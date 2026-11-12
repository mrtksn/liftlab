#!/usr/bin/env python3
"""LiftLab's page as the command module over Wi-Fi: a relay between the page and the drone.

A browser can't send UDP, and the Wi-Fi link (fc/radio_link.h, RLINK_WIFI) is UDP: the drone's Pi (dfb_pi --radio
wifi,ap,CHANNEL or wifi,sta) takes the packet layer's packets (fc/plink.h) in datagrams on port 14570. So the page
(js/live.js, "Connect to drone" with the Ground tab's link set to Wi-Fi) opens a WebSocket to this program on this
computer, and each binary message is one packet: what the page sends goes to the drone in one datagram, and each
datagram from the drone comes to the page in one message. Nothing is read or changed here: the packets are signed
with the binding phrase at both ends (this program doesn't know it), so a page without the phrase flies nothing.

    python3 runner/ground/wifi_bridge.py --drone 192.168.2.127          (the drone's Pi; :PORT if not 14570)
    python3 runner/ground/wifi_bridge.py --drone liftlab-drone.local --port 14571 --origin https://example.org

It listens on 127.0.0.1 only (--port, 14571 by default): pages on this computer, opened from LiftLab's address
(https://mrtksn.github.io), from a local server (http://localhost or 127.0.0.1, any port) or from a file, and any
--origin added. One page at a time: a second is refused while the first is connected. On connecting, the page gets
one text message, {"drone": "HOST:PORT"}: where the packets go. Python 3 alone, no packages.
"""
import argparse, base64, hashlib, json, select, socket, struct, sys, threading, time

DEFAULT_PORT = 14571
DRONE_PORT = 14570                      # fc/radio_link.h RLINK_UDP_PORT
MAX_PACKET = 1024                       # (plink's are 250 at most: PLINK_MTU)
ORIGINS = ('https://mrtksn.github.io',)
WS_GUID = b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


def origin_ok(origin, extra=()):
    """The pages allowed: LiftLab's site, a local server, a file (Origin "null"), and those added."""
    if origin is None: return False
    o = origin.strip().rstrip('/')
    if o in ORIGINS or o in extra or o == 'null': return True
    for host in ('http://localhost', 'http://127.0.0.1', 'http://[::1]'):
        if o == host or (o.startswith(host + ':') and o[len(host) + 1:].isdigit()): return True
    return False


def accept_key(key):
    return base64.b64encode(hashlib.sha1(key.encode() + WS_GUID).digest()).decode()


def read_request(sock, limit=8192):
    """The HTTP upgrade request: (path, headers with lowercase names), or None."""
    data = b''
    while b'\r\n\r\n' not in data:
        chunk = sock.recv(1024)
        if not chunk or len(data) > limit: return None
        data += chunk
    head = data.split(b'\r\n\r\n', 1)[0].decode('latin-1').split('\r\n')
    parts = head[0].split(' ')
    if len(parts) < 3 or parts[0] != 'GET': return None
    headers = {}
    for line in head[1:]:
        if ':' in line:
            k, v = line.split(':', 1); headers[k.strip().lower()] = v.strip()
    return parts[1], headers


def frame(opcode, payload=b''):
    """One unmasked frame (server to page)."""
    n = len(payload)
    if n < 126: head = struct.pack('!BB', 0x80 | opcode, n)
    elif n < 65536: head = struct.pack('!BBH', 0x80 | opcode, 126, n)
    else: head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
    return head + payload


class FrameReader:
    """The page's frames (masked, as RFC 6455 has a client's), from the bytes as they come."""
    def __init__(self): self.buf = b''; self.parts = b''; self.op = 0

    def feed(self, data):
        self.buf += data; out = []
        while True:
            if len(self.buf) < 2: return out
            b0, b1 = self.buf[0], self.buf[1]; fin, op, masked, n = b0 & 0x80, b0 & 0x0F, b1 & 0x80, b1 & 0x7F
            i = 2
            if n == 126:
                if len(self.buf) < 4: return out
                n = struct.unpack('!H', self.buf[2:4])[0]; i = 4
            elif n == 127:
                if len(self.buf) < 10: return out
                n = struct.unpack('!Q', self.buf[2:10])[0]; i = 10
            if not masked: raise ValueError('an unmasked frame from the page')
            if n > MAX_PACKET * 4: raise ValueError('a frame too big for a packet')
            if len(self.buf) < i + 4 + n: return out
            mask = self.buf[i:i + 4]; body = bytes(c ^ mask[k & 3] for k, c in enumerate(self.buf[i + 4:i + 4 + n]))
            self.buf = self.buf[i + 4 + n:]
            if op >= 8: out.append((op, body)); continue            # (control frames: never fragmented)
            if op: self.op = op; self.parts = body
            else: self.parts += body
            if fin: out.append((self.op, self.parts)); self.parts = b''


class Bridge:
    def __init__(self, drone, port=DEFAULT_PORT, extra_origins=(), bind='127.0.0.1', log=print):
        self.drone = drone; self.port = port; self.extra = tuple(o.rstrip('/') for o in extra_origins); self.bind = bind
        self.log = log; self.busy = threading.Lock(); self.counts = {'up': 0, 'down': 0}
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((bind, port)); self.srv.listen(4); self.port = self.srv.getsockname()[1]
        self.stopping = False

    def serve_forever(self):
        while not self.stopping:
            try: c, _ = self.srv.accept()
            except OSError: break
            threading.Thread(target=self.client, args=(c,), daemon=True).start()

    def close(self):
        self.stopping = True
        try: self.srv.close()
        except OSError: pass

    def refuse(self, c, code, why):
        try: c.sendall(f'HTTP/1.1 {code}\r\nContent-Type: text/plain\r\nContent-Length: {len(why)}\r\nConnection: close\r\n\r\n{why}'.encode())
        except OSError: pass
        c.close()

    def client(self, c):
        c.settimeout(5)
        try: req = read_request(c)
        except OSError: req = None
        if not req: c.close(); return
        path, h = req
        origin = h.get('origin')
        if 'websocket' not in h.get('upgrade', '').lower() or 'sec-websocket-key' not in h:
            return self.refuse(c, '426 Upgrade Required', 'LiftLab Wi-Fi bridge: connect from LiftLab (Connect to drone)\n')
        if not origin_ok(origin, self.extra):
            self.log(f'refused a page from {origin}: not LiftLab (add it with --origin)')
            return self.refuse(c, '403 Forbidden', 'this page is not allowed\n')
        if not self.busy.acquire(blocking=False):
            self.log(f'refused a second page from {origin}: one at a time')
            return self.refuse(c, '409 Conflict', 'another page is connected\n')
        try:
            c.sendall(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                       f'Sec-WebSocket-Accept: {accept_key(h["sec-websocket-key"])}\r\n\r\n').encode())
            c.settimeout(None)
            self.log(f'page connected ({origin}): packets to {self.drone[0]}:{self.drone[1]}')
            self.relay(c)
        except OSError as e:
            self.log(f'page connection: {e}')
        finally:
            self.busy.release()
            try: c.close()
            except OSError: pass
            self.log('page disconnected')

    def relay(self, c):
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            u.connect(self.drone)                                  # (only the drone's datagrams come back here)
            c.sendall(frame(1, json.dumps({'drone': f'{self.drone[0]}:{self.drone[1]}'}).encode()))
            r = FrameReader(); errs = 0; warned = 0
            while True:
                ready, _, _ = select.select([c, u], [], [], 1.0)
                if c in ready:
                    data = c.recv(65536)
                    if not data: return
                    for op, body in r.feed(data):
                        if op == 8: c.sendall(frame(8, body[:2])); return
                        if op == 9: c.sendall(frame(10, body)); continue
                        if op == 2 and 0 < len(body) <= MAX_PACKET:
                            try: u.send(body); self.counts['up'] += 1
                            except OSError as e:               # (no route yet, the Pi's address not answering: a lost packet)
                                errs += 1
                                if time.monotonic() - warned > 5: warned = time.monotonic(); self.log(f'to the drone: {e}')
                if u in ready:
                    try: d = u.recv(MAX_PACKET + 1)
                    except OSError: d = b''                      # (ICMP port unreachable: dfb_pi isn't running there)
                    if 0 < len(d) <= MAX_PACKET: c.sendall(frame(2, d)); self.counts['down'] += 1
        except ValueError as e:
            self.log(f'page: {e}')
            try: c.sendall(frame(8, struct.pack('!H', 1002)))
            except OSError: pass
        finally:
            u.close()


def parse_drone(s):
    host, _, port = s.rpartition(':') if s.count(':') == 1 else (s, '', '')
    host = host or s
    try: p = int(port) if port else DRONE_PORT
    except ValueError: raise argparse.ArgumentTypeError(f'{s}: HOST or HOST:PORT')
    try: ip = socket.gethostbyname(host)
    except OSError as e: raise argparse.ArgumentTypeError(f"can't find {host}: {e} (is this computer on the drone's network?)")
    return ip, p


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--drone', required=True, type=parse_drone, help="the drone's Pi on the Wi-Fi network: HOST[:PORT] (port 14570)")
    ap.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'where the page connects, on 127.0.0.1 ({DEFAULT_PORT})')
    ap.add_argument('--origin', action='append', default=[], help='another page address allowed to connect (repeatable)')
    a = ap.parse_args(argv)
    try: b = Bridge(a.drone, a.port, a.origin)
    except OSError as e: sys.exit(f'wifi_bridge: port {a.port}: {e}')
    print(f'LiftLab Wi-Fi bridge: pages connect to ws://127.0.0.1:{b.port}, packets go to {a.drone[0]}:{a.drone[1]}. Ctrl+C stops it.', flush=True)
    def stats():
        last = dict(b.counts)
        while True:
            time.sleep(10)
            if b.counts != last: print(f"packets: {b.counts['up']} up, {b.counts['down']} down", flush=True); last = dict(b.counts)
    threading.Thread(target=stats, daemon=True).start()
    try: b.serve_forever()
    except KeyboardInterrupt: print()
    finally: b.close()


if __name__ == '__main__':
    main()
