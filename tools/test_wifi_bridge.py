#!/usr/bin/env python3
"""runner/ground/wifi_bridge.py on this computer: a WebSocket client as the page, a UDP socket as the drone's Pi.
Packets both ways, one message to one datagram; the hello; pages from elsewhere and a second page refused; ping;
a close. Python 3 alone: python3 tools/test_wifi_bridge.py"""
import base64, json, os, socket, struct, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'runner', 'ground'))
import wifi_bridge as W

def ws_connect(port, origin='https://mrtksn.github.io'):
    s = socket.create_connection(('127.0.0.1', port), timeout=3)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f'GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n' + (f'Origin: {origin}\r\n' if origin else '') + '\r\n').encode())
    head = b''
    while b'\r\n\r\n' not in head:
        d = s.recv(1);
        if not d: break
        head += d
    status = head.split(b'\r\n')[0].decode()
    if ' 101 ' in status: assert W.accept_key(key).encode() in head, 'Sec-WebSocket-Accept'
    return s, status

def send(s, op, payload):
    mask = os.urandom(4); n = len(payload)
    head = struct.pack('!BB', 0x80 | op, 0x80 | n) if n < 126 else struct.pack('!BBH', 0x80 | op, 0x80 | 126, n)
    s.sendall(head + mask + bytes(c ^ mask[i & 3] for i, c in enumerate(payload)))

def recv(s):
    def take(n):
        b = b''
        while len(b) < n:
            d = s.recv(n - len(b))
            if not d: raise EOFError
            b += d
        return b
    b0, b1 = take(2); n = b1 & 0x7F
    if n == 126: n = struct.unpack('!H', take(2))[0]
    assert not b1 & 0x80, 'the server does not mask'
    return b0 & 0x0F, take(n)

def main():
    drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); drone.bind(('127.0.0.1', 0)); drone.settimeout(3)
    logs = []
    B = W.Bridge(drone.getsockname(), 0, ['https://other.example'], log=logs.append)
    threading.Thread(target=B.serve_forever, daemon=True).start()
    try:
        # origins
        assert W.origin_ok('https://mrtksn.github.io') and W.origin_ok('http://localhost:8765') and W.origin_ok('http://127.0.0.1') and W.origin_ok('null')
        assert not W.origin_ok('https://evil.example') and not W.origin_ok('http://localhost.evil.example') and not W.origin_ok(None)
        _, st = ws_connect(B.port, 'https://evil.example'); assert ' 403 ' in st, st
        _, st = ws_connect(B.port, None); assert ' 403 ' in st, st
        print('origins: LiftLab, local pages and --origin allowed; others refused')
        # a page: the hello, then packets both ways
        p, st = ws_connect(B.port); assert ' 101 ' in st, st
        op, hello = recv(p); assert op == 1 and json.loads(hello)['drone'] == '%s:%d' % drone.getsockname(), hello
        pkts = [bytes([k]) * (k * 7 % 250 + 1) for k in range(1, 40)]
        for x in pkts: send(p, 2, x)
        got = [drone.recvfrom(2048) for _ in pkts]
        assert [g[0] for g in got] == pkts, 'one message, one datagram, in order'
        back = got[0][1]
        for x in pkts[:10]: drone.sendto(x[::-1], back)
        for x in pkts[:10]:
            op, d = recv(p); assert op == 2 and d == x[::-1]
        print('packets: 39 up as 39 datagrams, 10 down as 10 messages, unchanged')
        # not from the drone's address: dropped (the UDP socket is connected)
        stranger = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); stranger.sendto(b'x' * 20, back); stranger.close()
        drone.sendto(b'after', back); op, d = recv(p); assert d == b'after', d
        print('a datagram from elsewhere: not passed to the page')
        # a second page while the first is connected
        _, st = ws_connect(B.port); assert ' 409 ' in st, st
        print('a second page: refused while the first is connected')
        # ping, too big, close
        send(p, 9, b'hi'); op, d = recv(p); assert op == 10 and d == b'hi'
        send(p, 2, b'z' * 2000); drone.settimeout(0.3)
        try: drone.recvfrom(4096); assert False, 'a message too big for a packet went'
        except socket.timeout: pass
        send(p, 8, struct.pack('!H', 1000)); op, _ = recv(p); assert op == 8
        p.close(); time.sleep(0.3)
        p2, st = ws_connect(B.port, 'http://localhost:8765'); assert ' 101 ' in st, st; recv(p2); p2.close()
        print('ping answered, an oversized message dropped, close; the next page connects')
        assert B.counts['up'] == 39 and B.counts['down'] == 11, B.counts
    finally:
        B.close()
    print('wifi bridge: ok')

if __name__ == '__main__':
    main()
