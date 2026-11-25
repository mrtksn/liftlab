#!/usr/bin/env python3
"""Local-only bench panel for LiftLab flight firmware. Access via an SSH tunnel."""
import http.server, json, math, os, secrets, struct, threading, time, zlib
from pathlib import Path
import serial

HERE = Path(__file__).resolve().parent
TOKEN = secrets.token_urlsafe(24)
lock = threading.RLock()
state = {'telemetry': None, 'nav': None, 'reports': [], 'updated': 0, 'nav_updated': 0, 'error': None, 'wiring_ok': False}
run = {'until': 0, 'values': [0, 0, 0, 0], 'seq': 0}

def frame(kind, payload=b''):
    b = bytes([kind]) + struct.pack('<I', len(payload)) + payload
    return b'DF' + b + struct.pack('<I', zlib.crc32(b) & 0xffffffff)

def command(motor=-1, value=0):
    # A 48-byte, disarmed command requests NAV samples; never arms flight.
    return frame(3, struct.pack('<12f', 0, 0, 0, 0, 0, motor, value, 0, 0, 0, 0, 0))

def reader():
    try:
        with serial.Serial('/dev/serial0', 921600, timeout=0.02, exclusive=True) as s:
            buf = b''; next_cmd = 0; next_show = 0
            while True:
                now = time.monotonic()
                if now >= next_cmd:
                    with lock:
                        active = state['wiring_ok'] and now < run['until'] and state['telemetry'] is not None and now - state['updated'] < 1 and int(state['telemetry'][13]) & 16
                        values = list(run['values']) if active else None
                    s.write(frame(15, struct.pack('<4f', *values)) if values is not None else command()); next_cmd = now + .1
                if now >= next_show:
                    s.write(frame(5, b'show')); next_show = now + 10
                buf += s.read(4096)
                while True:
                    i = buf.find(b'DF')
                    if i < 0: buf = buf[-1:]; break
                    buf = buf[i:]
                    if len(buf) < 7: break
                    n = struct.unpack_from('<I', buf, 3)[0]
                    if n > 4096: buf = buf[2:]; continue
                    if len(buf) < n + 11: break
                    body = buf[2:7+n]; crc = struct.unpack_from('<I', buf, 7+n)[0]; buf = buf[11+n:]
                    if zlib.crc32(body) & 0xffffffff != crc: continue
                    kind, data = body[0], body[5:]
                    with lock:
                        if kind == 0x83 and len(data) == 144:
                            v = struct.unpack('<36f', data)
                            if all(math.isfinite(x) for x in v): state['telemetry'] = v; state['updated'] = time.monotonic()
                        elif kind == 0x84 and len(data) == 64:
                            v = struct.unpack('<16f', data)
                            if all(math.isfinite(x) for x in v): state['nav'] = v; state['nav_updated'] = time.monotonic()
                        elif kind in (0x81, 0x82):
                            msg = data.decode('utf-8', 'replace'); state['reports'] = (state['reports'] + [msg])[-18:]
                            if msg.startswith('motors='): state['wiring_ok'] = 'motors=18,22,23,19 ' in msg and 'motor_driver=0,0,0,0 ' in msg and 'esc_hz=490 ' in msg and 'esc_us=1070,1940,550 ' in msg
    except Exception as e:
        with lock: state['error'] = str(e); run['until'] = 0

class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def reply(self, code, data, mime='application/json'):
        self.send_response(code); self.send_header('Content-Type', mime); self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff'); self.end_headers(); self.wfile.write(data)
    def valid(self):
        return self.headers.get('Host') in ('127.0.0.1:8766', 'localhost:8766') and self.headers.get('X-Bench-Token') == TOKEN and self.headers.get('Origin') in (None, 'http://127.0.0.1:8766', 'http://localhost:8766')
    def do_GET(self):
        if self.path == '/':
            html = (HERE / 'index.html').read_text().replace('__TOKEN__', TOKEN)
            self.reply(200, html.encode(), 'text/html; charset=utf-8')
        elif self.path == '/state' and self.valid():
            with lock:
                d = dict(state); d['age'] = time.monotonic() - d.pop('updated'); d['nav_age'] = time.monotonic() - d.pop('nav_updated'); d['running'] = time.monotonic() < run['until']
            self.reply(200, json.dumps(d, allow_nan=False).encode())
        else: self.reply(404, b'{}')
    def do_POST(self):
        if not self.valid() or self.path not in ('/run', '/stop'):
            return self.reply(403, b'{}')
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length > 512: raise ValueError('request too large')
            data = json.loads(self.rfile.read(length) or b'{}')
            with lock:
                seq = data['seq']
                if type(seq) is not int or seq <= run['seq']: raise ValueError('stale request')
                run['seq'] = seq
                if self.path == '/stop': run['until'] = 0
                else:
                    values = data['values']
                    t = state['telemetry']; now = time.monotonic()
                    if not isinstance(values, list) or len(values) != 4 or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not 0 <= v <= 1 for v in values) or not any(values): raise ValueError('invalid motor values')
                    if not state['wiring_ok'] or state['error'] or not t or now-state['updated'] > 1 or int(t[1]) not in (0,4) or not (int(t[13]) & 16): raise ValueError('fresh disarmed/test telemetry and airframe required')
                    run.update(values=list(values), until=now+.35)
            self.reply(200, b'{}')
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as e: self.reply(400, json.dumps({'error':str(e)}).encode())

if __name__ == '__main__':
    threading.Thread(target=reader, daemon=True).start()
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 8766), Handler)
    print('Bench panel at http://127.0.0.1:8766/ (SSH tunnel required)', flush=True)
    server.serve_forever()
