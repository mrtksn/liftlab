#!/usr/bin/env python3
"""Talk to the flight firmware (runner/fc/esp32) over its USB serial port: from the Raspberry Pi, or a Mac.

    python3 fly.py PORT watch                      telemetry and messages
    python3 fly.py PORT status                     what it's doing, and its wiring
    python3 fly.py PORT airframe drone.dfa         send the airframe (simulator: Airframe → Export for the flight controller)
    python3 fly.py PORT program formulas.rnp       send flight formulas (hot reload, as send_program.py)
    python3 fly.py PORT set motors=25,26,27,14 servos=16,17   change the wiring (then: save, reboot)
    python3 fly.py PORT save | reboot | show
    python3 fly.py PORT test 1 0.1                 spin motor 1 at 10% for 2 s (PROPS OFF)
    python3 fly.py PORT fly [--gamepad]            fly: keyboard (tethered bench tests) or a gamepad

PORT: /dev/ttyUSB0 on the Pi, /dev/cu.usbserial-0001 on a Mac. 115200 baud unless --baud.

Flying: the firmware needs commands 50 times a second; fly.py sends them while it runs. If fly.py stops, the
cable comes out or the computer hangs, the drone goes to its failsafe after 0.5 s: it levels, comes down at about
1 m/s and switches off when it lands. Keys (the sticks spring back to the middle when you let go):
    i / k   lean forward / back          j / l   lean left / right        u / o   turn left / right
    w / s   throttle up / down (stays)   h       throttle to the middle (hover, or hold height with a barometer)
    Enter   arm (throttle at 0)          space   DISARM: motors off at once          q   disarm and quit
A keyboard is only good for tethered tests. Gamepad (pip install pygame): left stick throttle (up/down) and turn,
right stick lean; A arms, B disarms. Needs pyserial (pip install pyserial).
"""
import argparse, os, select, struct, sys, threading, time, zlib

PROGRAM, STATUS, CMD, AIRFRAME, SETTING = 1, 2, 3, 4, 5
EVENT, REPORT, TELEM = 0x81, 0x82, 0x83
STATES = ['disarmed', 'ARMED', 'FAILSAFE', 'CRASHED', 'motor test']

def frame(ftype, payload=b''):
    body = bytes([ftype]) + struct.pack('<I', len(payload)) + payload
    return b'DF' + body + struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)

def cmd_frame(arm=0, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, test_motor=-1, test_throttle=0.0):
    return frame(CMD, struct.pack('<7f', arm, roll, pitch, yaw, throttle, test_motor, test_throttle))

class Link:
    """Frames and plain text (the boot log) from the serial port."""
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.02); self.buf = b''; self.lock = threading.Lock()
    def send(self, data):
        with self.lock: self.s.write(data)
    def read(self):
        self.buf += self.s.read(4096); out = []
        if not hasattr(self, 'text'): self.text = b''
        while self.buf:
            i = self.buf.find(b'DF')
            if i != 0:                                    # plain text before the next frame (the boot log, printf)
                cut = len(self.buf) - (1 if self.buf.endswith(b'D') else 0) if i < 0 else i
                self.text += self.buf[:cut]; self.buf = self.buf[cut:]
                *lines, self.text = self.text.split(b'\n')
                out += [('text', l.decode('utf-8', 'replace').rstrip()) for l in lines if l.strip()]
                if i < 0: return out
                continue
            if len(self.buf) < 7: return out
            ftype, n = self.buf[2], struct.unpack('<I', self.buf[3:7])[0]
            if n > 4096: self.text += b'DF'; self.buf = self.buf[2:]; continue
            if len(self.buf) < 11 + n: return out
            body, crc = self.buf[2:7 + n], struct.unpack('<I', self.buf[7 + n:11 + n])[0]
            if zlib.crc32(body) & 0xFFFFFFFF != crc: self.text += b'DF'; self.buf = self.buf[2:]; continue
            self.buf = self.buf[11 + n:]; out.append((ftype, body[5:]))
        return out

def telem(payload):
    t = struct.unpack('<36f', payload)
    return {'t': t[0], 'state': int(t[1]), 'roll': t[2], 'pitch': t[3], 'yaw': t[4], 'rates': t[5:8], 'alt': t[8], 'vz': t[9],
            'batt': t[10], 'loop': t[11], 'loop_max': t[12], 'flags': int(t[13]), 'slot': int(t[14]), 'trap': int(t[15]),
            'motors': t[16:28], 'servos': t[28:36]}

def telem_line(d, n_motors=4):
    f = d['flags']
    s = STATES[d['state']] if 0 <= d['state'] < len(STATES) else '?'
    parts = [f"{s:10s}", f"roll {d['roll']:6.1f} pitch {d['pitch']:6.1f} yaw {d['yaw']:6.1f}"]
    parts.append(f"height {d['alt']:5.2f} m {d['vz']:+5.2f} m/s" if f & 2 else f"vz {d['vz']:+5.2f} m/s (no baro)")
    if d['batt'] > 0: parts.append(f"{d['batt']:5.2f} V")
    parts.append('motors ' + ' '.join(f"{m:4.2f}" for m in d['motors'][:n_motors]))
    parts.append(f"loop {d['loop']:.0f}/{d['loop_max']:.0f} us")
    if not f & 1: parts.append('NO GYRO')
    elif not f & 4: parts.append('settling')
    if not f & 16: parts.append('NO AIRFRAME')
    return ' | '.join(parts)

def show(ftype, payload, n_motors):
    if ftype == 'text': print('  ' + payload)
    elif ftype == EVENT: print('> ' + payload.decode('utf-8', 'replace'))
    elif ftype == REPORT: print('= ' + payload.decode('utf-8', 'replace'))
    elif ftype == TELEM and len(payload) == 144: return telem(payload)

def listen(L, seconds, until=None, n_motors=4):
    end = time.time() + seconds
    while time.time() < end:
        for ftype, p in L.read():
            show(ftype, p, n_motors)
            if until and ftype in (EVENT, REPORT) and until(p.decode('utf-8', 'replace')): end = min(end, time.time() + 0.3)

class Sender(threading.Thread):
    """Sends the current command 50 times a second."""
    def __init__(self, L):
        super().__init__(daemon=True); self.L = L; self.c = dict(arm=0, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, test_motor=-1, test_throttle=0.0)
        self.run_ = True
    def run(self):
        nxt = time.time()
        while self.run_:
            self.L.send(cmd_frame(**self.c)); nxt += 0.02; time.sleep(max(0, nxt - time.time()))
    def stop(self):
        self.c.update(arm=0, throttle=0.0, test_motor=-1)
        for _ in range(5): self.L.send(cmd_frame(**self.c)); time.sleep(0.02)
        self.run_ = False

def fly(L, gamepad):
    S = Sender(L); S.start(); c = S.c
    pad = None
    if gamepad:
        import pygame
        pygame.init(); pygame.joystick.init()
        if not pygame.joystick.get_count(): sys.exit('no gamepad found')
        pad = pygame.joystick.Joystick(0); pad.init(); print('gamepad:', pad.get_name())
    import termios, tty
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setcbreak(fd)
    held = {}; last = None; tshow = 0
    try:
        print(__doc__.split('Flying:')[1].split('A keyboard')[0])
        while True:
            now = time.time()
            while select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == ' ': c.update(arm=0, throttle=0.0); print('\nDISARM')
                elif k == 'q': return
                elif k in '\r\n':
                    if c['throttle'] > 0.05: print('\nthrottle to 0 first (s)')
                    else: c['arm'] = 1; print('\narming')
                elif k == 'w': c['throttle'] = min(1.0, round(c['throttle'] + 0.05, 2))
                elif k == 's': c['throttle'] = max(0.0, round(c['throttle'] - 0.05, 2))
                elif k == 'h': c['throttle'] = 0.5
                elif k in 'ikjluo': held[k] = now
            if pad:
                import pygame
                pygame.event.pump()
                ax = lambda i: pad.get_axis(i) if pad.get_numaxes() > i else 0.0
                dz = lambda v: 0.0 if abs(v) < 0.08 else v
                c['throttle'] = max(0.0, min(1.0, 0.5 - 0.5 * dz(ax(1)))) if c['arm'] else 0.0
                c['yaw'] = -dz(ax(0)); c['roll'] = dz(ax(3)); c['pitch'] = -dz(ax(4))
                if pad.get_numbuttons() > 1:
                    if pad.get_button(1): c.update(arm=0, throttle=0.0)
                    elif pad.get_button(0) and not c['arm']: c['arm'] = 1
            else:   # a key counts as held while the terminal repeats it
                on = lambda k: now - held.get(k, 0) < 0.6
                c['pitch'] = 0.5 * (on('i') - on('k')); c['roll'] = 0.5 * (on('l') - on('j')); c['yaw'] = 0.5 * (on('u') - on('o'))
            for ftype, p in L.read():
                if ftype == TELEM and len(p) == 144: last = telem(p)
                elif ftype in (EVENT, REPORT, 'text'):
                    print('\n' + ('> ' if ftype == EVENT else '  ') + (p if isinstance(p, str) else p.decode('utf-8', 'replace')))
            if last and now - tshow > 0.1:
                tshow = now
                sys.stdout.write('\r' + telem_line(last) + f" | sticks thr {c['throttle']:.2f} r {c['roll']:+.1f} p {c['pitch']:+.1f} y {c['yaw']:+.1f}  "); sys.stdout.flush()
            time.sleep(0.01)
    finally:
        S.stop(); termios.tcsetattr(fd, termios.TCSADRAIN, old); print('\ndisarmed, bye')

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('port'); ap.add_argument('what', nargs='+'); ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--gamepad', action='store_true'); ap.add_argument('--yes', action='store_true', help="don't ask about props for a motor test")
    a = ap.parse_args()
    L = Link(a.port, a.baud); w = a.what
    if w[0] == 'watch':
        last = 0
        try:
            while True:
                for ftype, p in L.read():
                    d = show(ftype, p, 4)
                    if d and time.time() - last > 0.2: last = time.time(); print(telem_line(d))
        except KeyboardInterrupt: pass
    elif w[0] == 'status':
        L.send(frame(STATUS)); L.send(frame(SETTING, b'show')); listen(L, 1.5)
    elif w[0] in ('show', 'save', 'reboot', 'defaults'):
        L.send(frame(SETTING, w[0].encode())); listen(L, 1.5)
    elif w[0] == 'set':
        for kv in w[1:]: L.send(frame(SETTING, kv.encode())); listen(L, 0.5)
        print('(send "save" and "reboot" to use the new wiring)')
    elif w[0] == 'airframe':
        blob = open(w[1], 'rb').read()
        if blob[:4] != b'DFBA': sys.exit('not an airframe file (.dfa from the simulator)')
        L.send(frame(AIRFRAME, blob)); print(f'sent {len(blob)} bytes')
        listen(L, 3, until=lambda s: s.startswith('airframe'))
    elif w[0] == 'program':
        img = open(w[1], 'rb').read()
        if img[:4] != struct.pack('<I', 0x52464244): sys.exit('not a program image')
        L.send(frame(PROGRAM, img)); print(f'sent {len(img)} bytes')
        listen(L, 10 + len(img) / 10000, until=lambda s: s.startswith(('program swapped', 'program rejected')))
    elif w[0] == 'test':
        m, thr = int(w[1]), float(w[2]); secs = min(3.0, float(w[3])) if len(w) > 3 else 2.0
        if not a.yes and input(f'Spin motor {m} at {thr:.0%} for {secs:.1f} s. Props off? [y/N] ').strip().lower() != 'y': return
        S = Sender(L); S.c.update(test_motor=m - 1, test_throttle=thr); S.start()
        try: listen(L, secs)
        finally: S.stop()
        listen(L, 0.3)
    elif w[0] == 'fly': fly(L, a.gamepad)
    else: ap.error('unknown: ' + w[0])

if __name__ == '__main__': main()
