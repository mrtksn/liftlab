#!/usr/bin/env python3
"""Fly the A13 toy drone from a Mac: live video from its camera board plus stick commands over Wi-Fi.

    .venv/bin/python tools/a13_pilot.py              join the drone's Wi-Fi first (192.168.1.1)
    .venv/bin/python tools/a13_pilot.py --gamepad    left stick throttle and turn, right stick lean

The protocol is in docs/a13-wifi-protocol.md: a 21-byte control packet to UDP 7099 about 18 times a second,
a heartbeat once a second, and MJPEG video at rtsp://192.168.1.1:7070/webcam. Needs pygame (pip install pygame)
and ffmpeg (brew install ffmpeg).

Keys (sticks spring back to the middle when you let go; the drone's throttle middle means hold height):
    w / s   throttle up / down        a / d   turn left / right
    arrows  lean forward, back, left, right
    1 2 3   stick strength 30 / 60 / 100 %
    t       take off      l or space   land      c   switch camera      q or Esc   land and quit
Gamepad: A takes off, B lands, X switches camera.

Nothing here is a kill switch: the stock app sends no motor-off command that we've seen. Fly with props off
until the failsafe (what the drone does when packets stop) has been checked.
"""
import argparse, socket, subprocess, sys, threading, time

CONTROL_PORT, VIDEO_URL = 7099, 'rtsp://{host}:7070/webcam'
WIDTH, HEIGHT = 640, 352
HEARTBEAT, GOODBYE = b'\x01\x01', b'\x08\x01'
TAKEOFF, LAND = 0x01, 0x02
SEND_HZ, FLAG_HOLD = 18, 1.0           # the app's packet rate, and how long it holds take-off/land

def camera_packet(n):
    return bytes([0x06, n])

def stick_byte(v):
    """-1..1 to the drone's 0x00..0xFF with 0x80 in the middle."""
    v = max(-1.0, min(1.0, v))
    return max(0, min(255, round(0x80 + v * (0x7F if v > 0 else 0x80))))

def control_packet(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0, flags=0, mode=0x02):
    body = bytes([stick_byte(roll), stick_byte(pitch), stick_byte(throttle), stick_byte(yaw), flags, mode]) + bytes(10)
    chk = 0
    for b in body: chk ^= b
    return bytes([0x03, 0x66, 0x14]) + body + bytes([chk, 0x99])

class Link:
    """Sends control packets and heartbeats on its own thread, so the rate holds while the window is busy."""
    def __init__(self, host):
        self.addr = (host, CONTROL_PORT)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.s.setblocking(False)
        self.sticks = dict(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0)
        self.flag, self.flag_until = 0, 0.0
        self.last_reply, self.replies = 0.0, 0
        self.lock = threading.Lock(); self.running = True
        self.t = threading.Thread(target=self.run, daemon=True)

    def start(self): self.t.start(); return self

    def send(self, data):
        try: self.s.sendto(data, self.addr)
        except OSError: pass                   # not on the drone's Wi-Fi yet: keep trying

    def command(self, flag):
        with self.lock: self.flag, self.flag_until = flag, time.time() + FLAG_HOLD

    def run(self):
        beat = 0.0
        while self.running:
            now = time.time()
            if now - beat >= 1.0: self.send(HEARTBEAT); beat = now
            with self.lock:
                if now > self.flag_until: self.flag = 0
                pkt = control_packet(flags=self.flag, **self.sticks)
            self.send(pkt)
            try:
                while True: self.s.recv(64); self.last_reply = time.time(); self.replies += 1
            except (BlockingIOError, OSError): pass
            time.sleep(1 / SEND_HZ)

    def stop(self):
        with self.lock: self.sticks = dict(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0)
        self.command(LAND); time.sleep(0.3)
        self.running = False; self.t.join(1)
        self.send(GOODBYE); self.s.close()

class Video:
    """ffmpeg decodes the RTSP stream to raw frames; only the newest frame is kept, so there's no backlog."""
    def __init__(self, url):
        self.url, self.frame, self.frames, self.p = url, None, 0, None
        self.running = True; self.restart = False
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        size = WIDTH * HEIGHT * 3
        while self.running:
            self.restart = False
            timeout = ['-timeout', '3000000'] if self.url.startswith('rtsp') else []    # give up on a silent stream
            self.p = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-fflags', 'nobuffer', '-flags', 'low_delay',
                                       *timeout, '-i', self.url, '-vf', f'scale={WIDTH}:{HEIGHT}',
                                       '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
            while self.running and not self.restart:
                buf = self.p.stdout.read(size)
                if len(buf) < size: break
                self.frame = buf; self.frames += 1
            self.p.kill(); self.p.wait()
            if self.running: time.sleep(0.3 if self.restart else 1)    # give the board a moment, or retry a dropped stream

    def reopen(self):          # after a camera switch the app re-opens the stream too
        self.restart = True
        if self.p: self.p.kill()

    def stop(self):
        self.running = False
        if self.p: self.p.kill()

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--host', default='192.168.1.1')
    ap.add_argument('--video', help='video URL (default: the drone\'s RTSP stream)')
    ap.add_argument('--gamepad', action='store_true')
    ap.add_argument('--seconds', type=float, help='quit after this long (for tests)')
    a = ap.parse_args()

    import pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT + 40))
    pygame.display.set_caption('A13 pilot')
    font = pygame.font.SysFont('menlo', 14)
    pad = None
    if a.gamepad:
        pygame.joystick.init()
        if not pygame.joystick.get_count(): sys.exit('no gamepad found')
        pad = pygame.joystick.Joystick(0); pad.init(); print('gamepad:', pad.get_name())

    link = Link(a.host).start()
    video = Video(a.video or VIDEO_URL.format(host=a.host))
    camera, strength, note, note_until = 1, 0.6, '', 0.0
    clock, t0, fps_t, fps_n, fps = pygame.time.Clock(), time.time(), time.time(), 0, 0.0
    try:
        while not a.seconds or time.time() - t0 < a.seconds:
            now = time.time()
            for e in pygame.event.get():
                if e.type == pygame.QUIT: return
                if e.type == pygame.KEYDOWN:
                    k = e.key
                    if k in (pygame.K_q, pygame.K_ESCAPE): return
                    elif k == pygame.K_t: link.command(TAKEOFF); note = 'TAKE OFF'
                    elif k in (pygame.K_l, pygame.K_SPACE): link.command(LAND); note = 'LAND'
                    elif k == pygame.K_c:
                        camera = 2 if camera == 1 else 1
                        link.send(camera_packet(camera)); video.reopen(); note = f'camera {camera}'
                    elif k in (pygame.K_1, pygame.K_2, pygame.K_3):
                        strength = {pygame.K_1: 0.3, pygame.K_2: 0.6, pygame.K_3: 1.0}[k]; note = f'strength {strength:.0%}'
                    else: continue
                    note_until = now + 1.5
                if e.type == pygame.JOYBUTTONDOWN:
                    if e.button == 0: link.command(TAKEOFF); note = 'TAKE OFF'
                    elif e.button == 1: link.command(LAND); note = 'LAND'
                    elif e.button == 2:
                        camera = 2 if camera == 1 else 1
                        link.send(camera_packet(camera)); video.reopen(); note = f'camera {camera}'
                    note_until = now + 1.5

            if pad:
                ax = lambda i: pad.get_axis(i) if pad.get_numaxes() > i else 0.0
                dz = lambda v: 0.0 if abs(v) < 0.08 else v
                sticks = dict(throttle=-dz(ax(1)), yaw=dz(ax(0)), roll=dz(ax(3)), pitch=-dz(ax(4)))
            else:
                key = pygame.key.get_pressed()
                axis = lambda plus, minus: strength * (key[plus] - key[minus])
                sticks = dict(throttle=axis(pygame.K_w, pygame.K_s), yaw=axis(pygame.K_d, pygame.K_a),
                              pitch=axis(pygame.K_UP, pygame.K_DOWN), roll=axis(pygame.K_RIGHT, pygame.K_LEFT))
            with link.lock: link.sticks = sticks

            frame = video.frame
            if frame:
                screen.blit(pygame.image.frombuffer(frame, (WIDTH, HEIGHT), 'RGB'), (0, 0))
            else:
                screen.fill((20, 20, 20), (0, 0, WIDTH, HEIGHT))
                screen.blit(font.render('waiting for video…', True, (200, 200, 200)), (WIDTH // 2 - 80, HEIGHT // 2))
            if video.frames != fps_n and now - fps_t >= 1:
                fps, fps_t, fps_n = (video.frames - fps_n) / (now - fps_t), now, video.frames
            linked = now - link.last_reply < 1.0
            screen.fill((0, 0, 0), (0, HEIGHT, WIDTH, 40))
            status = (f"{'LINK' if linked else 'no link'}  cam {camera}  {fps:4.1f} fps   "
                      f"thr {sticks['throttle']:+.2f} yaw {sticks['yaw']:+.2f} "
                      f"pit {sticks['pitch']:+.2f} rol {sticks['roll']:+.2f}")
            screen.blit(font.render(status, True, (120, 230, 120) if linked else (230, 120, 120)), (8, HEIGHT + 4))
            help_line = note if now < note_until else 'w/s thr  a/d turn  arrows lean  t take off  l land  c camera  q quit'
            screen.blit(font.render(help_line, True, (220, 220, 220)), (8, HEIGHT + 21))
            pygame.display.flip()
            clock.tick(60)
    finally:
        link.stop(); video.stop(); pygame.quit()
        print(f'sent goodbye; {link.replies} replies from the drone, {video.frames} video frames')

if __name__ == '__main__':
    main()
