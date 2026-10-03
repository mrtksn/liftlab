#!/usr/bin/env python3
"""Push buttons on a Raspberry Pi's GPIO for the command module: each button (to ground) sends press/release to
dfb_ground over UDP. Run dfb_ground on the same Pi, with --latch arm,fly so the arm and fly buttons toggle.

  sudo apt install python3-gpiozero
  ./dfb_ground --tx /dev/ttyUSB0 --latch arm,fly &
  python3 buttons_pi.py

Change PINS for your wiring (BCM numbers). The stick buttons are repeated while held: dfb_ground lets a stick go
back to the centre a second after it last heard of it."""
import socket, time
from gpiozero import Button

PINS = {'arm': 5, 'fly': 6, 'up': 13, 'down': 19, 'fwd': 26, 'back': 21, 'left': 20, 'right': 16, 'yawl': 12, 'yawr': 25, 'hold': 24, 'home': 23}
STICK = {'right', 'left', 'fwd', 'back', 'up', 'down', 'yawr', 'yawl'}
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
say = lambda line: s.sendto(line.encode(), ('127.0.0.1', 14561))
buttons = {n: Button(p) for n, p in PINS.items()}
was = {n: False for n in PINS}
while True:
    for n, b in buttons.items():
        down = b.is_pressed
        if down and (n in STICK or not was[n]): say(f'press {n}')
        if not down and was[n]: say(f'release {n}')
        was[n] = down
    time.sleep(0.05)
