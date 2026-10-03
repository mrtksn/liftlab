#!/usr/bin/env python3
"""A gamepad for the command module on a Mac (or anywhere pygame runs): reads the pad and sends its sticks and
buttons to dfb_ground over UDP, 50 times a second. (On Linux, dfb_ground --joystick reads a pad itself.)

  pip install pygame
  ./dfb_ground --tx /dev/tty.usbserial-XXXX --latch arm,fly    (in another terminal)
  python3 gamepad.py [HOST]

Mode 2, an Xbox-style layout: left stick throttle and yaw, right stick pitch and roll; LB arm, RB take off/land
(both latch: press once on, once off), A hold, Y home, X calibrate. Change AXES and BUTTONS for your pad."""
import socket, sys, time
import pygame

HOST = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
AXES = {'roll': (2, 1), 'pitch': (3, -1), 'throttle': (1, -1), 'yaw': (0, 1)}   # name: (pad axis, sign)
BUTTONS = {'arm': 4, 'fly': 5, 'hold': 0, 'home': 3, 'cal': 2}                # name: pad button

pygame.init(); pygame.joystick.init()
if not pygame.joystick.get_count(): sys.exit('no gamepad found')
pad = pygame.joystick.Joystick(0); pad.init(); print('gamepad:', pad.get_name())
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
was = {}
while True:
    pygame.event.pump()
    lines = [f'stick {n} {sign * pad.get_axis(a):.3f}' for n, (a, sign) in AXES.items() if a < pad.get_numaxes()]
    for n, b in BUTTONS.items():
        down = b < pad.get_numbuttons() and pad.get_button(b)
        if down != was.get(n): lines.append(f'{"press" if down else "release"} {n}'); was[n] = down
    s.sendto('\n'.join(lines).encode(), (HOST, 14561))
    time.sleep(0.02)   # the sticks lapse after a second without news, so if this stops, they centre
