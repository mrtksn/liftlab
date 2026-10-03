#!/usr/bin/env python3
"""Flies a square with the command module, as a script: dfb_ground (on this computer, or another one on the
network) takes text commands over UDP and sends them up the radio as channels and commands.

  ./dfb_ground --tx /dev/tty.usbserial-XXXX          (in another terminal)
  python3 fly_square.py [HOST] [SIDE_METRES]

The drone does the flying: the go-tos are targets for its navigation, within its own limits and failsafes.
Ctrl-C lands it (the fly switch off)."""
import socket, sys, time

HOST = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
SIDE = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(0.5)

def say(line):
    s.sendto(line.encode(), (HOST, 14561))
    try: return s.recvfrom(2000)[0].decode()
    except socket.timeout: return ''

def wait(seconds):
    end = time.time() + seconds
    while time.time() < end:
        print(say('status')); time.sleep(1)

try:
    say('press arm'); time.sleep(1)        # arm first (the flight core won't arm with the throttle up),
    say('press fly'); wait(8)               # then take off; it waits for its position to settle
    for x, y in [(SIDE, 0), (SIDE, -SIDE), (0, -SIDE), (0, 0)]:   # x north, y west: -y is east
        print(say(f'goto {x} {y} 2'))
        wait(6)
finally:
    print(say('release fly'))               # lands where it is
    wait(6)
    say('release arm')
