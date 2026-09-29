#!/usr/bin/env python3
"""The companion computer's side of the link (runner/rn_link.h): send a program image to the flight controller
over a serial port and print what it reports.

    python3 send_program.py flight-formulas.rnp /dev/serial0 [--baud 921600]
    python3 send_program.py --status /dev/serial0
    python3 send_program.py flight.rnp /dev/serial0 --watch 8     (keep listening 8 s after it swaps in)

The image is what the simulator's "Download program for the drone" gives (or tools/export_program.js). The
drone checks it (every step's addresses, its self-tests, its formulas' inputs), flies it in the background for a
second, blends it in and reports each step; a bad program is rejected and the flying one keeps flying.
Needs pyserial (pip install pyserial).
"""
import argparse, struct, sys, time, zlib

PROGRAM, STATUS, EVENT, REPORT = 1, 2, 0x81, 0x82
MAX_PAYLOAD = 4096          # the drone only sends lines of text

def frame(ftype, payload=b''):
    body = bytes([ftype]) + struct.pack('<I', len(payload)) + payload
    return b'DF' + body + struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)

class Reader:
    """Collects frames from a byte stream (the same states as rn_link_feed)."""
    def __init__(self): self.buf = b''
    def feed(self, data):
        self.buf += data; out = []
        while True:
            i = self.buf.find(b'DF')
            if i < 0: self.buf = self.buf[-1:]; return out
            self.buf = self.buf[i:]
            if len(self.buf) < 7: return out
            ftype, n = self.buf[2], struct.unpack('<I', self.buf[3:7])[0]
            if n > MAX_PAYLOAD: self.buf = self.buf[2:]; continue          # not a real frame start
            if len(self.buf) < 11 + n: return out
            body, crc = self.buf[2:7 + n], struct.unpack('<I', self.buf[7 + n:11 + n])[0]
            if zlib.crc32(body) & 0xFFFFFFFF != crc: self.buf = self.buf[2:]; continue
            self.buf = self.buf[11 + n:]; out.append((ftype, body[5:]))

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('image', nargs='?'); ap.add_argument('port'); ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--status', action='store_true'); ap.add_argument('--wait', type=float, default=5.0)
    ap.add_argument('--watch', type=float, default=0.2, help='seconds to keep listening after the drone answers')
    a = ap.parse_args()
    import serial
    s = serial.Serial(a.port, a.baud, timeout=0.1)
    if a.status: s.write(frame(STATUS))
    else:
        img = open(a.image, 'rb').read()
        if img[:4] != struct.pack('<I', 0x52464244): sys.exit('not a program image')
        s.write(frame(PROGRAM, img)); print(f'sent {len(img)} bytes')
    r, end = Reader(), time.time() + a.wait
    while time.time() < end:
        for ftype, payload in r.feed(s.read(4096)):
            print(('report: ' if ftype == REPORT else '') + payload.decode('utf-8', 'replace'))
            if ftype == REPORT or payload.startswith((b'swapped', b'rejected')): end = time.time() + a.watch

if __name__ == '__main__': main()
