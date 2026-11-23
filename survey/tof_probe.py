"""Read one HCJ TOF250 sample on the Pi's header bus. Argument is a 7-bit address."""
import errno
import fcntl
import json
import sys
import time

with open('/dev/i2c-1','r+b',buffering=0) as bus:
    fcntl.ioctl(bus, 0x0703, int(sys.argv[1]))
    try:
        bus.write(bytes([0]))
        time.sleep(.001)
        value = bus.read(2)
    except OSError as exc:
        if exc.errno in (errno.EIO, errno.ENXIO, errno.EREMOTEIO):
            sys.exit(f"No response at 0x{int(sys.argv[1]):02x}. Check power, wiring and the 7-bit address.")
        raise
    if len(value) != 2:
        raise IOError('Incomplete TOF250 sample')
    print(json.dumps(dict(raw=int.from_bytes(value,'big'), pi_read_ns=time.monotonic_ns())))
