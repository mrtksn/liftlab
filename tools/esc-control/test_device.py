"""Exercise the device watchdog and command gating without hardware."""
import contextlib
import io
import pathlib
import runpy
import sys
import types
from unittest.mock import patch

clock = [0]
events = []
schedule = [(0, 'CONFIG 300\n'), (300, 'ARM\n'), (600, 'RUN 1500\n'),
            (1700, 'RUN 1000\n'), (2000, 'ARM\n'), (2300, 'RUN 1000\n'),
            (2600, 'RUN 3000\n'), (2900, 'RUN 3010\n'),
            (3200, 'CONFIG 490\n'), (3500, 'STOP\n'), (3800, 'QUIT\n')]
queue = []

class Input:
    def read(self, n):
        return queue.pop(0)

class Poll:
    def register(self, *args): pass
    def poll(self, ms):
        clock[0] += ms
        while schedule and schedule[0][0] <= clock[0]:
            queue.extend(schedule.pop(0)[1])
        return bool(queue)

class Pin:
    IN = 0
    def __init__(self, *args): pass

class PWM:
    def __init__(self, pin, freq, duty_u16): events.append(('pwm', clock[0], duty_u16))
    def duty_u16(self, value): events.append(('pwm', clock[0], value))
    def deinit(self): events.append(('deinit', clock[0], None))

machine = types.SimpleNamespace(Pin=Pin, PWM=PWM)
select = types.SimpleNamespace(poll=Poll, POLLIN=1)
time = types.SimpleNamespace(ticks_ms=lambda: clock[0], ticks_diff=lambda a, b: a-b)
output = io.StringIO()
with patch.dict(sys.modules, {'machine': machine, 'select': select, 'time': time}), \
     patch.object(sys, 'stdin', Input()), contextlib.redirect_stdout(output):
    runpy.run_path(str(pathlib.Path(__file__).with_name('device.py')))

stop = 550 * 300 * 65535 // 1000000
run1500 = 1500 * 300 * 65535 // 1000000
run1000 = 1000 * 300 * 65535 // 1000000
run3000 = 3000 * 300 * 65535 // 1000000
run3010 = 3010 * 300 * 65535 // 1000000
values = [value for kind, _, value in events if kind == 'pwm']
assert 'ESC TIMEOUT' in output.getvalue(), output.getvalue()
assert values.count(run1500) == 1
assert values.count(run1000) == 1, 'RUN after timeout must be ignored until rearmed'
assert values.count(run3000) == 1 and run3010 not in values
assert 'ESC CONFIG ERROR' in output.getvalue(), 'frequency cannot change after PWM starts'
assert values[-1] == stop
assert events[-1][0] == 'deinit'
print('Device watchdog and command gating: passed')
