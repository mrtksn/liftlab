import machine, sys, select, time

STOP_US = 550
FREQ = 490
LIMITS = {490: 2000, 400: 2450, 300: 3000}
LEASE_MS = 650
PWM_PIN = 18
pwm = None
armed = False
running = False
last_run = 0
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
buf = ''

def pulse(us):
    global pwm
    duty = int(us * FREQ * 65535 // 1000000)
    if pwm is None:
        pwm = machine.PWM(machine.Pin(PWM_PIN), freq=FREQ, duty_u16=duty)
    else:
        pwm.duty_u16(duty)

def stop():
    global running
    if pwm is not None:
        pulse(STOP_US)
    running = False

print('ESC READY')
try:
    while True:
        if running and time.ticks_diff(time.ticks_ms(), last_run) > LEASE_MS:
            stop()
            armed = False
            print('ESC TIMEOUT')
        if poller.poll(20):
            ch = sys.stdin.read(1)
            if not ch:
                stop()
                armed = False
                continue
            if ch in '\r\n':
                cmd = buf.strip()
                buf = ''
                if cmd.startswith('CONFIG '):
                    try:
                        requested = int(cmd[7:])
                    except ValueError:
                        requested = 0
                    if pwm is None and requested in LIMITS:
                        FREQ = requested
                        print('ESC CONFIGURED', FREQ)
                    else:
                        print('ESC CONFIG ERROR')
                elif cmd == 'ARM':
                    stop()
                    pulse(STOP_US)
                    armed = True
                    print('ESC ARMED')
                elif cmd == 'STOP':
                    stop()
                    print('ESC STOPPED')
                elif cmd == 'DISARM':
                    stop()
                    armed = False
                    print('ESC DISARMED')
                elif cmd.startswith('RUN ') and armed:
                    try:
                        value = int(cmd[4:])
                    except ValueError:
                        value = 0
                    if 550 <= value <= LIMITS[FREQ]:
                        pulse(value)
                        running = True
                        last_run = time.ticks_ms()
                elif cmd == 'QUIT':
                    stop()
                    break
            elif len(buf) < 32:
                buf += ch
            else:
                buf = ''
finally:
    stop()
    if pwm is not None:
        pwm.deinit()
    machine.Pin(PWM_PIN, machine.Pin.IN)
    print('ESC EXIT')
