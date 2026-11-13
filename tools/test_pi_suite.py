#!/usr/bin/env python3
"""runner/pi/suite/liftlab (LiftLab on the drone's computer), without a Pi: liftlab.conf read and checked, the systemd
units written from it (paths, quoting, conflicts), apply (units written, removed, enabled, started), migrate from the
earlier layout (~/dfb, dfb.service, ~/liftlab-bench), dfb_pi's status read, and the health checks. systemctl and
sudo are stand-ins here. Python 3 alone: python3 tools/test_pi_suite.py"""
import importlib.machinery, importlib.util, os, sys, tempfile, types
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / 'runner' / 'pi' / 'suite' / 'liftlab'

def load(root, units):
    os.environ['LIFTLAB_HOME'] = str(root); os.environ['LIFTLAB_UNIT_DIR'] = str(units)
    loader = importlib.machinery.SourceFileLoader('liftlab_' + str(abs(hash(str(root)))), str(SRC))
    spec = importlib.util.spec_from_loader(loader.name, loader); m = importlib.util.module_from_spec(spec); loader.exec_module(m)
    return m

def fake_systemd(m, units):
    """sudo and systemctl as a dict of unit states; tee/rm on the unit folder do the file operations."""
    st = {}; calls = []
    def sudo(argv, check=True, input=None):
        calls.append(argv)
        if argv[0] == 'tee': Path(argv[1]).write_text(input)
        elif argv[0] == 'rm': Path(argv[-1]).unlink(missing_ok=True)
        elif argv[0] == 'systemctl':
            verb, *rest = argv[1:]
            for u in [r for r in rest if r.endswith('.service')]:
                s = st.setdefault(u, {'Id': u, 'ActiveState': 'inactive', 'SubState': 'dead', 'UnitFileState': 'disabled', 'NRestarts': '0'})
                if verb in ('start', 'restart'): s['ActiveState'] = 'active'
                if verb == 'stop': s['ActiveState'] = 'inactive'
                if verb == 'enable': s['UnitFileState'] = 'enabled'
                if verb == 'disable': s['UnitFileState'] = 'disabled'
                if '--now' in rest: s['ActiveState'] = 'inactive' if verb == 'disable' else 'active'
        return types.SimpleNamespace(returncode=0, stdout='', stderr='')
    m.sudo = sudo
    m.systemctl = lambda *a, check=True: sudo(['systemctl', *a], check)
    m.unit_state = lambda svcs: {s.unit: dict(st[s.unit]) for s in svcs if s.unit in st}
    return st, calls

def main():
    tmp = Path(tempfile.mkdtemp()); root = tmp / 'liftlab'; units = tmp / 'units'; root.mkdir(); units.mkdir()
    m = load(root, units)

    # liftlab.conf: read and checked
    (root / 'liftlab.conf').write_text('[liftlab]\nuser = pilot\n\n[flight]\ncommand = bin/dfb_pi --nav drone.dnc --bind "my phrase" --radio wifi,sta\nworkdir = design\nboot = yes\nprobe = dfb_pi 14560\n\n'
                                       '[bench]\ncommand = python3 -u apps/bench/server.py\nboot = no\nrestart = on-failure\nconflicts = flight\n')
    cp = m.read_conf(); sv = {s.name: s for s in m.services(cp)}
    assert list(sv) == ['flight', 'bench'] and sv['flight'].boot and not sv['bench'].boot and sv['bench'].conflicts == ['flight']
    assert m.run_user(cp) == 'pilot'
    for bad, why in (('[x y]\ncommand = a\n', 'a name'), ('[x]\nboot = yes\n', 'no command'), ('[x]\ncommand = a\nrestart = sometimes\n', 'restart')):
        (tmp / 'bad.conf').write_text(bad)
        try: m.services(m.read_conf(tmp / 'bad.conf')); assert False, bad
        except SystemExit as e: assert why in str(e), e
    print('liftlab.conf: read; a bad name, no command, an unknown restart refused')

    # the units: the program from the folder, the arguments as given (quoted for systemd), the working folder, conflicts
    u = m.unit_text(sv['flight'], 'pilot', all_names=['flight', 'bench'])
    assert f'ExecStart={root.resolve()}/bin/dfb_pi --nav drone.dnc --bind "my phrase" --radio wifi,sta' in u, u
    assert f'WorkingDirectory={(root / "design").resolve()}' in u and 'User=pilot' in u and 'Restart=always' in u and 'WantedBy=multi-user.target' in u
    ub = m.unit_text(sv['bench'], 'pilot', all_names=['flight', 'bench'])
    assert 'Conflicts=liftlab-flight.service' in ub and 'Restart=on-failure' in ub and ('ExecStart=/' in ub), ub
    assert m.sd_quote('a b') == '"a b"' and m.sd_quote('50%') == '50%%' and m.sd_quote('$HOME') == '$$HOME' and m.sd_quote('x"y') == '"x\\"y"'
    print('units: the program from the folder, quoted arguments, its working folder, user, restart, conflicts')

    # apply: written, enabled and started (boot = yes), the others written only; again: nothing; a stale one removed
    st, calls = fake_systemd(m, units)
    said = m.apply(quiet=True)
    assert (units / 'liftlab-flight.service').exists() and (units / 'liftlab-bench.service').exists()
    assert st['liftlab-flight.service']['UnitFileState'] == 'enabled' and st['liftlab-flight.service']['ActiveState'] == 'active', st
    assert st.get('liftlab-bench.service', {}).get('ActiveState', 'inactive') == 'inactive'
    n = len(calls); said2 = m.apply(quiet=True)
    assert not [c for c in calls[n:] if c[0] == 'tee' or c[1:2] in (['start'], ['restart'], ['enable'])], calls[n:]
    m.systemctl('stop', 'liftlab-flight.service'); m.apply(quiet=True)
    assert st['liftlab-flight.service']['ActiveState'] == 'inactive', 'stopped by hand: apply leaves it stopped'
    (units / 'liftlab-old.service').write_text('x'); m.apply(quiet=True)
    assert not (units / 'liftlab-old.service').exists()
    m.set_boot('bench', True)
    assert 'boot = yes' in (root / 'liftlab.conf').read_text().split('[bench]')[1] and st['liftlab-bench.service']['UnitFileState'] == 'enabled'
    print('apply: written, enabled and started once; nothing the second time; stopped by hand stays stopped; stale units removed; boot on')

    # migrate: from ~/dfb and dfb.service and ~/liftlab-bench, into the folder; nothing deleted
    home = tmp / 'home'; (home / 'dfb' / 'runner' / 'pi').mkdir(parents=True); (home / 'liftlab-bench').mkdir()
    for f in ('drone.dnc', 'drone.dfa', 'drone.dlc'): (home / 'dfb' / f).write_text(f)
    (home / 'dfb' / 'runner' / 'pi' / 'dfb_pi').write_text('binary'); (home / 'liftlab-bench' / 'server.py').write_text('print()')
    old = tmp / 'old.service'; old.write_text('[Service]\nExecStart=/home/pi/dfb/runner/pi/dfb_pi --nav drone.dnc --no-navigation --tuning drone.dft --radio wifi,sta --bind liftlab\n')
    root2 = tmp / 'liftlab2'; root2.mkdir(); m2 = load(root2, units); fake_systemd(m2, units)
    m2.cmd_migrate(home=home, old_unit=old)
    c2 = m2.read_conf(); s2 = {s.name: s for s in m2.services(c2)}
    assert set(s2) == {'flight', 'bench', 'monitor'}, list(s2)
    assert s2['flight'].command == 'bin/dfb_pi --nav drone.dnc --no-navigation --tuning drone.dft --radio wifi,sta --bind liftlab' and s2['flight'].workdir == 'design' and s2['flight'].boot
    assert s2['bench'].conflicts == ['flight'] and not s2['bench'].boot and s2['monitor'].command == './liftlab watch'
    assert (root2 / 'design' / 'drone.dnc').read_text() == 'drone.dnc' and (root2 / 'bin' / 'dfb_pi').exists() and (root2 / 'src' / 'runner' / 'pi' / 'dfb_pi').exists()
    assert (home / 'liftlab-bench').is_symlink() and (root2 / 'apps' / 'bench' / 'server.py').exists() and (home / 'dfb' / 'drone.dnc').exists() and old.exists()
    assert 'already' not in ' '.join(m2.cmd_migrate(home=home, old_unit=old)) and set(x.name for x in m2.services()) == {'flight', 'bench', 'monitor'}
    print('migrate: the design, dfb_pi, the source, the bench panel moved in (a link left); [flight] from dfb.service; again: nothing new')

    # the cheap look between systemd's: the cgroup's processes, the boot link; systemd itself once per max_age
    cg = tmp / 'cgroup'; (cg / 'liftlab-flight.service').mkdir(parents=True); (cg / 'liftlab-flight.service' / 'cgroup.procs').write_text(f'{os.getpid()}\n')
    (cg / 'liftlab-bench.service').mkdir(); (cg / 'liftlab-bench.service' / 'cgroup.procs').write_text('')
    m.CGROUP = cg; (units / 'multi-user.target.wants').mkdir(exist_ok=True)
    (units / 'multi-user.target.wants' / 'liftlab-flight.service').write_text('link')
    asked = []; m.unit_state = lambda svcs: (asked.append(1), {s.unit: {'Id': s.unit, 'ActiveState': 'failed', 'NRestarts': '3', 'MainPID': '0'} for s in svcs})[1]
    m.forget_state(); svs = m.services()
    a1 = m.live_state(svs); a2 = m.live_state(svs)
    assert len(asked) == 1, 'systemd asked once in max_age'
    assert a1['liftlab-flight.service']['ActiveState'] == 'active' and a1['liftlab-flight.service']['MainPID'] == str(os.getpid()) and a1['liftlab-flight.service']['UnitFileState'] == 'enabled', a1
    assert a1['liftlab-bench.service']['ActiveState'] == 'failed' and a1['liftlab-bench.service']['NRestarts'] == '3', 'no processes: systemd\'s word (failed) kept'
    m.forget_state(); m.live_state(svs); assert len(asked) == 2, 'after an action: systemd asked again'
    print('the cheap look: running from the cgroup, at boot from the link, systemd once a while and after actions')

    # dfb_pi's status
    d = m.dfb_status('angle mode: sticks arm on, throttle 0.00, roll +0.00, pitch +0.00, yaw -0.00; the ESP32 armed; wifi with 192.168.2.191:60195: packets sent 10 (0 failed), got 20 (LQ 98%), bad 0')
    assert d['ok'] and d['esp32'] == 'armed' and d['arm'] and d['throttle'] == 0 and d['lq'] == 98 and d['peer'] == '192.168.2.191:60195', d
    d = m.dfb_status('angle mode: no channels from the radio (nothing sent to the ESP32); the ESP32 not heard yet; wifi with nobody yet: packets sent 0 (0 failed), got 0 (LQ 0%)')
    assert not d['ok'] and d['channels'] is False and d['peer'] is None and 'no pilot' in d['text'], d
    d = m.dfb_status('disarmed; at 0.00 0.00 0.00, speed 0.00 0.00 0.00; position not settled yet')
    assert d['ok'] and 'esp32' not in d
    print("dfb_pi's status: the ESP32, the pilot's link, the sticks; not heard is a problem")

    # the control screen: what it sends, its keys, dfb_pi's answers
    st = m.ctl_status('angle mode: text sticks arm on, throttle 0.00, roll +0.00, pitch +0.00, yaw +0.00; the ESP32 armed; attitude roll +1.5 pitch -2.5 yaw 90 deg, height 0.12 m, vz +0.00 m/s, flags 31; motors 0.06 0.06 0.06 0.06; wifi with nobody yet: packets sent 0 (0 failed), got 0 (LQ 0%)')
    assert st['esp32'] == 'armed' and st['source'] == 'this panel' and st['roll'] == 1.5 and st['pitch'] == -2.5 and st['height'] == 0.12 and st['motors'] == [0.06] * 4 and st['angle_mode'], st
    assert m.ctl_status('disarmed; at 0 0 0')['angle_mode'] is False
    clock = [100.0]; sent = []
    def fake(text):
        sent.append(text)
        if text == 'status': return 'angle mode: text sticks arm off, throttle 0.00, roll +0.00, pitch +0.00, yaw +0.00; the ESP32 disarmed; attitude roll +0.0 pitch +0.0 yaw 0 deg, height 0.00 m, vz +0.00 m/s, flags 31; motors 0.00 0.00 0.00 0.00'
        if text.startswith('motor 7'): return 'motor: 7 isn\'t one of this airframe\'s 4 motors'
        return 'sticks: arm off' if text.startswith('sticks') else 'motor test: ' + text
    c = m.Control(14560, send=fake, now=lambda: clock[0])
    assert c.line() == 'sticks 0 0.00 +0.00 +0.00 +0.00', c.line()
    c.tick(True); assert c.status['esp32'] == 'disarmed' and sent == [c.line(), 'status']
    c.key('w'); c.key('w'); assert c.thr == 0.1
    assert 'bottom' in c.key(' ') and not c.arm, 'arming needs the bottom throttle'
    c.key('0'); assert 'again' in c.key(' ') and not c.arm
    clock[0] += 3; assert 'again' in c.key(' ') and not c.arm, 'the second press within 2 s'
    clock[0] += 1; assert c.key(' ') == 'arm' and c.arm and c.line().startswith('sticks 1 0.00')
    assert 'disarmed' in c.key('1') and c.motor is None, 'no motor test armed'
    c.key('right'); c.key('up'); c.key('up'); c.key('d'); c.key('h')
    assert c.line() == 'sticks 1 0.50 +0.10 +0.20 -0.10', c.line()
    c.key('c'); c.key('x'); assert not c.arm and c.line() == 'sticks 0 0.00 +0.00 +0.00 +0.00', c.line()
    c.key('t'); assert c.key('2') == 'motor 2 at 0.15' and c.line() == 'motor 2 0.15'
    assert 'released' in c.key('3') and c.motor is None, 'another motor: the test released first'
    c.key('2'); assert c.key('2') == 'motor test off' and c.motor is None
    c.key('7'); assert c.motor is None, 'beyond the airframe\'s motors (from the status)'
    c.motor = 7; c.tick(False); assert 'isn\'t one' in c.said, 'a refusal shown'
    c.motor = None; c.arm = True; c.thr = 0.0; sent.clear(); c.leave(); assert sent == ['sticks 0 0'] * 3 + ['sticks off'] and not c.arm, 'leaving on the ground: disarmed, then stopped'
    c.arm = True; c.thr = 0.5; sent.clear(); c.leave(); assert sent == ['sticks off'], 'leaving in the air: just stopped (the failsafe lands it)'
    # safety and level: one-off commands, sent before the sticks; the ESP32's word in the status
    st = m.ctl_status('angle mode: no channels from the radio (nothing sent to the ESP32); the ESP32 disarmed; attitude roll +4.0 pitch -19.8 yaw -5 deg, height -0.32 m, vz +0.18 m/s, flags 87, SAFETY OFF, levelled; motors 0.00 0.00 0.00 0.00; the ESP32 said 3 s ago: disarmed: won\'t arm: not level (or: safety off); wifi with 192.168.2.191:60569: packets sent 5 (0 failed), got 4 (LQ 19%)')
    assert st['safety_off'] and st['levelled'] and st['pitch'] == -19.8 and st['esp32_said'].startswith("disarmed: won't arm") and st['esp32_said_s'] == 3, st
    c = m.Control(14560, send=fake, now=lambda: clock[0]); c.status = m.ctl_status('angle mode: no channels; the ESP32 disarmed'); sent.clear()
    assert 'again' in c.key('!') and not c.queue; assert c.key('!') == 'safety off' and c.queue == ['safety off']
    c.tick(False); assert sent[:2] == ['safety off', c.line()] and not c.queue, sent
    c.status['safety_off'] = True; assert c.key('!') == 'safety on' and c.queue == ['safety on']; c.queue.clear()
    assert 'still' in c.key('L') and c.queue == ['level']; c.queue.clear()
    c.status['esp32'] = 'armed'; assert 'disarmed only' in c.key('L') and not c.queue; c.status['safety_off'] = False; assert 'disarmed' in c.key('!')
    print('control: safety off (twice) and on, level now; the ESP32\'s last word and the flags read')

    # dfb_send over UDP, against a stand-in dfb_pi
    import socket, threading
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); srv.bind(('127.0.0.1', 0)); port = srv.getsockname()[1]
    def echo():
        b, a = srv.recvfrom(512); srv.sendto(b'you said ' + b, a)
    threading.Thread(target=echo, daemon=True).start()
    assert m.dfb_send(port, 'health') == 'you said health'
    assert m.dfb_send(port, 'health', timeout=0.2) is None, 'nobody answers: None'
    assert m.dfb_port(types.SimpleNamespace(probe='dfb_pi 14560')) == 14560 and m.dfb_port(types.SimpleNamespace(probe='tcp 8766')) is None
    print('control: the sticks it sends, arming twice at the bottom, motor tests disarmed and one at a time, leaving safely; send')

    # health
    sy = {'power_raw': 0x50005, 'temp_c': 80, 'mem_avail_mb': 30, 'disk_pct': 95, 'wifi': ('wlan0', 20, -85)}
    progs = [{'name': 'flight', 'installed': True, 'boot': True, 'state': 'failed', 'result': 'exit-code', 'sub': 'failed', 'probe': None, 'up_s': None},
             {'name': 'monitor', 'installed': True, 'boot': True, 'state': 'active', 'result': '', 'sub': 'running', 'probe': {'ok': False, 'text': 'no answer to status'}, 'up_s': 30},
             {'name': 'bench', 'installed': False, 'boot': False, 'state': 'inactive', 'result': '', 'sub': '', 'probe': None, 'up_s': None}]
    pr = m.problems({'system': sy, 'programs': progs}); txt = ' | '.join(t for _, t in pr)
    for want in ('under-voltage now', 'throttled', '80', 'memory', 'disk 95%', 'Wi-Fi weak', 'flight: failed', 'monitor: no answer', 'bench: not set up'):
        assert want in txt, (want, txt)
    assert pr[0][0] == 2
    ok = m.problems({'system': {'power_raw': 0, 'temp_c': 50, 'mem_avail_mb': 300, 'disk_pct': 40, 'wifi': ('wlan0', 90, -50)}, 'programs': []})
    assert ok == [], ok
    print('health: under-voltage, throttling, heat, memory, disk, weak Wi-Fi, a stopped program, a probe without answer, not set up')
    print('pi suite: ok')

if __name__ == '__main__':
    main()
