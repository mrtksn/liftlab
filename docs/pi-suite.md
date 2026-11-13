# LiftLab on the drone's Raspberry Pi

Everything the drone runs on its Pi lives in one folder, `~/liftlab`, with a control panel, `liftlab`, that you reach
over SSH. The source is `runner/pi/suite/`.

```
~/liftlab/
  liftlab          the control panel and its commands (also /usr/local/bin/liftlab)
  liftlab.conf     the programs it runs (one [section] each)
  bin/dfb_pi       the drone's companion program
  design/          this design's files from LiftLab (drone.dnc, drone.dfa, drone.dlc; autotune writes drone.dft)
  src/runner/      the source, for building on the Pi
  apps/bench/      the motor bench panel (tools/pi-bench), when there is one
```

## Setting it up

From your computer, in the repository's folder (with [zig](https://ziglang.org): `brew install zig`):

```sh
sh tools/deploy_pi.sh pi@raspberrypi.local
```

It builds the Pi's programs on your computer (`tools/build_pi_cross.sh`, seconds instead of minutes on a Pi Zero),
copies them with the source to `~/liftlab/src`, and runs `runner/pi/suite/install.sh` there. Without zig, copy the
source and build on the Pi: LiftLab's install guide for the Pi (Computers → the Pi → Install / export…) has the
commands, and the one line that makes this design's `dfb_pi` the **flight** program:

```sh
liftlab add flight --workdir design --boot yes --probe "dfb_pi 14560" --command 'bin/dfb_pi --nav drone.dnc ...'
```

`install.sh` also moves in an earlier setup: `~/dfb` (the design, `dfb_pi`, the source) and its `dfb.service` become
the folder and its **flight** program, `~/liftlab-bench` becomes `apps/bench` (a link stays where it was). Nothing is
deleted. It keeps ModemManager off the flight controller's serial port (a udev rule).

## The control panel

```sh
ssh -t pi@raspberrypi.local liftlab
```

A full-screen view, refreshed each second:

- the Pi: address, uptime, temperature, load, free memory, disk, Wi-Fi quality, and **power**: the Pi's own
  under-voltage and throttling flags (`vcgencmd get_throttled`), the first thing to check when a drone's computer
  misbehaves;
- each program: running or not, at boot or not, how long it has run, how often it was restarted, its memory, and its
  health: for `dfb_pi`, what it says itself (the flight controller's state, the pilot's link and its quality, the sticks
  in angle mode);
- the problems now, and the selected program's last log lines.

Keys: ↑↓ (or `j` `k`) a program · `s` start · `x` stop · `r` restart · `b` start at boot or not · `l` its whole log · `c` control
the drone (below) · `:` a command to `dfb_pi` · `a` write
`liftlab.conf` to systemd · `R` reboot · `P` power off · `q` quit.

## Controlling the drone from the panel

`c` on the **flight** program (any program whose probe is `dfb_pi PORT`) opens its control screen: the sticks from
the keyboard, sent to `dfb_pi` 20 times a second as a radio's would be, and what the ESP32 says back: its state, its
attitude, height and vertical speed, each motor's throttle. It's for testing on the bench, without LiftLab's page or a
radio; it needs angle mode (`dfb_pi --no-navigation`), and a radio's pilot, when one is connected, wins.

| | |
|---|---|
| `space` | disarm; to arm, twice within 2 s, with the throttle at the bottom (the flight controller's rule) |
| `w` `s` (`W` `S`) | throttle ±0.05 (±0.01); `0` the bottom, `h` the middle, which holds the height |
| ← → ↑ ↓, `a` `d` | roll, pitch, yaw ±0.1; `c` centres them. The sticks stay where the keys put them (a terminal says when a key is pressed, not when it's let go) |
| `1`…`9` | a motor test, disarmed: that motor alone at the test throttle (`t` `g` ±0.05); the same key or `m` stops it |
| `x` | all stop: disarmed, sticks centred, throttle at the bottom |
| `!` | safety off (twice: no level check when arming, no crash cut-off; for bench work with the sensor out) / on |
| `L` | level: as it sits now is level (disarmed, still; until a reboot). See [hardware-wiring.md](hardware-wiring.md#which-way-is-level) |
| `:` | any text command to `dfb_pi` (also on the main screen), its answer in the bottom line |
| `q` | leave: the sticks stop. On the ground it disarms first; in the air the ESP32's failsafe lands it (as when a radio is lost) |

Only `q` leaves (not Esc: half an arrow key mustn't stop the sticks). If the panel or the SSH connection dies, the
sticks stop within half a second and the failsafe takes over the same way.

Underneath, these are `dfb_pi` text commands on its UDP port, which a script can send too:

```sh
liftlab send flight status
liftlab send flight sticks 1 0          # arm on, throttle 0 (good for half a second: send it again to keep it)
liftlab send flight motor 2 0.1         # motor 2 alone at 0.1, disarmed
liftlab send flight sticks off
```

The screen also shows what the ESP32 said last (why it won't arm, for one), and **SAFETY OFF** while it is.
`safety on|off`, `level`, `level none` and `esp32 LINE` (any settings line: `show`, `level=boot`, `save` ...) go to the
ESP32 the same way; its answer shows in `status` and the log.

`sticks ARM THROTTLE [ROLL PITCH YAW]` (arm 0/1, throttle 0..1, the rest −1..1, yaw as the flight controller's: negative
turns right), `motor N THROTTLE` (N from 1 in the airframe's order; `--airframe` lets `dfb_pi` check it), `motor off`.

The same as commands, for scripts or a quick look:

| | |
|---|---|
| `liftlab status` (`--json`) | the panel's view, once |
| `liftlab check` | the problems only; exit status 1 if there are any |
| `liftlab start\|stop\|restart NAME\|all` | |
| `liftlab enable\|disable NAME` | start at boot or not |
| `liftlab logs NAME [-f]` | its log (`-f`: follow it) |
| `liftlab add NAME --command ... [--boot yes] [--workdir DIR] [--probe ...] [--conflicts NAME]`, `liftlab remove NAME` | |
| `liftlab apply` | after editing `liftlab.conf` by hand |
| `liftlab build` | `dfb_pi` from `src/` into `bin/` |
| `liftlab send NAME TEXT...` | a text command to a `dfb_pi` program, and its answer |

## Programs and monitoring

Each `[section]` of `liftlab.conf` (see `liftlab.conf.example`) is a systemd service, `liftlab-NAME`: started at
boot when `boot = yes`, started again when it stops (`restart`). `conflicts` names programs that can't run beside it:
the bench panel and `dfb_pi` both need the serial port, so starting one stops the other.

The **monitor** program (`liftlab watch`, at boot) checks every 10 s and writes each change to the journal: an
under-voltage, a program that stopped, the flight controller not heard, Wi-Fi getting weak, and when each clears.
`liftlab logs monitor` shows what happened, after a flight too.

Starting and stopping use `sudo` (Raspberry Pi OS gives the first user it without a password); looking needs none.
