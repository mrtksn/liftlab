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

Keys: ↑↓ (or `j` `k`) a program · `s` start · `x` stop · `r` restart · `b` start at boot or not · `l` its whole log · `a` write
`liftlab.conf` to systemd · `R` reboot · `P` power off · `q` quit.

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

## Programs and monitoring

Each `[section]` of `liftlab.conf` (see `liftlab.conf.example`) is a systemd service, `liftlab-NAME`: started at
boot when `boot = yes`, started again when it stops (`restart`). `conflicts` names programs that can't run beside it:
the bench panel and `dfb_pi` both need the serial port, so starting one stops the other.

The **monitor** program (`liftlab watch`, at boot) checks every 10 s and writes each change to the journal: an
under-voltage, a program that stopped, the flight controller not heard, Wi-Fi getting weak, and when each clears.
`liftlab logs monitor` shows what happened, after a flight too.

Starting and stopping use `sudo` (Raspberry Pi OS gives the first user it without a password); looking needs none.
