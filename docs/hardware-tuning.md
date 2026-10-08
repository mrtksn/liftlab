# Calibration, learning and PID autotune on Pi + ESP32

The ESP32 runs PID control. The Pi runs airframe calibration, passive model learning,
response measurement and tuning verification. Hardware commands go to the running
`dfb_pi` process; the browser's Tune/Measured autotune controls still operate the simulator.

## Install and connect

Update **both** the ESP32 flight firmware and the Pi source. Older firmware can still
fly and send ordinary learning telemetry, but cannot acknowledge hardware tuning.
The bundled firmware in `firmware/manifest.json` may predate this source; build current
source or publish its CI firmware artifact before flashing. The install dialog's `version`
check identifies the installed build. Updating only the Pi does not enable autotune.

Use the normal Computers → Install workflow to export matching `drone.dfa`, `drone.dnc`
and `drone.dlc`. Both boards must use the same airframe. On the Pi, build and run:

```sh
cd ~/dfb
sh runner/pi/build.sh
./runner/pi/dfb_pi --nav drone.dnc --airframe drone.dfa --pi drone.dlc --tuning drone.dft --gps /dev/ttyUSB0
```

Keep the existing `--link`, GPS baud and radio arguments for your wiring. The
flight UART must match on both ends, at 460800 baud or faster (921600 is the default);
enhanced tuning telemetry is not requested on slower links. The example
GPS device is a placeholder. Navigation needs an actual position reference; the Pi's
current physical sensor path supports serial NMEA GPS. Simulator optical flow does not
provide a physical Pi camera driver. Do not disable learning or the health supervisor.
The installer adds `--tuning drone.dft` when both tasks are assigned to the Pi.

While running interactively, type commands directly into `dfb_pi`. When it runs as a
service, use the included Python client, on the Pi over SSH or from your computer:

```sh
python3 runner/pi/control.py 127.0.0.1 status
python3 runner/pi/control.py 127.0.0.1 learning
python3 runner/pi/control.py 127.0.0.1 autotune status
```

From your computer, replace `127.0.0.1` with the Pi's hostname/IP. The client uses the
existing UDP command port 14560 (`--port` overrides it). It never opens the flight UART,
so the service keeps running. A client timeout means the reply is missing; query status
before repeating a command. This existing command endpoint belongs on your trusted
local network; it is separate from the binding-phrase-protected pilot radio.

## Activate it while hovering

First establish a stable guided position hold with the normal pilot controls. Calibration
and autotune **require flight**, not a stationary bench or a hand-held airframe. Release
the controls during tests, keep a pilot ready to take over, and use a clear test area.
This implementation has native simulated-flight verification; real-flight validation is
still pending. Do ordinary hardware bring-up and flight checks before using it.

Send each command when its preceding step has finished, rather than pasting this whole
sequence at once:

1. `calibrate` — tests the actuators while hovering. Query `learning` or follow
   `journalctl -u dfb -f` until calibration finishes with an accepted model. `stop` ends
   calibration while keeping the previous accepted model. `learned` explicitly selects
   the accepted model; `description` selects the airframe description. Autotune requires
   a completed calibration, and any model switch invalidates the current measurement.
2. `keep off` — recommended while tuning. `keep on` separately enables passive,
   bounded actuator-model adaptation during ordinary flight; it does **not** tune PID.
   Autotune suspends this preference during measurement/review/verification and restores
   it afterward. An accepted model change requires a fresh attitude measurement.
3. `autotune attitude` — measures bounded roll, pitch and yaw response. Query
   `autotune status` for progress. Wait for `review`; status then shows current and
   proposed P/D/I gains. Measurement alone leaves the gains unchanged.
4. `autotune apply` — stages temporary gains and repeats the flight tests. Wait for
   `done` and the verified/accepted message. A rejection or interruption during verification
   restores the preceding accepted gains.
5. `autotune position` — available after attitude verification on this model. Wait for
   `review`, then `autotune apply` again. It measures a small horizontal target sweep;
   position gains are shared across axes, so independent vertical validation remains pending.
6. After completing tuning and landing, `autotune save` explicitly writes the verified
   attitude and position gains to `drone.dft`. The file is on the Pi, not ESP32 flash.

For example, to start the first test remotely:

```sh
python3 runner/pi/control.py raspberrypi.local calibrate
# Wait for accepted calibration; then send individually:
python3 runner/pi/control.py raspberrypi.local autotune attitude
python3 runner/pi/control.py raspberrypi.local autotune status
```

`autotune stop` cancels a measurement/recommendation/trial. Moving the pilot's target,
changing heading or flight/model state, a supervisor intervention, excessive motion,
insufficient motor headroom, or stale/mismatched telemetry also stops it. Calibration
commands and other pilot commands cancel tuning first. Rejection in poor GPS/noisy
conditions is expected; limits are not relaxed to manufacture a recommendation.

## Gains, persistence and limits

Model fitting/recommendation runs on a background Pi worker with immutable inputs;
its completion cannot revive a cancelled or replaced measurement. Navigation and
telemetry guards continue during the search. Pi Zero timing still needs device profiling.

Only the bundled standard control formulas, fixed motors and tilt mode are supported.
Loaded custom program slots cannot tune. Simulator tuning edits are not automatically
uploaded to either board. Gains use LiftLab's mass/inertia-normalized acceleration units,
so other flight-controller PID numbers cannot be pasted into this protocol. Use an
unloaded airframe: the physical path has no cable-tension sensor and cannot distinguish
a swinging payload from actuator response.

The flight controller reports synchronized gyro and averaged commanded angular
acceleration from the same telemetry interval, current gains, transaction state, guided-mode readback and its
airframe fingerprint. Attitude/position reference tests expire after 100 ms without
refresh. Trial gains have an independent 250 ms lease on each board and revert if the
Pi stops refreshing them, flight becomes unsafe or the standard program is replaced.
Normal flight commands still have their separate existing failsafe. The Pi preserves
partial UART writes and stops on a 40 ms output backlog instead of replaying old commands.

Successful verification sends an explicit acceptance transaction. At this point gains
become the runtime baseline. If its reply is lost, the status says acceptance was
interrupted and requires checking the reported current gains; it does not promise an
already accepted transaction was undone. Saving is a separate explicit operation.

The Pi also requires a valid GPS fix received within the last two seconds; a lost
position reference stops tuning even if inertial navigation continues.

The saved file contains validated gains, a checksum and fingerprints of both exported
airframe and navigation configuration. A design mismatch/corrupt file is refused.
At startup the Pi restores both gain sets while disarmed and waits for the ESP32 readback;
it refuses to fly if restoration cannot be confirmed. The learning model and attitude
response fit are not saved in this file: fresh calibration/attitude measurement are
needed for another position tuning run after restart. Loading gains does not count
as freshly verifying them on a changed physical airframe.

Saving requires writable storage; a read-only Pi overlay will refuse the save and retain
the previous file. To discard saved gains, stop the service, move/remove `drone.dft`,
reboot the ESP32 to clear its runtime overrides, and restart the Pi. Neither rebooting
only the Pi nor deleting the file changes gains already accepted by the still-running ESP32.

## Regression checks

`sh tools/test_hardware_tuning.sh` checks the native measured-response math, delayed
command/readback transactions on an independent rigid-body plant, attitude/position
measurement and verification, rollback and guard conditions, saved-file validation,
UART backpressure and the UDP client. These checks do not prove real actuator behavior
or successful ESP-IDF firmware compilation; CI builds the real board images separately.
