# Pi bench panel

The full-range panel requires flight firmware built from the matching source; older builds still cap tests at 30% and 3 seconds. The panel talks to the LiftLab flight firmware over the Pi's `/dev/serial0` at 921600 baud. It binds only to the Pi's loopback interface. The Pi needs Python 3 and pyserial. It is a bench tool, not a flight controller.

On the Pi, run `python3 ~/liftlab-bench/server.py`. On your computer, open a terminal and keep this SSH tunnel running:

```sh
ssh -N -L 8766:127.0.0.1:8766 pi@192.168.2.127
```

Open http://127.0.0.1:8766/ on that computer. Stop the panel with Ctrl-C if run in the foreground. The server sends disarmed commands continuously to request IMU samples, and owns the serial port exclusively; do not run it alongside flight or other serial control software. It does not start automatically after a Pi reboot.

The page displays controller telemetry and specific force, including gravity. The altitude and attitude are controller estimates, not raw pressure or independent sensor values. An unloaded airframe leaves the estimator unavailable. Sensor detection does not establish sensor accuracy or physical wiring.

Motor test uses the flight firmware's disarmed four-output bench command, covering the full configured throttle range while commands continue. Hold an unselected slider to run one motor and release to stop; select motors first to set speeds without running, then use Start selected motors and Stop all motors. The page stops on blur or when hidden. The panel adds a 350 ms browser lease, stops on release, and disables the button without fresh telemetry, an airframe, the observed output order and ESC settings. The existing airframe determines output order; verify it matches the physical labels before powering motors. A test can only run after a valid airframe is loaded through the existing LiftLab installer; this panel does not upload one. The observed settings are 490 Hz, 1070–1940 µs with 550 µs stop, pins 18,22,23,19. Actual behavior, including signal loss, remains unverified for this complete setup.

Remove all propellers and keep a physical battery disconnect within reach. The software stop and timeouts do not remove power. Disconnect the battery if the motor does not stop. Recheck stop behavior and wiring at low output before further testing.
