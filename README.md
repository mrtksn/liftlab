# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo-tilted motors, rigid masses and masses on cables, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

Every physical law and control law is a plain function in `js/laws.js`, and you can read and edit each one live in the **Formulas** tab.

## Run it

Open `index.html` in a browser. There is no build step. It needs an internet connection to load three.js (r128, from cdnjs) and the Google Fonts it uses.

## Sensors and estimation

The controller doesn't see the true state. It flies on what its sensors report, through two estimators, just like real flight software. Sensors are parts you attach on the Airframe tab, each with a position and mount angle you can set:

| Sensor | Imperfections |
|---|---|
| IMU (gyro + accelerometer) | Noise, turn-on bias, gyro bias drift, range limits, sample rate, delay. Motor vibration (a sinusoid per motor at its rotation frequency) is stronger near busy motors, and a slow sample rate aliases it. An off-center accelerometer also feels the drone's rotation. |
| Compass | Noise, hard-iron offset, and interference from motor currents that grows with throttle and falls off with distance. |
| Barometer | Noise and slow drift. |
| Position fix | GPS, RTK GPS or motion-capture presets: noise, a slowly wandering error, update rate, delay, and a "signal lost" switch. |

Each sensor can be marked as known or unknown to the controller. When it's unknown, the controller assumes the sensor sits at the hub with no rotation, which is how you model a misplaced or misaligned sensor.

- **Attitude estimator:** a Mahony complementary filter. It trusts the accelerometer for "up" only when the reading is near 1 g, because a multirotor's accelerometer feels thrust, not gravity, while it accelerates.
- **Position estimator:** a complementary filter over the accelerometer, position fix and barometer. It compares each delayed fix with the estimate from when the fix was measured. Without a fix, it falls back on drag fusion: the sideways accelerometer reading is air drag, which reveals airspeed.

The **State estimate** panel shows estimate-minus-truth errors and warns about missing references. The dashed outline in the 3D view is where the flight software thinks the drone is. **Controller flies on: Ground truth** bypasses the sensors for comparison.

Sensor noise comes from a seeded generator, so every reset replays the same noise.

## Flying it

The pads on the 3D view and the keyboard steer the drone. They move the target the controller holds, at a commanded velocity that is also fed forward to the position law, so every airframe you build flies with the same controls.

| Key | Action |
|---|---|
| W / S | Climb / descend |
| A / D | Turn left / right |
| ↑ / ↓ | Forward / back, relative to the heading |
| ← / → | Left / right, relative to the heading |
| Space | Stop and hold the current position |
| H | Fly back to the start point |
| 1 / 2 / 3 | Gentle (1 m/s) / Normal (3 m/s) / Sport (6 m/s) |
| C | Chase camera: keep the view behind the drone |
| P (hold) | Charge a poke; release to hit. Tap for a nudge, hold 1.5 s for the strongest. The Poke button works the same way |

Keys are ignored while you type in a text field or the formula editor. In the published page, click the 3D view first so the page receives the keys.

## Layout

| File | What it holds |
|---|---|
| `js/laws.js` | **The governing formulas**: 22 functions for the physics, sensors, estimators and controller, plus the text shown for each in the Formulas tab |
| `js/runtime.js` | Law registry: compiles edits, validates what each formula returns, falls back to the default when an edit fails |
| `js/math.js` | Vector, matrix and quaternion helpers and the bounded least-squares solver. Everything here can be used inside formulas |
| `js/sim.js` | Airframe presets, mass properties, controller plumbing, physics stepping and the flight-envelope check |
| `js/sensors.js` | Sensor parts, sampling at each sensor's rate with delay, vibration and magnetic interference, and fusing readings for the estimators |
| `js/view3d.js` | three.js scene and camera |
| `js/pilot.js` | Keyboard and on-screen flight controls |
| `js/formulas-ui.js` | The Formulas tab |
| `js/ui.js` | Airframe editor, telemetry, traces, header controls, persistence and the boot loop |
| `css/style.css` | Styles, light and dark |

## The formulas

**Physics (the plant):** `rigidBody`, `gravity`, `rotorWrench`, `tiltAxis`, `motorResponse`, `servoResponse`, `bodyDrag`, `cableTension`, `payloadDrag`, `groundContact`.

**Sensors:** `imuModel`, `magModel`, `baroModel`, `posFixModel`.

**Estimation:** `attitudeEstimator`, `positionEstimator`.

**Controller:** `positionControl`, `thrustAxisTarget`, `attitudeError`, `attitudeControl`, `forceDemand`, `allocation`.

The simulator only calls these by name through `run(key, …)`. To change a default, edit the function in `js/laws.js`.

Edits made in the Formulas tab:
- take effect on the next time step, mid-flight;
- are test-called with sample inputs before they're accepted, so syntax errors and wrong return shapes are rejected with a message;
- are switched off automatically if they throw or return something unusable during flight, and the default takes over;
- are saved in your browser's local storage. **Copy edited formulas** gives you text to paste back into `js/laws.js`.

`rotorWrench` and `tiltAxis` are shared by the plant, the controller's effectiveness matrix and the envelope check. The controller evaluates `rotorWrench` at T = 1 N, so it assumes the law is linear in T.

## What it models

- One rigid body with 6 degrees of freedom, integrated at 2 kHz. The controller runs at 1 kHz.
- **Motor:** thrust along its axis, first-order spin-up lag, drag torque κ·T opposite to its spin, adjustable health.
- **Motor on servo:** thrust direction rotates about a hinge, with a servo angle limit and a maximum servo speed.
- **Rigid mass:** box, sphere or vertical cylinder, contributing mass, center-of-gravity shift and inertia.
- **Mass on cable:** a point mass on a tension-only spring-damper cable that can swing, go slack and touch the ground.
- Masses and cables can be hidden from the controller ("Controller knows" off), so it must absorb them with integral action.

## Control and allocation

- Position PID produces a desired force. Attitude uses geometric control on SO(3) with integral action.
- Gains are in acceleration units and multiplied by the modeled mass and inertia, so they carry over to new geometry.
- Allocation is two-stage bounded weighted least squares. Stage 1 picks servo angles using virtual inputs (T·cos θ, T·sin θ). Stage 2 solves motor thrusts at the servos' actual angles.
- Two steering modes: "Tilt body" (4 controlled axes: climb, roll, pitch, yaw) and "Stay level" (all 6 axes, needs thrust vectoring).

## Flight envelope

The attainable set of accelerations is a zonotope built from each actuator's contribution, linearized over servo range. It is compared with what hover requires, including static cable loads, and reported as Flyable / Marginal / Cannot hover / Not controllable, plus per-axis headroom.

## Known simplifications

- Sensors have no temperature effects, cross-axis sensitivity or scale-factor error yet.
- Magnetic interference comes only from motor currents, not from wiring or the battery.
- A tilting motor's mass stays at its pivot. Gyroscopic torque from spinning props is ignored.
- No aerodynamic interaction between rotors, the frame and the payload beyond simple linear drag.
- Edited formulas run in the page itself, so an infinite loop in one will freeze the tab.
