# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo-tilted motors, rigid masses and masses on cables, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

Every physical law and control law is a plain function in `js/laws.js`, and you can read and edit each one live in the **Formulas** tab.

## Run it

Open `index.html` in a browser. There is no build step. It needs an internet connection to load three.js (r128, from cdnjs) and the Google Fonts it uses.

## Learning the airframe

The controller commands **throttle fractions (0–1)**, not Newtons, and doesn't need to know prop sizes, mass or inertia. What it needs is the **effectiveness matrix B**: how much linear and angular acceleration each actuator input produces. B comes from one of two sources.

- **Description:** computed from the airframe you entered, including only the parts marked as known to the controller.
- **Learned:** identified from flight data by recursive least squares (`identifyEffectiveness`), starting from the description. The learner:
  - uses the accelerometer and gyro as outputs, and the throttle sent as inputs;
  - band-passes both sides (0.3–12 Hz), so steady offsets like drag can't leak in;
  - removes the IMU's lever-arm swing.

  A tilting rotor is two inputs, u·cosθ and u·sinθ, so its effect at any servo angle is known.

**Calibrate** (Controller model panel) runs about 10–20 s of small test moves while hovering:
1. Settle.
2. Pulse each motor in turn, twice. While one motor is pulsed, the others keep the throttle they had when the pulse began (**Freeze other motors during pulses**). Otherwise the controller answers every pulse with the other motors, and inputs that always move together can't be told apart.
3. Sweep each servo.
4. Excite everything together.
5. Validate on a fresh signal.

It then scores the learned model and the description on the same validation data, and switches to the learned model only if it predicts better. On the stock presets the description is already near-perfect and is kept. With hidden masses, weak motors or unknown parts, the learned model wins and the drone flies noticeably better.

**Keep learning in flight** continues the identification with 30 s of memory and a 2% dither, to track slow changes such as the battery draining. Run Calibrate again after big changes.

While a calibration runs, the controller keeps flying on the model it had when the calibration started, and switches only when the calibration decides.

The panel shows how close each actuator's learned effect is to the truth. The truth comes from linearizing the real simulated physics (airflow and battery included) by nudging each input.

## Throw start

**Reset to: Throw** (or **T**) starts the drone the way Blaha, Smeur and Remes (TU Delft, 2024) do: it is held still for a moment, then thrown upward with its motors off and a random tumble. It knows its sensors and how many actuators it has, and nothing about its geometry, mass, props or motors.

1. **Climb.** It rides the throw with the motors off.
2. **Pulse near the top of the arc.** Each motor fires on its own at 50% throttle. A pulse ends after 80 ms, or earlier once the drone's rotation has changed by 4 rad/s, which keeps well inside the gyro's range. Servo rotors are pulsed twice, near each end of their range, so both halves of a tilting rotor are seen. It pulses near the top because air rushing through the props while climbing or falling changes their thrust.
3. **Fit** (`identifyThrow`). In free fall the accelerometer feels no gravity, only the rotors and its own swing around the center of gravity. One least-squares fit on under a second of data gives:
   - the effectiveness matrix;
   - where the IMU sits relative to the balance point;
   - the gyroscopic coupling between axes;
   - the motor lag, found by fitting several candidate lags side by side and keeping the best.

   Nothing fights the pulses, so each motor's effect comes out clean.
4. **Catch.** The controller takes over on the model it just learned. It gets upright first and turns to the target heading afterwards.
5. **Refine** (optional, on by default). A hover calibration runs, starting from and competing against the throw model.

If the fit is poor, it catches itself on the airframe description instead and says so. The panel suggests a minimum throw height for the current airframe, since more actuators mean more pulses and a longer fall.

Results on the stock presets:
- **Identification:** the fit explains 94–100% of the throw data. The IMU offset comes out within about 4 mm.
- **Hover:** judged in hover, the throw model is rough, anywhere from 0 to about 90% right per actuator. The props see very different air while tumbling and falling than in hover. That's still enough to catch itself, and the hover calibration afterwards brings each actuator to roughly 75–95%.
- **Recovery:** all six presets catch themselves. The main-lifter layout needs about 10 pulses, and from the default 4 m it touches the ground before it recovers, so throw it higher.

## Airflow (physics only)

The simulated world has effects the controller is never told about:

- **Rotor wakes** (`wakeVelocity`): momentum-theory downwash that speeds up and contracts below each disc. A rotor in another's wake loses thrust.
- **Rotor aerodynamics** (`rotorAero`):
  - Glauert inflow: climbing and wake inflow cost thrust, and forward flight gains a little (translational lift);
  - ground effect, from the Cheeseman–Bennett formula;
  - rotor drag, which grows with thrust and airspeed.
- **Downwash on parts** (`wakeLoad`): rotor wash pushes the hub, rigid masses and cable payloads.
- **Battery** (`batteryModel`): drains with load and sags, so the same throttle gives less thrust over a flight.

Prop radius is a motor setting. **Airflow** on the 3D view shows the wake columns.

## Editing the airframe

- **Type numbers:** every value on the Airframe tab has a box you can type into. Typed values can go beyond the slider's range, for example positions up to ±2 m.
- **Edit mode:** press **Edit** on the 3D view (or **E**). The simulation pauses and the airframe is drawn level in its own body axes. Hover to see a part's name, click to select it (or click its card), then drag:
  - **arrows** to move along X (red), Y (green) or Z (blue);
  - **squares** to move within a plane;
  - **rings** to rotate: a motor's thrust axis, a servo's hinge direction, or an IMU's or compass's mount.
- **Snapping and exiting:** positions snap to 5 mm and angles to 5°; hold **Shift** for 1 mm and 1°. **Esc** deselects, then leaves edit mode. **Done** or **Run** resumes the simulation.
- **Live feedback:** the airframe check, mass properties and the part's card update as you drag.

## Sensors and estimation

The controller doesn't see the true state. It flies on what its sensors report, through two estimators, just like real flight software. Sensors are parts you attach on the Airframe tab, each with a position and mount angle you can set:

| Sensor | Imperfections |
|---|---|
| IMU (gyro + accelerometer) | Noise, turn-on bias, gyro bias drift, range limits, sample rate, delay. Motor vibration (a sinusoid per motor at its rotation frequency) is stronger near busy motors, and a slow sample rate aliases it. An off-center accelerometer also feels the drone's rotation. |
| Compass | Noise, hard-iron offset, and interference from motor currents that grows with throttle and falls off with distance. |
| Barometer | Noise and slow drift. |
| Position fix | GPS, RTK GPS or motion-capture presets: noise, a slowly wandering error, update rate, delay, and a "signal lost" switch. |
| Optical flow + rangefinder | A downward camera that tracks how the ground slides past, plus a distance sensor. Flow noise grows as tracking quality drops; each axis has its own scale error; it saturates above a maximum flow rate. Quality depends on the ground's texture, the light and the height. The rangefinder has minimum and maximum range and noise that grows with distance. |

Each sensor can be marked as known or unknown to the controller. When it's unknown, the controller assumes the sensor sits at the hub with no rotation, which is how you model a misplaced or misaligned sensor.

- **Attitude estimator:** a Mahony complementary filter. It trusts the accelerometer for "up" only when the reading is near 1 g, because a multirotor's accelerometer feels thrust, not gravity, while it accelerates.
- **Position estimator:** a complementary filter over the accelerometer, position fix, optical flow, rangefinder and barometer. It compares each delayed reading with the estimate from when it was measured, and learns the accelerometer's horizontal bias from persistent velocity errors. Without a fix or flow, it falls back on drag fusion: the sideways accelerometer reading is air drag, which reveals airspeed.

### Optical flow

The camera sees the ground slide by at flow ≈ ω − v/d: rotation makes the image move even when the drone doesn't, and the same speed gives less flow from higher up. `flowVelocity` removes the rotation with the gyro and multiplies by the rangefinder distance to get velocity over the ground. The estimator fuses that velocity, and uses the rangefinder for height (over flat ground) in place of the barometer.

- The **Indoor quad (optical flow, no GPS)** preset is a quad with an IMU, compass, barometer and a flow sensor, and no position fix.
- **Target & environment** has *Ground texture* and *Light* sliders. Over water, in the dark, or above the rangefinder's range, flow drops out and the drone drifts slowly on the accelerometer and drag alone.
- The sensor looks along its own −Z. A flow sensor that is rotated but marked unknown to the controller reads velocity in the wrong direction, and the drone flies away, which is what happens on a real drone with a mis-set sensor orientation.
- With a GPS as well, flow makes the velocity estimate several times better, but position still follows the GPS's slow wander, because nothing can tell that wander apart from real motion.

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
| T | Throw start: throw the drone with its motors off and let it learn itself in free fall |
| 1 / 2 / 3 | Gentle (1 m/s) / Normal (3 m/s) / Sport (6 m/s) |
| C | Chase camera: keep the view behind the drone |
| E | Edit mode: select and drag parts in the 3D view |
| P (hold) | Charge a poke; release to hit. Tap for a nudge, hold 1.5 s for the strongest. The Poke button works the same way |

Keys are ignored while you type in a text field or the formula editor. In the published page, click the 3D view first so the page receives the keys.

## Layout

| File | What it holds |
|---|---|
| `js/laws.js` | **The governing formulas**: 31 functions for the physics, airflow, sensors, estimators, identification and controller, plus the text shown for each in the Formulas tab |
| `js/runtime.js` | Law registry: compiles edits, validates what each formula returns, falls back to the default when an edit fails |
| `js/math.js` | Vector, matrix and quaternion helpers and the bounded least-squares solver. Everything here can be used inside formulas |
| `js/sim.js` | Airframe presets, mass properties, controller plumbing, physics stepping and the flight-envelope check |
| `js/learn.js` | Controller model: described vs. learned effectiveness, the calibration cycle and learning in flight |
| `js/sensors.js` | Sensor parts, sampling at each sensor's rate with delay, vibration and magnetic interference, and fusing readings for the estimators |
| `js/view3d.js` | three.js scene and camera |
| `js/pilot.js` | Keyboard and on-screen flight controls |
| `js/editor.js` | Edit mode: picking parts and the move and rotate handles |
| `js/formulas-ui.js` | The Formulas tab |
| `js/ui.js` | Airframe editor, telemetry, traces, header controls, persistence and the boot loop |
| `css/style.css` | Styles, light and dark |

## The formulas

**Physics (the plant):** `rigidBody`, `gravity`, `rotorWrench`, `tiltAxis`, `motorResponse`, `servoResponse`, `bodyDrag`, `cableTension`, `payloadDrag`, `groundContact`.

**Sensors:** `imuModel`, `magModel`, `baroModel`, `posFixModel`, `flowModel`, `rangeModel`.

**Airflow and battery (physics):** `wakeVelocity`, `rotorAero`, `wakeLoad`, `batteryModel`.

**Estimation:** `attitudeEstimator`, `flowVelocity`, `positionEstimator`.

**Identification:** `identifyThrow`, `identifyEffectiveness`.

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
- Airflow uses fast engineering models (momentum theory, Glauert inflow), not CFD. Wakes are straight columns and aren't bent by wind or forward flight.
- The motor command-to-thrust curve is linear. Real ESCs need thrust linearization, which isn't identified yet.
- Servo angles are assumed measurable (servo feedback) for identification.
- The throw start needs the IMU's mounting angle to be known, and uses the commanded throttle rather than measured motor RPM, which the Delft work uses. Their method also identifies the throttle curve and the spin-up reaction torque; the simulated motors don't have those.
- Edited formulas run in the page itself, so an infinite loop in one will freeze the tab.
