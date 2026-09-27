# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo joints, rigid masses, masses on cables and sensors, anything on anything, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

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

  A motor on servo joints is several inputs: its thrust times each product of (1, cos θ, sin θ) over the joints it sits on. That's 3 columns for one joint and 9 for two. Turning a rigid part about a hinge is linear in cos θ and sin θ, so this is exact, and its effect at any joint angles is a fixed sum of learned columns.

**Calibrate** (Controller model panel) runs about 12 s of test moves while hovering, or about 25 s with servos:
1. **Settle.**
2. **Test each motor on its own.** It steps up then down, by 6% and then by 16%, while every other motor and servo keeps what it had (**Freeze other motors during pulses**). Each test waits until the drone is calm first. Freezing matters because otherwise the controller answers every pulse with the other motors, and inputs that always move together can't be told apart.
3. **Test each servo on its own.** It swings one way, then the other, by 35% of its range, while the motors and every other servo hold still.
4. **Sweep each servo.** The sweep rides on top of what the controller asks for, so it keeps its servo authority; a tricopter's tail servo is its only real yaw control.
5. **Excite everything together.**
6. **Validate** on a fresh signal.

It then scores the learned model and the description on the same validation data, and switches to the learned model only if it predicts better. On the stock presets the description is already near-perfect and is kept. With hidden masses, weak motors or unknown parts, the learned model wins and the drone flies noticeably better.

**Keep learning in flight** continues the identification with 30 s of memory and a 2% dither, to track slow changes such as the battery draining. Run Calibrate again after big changes.

While a calibration runs, the controller keeps flying on the model it had when the calibration started, and switches only when the calibration decides.

### Actuator response

The single-actuator tests learn how each actuator responds over time, not just how strong it is. The results appear under **Actuator response** next to the true values.

| Learned | From | Used for |
|---|---|---|
| Motor lag | `identifyMotorResponse`: which spin-up time best explains the response to a step | The effectiveness identification, which used to assume 35 ms for every motor |
| Servo speed and lag | `identifyServoResponse`: the speed limit and lag that best explain how the drone's response traces the servo's real angle, with no angle feedback needed | `servoPredictor`, the servo angle the controller uses when a servo has no feedback |
| Throttle-curve bend | `identifyMotorResponse`: whether a step up gives more than the same step down | `thrustLinearization`, only if you turn on **Linearize thrust with the measured curve**. Until then it assumes a typical brushless curve (bend 0.7) |

In the tests on the stock presets:
- **Motor lag:** comes out within 5 ms on every multirotor. The main-lifter's big rotor is usually within 5 ms too, but its fit is poor.
- **Servo speed and lag:** the measured speed is what the servo really manages on a short step, below its no-load speed (340 against 360°/s on the tilt-rotor quad, 260 against 300°/s on the tricopter's loaded tail). The command delay comes out within 5 ms.
- **Throttle-curve bend:** the real curve now comes from the motor physics (about 0.8). The tests see it only roughly, anywhere from 0.1 to 0.9, because the air a pulse pushes through the prop and the battery sagging under load produce effects of the same size. So it's shown but not used unless you ask. On real hardware this is usually measured on a thrust stand.

The hardware has traits the controller is never told, marked **hidden** on the part cards, so there's something to learn:
- **Spin-up time** of each motor (sets its prop and rotor inertia).
- **Servo stall torque** (0.8 N·m), **command delay** (20 ms) and **trim error** (0°).
- **Servo feedback** (off by default, like hobby servos). Without it, the controller never sees servo angles and relies on its prediction.


The panel shows how close each actuator's learned effect is to the truth. The truth comes from linearizing the real simulated physics (airflow and battery included) by nudging each input.

## Throw start

**Reset to: Throw** (or **T**) starts the drone the way Blaha, Smeur and Remes (TU Delft, 2024) do: it is held still for a moment, then thrown upward with its motors off and a random tumble. The throw itself takes 0.12 s of hand push, which the IMU feels, so the drone knows it's climbing. It knows its sensors and how many actuators it has, and nothing about its geometry, mass, props or motors.

1. **Climb.** It rides the throw with the motors off.
2. **Pulse over the top of the arc.** Each motor fires on its own at 50% throttle. A pulse ends after 80 ms, or earlier once the drone's rotation has changed by 4 rad/s, which keeps well inside the gyro's range. A motor on steering joints is pulsed with each of those joints in the middle, at one end and at the other end, so all its columns can be told apart. The pulses are timed to finish just after the top; on the way down it soon needs its height to recover. If it runs out of room (it needs about 0.45 s to spin up and turn upright, then brakes at 0.8 g), it stops pulsing early. A motor it only tried in the middle is then treated as fixed there, and the servos are held in the middle, with the drone leaning to move, until a calibration has measured them.
3. **Fit** (`identifyThrow`). In free fall the accelerometer feels no gravity, only the rotors and its own swing around the center of gravity. One least-squares fit on under a second of data gives:
   - the effectiveness matrix;
   - where the IMU sits relative to the balance point;
   - the gyroscopic coupling between axes;
   - the spin-up reaction (B₂, their G₂): a motor speeding up twists the frame the other way, several times harder than its steady drag torque. Without this term a pulse from standstill looks like a huge yaw effect;
   - the motor lag. The drone doesn't measure prop speed, so it runs a generic brushless motor model (back-EMF, a current limit, prop drag) with its time constant unknown.

   It's built to run on a microcontroller. While falling it keeps one running fit per candidate lag (all motors the same), a fixed cost per step, and a compact 250 Hz log. At the moment it has to catch itself it picks the best of those fits at once. Then, on the spare core, it works out each motor's own lag from the log, one motor at a time, and switches to that if it explains the fall better. A big slow rotor and small fast ones can then share a frame.

   Nothing fights the pulses, so each motor's effect comes out clean.
4. **Catch.** The controller takes over on the model it just learned. It gets upright first and turns to the target heading afterwards.
5. **Refine** (optional, on by default). A hover calibration runs, starting from and competing against the throw model.

If the fit is poor, it catches itself on the airframe description instead and says so. The panel suggests a minimum throw height for the current airframe, since more actuators mean more pulses and a longer fall.

Results on the stock presets (thrown to 4 m):
- **Identification:** the fit explains 97–100% of the rotation and of the force. Motor lag comes out at the true 30 ms; the IMU offset within about 3 mm.
- **Recovery:** the quad, hexacopter, tricopter and tilt-rotor quad catch themselves reliably. The tilt-rotor quad gets through about half its pulses before it has to stop, and holds its servos until the calibration. The main-lifter layout is marginal: its big rotor's slow spin-up and strong gyroscopic torque leave it skimming the ground or crashing, whatever the throw height.
- **Helicopter:** the throw start doesn't work. Its only way to roll and pitch is the two-servo rotor head, and the free fall is too short to measure the head's nine basis columns well enough to fly on.
- **Afterwards:** the hover calibration brings the learned model to 92–100% of the force and 96–99% of the rotation on the multirotors and the tilt-rotor quad.

## Helicopter

**Helicopter (main rotor + tail rotor)** under Start from:
- **Main rotor:** a 0.6 m collective-pitch rotor on a two-servo head that tilts it fore–aft and sideways. This stands in for a swashplate's cyclic: tilting the lift off-centre is what rolls and pitches the body.
- **Tail rotor:** out on a 45 cm boom, pushing sideways.
- **Counter-torque:** the main rotor's drag twists the body the opposite way to its spin, and the tail rotor pushes against that.
- **Hover:** the tail rotor's sideways push makes it hang a few degrees to one side, as real helicopters do.

It hovers, manoeuvres and calibrates (98% of the rotation and 100% of the force explained). With a single rotor and nothing to counter its torque, the body spins the opposite way to the rotor.

## Airflow (physics only)

The simulated world has effects the controller is never told about:

- **Rotor wakes** (`wakeVelocity`): momentum-theory downwash that speeds up and contracts below each disc. Wind and forward flight blow the wake sideways as it travels down, so the rear rotors fly into the front rotors' wash. A rotor in another's wake loses thrust.
- **Rotor aerodynamics** (`rotorAero`):
  - Glauert inflow: climbing and wake inflow cost thrust, and forward flight gains a little (translational lift);
  - ground effect, from the Cheeseman–Bennett formula;
  - rotor drag, which grows with thrust and airspeed;
  - vortex ring state: descending straight down at around the rotor's own induced velocity costs it up to 30% of its thrust.
- **Downwash on parts** (`wakeLoad`): rotor wash pushes the hub, rigid masses and cable payloads.
- **Battery** (`batteryModel`): a 4-cell 1.3 Ah pack. It supplies the current the motors really draw, drains with it and sags under it, so the same throttle gives less thrust over a flight and during hard manoeuvres.

Prop radius is a motor setting. **Airflow** on the 3D view shows the wake columns.

## Servo joints and rods

A **servo joint** is a hinge mounted on the frame, on a rod or on another joint. A **rod / lever** is a rigid stick with a mass; whatever you attach to it rides at its far end. Anything can be attached to either: motors, rigid masses, cable payloads, sensors, rods and further joints. Chaining them builds an arm: shoulder servo → upper-arm rod → elbow servo → forearm rod → hand and camera. **+ Motor on servo** adds a joint with a motor at the same point, the usual tilt-rotor.

**The parts list is the attachment tree.** The frame is at the top. Each servo or rod shows what it carries, indented beneath it with guide lines, and can be folded away. You attach things by dragging:
- **Onto a servo or rod:** drag a part by its grip (⠿) onto it to attach it there.
- **Onto Frame:** to take a part off.
- **Without dragging:** each card's **Attached to** field does the same.

Placement follows what you drop onto:
- A part put on a rod goes to the rod's far end.
- A motor or rod put on a servo goes onto the servo's pivot (a tilt-rotor, an arm); anything else goes just below it.
- A servo given its first motor is handed to the allocator to steer (new servos start that way).
- Moving or turning a servo or rod (typed values, sliders or edit-mode handles) carries everything on it along. Lengthening a rod moves what's at its end.

**How a servo is mounted.** You describe a servo by which way it swings what it carries. What it carries sticks out from it one way: toward the parts, or along a rotor's thrust when the rotor sits on the pivot. The servo swings that load toward a chosen direction and back; the hinge axis follows from the two.
- **In edit mode:** select a servo and the view centres on it and plays its swing, moving everything on it (a rotor's thrust arrow shows where the thrust points). A ring round the servo has two arrows where the load swings to; drag them round to swing it any other way (5° steps, 1° with Shift). Drag either dot at the ends of the fan to change the travel. The edit bar's servo panel has three rows: **Swings** (quick picks: forward–back, left–right, up–down, along the rod; the exact angle; ⟲ ⟳ to turn it 15°), **Travel** (− / +), and **Preview** (play/pause, hold at either limit, or a slider to hold any angle).
- **On its card**, grouped as Pivot, Swing, Control and Servo hardware: **Swings** (the same quick picks), **Swing direction** in degrees, and **Hinge lean**, which tilts the hinge so the load sweeps a cone instead of a flat arc.
- **What "relative" means:** the angle is measured from the mount's own X (its Z when X runs along the load). On the frame or on another servo's output that's the body axes; on a rod, X runs along the rod and Z is as near to up as the rod allows. Turning the rod keeps the servo at the same angle to it, and swings everything it carries along.

**Seeing the travel.** In edit mode each servo shows what it can sweep:
- **The fan:** an amber sector in its plane of motion (amber is used for servo travel everywhere, so it doesn't mix with the rotors' blue), out to the farthest part it carries (a rotor's rim), between its limits either side of 0°. The 0° line is dashed; it is where the parts sit as placed.
- **The paths:** each carried part's path as a dashed arc.
- **The hinge axis:** a dashed line through the pivot, and the servo's case and horn are drawn on it.

The servo you've selected is drawn brighter, as is any servo carrying the part you've selected. In flight, with Forces on, a servo steering a rotor keeps a faint fan so you can see the rotor tip across it.

A rod points by presets (down, forward, back, left, right, up) or by exact angles, and its rings swing it with everything it carries.

- **Positions and mounts** are entered with every joint at 0°, in body axes. The joints above a part carry it from there (`jointRotation`), nearest first.
- **Control.** A joint carrying a motor can be steered by the allocator. How much it's used depends on the steering mode: under **Tilt body** the allocator tips the whole drone and moves such a servo only a little; under **Mixed** or **Stay level** it swings the rotor to push the drone along. Any joint can be **Set by me**, a live angle you can change in flight, which is how you'd swing a robot hand. A joint with no motor on it is always set by you.
- **Servo hardware.** Range, no-load speed, and its own mass, plus the hidden traits the controller isn't told: stall torque, command delay, trim error, and whether it reports its angle.

What moving parts do in the physics:
- **Full coupled dynamics.** Each joint's output is its own rigid body, solved together with the frame (see What it models). Swinging an arm moves and turns the frame the other way, the servo has to fight gravity, thrust and the frame's own motion, and nothing is approximated as quasi-static.
- **Rotors, cables, contacts.** Rotors, wakes, cable attachments and ground contact points all move with their joints.
- **Sensors.** A sensor on a joint moves and turns with it. Its gyro also feels the joint turning, its accelerometer feels the arm's own acceleration, and a camera or GPS antenna on it also feels the joint's motion.

What the controller does with them:
- **Its own model.** It moves its CoG and inertia with the joint angles it believes (feedback, or its prediction), for the masses it knows about.
- **Known sensors on joints.** Readings are rotated by the believed joint pose. A joint-mounted gyro has the believed joint rate taken out, and an optical-flow camera or GPS antenna on a moving arm has the arm's own motion taken out.
- **Allocation.** A steering joint is an input whose effect is what turning it does to every motor it carries.

Tests (quad unless noted):

| Setup | Result |
|---|---|
| 0.25 kg "hand" on an 18 cm arm below, swung 0° → 60° → −60° → 0° in flight | Holds; tilt under 8° |
| Same, with the IMU on the moving arm (mount known) | Holds; attitude estimate within 4.6° |
| Tilt-rotor quad with one rotor's servo on a second, folding joint (9 learned columns for that rotor) | Calibrates and flies while the fold moves to 15° |
| Throw start with the arm fitted | Catches itself and calibrates |
| Shoulder and elbow servos with two rods, a hand and the optical-flow camera at the end, both joints swinging continuously | Holds position, with GPS or indoors on flow alone |
| Swivel (vertical-axis) servo with a rod forward, hand and IMU at the end, swinging ±80° | Holds; tilt under 8° |
| Hand or IMU unknown to the controller | Flips. A heavy offset load or a misread IMU is more than the integrators can absorb, as on real hardware |
| All stock presets | Unchanged: the same hover, manoeuvre, calibration, throw and optical-flow results as before |

## Designs, undo and redo

The **Design** section at the top of the Airframe panel:
- **Save** keeps the current airframe under the name in the box. Saving under a name that's already in the list replaces it; a new name makes a new entry. The header shows "unsaved changes" once you edit a saved design.
- **Saved designs** lists them newest first, with their motor and servo counts. Click one to open it; it also appears under Start from. ⤓ saves it to a file, and × deletes it (click twice).
- **Where they're kept:** as a published artifact, in your account, private to you and there on any device. Opened from the repository, in that browser.
- **Save to file** and **Open file…** write and read a design as JSON (`"format": "drone-force-bench-design"`), so a design can live in the repository or go to another machine.
- **Undo and Redo** (buttons here and in the edit bar, or Ctrl+Z / ⌘Z and Ctrl+Shift+Z / ⌘⇧Z / Ctrl+Y) step through every change to the airframe: moving, rotating, adding and removing parts, attaching them, editing their cards, loading a layout or opening a design. A continuous drag of a handle or slider is one step. Up to 300 steps.

A design is the airframe alone: frame mass, every part and the steering mode. Flight state, what the drone has learned and edited formulas aren't part of it.

## Editing the airframe

- **Type numbers:** every value on the Airframe tab has a box you can type into. Typed values can go beyond the slider's range, for example positions up to ±2 m.
- **Edit mode:** press **Edit** on the 3D view (or **E**). The simulation pauses and the airframe is drawn level in its own body axes. Hover to see a part's name, click to select it (or click its card), then drag:
  - **arrows** to move along X (red), Y (green) or Z (blue);
  - **squares** to move within a plane;
  - **rings** to rotate: a motor's thrust axis, a servo joint's hinge direction, or an IMU's or compass's mount.
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
| `js/laws.js` | **The governing formulas**: 36 functions for the physics, airflow, sensors, estimators, identification and controller, plus the text shown for each in the Formulas tab |
| `js/runtime.js` | Law registry: compiles edits, validates what each formula returns, falls back to the default when an edit fails |
| `js/budget.js` | Flight computer budget: counts what the flight code costs per control step and keeps in memory, for an ESP32 |
| `js/math.js` | Vector, matrix and quaternion helpers and the bounded least-squares solver. Everything here can be used inside formulas |
| `js/sim.js` | Airframe presets, mass properties, controller plumbing, physics stepping and the flight-envelope check |
| `js/multibody.js` | Articulated-body dynamics: the frame and every servo joint solved together (recursive Newton–Euler) |
| `js/joints.js` | Servo joints and rods: the attachment tree, poses from the joint angles (true and believed), carrying parts along, servo state |
| `js/learn.js` | Controller model: described vs. learned effectiveness, the calibration cycle and learning in flight |
| `js/sensors.js` | Sensor parts, sampling at each sensor's rate with delay, vibration and magnetic interference, and fusing readings for the estimators |
| `js/view3d.js` | three.js scene and camera |
| `js/pilot.js` | Keyboard and on-screen flight controls |
| `js/editor.js` | Edit mode: picking parts and the move and rotate handles |
| `js/formulas-ui.js` | The Formulas tab |
| `js/ui.js` | Airframe editor, telemetry, traces, header controls, persistence and the boot loop |
| `js/designs.js` | Undo and redo, saved designs (your account or this browser) and design files |
| `css/style.css` | Styles, light and dark |

## The formulas

**Physics (the plant):** `rigidBody`, `gravity`, `rotorWrench`, `jointRotation`, `motorDynamics`, `servoTorque`, `bodyDrag`, `cableTension`, `payloadDrag`, `groundContact`.

**Sensors:** `imuModel`, `magModel`, `baroModel`, `posFixModel`, `flowModel`, `rangeModel`.

**Airflow and battery (physics):** `wakeVelocity`, `rotorAero`, `wakeLoad`, `batteryModel`.

**Estimation:** `attitudeEstimator`, `flowVelocity`, `servoPredictor`, `positionEstimator`.

**Identification:** `identifyThrow`, `identifyMotorResponse`, `identifyServoResponse`, `identifyEffectiveness`.

**Controller:** `positionControl`, `thrustAxisTarget`, `attitudeError`, `attitudeControl`, `forceDemand`, `allocationPreferences`, `allocation`, `thrustLinearization`.

The simulator only calls these by name through `run(key, …)`. To change a default, edit the function in `js/laws.js`.

Edits made in the Formulas tab:
- take effect on the next time step, mid-flight;
- are test-called with sample inputs before they're accepted, so syntax errors and wrong return shapes are rejected with a message;
- are switched off automatically if they throw or return something unusable during flight, and the default takes over;
- are saved in your browser's local storage. **Copy edited formulas** gives you text to paste back into `js/laws.js`.

`rotorWrench` and `jointRotation` are shared by the plant, the controller's effectiveness matrix and the envelope check. The controller evaluates `rotorWrench` at T = 1 N, so it assumes the law is linear in T.

## What it models

The physics isn't simplified for speed; every step (2 kHz) does the full version. The controller runs at 1 kHz.

- **Articulated rigid bodies** (`js/multibody.js`, `rigidBody`). The frame is a free-floating body and every servo joint adds another: the servo's output and everything rigidly on it, including rods, further joints and what they carry. The equations of motion for the frame's 6 degrees of freedom plus every joint angle are built each step with the recursive Newton–Euler algorithm (Featherstone) and solved together. So a swinging arm pushes the frame the other way, a load drags its servo, the whole thing conserves momentum in free fall (checked: angular momentum to 0.2%, linear to 0.01% while an arm swings ±60°), and a sensor on an arm feels the arm's own acceleration.
- **Motors** (`motorDynamics`). Each motor is a brushless motor, ESC and prop sized from its card: throttle sets the voltage, back-EMF and winding resistance set the current, current sets the torque, the prop's inertia sets how fast it spins up, and thrust and drag torque grow with speed squared. From that come a throttle curve that bends upward, spin-up faster than spin-down, a current-limited start from standstill, thrust that fades with the battery, the frame feeling each motor's torque while it speeds up (the spin-up reaction), and the gyroscopic torque of a spinning prop when the frame or its servo turns it. Health scales the thrust. Hover hints (rpm and amps) show on each motor's bar.
- **Collective-pitch rotors** (a motor's **Blade pitch** setting), as on a helicopter. The ESC's governor holds the rotor at a set speed and the blade pitch sets the thrust, so thrust follows the command after the pitch servo's 30 ms lag. The rotor never speeds up or slows down, so there's no spin-up twist, but more pitch means more drag torque, which twists the frame. The blades flap: the disc follows the mast a few milliseconds behind (flapping time constant 16/(γΩ), Lock number γ ≈ 4), so turning the airframe doesn't meet the rotor's gyroscopic stiffness the way a rigid prop does. With rigid blades, a helicopter-sized rotor couples the axes so strongly the controller can't hold it.
- **Servo joints** (`servoTorque`). A hobby servo is a geared motor with a position loop: full stall torque when stopped, none at its no-load speed, a 3° proportional band, the gearbox's reflected inertia, a command delay, and hard stops just past its travel. It moves by the multibody dynamics, so a light arm snaps to its target, a heavy one lags and overshoots, and thrust or weight on an arm holds it slightly off target (checked: a 100 g weight on a 15 cm arm sags it 0.5°).
- **Rigid mass:** box, sphere or vertical cylinder, riding on whichever body it's attached to.
- **Mass on cable:** a point mass on a tension-only spring-damper cable that can swing, go slack and touch the ground.
- **Sensors:** each rides on its body. The IMU has scale errors and axis misalignment as well as noise, bias and drift; the magnetometer has soft-iron distortion as well as hard iron and motor interference.
- Masses and cables can be hidden from the controller ("Controller knows" off), so it must absorb them with integral action.

## Control and allocation

- Position PID produces a desired force. Attitude uses geometric control on SO(3) with integral action.
- Gains are in acceleration units and multiplied by the modeled mass and inertia, so they carry over to new geometry.
- Allocation is two-stage bounded weighted least squares. Stage 1 picks servo angle changes, each capped by what the servo can reach in the planning horizon. Stage 2 solves motor thrusts at the servos' actual angles. Each stage first finds the best achievable move, then chooses among equal ways of making it (see Allocation above).
- Three steering modes:
  - **Tilt body** (4 controlled axes: climb, roll, pitch, yaw). The body leans to move sideways; servos only help turn.
  - **Mixed.** Servos make a share of the sideways force (a slider under Allocation, 50% by default), and the body leans for the rest. If the servos can't deliver their share because they're saturated, too slow, or absent, the share drops automatically within about 0.3 s, so the drone leans more instead. With no servos, it flies like Tilt body.
  - **Stay level** (all 6 axes, needs thrust vectoring). Servos make all the sideways force.
- In the test manoeuvre, Mixed on the tilt-rotor quad kept the tilt under 17° (Tilt body: 30°) and tracked as well as Stay level, with more motor margin to spare (34% against 22%). On the main-lifter layout it cut the tilt from 29° to 19°.

## Allocation: limits, margin, power and servo speed

Every throttle stays within 0–1 and every servo within its range; those are hard limits. On top of that, the allocation (`allocation`) works in three passes:
1. **The move itself.** Get as close to the wanted lift, roll and pitch (and sideways force, where servos make it) as the limits allow.
2. **Then yaw.** As much yaw as it can get without giving any of that up. Most flight controllers put yaw last. Otherwise, when yaw can't be had, the cheapest way to cut the yaw error is to cut the thrust: a lone rotor would never lift, and saturated motors would drop the drone to hold its heading.
3. **The choice.** Keep exactly that move, and where it can be made in more than one way, pick using `allocationPreferences`. Examples: a hexacopter's spare motors, a servo against a motor, or a big lifting rotor against small steering ones.

The preferences never give up any of the move. They are three sliders under **Allocation**:
- **Keep margin (allowance).** Pulls each device toward the middle of its range, gently in the middle and about 100× harder near a limit. It weighs most on the devices that do the steering, so a big lifting rotor near its limit matters less than a steering motor near its limit.
- **Save power (efficiency).** Rotor power grows with thrust^1.5, so it spreads lift toward the big, efficient discs. Each motor's figure of merit is a setting on its card.
- **Servo move cost.** Moving a servo costs in proportion to how much of its reach the move uses. Reach is the learned speed × (planning horizon − learned lag). A slow or laggy servo therefore gets the steady part of the work, and the motors get the quick corrections. Each step's servo change is also capped at that reach, and the motors cover whatever the servo hasn't reached yet.

The panel header shows the estimated rotor power and the device closest to a limit.

What it changed in the tests:

| Case | Before | After |
|---|---|---|
| Tilt-rotor quad with a 0.45 kg load over one arm, time a motor sits at a limit (hover / manoeuvre) | 67% / 66% | 3–8% / 22–60% |
| Main-lifter layout: servo angle, hover power, tightest margin in hover | 44° (at its limit), 157 W, 0% | 8°, 136 W, 40% |
| Main-lifter layout: servo response tests that pass | 0–2 of 4 | 4 of 4 |
| Quad, hexacopter, tricopter | | Unchanged; there is no real choice to make, or the old tie-break already picked the same |

The cost: with slow, laggy servos, the motors take more of the quick work, so their tightest margin during a manoeuvre drops (25% → 10% in the test).

## Flight computer budget

The **Flight computer** panel shows what the flight software alone would cost on the drone's own computer: an ESP32, ESP32-S3 or ESP32-C3. The simulator's physics isn't counted. Only what runs inside the control step is: estimation, learning, control, allocation and the code around them, plus the throw's background refinement.

- **Operations:** the math helpers add up the arithmetic they do while a control step runs, and formulas with loops of their own are counted from their sizes. A multiply-add counts 2, a square root or trig call about 15. Time assumes plain C in 32-bit floats: about 60 million operations per second on the ESP32, 80 on the S3 and 4 on the C3, which has no float unit.
- **Memory:** the numbers the flight code keeps between steps, at 4 bytes each.
- **One-off fits:** the fits at the end of each actuator test, and the throw's first fit, are listed separately. On the drone the test fits belong on the second core.

Measured on the presets, ESP32 at 1 kHz:

| Preset | Hover | Calibrating | Throw, while falling | Memory, most | Throw's first fit |
|---|---|---|---|---|---|
| Quad | 7% of a core | 7% | 14% | 47 KB | 1.4 ms |
| Hexacopter | 11% | 11% | 21% | 60 KB | 3 ms |
| Tricopter | 12% | 12% | 19% | 52 KB | 2 ms |
| Tilt-rotor quad | 28% | 44% | 52% | 113 KB | 14 ms |
| Main lifter + 4 steering | 32% | 62% | 58% | 125 KB | 17 ms |
| Helicopter | 20% | 23% | (throw start not supported) | 91 KB | — |

The biggest items are the in-flight learning (its matrix grows with the square of the inputs, and a motor on a servo is three inputs), the allocation (three passes: lift and tilt, then yaw, then preferences), and rebuilding the controller's model as servos move. The end-of-test fits take 8–30 ms each. The per-motor lag search after a throw takes 0.1–0.3 s on the spare core. The ESP32-C3 can't run the 1 kHz loop; the panel shows the fastest loop it could keep.

## Flight envelope

The attainable set of accelerations is the sum of what each rotor can make: anything from zero to full thrust, along any direction its servos can swing it to (sampled across each servo's range). A swung rotor's sideways push costs it lift (T cos θ up, T sin θ across); an earlier straight-line version treated the two as independent and could call a layout flyable that couldn't hold its yaw. The set is compared with what hover requires, including static cable loads, and reported as Flyable / Marginal / Cannot hover / Not controllable, plus per-axis headroom.

**Lift is never traded away completely.** When a torque can't be cancelled (a big rotor off the balance point, a spin reaction nothing can counter), the allocation keeps at least 75% of the lift asked for and lets the attitude take the rest of the shortfall. Before this, the cheapest answer to such a torque was to switch the big rotor off, and the drone dropped as if that rotor made no thrust.

## Known simplifications

- Sensors have no temperature effects.
- Magnetic interference comes only from motor currents, not from wiring or the battery.
- Airflow uses engineering models (momentum theory, Glauert inflow, a skewed wake), not CFD. The prop's drag torque doesn't change with inflow.
- Structure is rigid: frames, rods and servo horns don't flex, and gears have no backlash.
- The controller's effectiveness model is static. The rotors' gyroscopic torque and the spin-up reaction are real in the physics; the learning measures the spin-up reaction (B₂) so it doesn't corrupt the rest, but the controller doesn't yet use it to cancel those twists, as the Delft INDI controller does. On the main-lifter layout the big rotor's gyroscopic torque is large, and its calibration explains only about 70% of the rotation.
- Learning treats the CoG as fixed. When a known mass swings on a joint, the controller's model follows it, but the learned columns stay as they were at calibration, and keep-learning catches up over about 30 s.
- Without servo feedback, identification and allocation use the predicted servo angle, so a servo that stalls or sags under load isn't noticed.
- The vertical position integral can trim up to 5 m/s², enough to absorb an unknown hover throttle; sideways it stays at 2 m/s².
- In the throw start, a motor on two steering joints only has each joint varied on its own, so the cross terms of its 9 columns come from the hover calibration. The throw start also needs the IMU's mounting angle to be known. It uses the commanded throttle and a generic motor model rather than measured motor RPM, which the Delft work uses; the simulated motors have the same structure as that model, so real motors will fit it less exactly.
- Edited formulas run in the page itself, so an infinite loop in one will freeze the tab.
