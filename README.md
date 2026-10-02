# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo joints, rigid masses, masses on cables and sensors, anything on anything, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

Every physical law and control law is a plain function in `js/laws.js`, and you can read and edit each one live in the **Formulas** tab.

## Run it

Open `index.html` in a browser. There is no build step. It needs an internet connection to load three.js (r128, from cdnjs) and the Google Fonts it uses.

The header has three groups:
- **Airframe:** start from a layout or one of your saved designs. **Blank** is a bare frame with a battery and an IMU; it opens in edit mode, resting on the ground, for you to add motors to. If the airframe on screen has changes you haven't saved, you're asked first: save it (under a name you give, never over a different design), don't save, or cancel. Opening a saved design or a design file asks the same.
- **Simulation:** ▶ / ❚❚ runs and pauses (**K**), ↺ resets (**R**), **Hover / Throw** chooses what a reset does, and ¼× ½× 1× sets the speed.
- **Steering:** Tilt body, Mixed or Stay level.

**Poke** is with the flight controls on the 3D view, beside Hold and Home.

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

The learning runs at 200 Hz, as it does on the drone, where it has the ESP32's second core to itself. Between its updates the 1 kHz control steps average what it learns from (the inputs, accelerometer, gyro and motor commands), which is its anti-aliasing filter.

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
- **The rule is the same for every part:** moving or turning a part (typed values, sliders or edit-mode handles) carries everything attached to it, and everything attached to that, and so on; what it is attached to stays put. With A carrying B carrying C: move or turn A and B and C follow; move or turn B and C follows while A stays. Turns are rigid, so what's on a part keeps its angles to it. Lengthening a rod moves what's at its end.
- A rod can also be **rolled about its length** (its card, or its ring along the rod), which turns what's on it round the rod.

**How a servo is mounted.** You describe a servo by which way it swings what it carries. What it carries sticks out from it one way: toward the parts, or along a rotor's thrust when the rotor sits on the pivot. The servo swings that load toward a chosen direction and back; the hinge axis follows from the two.
- **In edit mode:** select a servo and the view centres on it and plays its swing, moving everything on it (a rotor's thrust arrow shows where the thrust points). Like a motor or a rod, it has three rotation rings (red, green, blue): drag one to turn the servo about that axis, and everything attached to it turns with it about its pivot. Drag either dot at the ends of the fan to change the travel. The edit bar's servo panel has three rows: **Swings** (quick picks that change the swing direction while leaving the load where it is: forward–back, left–right, up–down, along the rod; the exact angle; ⟲ ⟳ to turn it 15°), **Travel** (− / +), and **Preview** (play/pause, hold at either limit, or a slider to hold any angle).
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
- **Before something replaces the airframe** (Start from, opening a saved design, Open file…), you're asked to save it if you changed it since it was loaded or last saved.
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
  - **rings** to rotate: a motor's shaft, a servo (with everything on it), a rod (with everything on it), or an IMU's or compass's mount.
- **What you can pick:** only the solid part under the pointer (arrows, arcs, fans and hidden shapes don't count, and the frame hides what's behind it). Clicking the arm a part hangs on picks that part. The hover label for a motor says whether it pulls or pushes and which way it spins, and in edit mode every motor shows a small thrust arrow (the selected one's is bold and drawn on top), so pushers stand out.
- **Snapping and exiting:** positions snap to 5 mm and angles to 5°; hold **Shift** for 1 mm and 1°. **Esc** deselects, then leaves edit mode. **Done** or **Run** resumes the simulation.
- **Live feedback:** the airframe check, mass properties and the part's card update as you drag.

## Views

**Show** (top right of the 3D view) has a switch for everything drawn over the scene, grouped:
- **Forces:** thrust, weight, wind.
- **Torque:** rotor torque, net torque, wanted torque. All off by default; **Q** turns rotor and net torque on and off together.
- **Airframe:** prop spin, servo range, centre of mass, sensor beams, airflow.
- **Flight:** trail, estimate, target.
- **Scene:** ground grid, readouts, legend.

Presets switch everything back to the defaults, everything on, or just the drone. The button counts how many switches differ from the defaults, the legend lists only what's shown, and your choice is kept in this browser.

The box under the view buttons (top right of the 3D view) sets how you look at the drone, in flight and while editing:
- **Orientation triad:** the drone's X (forward), Y (left) and Z (up) as the camera sees them. Click an axis end to look from that side: the filled ends are front, left and top, the hollow ones back, right and bottom.
- **Persp / Ortho:** perspective, or orthographic (no foreshortening, so parts line up and lengths compare directly). Zoom works the same in both.
- **Top, Front, Side, Iso** buttons. The camera glides there in a quarter second.
- **Keys:** numpad 1, 3, 7 for front, right, top (with Ctrl: back, left, bottom), numpad 0 for iso, **O** or numpad 5 for perspective/orthographic, and **V** steps through top, front, right and iso on a keyboard without a numpad.

Named views are relative to the drone: in flight "front" means facing its nose wherever it's heading (and turns Chase off); while editing, body axes. Selecting a part while editing centres the view on it. You can orbit all the way over the top and underneath.

### Torque

Off by default: turn it on in the **Show** menu (or with **Q**). It draws the torques on the drone in pink, as turning arrows. An arc goes round the torque's axis the way it turns (right-hand rule: thumb along the axis, fingers the way it turns), and sweeps further the bigger the torque:
- **At each rotor:** its reaction torque, the motor pushing the frame back the opposite way to the prop's spin. The thrust arrows don't show this, and it is what makes a quad yaw: the rotors spinning one way against the ones spinning the other.
- **At the centre of mass:** the **net torque**, everything that turns the drone together: thrust lever arms, rotor reactions, gyroscopic torques, air drag, cables and the ground. Weight adds none there. It is drawn as an arc with an axis arrow, seen through the airframe, and smoothed over 0.15 s (the rotor arcs over 0.1 s). It equals J·ω̇ + ω × Jω, so it is exactly what changes the drone's spin.
- **Wanted torque** (a faint arrow, a separate switch): the torque the controller **asked for**, smoothed the same way. It differs from the net torque mostly while it's changing, because the motors take a few tens of milliseconds to follow.

The readout under the speed line gives the net torque in body axes: roll about X (forward), pitch about Y (left), yaw about Z (up), in N·m. In a steady hover it hovers around zero.

## Sensors and estimation

The controller doesn't see the true state. It flies on what its sensors report, through two estimators, just like real flight software. Sensors are parts you attach on the Airframe tab, each with a position and mount angle you can set:

| Sensor | Imperfections |
|---|---|
| IMU (gyro + accelerometer) | Noise, turn-on bias, gyro bias drift, range limits, sample rate, delay. Motor vibration (a sinusoid per motor at its rotation frequency) is stronger near busy motors, and a slow sample rate aliases it. An off-center accelerometer also feels the drone's rotation. |
| Compass | Noise, hard-iron offset, and interference from motor currents that grows with throttle and falls off with distance. |
| Barometer | Noise and slow drift. |
| Position fix | GPS, RTK GPS or motion-capture presets: noise, a slowly wandering error, update rate, delay, and a "signal lost" switch. |
| Optical flow + rangefinder | A downward camera that tracks how the ground slides past, plus a distance sensor. Flow noise grows as tracking quality drops; each axis has its own scale error; it saturates above a maximum flow rate. Quality depends on the ground's texture, the light and the height. The rangefinder has minimum and maximum range and noise that grows with distance. |

Every sensor has a mass, editable on its card (defaults: IMU 3 g, compass 2 g, barometer 2 g, GPS 15 g, optical flow 5 g). It counts toward the weight, the centre of mass and the inertia, rides on whatever it is attached to, and touches the ground. Designs saved before sensors had a mass get these defaults when opened.

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

## Heat, failures and the supervisor

Every motor, servo and the battery has a temperature and a health, and each can fail. A **supervisor** watches them the way a Raspberry Pi next to the ESP32 would, and rewrites the flight controller's settings when something goes wrong. The flight controller itself doesn't change: it keeps running the same allocation, just on an edited table.

### Heat (`thermalModel`)

Each part is one lump of heat: C dT/dt = P − G (T − T_air). The air temperature is a slider on the Airframe tab.

| Part | Heat in (P) | Holds (C) and loses (G) | What heat does |
|---|---|---|---|
| Motor | i² R: its copper loss, with R rising 0.39%/K; its back-EMF constant falls 0.12%/K, so a hot motor needs more current for the same thrust | C from the motor's mass. G is set so full load settles 25% past the limit with Cooling = 1, and grows with speed (its own prop cools it) | Past its limit it loses thrust for good, faster the hotter it is. 35 K past the limit it fails the way you set: stops, or loses the thrust percentage you gave it |
| Battery | I² R_int, with R_int higher when cold and when worn | From the pack's mass (38 g per Ah per cell) | Past its limit it wears (less capacity, more resistance). 25 K past it, it loses a cell or cuts out |

Motor cards have **Temperature limit**, **Cooling**, **Overheating damages it** and **When it fails**. The battery has its own section on the Airframe tab: cells, capacity, internal resistance, limit, sensors and failure mode.

### Breaking things

The Health panel (right column) lists every part with its true temperature and state, what the drone's sensors say about it, and what the supervisor has done. Each row has a **Break…** menu that fails the part mid-flight:

- **Motor:** stop it, or lose its set percentage of thrust.
- **Servo:** jam it where it is, or make it go limp (friction only, no torque).
- **Battery:** lose a cell, or cut out.

**Repair all** undoes every failure and the supervisor's changes without resetting the flight.

### What the drone can sense

The health sensors run on their own random stream, so switching them on doesn't change the flight's other noise.

| Sensor | Where it's set | Rate, delay, noise |
|---|---|---|
| Motor temperature sensor | Motor card, off by default | 10 Hz, 1.5 s thermal lag, ±0.3 °C |
| ESC telemetry (rpm and current) | Motor card, on by default | 50 Hz |
| Battery voltage and current | Battery section | 50 Hz |
| Battery temperature | Battery section | 2 Hz, 5 s lag |

With no temperature sensor but with ESC current, the supervisor runs the same heat model itself to estimate the motor's temperature. With neither, a motor's heat can't be seen until it fails.

With a voltage sensor, the flight controller corrects its throttle for sag (`voltageCompensation`: u_sent = u · V_ref / V), so thrust per command, and so the table, stays true as the pack drains.

### The supervisor (Formulas: Supervisor group)

It runs at 10 Hz and talks to the flight controller over a link that is 40 ms late each way. It reads the health sensors and a 50 Hz data stream from the controller: for each motor its column × thrust, for each steering servo how its column changes with angle, and the measured force and rotation. It runs three formulas in a chain:

1. **`actuatorHealth`** compares what the table predicts with what the IMU measures. From how the error changes, it fits one explanation per part: "motor *i* makes only η of its thrust" or "servo *j* is δ away from where it's told". Each fit gets a confidence from how much of the error it explains and how much that part has been moving. It doesn't judge until it has about 3 s of flying, and it ignores the ground, the first 1.5 s and throws.
2. **`faultDecision`** turns that into settings:
   - **Failed motor** (ESC rpm under 30% of what the command should give for 0.2 s, or η < 0.25 with high confidence for 0.5 s): removed from the table.
   - **Weakened motor** (η < 0.88 for 1.5 s): its column in the table, learned or described, is scaled by η.
   - **Hot motor:** its throttle is capped, from 100% at 20 K under its limit to 55% at the limit, so the others take the load before it's damaged.
   - **Stuck servo** (δ confidently above 3°, or its feedback disagrees by 5°, for 0.5 s): taken out of the steering and held at the angle the supervisor believes it's really at, so the motors on it are modelled where they really point.
3. **`flightPolicy`** decides how to fly on what's left, from the lift margin the remaining motors give, whether roll, pitch and yaw can still be held, the battery and the temperatures:

| Mode | When | Limits |
|---|---|---|
| Careful | a motor over 85% of its limit, a hot battery, margin < 1.6× | slower, less lean |
| Return home | a motor or a battery cell failed, margin < 1.35×, battery < 20% or < 3.3 V per cell | flies home at 1.5 m/s, then lands |
| Land | roll or pitch lost for 1 s, margin < 1.08×, battery < 8%, overheating | straight down, motors stop on the ground |

It only steps up, never back down. A sudden voltage drop of about a cell's worth is read as a lost cell: the supervisor counts one cell fewer when it works out the charge, and heads home.

### What it can and can't save (supervisor on vs off)

| Failure | Supervisor on | Supervisor off |
|---|---|---|
| Hex, one motor stops | Spotted in 0.4 s from ESC telemetry (0.7 s from the IMU alone), lands | Crashes |
| Quad, one motor stops | Crashes | Crashes |
| Quad or hex, a motor loses 50% | Table scaled, flies on | Flies, less precisely |
| Battery loses a cell | Goes home and lands | Keeps flying on a weaker pack |
| Battery cuts out | Falls | Falls |
| Tilt-rotor quad, a servo jams | Spotted, taken out of the steering, flies on | Flies, fighting the stuck servo |
| Overheating motor (with a sensor or ESC current) | Caps it, flies carefully, goes home before damage | Burns out |

A quad that loses a motor needs a controller that lets the body spin and flies on three (as in the Delft and ETH work); that isn't modelled.

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
| P (hold) | Charge a poke; release to hit. Tap for a nudge, hold 1.5 s for the strongest. The Poke button on the view works the same way |
| K | Pause / run |
| R | Reset |

Keys are ignored while you type in a text field or the formula editor. In the published page, click the 3D view first so the page receives the keys.

## Layout

| File | What it holds |
|---|---|
| `js/laws.js` | **The governing formulas**: 41 functions for the physics, airflow, sensors, estimators, identification, controller and supervisor, plus the text shown for each in the Formulas tab |
| `js/runtime.js` | Law registry: compiles edits, validates what each formula returns, falls back to the default when an edit fails |
| `js/budget.js` | Flight computer budget: counts what the flight code costs per control step and keeps in memory, for an ESP32 |
| `js/math.js` | Vector, matrix and quaternion helpers and the bounded least-squares solver. Everything here can be used inside formulas |
| `js/sim.js` | Airframe presets, mass properties, controller plumbing, physics stepping and the flight-envelope check |
| `js/multibody.js` | Articulated-body dynamics: the frame and every servo joint solved together (recursive Newton–Euler) |
| `js/joints.js` | Servo joints and rods: the attachment tree, poses from the joint angles (true and believed), carrying parts along, servo state |
| `js/learn.js` | Controller model: described vs. learned effectiveness, the calibration cycle and learning in flight |
| `js/health.js` | Heat, failures and health sensors for every part; the flight controller's settings the supervisor can change; the supervisor itself |
| `js/health-ui.js` | The Battery section and the Health panel |
| `js/sensors.js` | Sensor parts, sampling at each sensor's rate with delay, vibration and magnetic interference, and fusing readings for the estimators |
| `js/view3d.js` | three.js scene and camera |
| `js/pilot.js` | Keyboard and on-screen flight controls |
| `js/editor.js` | Edit mode: picking parts and the move and rotate handles |
| `js/formulas-ui.js` | The Formulas tab |
| `js/ui.js` | Airframe editor, telemetry, traces, header controls, persistence and the boot loop |
| `js/designs.js` | Undo and redo, saved designs (your account or this browser) and design files |
| `js/rn-parse.js` | The step compiler's parser for the formula subset |
| `js/rn-compile.js` | The step compiler: types, fixed places in the arena, fused list steps, linking, the steps listing |
| `js/rn-sigs.js` | What each flight formula takes and returns (the drone passes exactly these) |
| `js/rn-ops.js` | The runner's instruction set, shared by the compiler, both runners and the C header |
| `js/rn-vm.js` | The JavaScript runner, the program image with its self-tests, and the WebAssembly runner's wrapper |
| `js/rn-wasm.js` | The C runner built to WebAssembly (generated by `runner/build_wasm.sh`) |
| `js/rn-bridge.js` | The simulator's flight code on the runner, and loading edits in flight |
| `js/fc-export.js` | The airframe file for the flight controller (Export for the flight controller) |
| `js/fc-fil.js`, `js/fc-wasm.js` | Firmware in the loop: the flight controller's C code built to WebAssembly, flying in the simulator |
| `runner/fc/` | The flight code (`fc_core.c`), its tests and WebAssembly build, and the ESP32 flight firmware (`esp32/`) |
| `runner/` | The runner in C (`rn.c`), the drone's program slots (`rn_host.c`), the Pi link (`rn_link.c`, `pi/send_program.py`), the built-in program, the ESP-IDF example (`esp32/`) and the tests |
| `tools/` | Node tools: program export, the formula check against recorded flights, test data |
| `css/style.css` | Styles, light and dark |

## The formulas

**Physics (the plant):** `rigidBody`, `gravity`, `rotorWrench`, `jointRotation`, `motorDynamics`, `servoTorque`, `bodyDrag`, `cableTension`, `payloadDrag`, `groundContact`.

**Sensors:** `imuModel`, `magModel`, `baroModel`, `posFixModel`, `flowModel`, `rangeModel`.

**Airflow, battery and heat (physics):** `wakeVelocity`, `rotorAero`, `wakeLoad`, `batteryModel`, `thermalModel`.

**Estimation:** `attitudeEstimator`, `flowVelocity`, `servoPredictor`, `positionEstimator`.

**Identification:** `identifyThrow`, `identifyMotorResponse`, `identifyServoResponse`, `identifyEffectiveness`.

**Controller:** `positionControl`, `thrustAxisTarget`, `attitudeError`, `attitudeControl`, `forceDemand`, `allocationPreferences`, `allocation`, `thrustLinearization`, `voltageCompensation`.

**Supervisor:** `actuatorHealth`, `faultDecision`, `flightPolicy`.

The simulator only calls these by name through `run(key, …)`. To change a default, edit the function in `js/laws.js`.

Edits made in the Formulas tab:
- take effect on the next time step, mid-flight;
- are test-called with sample inputs before they're accepted, so syntax errors and wrong return shapes are rejected with a message;
- are switched off automatically if they throw or return something unusable during flight, and the default takes over;
- are saved in your browser's local storage. **Copy edited formulas** gives you text to paste back into `js/laws.js`.

`rotorWrench` and `jointRotation` are shared by the plant, the controller's effectiveness matrix and the envelope check. The controller evaluates `rotorWrench` at T = 1 N, so it assumes the law is linear in T.

## Flight code on the drone: the step runner

The flight formulas (estimation, in-flight learning, control, allocation: 16 of them) don't have to be ported to C by hand. The simulator compiles them into a list of steps for a small runner written in C. The same runner is built three ways:
- for the ESP32, as an ESP-IDF component (`runner/`);
- for a PC, where the tests run;
- as WebAssembly, which the simulator flies on by default (Formulas tab → **Flight code runs on: Step runner**).

A formula edited in the Formulas tab therefore flies in the simulator exactly as the drone would run it, and **Download program for the drone** gives the file to send it.

### What a program is

- **Steps:** each step is an opcode and its operands. Operands are addresses in one array of 32-bit floats, the arena, fixed when the program is built, so a step looks nothing up by name.
- **Built-in math:** the heavy parts are native in the runner: 3×3 matrices, quaternions, the bounded least-squares allocation, and arithmetic over whole lists. The formulas are the steps between them.
- **The compiler** (`js/rn-compile.js`) takes a subset of JavaScript: numbers, arrays and small records, the math helpers, `if`, `for` loops with a bound it can work out, `.map`, `.reduce`, `.slice` and `.push`. It uses each formula's signature (`js/rn-sigs.js`) for the types of its inputs.
- **Lists of actuator inputs** have room for `RN_IN` = 24. A drone that gains parts in flight uses the spare places, and every loop's worst case is known.
- **Memory:** a formula's memory (the `st` argument) becomes named fields in the arena.
- **Size:** the default program is 29 KB of steps and 65 KB of working memory.

Each flight formula's card in the Formulas tab shows its compiled steps and how many run per call.

### Safety on the drone

- **At load:** the loader (`rn_load`) rejects any step whose fixed address falls outside the arena, any write to the constants, any jump outside its formula, and any block (a matrix, a list) that doesn't fit.
- **At run time:** computed addresses and list lengths are checked, and each formula has a step limit. A bad or corrupted program can at worst compute wrong numbers in its own arena. It can't write anywhere else or hang the flight loop.
- **Self-tests:** a program carries its own tests: real inputs recorded in flight, and the outputs the simulator's runner gave for them. The drone runs them on load, which catches a runner or firmware mismatch.

### Loading an edit in flight

The simulator (`js/rn-bridge.js`) and the drone (`runner/rn_host.c`) take a new program the same way:

1. **Check:** the loader's checks, the self-tests, and on the drone, every formula must take and return exactly what the built-in one does.
2. **Background run:** for 1 s the new program runs beside the flying one on the same inputs, starting from a copy of its memory (carried across by field name). Its answers aren't used. A trap, a step limit or a number that isn't finite rejects it.
3. **Blend:** over 0.3 s the answers used move from the old program's to the new one's.
4. **Swap:** the new program flies with the memory it built up. The old one is kept.
5. **Fall back:** if the new program traps in flight, the previous one takes over within the same control step, with the memory carried across. On the drone, the built-in program (compiled into the firmware) is always there last.

Nothing that flies changes before step 4. The Formulas tab shows where an edit is and keeps a log.

### The drone and the Raspberry Pi

- **`runner/rn_host.c`** keeps three program slots: built-in, flying and candidate. `rn_host_prepare()` does the heavy checking on the link core, and `rn_host_tick()`, called by the flight loop, starts the background run, the blend and the swap. Inputs and results are passed as flat float arrays in each formula's signature order. A formula used several times (`servoPredictor`, once per servo) keeps a memory per instance.
- **`runner/rn_link.c`** frames messages over the UART: `DF`, a type, a length, the payload and a CRC-32.
- **`runner/pi/send_program.py`** is the Pi's side: `python3 send_program.py flight-formulas.rnp /dev/serial0`. It prints what the drone reports: loaded, rejected (and why), swapped, fell back.
- **`runner/esp32/`** is an example ESP-IDF project: the flight task on core 1 at 1 kHz, the link task on core 0. The built-in program's steps run straight from flash.
- **Memory:** three slots take about 250 KB. That's comfortable on an ESP32-S3, and tight on a plain ESP32 without PSRAM; there, put the loaded slots in PSRAM or build for fewer inputs.
- **`tools/export_program.js`** makes a program image from `js/laws.js` (with `--edit key=file.js` for edits). `--c runner/rn_builtin.c` writes it as the firmware's built-in program.

### Cost

**Measured on an ESP32-D0WDQ6** (ESP32-WROOM-32, 240 MHz, no PSRAM) with the bench firmware (`runner/bench`), which runs the default program's formulas on their self-test inputs:

| | Where | Time | Load |
|---|---|---|---|
| Flight loop: every formula once per step (4 servo predictors), except the learning | core 1, 1 kHz | 447–456 µs per step, longest ~550 µs | 45% |
| In-flight learning | core 0, 200 Hz | ~850 µs per update once it is learning | 17% |

Per call: the learning 390–850 µs, the allocation 88 µs, the position estimator 73 µs, allocation preferences 64 µs, the attitude estimator 53 µs, the rest under 25 µs each. The steps and the step loop run from IRAM and the step dispatch is a jump table; from flash, with ESP-IDF's default compare-chain switch, the same loop took 2.6 ms. A simple step costs about 90 cycles, roughly 10× hand-written C, so the flight-budget panel (which assumes about 6 operations, ~25 cycles, per step) is optimistic: real ESP32 time is about 4× its figure. The learning has its own working space in the program (`ownPool` in `js/rn-sigs.js`), so it can run on the other core at the same time.

Memory on that board: the built-in program's arena (65 KB) and one slot for programs sent over the link (arena 65 KB, steps in IRAM, 40 KB receive buffer) fit, with 54 KB left; a third slot doesn't, so the board runs in two-slot mode. Over USB at 115200 baud, a 40 KB program arrives in about 4 s and is checked and self-tested in 33 ms.

### Checks

`runner/test.sh` builds the runner and runs:
- **`tools/check_formulas.js`:** every flight formula compiled and run on the JavaScript runner and on the C runner, against about 180 calls recorded in simulated flights. They match within 32-bit float rounding. For the allocation, the forces and torques it makes must match, since several motor splits can be equally good.
- **`runner/test_rnhost.c`:** the drone's loading steps, covering a good edit, one that gives NaN in the background, one that traps after taking over, a corrupted image, and one whose formula takes other inputs.
- **`runner/test_link.c`:** the framing, including frames made by the Pi's Python.

## The flight controller firmware

`runner/fc/` is the drone's flight code around the formulas, and `runner/fc/esp32/` the firmware for an ESP32 that runs it. The simulator can fly the same C code: **Airframe → Flight controller → Fly the firmware** runs it built to WebAssembly against the simulator's physics. The IMU, barometer and battery readings go in, and the throttles and servo angles it returns drive the simulated ESCs and servos. So what flies in the simulator is what will fly on the drone.

### What it flies

- **The airframe** comes from the simulator. **Export for the flight controller** saves a `.dfa` file with:
  - the controller's model: mass and inertia (or the learned table, when it flies on that), and each motor's effect per full thrust as terms of its joints' angles;
  - the servos' ranges and speeds, and the throttle curves;
  - the allocation preferences and the IMU's mount.
  
  Nothing about the layout is hard-coded in the firmware.
- **Angle mode.** The sticks set the lean, from a level hover up to 35°, and the turn rate. The throttle stick is centred at 0.5:
  - **with a barometer** (BMP280/BME280), it sets the climb or sink speed, up to 2 m/s, and the middle holds the height;
  - **without one**, it sets vertical acceleration, and the middle keeps the vertical speed.
  
  Either way the accelerometer trims the thrust until the drone accelerates as asked, so the airframe's weight needn't be exact. There's no horizontal position hold: that needs GPS or optical flow.
- **Each step** is the simulator's control step, calling the flight formulas through the program slots (`rn_host.c`). So formulas sent from the Pi still reload in flight:
  1. `attitudeEstimator`;
  2. `servoPredictor` for each servo;
  3. `thrustAxisTarget`, `attitudeError`, `attitudeControl`, `forceDemand`;
  4. allocation in two stages: servo angles, then thrusts;
  5. `thrustLinearization` and `voltageCompensation`.
- **Learning is off** on the drone for now. It flies the exported table.

### Safety

- **At power-on** every ESC gets its minimum pulse.
- **Arming** needs all of these: the arm switch seen off since the last disarm (so nothing re-arms by itself), an airframe loaded, a working output for each of its motors and servos, a gyro, a settled attitude, less than 15° of tilt, the throttle stick at the bottom, and, when a battery sense wire is set, a reading that fits the pack.
- **Disarmed**, the motors get the minimum pulse. The motor test spins one motor, at most at 30%, for 3 s from when it starts (it stops sooner if commands stop); another test needs the test switched off first.
- **Commands** are clamped to their ranges; one with a number that isn't finite is ignored.
- **Link lost:** 0.5 s without a command. At idle (throttle at the bottom, most likely on the ground) it disarms. Otherwise it goes to the failsafe and levels:
  - **With a barometer** it descends at 1 m/s and disarms once it asks to sink but its height stays put for 1.5 s (landed). The barometer also measures the accelerometer's bias in flight (a few hundredths of a g from vibration or temperature is normal), so the speed it flies on is right.
  - **Without one** the speed can only be guessed from the accelerometer, which drifts. So it always asks for a little downward acceleration (0.2–0.6 m/s²) on the thrust it learned hovers, and drag sets the descent speed: in the tests 0.7–2 m/s. It can't climb on a biased guess. It disarms after the bump of touching down, and in any case after 120 s (about 80 m of descent): **for anything above that height, fit a barometer.**
- **Cut-offs:** tilting past 75° (a crash, also during the failsafe) or losing IMU data for 0.2 s while flying switches the motors off. So does a flight formula failing, or giving a number that isn't finite, for 50 ms even after the program slots fell back to the built-in program; a single bad step holds the last outputs.
- **The battery reading** is used for voltage compensation only when it fits the pack (0.6–1.35 × `vref`), so a loose sense wire can't multiply the throttles.
- **The link** checks each frame's length against its type and drops a frame whose bytes stop coming for 50 ms, so a damaged header can't swallow the commands after it.

### Hardware

**IMU**
- An MPU-6050 (GY-521), MPU-6500 or MPU-9250 on I2C: SDA 21, SCL 22.
- A LIS3DH is recognised, but it has no gyro, so a board with only a LIS3DH won't arm. It is still useful for motor tests and for checking the wiring.
- At boot, if the drone stands still, the firmware measures the gyro's offset.

**Barometer (optional):** a BMP280 or BME280 on the same bus.

**ESCs**
- Standard PWM ESCs, 1000–2000 µs at 400 Hz.
- Default pins: motors 1–8 on GPIO 25, 26, 27, 14, 32, 33, 4, 13. Motors 9–12 can go on free pins too; they share the servos' 8 channels.
- Only GPIO 4, 13, 14, 16–19, 21–23, 25–27, 32 and 33 are accepted for outputs. The boot-strapping pins (0, 2, 5, 12, 15) are refused: something wired there can stop the ESP32 booting, and some toggle during boot. A pin can't be used twice, and the longest ESC pulse must fit its period.

**Servos**
- 50 Hz, 1500 µs ±500 µs for ±45°, adjustable per servo.
- Default pins: servos on GPIO 16, 17, 18, 19, 23.

**Battery (optional):** a resistor divider to an ADC1 pin (32–39), set with `battery=34,11`. `vref` is the pack voltage the airframe's thrust is for: 16 V (4S) as in the simulator; set `vref=12` for a 3S pack.

**Wiring settings:** the wiring is kept in flash. Change it with `fly.py PORT set …`, then `save` and `reboot`.

**Tasks**

| Core | Task | What it does |
|---|---|---|
| Core 1 | Control loop (1 kHz) | Runs the flight code |
| Core 0 | Sensor task | Reads the IMU every step and the barometer at 25 Hz |
| Core 0 | Link task | The Pi's commands, programs, airframe and settings; telemetry at 20 Hz |

### The Pi side: `runner/pi/fly.py`

| Command | What it does |
|---|---|
| `python3 fly.py PORT status` | Show what it's doing and its wiring |
| `python3 fly.py PORT airframe my-drone.dfa` | Send the airframe; it's kept in flash |
| `python3 fly.py PORT watch` | Show telemetry and messages |
| `python3 fly.py PORT test 1 0.1` | Spin motor 1 at 10% for 2 s (**props off**) |
| `python3 fly.py PORT fly --gamepad` | Fly with a gamepad (`pip install pygame`), or with the keyboard for tethered tests |
| `python3 fly.py PORT program formulas.rnp` | Send edited formulas; they reload in flight as before |

- **Commands** go out 50 times a second while `fly.py` runs. If it stops, the drone goes to its failsafe.
- **While a program is arriving** (a few seconds at 115200 baud), the firmware keeps flying on the last command, only as long as a frame that size takes and never out of a failsafe. In `fly`, start with `--program FILE.rnp` and press P to send it; don't run a second `fly.py` on the same port (it opens the port exclusively, and without the DTR/RTS toggle that resets most ESP32 boards).
- **Gamepad:** after arming, the throttle stays at 0 until you first push the stick up; in flight, full down is the fastest descent, never idle. On the keyboard, `s` stops at 0.06 while armed and `x` goes to idle.
- **Flashing** the merged image at offset 0 erases the saved airframe and wiring. Send them again afterwards.

### Checks

- **`runner/fc/test_fc.c`** flies airframes exported from the simulator (`runner/fc/testdata/`: quad X, tilt-rotor quad, tricopter) against a rigid body driven by their own model, with drag. It checks:
  - the loader;
  - every arming refusal;
  - take-off, height hold, climb and sink;
  - lean and turn tracking;
  - the failsafe landing, with and without a barometer, including an accelerometer bias of ±0.1–0.5 m/s² that appears in flight and a link lost while climbing fast;
  - motor failure → crash cut-off, and IMU loss;
  - motor-test limits;
  - servo steering;
  - biased sensors.
- **`runner/test.sh`** runs it with the rest.

**Known limit: drift after fast flight.** The default attitude estimator trusts the accelerometer's "up", and while the drone speeds up or slows down that "up" is off. Two effects follow:
- after a fast pass the drone levels out slowly and drifts on for a while;
- a hard lean reads a few degrees less than it is.

The firmware tracks what it believes exactly. The belief is the formula's, and you can improve it in the Formulas tab (a lower accelerometer gain, or a model of rotor drag) and send it to the drone.

## What it models

The physics isn't simplified for speed; every step (2 kHz) does the full version. The controller runs at 1 kHz.

- **Articulated rigid bodies** (`js/multibody.js`, `rigidBody`). The frame is a free-floating body and every servo joint adds another: the servo's output and everything rigidly on it, including rods, further joints and what they carry. The equations of motion for the frame's 6 degrees of freedom plus every joint angle are built each step with the recursive Newton–Euler algorithm (Featherstone) and solved together. So a swinging arm pushes the frame the other way, a load drags its servo, the whole thing conserves momentum in free fall (checked: angular momentum to 0.2%, linear to 0.01% while an arm swings ±60°), and a sensor on an arm feels the arm's own acceleration.
- **Motors** (`motorDynamics`). Each motor is a brushless motor, ESC and prop sized from its card: throttle sets the voltage, back-EMF and winding resistance set the current, current sets the torque, the prop's inertia sets how fast it spins up, and thrust and drag torque grow with speed squared. From that come a throttle curve that bends upward, spin-up faster than spin-down, a current-limited start from standstill, thrust that fades with the battery, the frame feeling each motor's torque while it speeds up (the spin-up reaction), and the gyroscopic torque of a spinning prop when the frame or its servo turns it. Health scales the thrust. Hover hints (rpm and amps) show on each motor's bar.
- **Pullers and pushers** (a motor's **Prop** setting). A motor is mounted along its shaft, which points from the motor to the prop (the shaft tilt and azimuth). A puller (tractor) makes thrust along the shaft, toward the prop, and blows air back past the motor. A pusher's prop is pitched the other way: its thrust points back along the shaft, toward the motor, and it blows air away past the prop. Spin is always seen facing the prop, so the drag torque and gyroscopic torque follow the prop's real rotation either way; a pusher's spin about its thrust axis is the reverse of its card. A quad of pushers hung under the arms (shafts down) flies the same as an ordinary quad. In edit mode the selected motor shows its thrust arrow. Two faint arcs with arrowheads on each prop disc show which way it turns, seen facing the prop.
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

The **Flight computer** panel shows what the flight software alone would cost on the drone's own computer: an ESP32, ESP32-S3 or ESP32-C3. With the step runner flying, it counts the runner's steps (see above); with JavaScript, the cost of the same formulas written as plain C, which is what the table below shows. The simulator's physics isn't counted. Only what runs inside the control step is: estimation, learning, control, allocation and the code around them, plus the throw's background refinement.

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
- Without servo feedback, identification and allocation use the predicted servo angle. The supervisor catches a servo that jams or goes limp from the IMU, but not one that only sags a little under load.
- Each part is one lump of heat: a motor's windings and its case aren't separate, and the battery's cells heat evenly.
- The supervisor tests one failure at a time. Two at once (a motor and a cell together, say) can be misread, and a limp servo that swings freely can sometimes get another servo blamed.
- The vertical position integral can trim up to 5 m/s², enough to absorb an unknown hover throttle; sideways it stays at 2 m/s².
- In the throw start, a motor on two steering joints only has each joint varied on its own, so the cross terms of its 9 columns come from the hover calibration. The throw start also needs the IMU's mounting angle to be known. It uses the commanded throttle and a generic motor model rather than measured motor RPM, which the Delft work uses; the simulated motors have the same structure as that model, so real motors will fit it less exactly.
- Edited formulas run in the page itself, so an infinite loop in one will freeze the tab. (On the step runner, a formula's step limit stops it.)
- The step runner covers the flight formulas, not the code around them. The order of the calls, the calibration cycle, the sensor wiring and the motor outputs are the simulator's JavaScript (`js/sensors.js`, `js/sim.js`, `js/learn.js`), and on the drone they're still firmware to write; `runner/esp32/main/main.c` shows where they go. The one-off fits (`identifyThrow`, `identifyMotorResponse`, `identifyServoResponse`) and the supervisor aren't compiled yet.
- The runner has run on an ESP32 bench (its self-tests, timings, and loading, rejecting and falling back from programs sent over USB), not in a drone yet.
- In 32-bit floats the allocation can pick a different split between motors where several are equally good; the forces and torques it makes match.
