# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo joints, rigid masses, masses on cables and sensors, anything on anything, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

Every physical law and control law is a plain function in `js/laws.js`, and you can read and edit each one live in the **Formulas** tab.

## Run it

Open `index.html` in a browser. There is no build step. It needs an internet connection to load three.js (r128, from cdnjs) and the Google Fonts it uses.

The header has two groups:
- **Airframe:** the button says what's on screen (the saved design's name, or the layout it started from, with * once it's changed) and opens the menu: start from a layout or one of your saved designs, or open a design shared with you. The share icon beside it shares what's on screen (greyed out for a layout as it comes). **Sharing a design.** The **share** icon (beside the Airframe button) packs the whole design into a link: the airframe, its computers and its edited formulas, compressed into the part after `#`, which the browser never sends to a server. Nothing is stored anywhere; the page that opens the link unpacks it (asking first if the airframe on screen has unsaved changes), and you save it to keep it. A typical design makes a link of 1,100–1,300 characters, one with edited formulas a few thousand. The link needs the page at an address the other person can open (hosted somewhere, e.g. GitHub Pages); until then, **Copy the code only** and they paste it into **Open a shared design…** (the Airframe menu) in their copy of the page. Design files keep everything too.

A design is the whole drone: the airframe, its flight computers (the boards and their tasks) and the formulas they run (the ones you edited; the rest at their defaults). Opening a design brings its computers and formulas (a design saved before formulas were part of it leaves yours as they are), undo covers formula edits too, and a layout comes with the default computers and formulas (a flight controller and a Pi Zero; the cargo layout adds the Cargo task to drive its latch). **Blank** is a bare frame with a battery and an IMU; it opens in edit mode, resting on the ground, for you to add motors to. If the airframe on screen has changes you haven't saved, you're asked first: save it (under a name you give, never over a different design), don't save, or cancel. Opening a saved design or a design file asks the same.
- **Simulation:** ▶ / ❚❚ runs and pauses (**K**), ↺ resets (**R**), and ¼× ½× 1× sets the speed. Every flight starts on the ground with the motors stopped; the simulator then arms the drone and takes off to the target height for you, as you would with a real one. (**Reset into Hover / Throw** shows when a board runs the learning task: Throw has the drone thrown from the hand, to 7 m by default; picking one resets into it.)

**Which way is forward.** A red arrow on the hub points to the drone's nose (red, as the X axis on the orientation triad). A fainter red arrow beside the drone, level and just outside the props, shows the way the forward key moves it: the heading it holds (with navigation), or its nose (without). **Show ▾ → Forward** turns it off.

**Keys ?** lists every keyboard shortcut, and picks the layout: **Handset** (the default: W/S climb and descend, A/D turn, the arrows move, as the left and right sticks of a Mode 2 radio, the command module's keys and most drone simulators) or **Game** (W A S D move, the arrows climb and turn; the move pad goes to the left). The choice is kept in this browser. The flight keys don't take keys a focused control needs: Space and Enter press a focused button or tick a box, arrows move a focused slider, and the tabs move with ←/→. Selects that reset the flight (the world, a task's board) apply a keyboard change on Enter or when you leave them, so stepping through them with the arrows doesn't reset anything.

**Poke** is with the flight controls on the 3D view, beside Hold and Home.

A **crash** shows as a small banner above the flight controls, with the reason and a Reset button (or **R**). Nothing covers the view: the flight controller cuts the motors, and the physics keeps running, so the drone tumbles and falls until you reset. You can still orbit the camera to look at the wreck.

## The world

**World** in the header picks where the drone flies:
- **Open field:** flat ground, nothing else.
- **Parkour city** (the default): a compact city about 1:8 around an open plaza at the start point. Next to the plaza are low obstacles: rows of gates to fly through, platforms on stilts to fly under or land on, tunnels, steps and low walls. Further out are towers, stepped and L-shaped buildings, some joined by bridges. Streets are 1.3 to 2.5 m wide and towers reach about 10 m.
- **Full-scale city:** the same layout at real size (×8): streets 10 to 20 m wide, towers up to 80 m.

The dice button builds another city from a new random seed. The same seed always gives the same city, and the choice is remembered. Changing the world restarts the flight in the plaza. Reset starts the drone at its target, unless the target is inside or against a building (say, after crashing into one): then it starts again at the start point. Everything is a plain box, with no textures (`js/terrain.js`).

**Collisions.** The airframe touches things through small spheres: the hub, each motor and servo, points along the arms, every box mass, rods along their length, and the sensors. A small box is its corners; a bigger one (a wing, a long battery, the frame shaped as a wing) is spread over its whole surface, no more than about 5 cm apart (a thin plate as spheres as thick as it), so a building's edge can't slip between its corners. Any of them inside the ground or a building gets the ground-contact spring, turned to face that surface. A part that hits a thin wall or slab hard is pushed back out the side it came in by, so it never tunnels through. Landing on anything (the ground, a roof, a bridge) faster than 3 m/s is a crash. Bumping into a wall isn't, but the props usually won't survive it.

**Props are fragile.** Each prop's rim is checked at 16 points. If a spinning prop touches anything, it breaks:
- the motor keeps spinning a stub, with no thrust and almost no drag;
- the frame shakes from the stub's imbalance, which the IMU feels;
- the view shows a red stub;
- the Health panel says "prop broken" and logs what it hit.

A quad that clips a wall usually loses its front props and flips. You can also break a prop on demand from the motor's Break… menu. Reset or Repair all fits new props.

**Sensors see the city.**
- The rangefinder and optical flow measure the distance along their beam to whatever is there (a roof or a wall), not the height above the street. A drone holding its height on a rangefinder therefore jumps up when it crosses a roof, as a real one would.
- Ground effect works over rooftops too.
- The barometer and GPS are unaffected.

**In the view:**
- Buildings between the camera and the drone turn see-through.
- A soft shadow under the drone, on the street or a roof, helps judge height. Its switch is Shadow in the Show menu.
- In the full-scale city, the view reaches further and you can zoom out to 300 m.

## Flight computers

What flies the drone is its own flight code, the same C that runs on the ESP32 and the Raspberry Pi, nothing else. The **Computers** tab lists the boards on the drone and what each runs:

- **Boards:** ESP32, ESP32-S3, ESP32-C3 (microcontrollers), Raspberry Pi Zero, Zero 2 W or 4 (Linux computers). Add up to four, rename them, remove them. The default is what you have: an ESP32 flight controller and a Pi Zero.
- **Tasks:**
  - **Flight core** (`runner/fc/fc_core.c`), 1000 times a second: attitude, control and mixing, arming and the failsafes. It needs exact timing, so it must run on a microcontroller, and every drone has exactly one.
  - **Navigation** (`runner/fc/nav_core.c`), 100 times a second: where the drone is (GPS, optical flow and its rangefinder, barometer) and holding or moving its position. It sends the flight core *guided commands*: which way to accelerate and where to face. It can run on the Pi (talking to the ESP32 over the serial link) or on the ESP32 itself. With no navigation, you fly in **angle mode**: the keys lean the drone, and nothing holds its position.
  - **Learning** (`runner/fc/learn_core.c`), on the flight core's telemetry 200 times a second: what each motor and servo really does, in flight, in a hover calibration, or from a throw (below). It asks the flight core for test moves and tells it which model to fly on. It runs only on a Linux computer (a Pi): the throw's fit alone keeps about 300 KB. Without it, the boards fly on the airframe's description, and the learning panel and the throw start are hidden.
  - **Health supervisor** (`runner/fc/super_core.c`), 10 times a second: failing, weakened or hot parts, and how to fly on what's left (see Heat, failures and the supervisor). Also Pi only. Without it, nothing watches for failures and its parts of the Health panel are hidden.
  - **Cargo** (`runner/fc/cargo_core.c`), 50 times a second: the latches are wired to this board. It opens and closes them on the pilot's command and reports what they hold (see Cargo). Any board: the flight controller, the Pi, or an ESP32 of its own (with the radio on another board, the receiver's board passes it the LATCH commands with the channels).
  - **Telemetry & radio** (`runner/fc/tlm_core.c`, `tlm_crsf.c`, `rc_core.c`), 200 times a second: the ExpressLRS receiver is wired to this board. Its channels fly the drone, and the other tasks' telemetry comes here and goes down the radio (see Telemetry and the radio). It runs on either kind of board. Without it the drone has no radio: the simulator's pilot reaches the boards directly, as before, and the Ground station tab stays empty.
- **On the ground:** the command module, the pilot's side of the radio, when the drone has one (see The command module): an ESP32, a Pi, or a Mac or PC.
- **Default:** the ESP32 runs the flight core and the radio; the Pi Zero runs the navigation, the learning and the supervisor. Take any of the Pi's tasks off to fly without it.
- **Programs:** each board loads a program with the formulas of its tasks only. The ESP32's is small (about 14 KB of steps); the Pi's, with the throw's fit, about 170 KB.
- **Wiring:** the IMU, compass and barometer (the GY-87 is all three) go to the flight core's board; the GPS and the flow camera to the board that navigates; the health sensors (motor temperatures, ESC telemetry, the battery's voltage, current and temperature) to the supervisor's board; the ExpressLRS receiver to the radio's board.
- **Link:** a board talks to the flight controller over a serial link (921600 baud), 6 ms late each way. The flight core sends:
  - the navigation its attitude, rates, accelerometer and barometer height 100 times a second; the navigation answers with a guided command each time. If the commands stop, the flight core goes to its failsafe and lands;
  - the learning and the supervisor its telemetry 200 times a second (what each motor was told and what the IMU felt). The learning answers with test moves and the model to fly on; the supervisor with its settings, which also go to the navigation (to fly home or land) and the learning (to rescale what it learned). Tasks on the same board pass these directly.
- **Load:** each board shows roughly how much of a core its tasks take and, on a microcontroller, how much memory the flight program needs. An overloaded board turns red.
- **Export:** the flight core's board exports the airframe (`.dfa`, for `fly.py airframe`). The Pi exports its navigation config (`.dnc`: mass, where the barometer, GPS antenna and flow camera sit, which of them there are), and with the learning or the supervisor the airframe again and the Pi config (`.dlc`: where the IMU sits, each motor's heat model, the battery) for `dfb_pi`.
- **Changing the airframe** in flight changes the physics at once, but the flight core keeps flying on the airframe it was given until the next reset, as on the drone, which takes a new airframe only on the ground.

How the simulator runs them: each board is one instance of the flight code built to WebAssembly (`runner/fc/build_wasm.sh` → `js/board-wasm.js`), with the flight program compiled from the formulas. The simulator supplies only what the hardware would: sensor readings at their rates and delays, what one board sends another (after the link's delay), and the pilot. Everything about flying (estimating, deciding, mixing, arming, failsafes) happens inside the boards.

The formulas are listed under the task that runs them, and the board it's on. Edit one in flight and every board running it loads the new program through its own loader, as it would on the drone: self-tests, a second in the background beside the current version, then the swap. The log under **The flight program** shows each board's steps. The physics and sensor models are listed last, under **The world**: they're the simulator's, not flight code.

**Every flight:** the drone starts on the ground under the target, motors stopped. The simulator arms it after the attitude settles (with a radio, by flipping the handset's arm switch). With navigation, it asks the navigation to fly; the navigation takes off once its position estimate has settled on its references, and home is where it took off. Without navigation, the simulator opens the throttle until the barometer shows it has climbed most of the way, then centres the stick (which holds the height). A slim status line low on the view (where a crash notice goes, clear of the drone) says what it's doing meanwhile, step by step (starting the computers, levelling, arming, finding its position, taking off, with how high it has got), and why it's stuck when it is: an arming refusal with the flight core's reason, or a position that takes long to settle. On the stock layouts the whole launch takes about 3 s (indoors on optical flow, about 6 s).

## Learning the airframe

The learning is a task on the Pi (Computers tab), the same C as `dfb_pi` runs on a real Pi: `runner/fc/learn_core.c` around the formulas `identifyEffectiveness`, `identifyMotorResponse`, `identifyServoResponse` and `identifyThrow`. It works on the flight core's telemetry, and it never drives a motor itself: it asks the flight core for test moves (which lapse 0.1 s after the last request, so if the Pi stops, the drone simply flies on) and tells it which model to fly on. The panel is called **Learning** and shows only when a board runs the task.

The controller commands **throttle fractions (0–1)**, not Newtons, and doesn't need to know prop sizes, mass or inertia. What it needs is the **effectiveness matrix B**: how much linear and angular acceleration each actuator input produces. B comes from one of two sources.

- **Description:** computed from the airframe you entered, including only the parts marked as known to the controller.
- **Learned:** identified from flight data by recursive least squares (`identifyEffectiveness`), starting from the description. The learner:
  - uses the accelerometer and gyro as outputs, and the throttle sent as inputs;
  - band-passes both sides (0.3–12 Hz), so steady offsets like drag can't leak in;
  - removes the IMU's lever-arm swing.

  A motor on servo joints is several inputs: its thrust times each product of (1, cos θ, sin θ) over the joints it sits on. That's 3 columns for one joint and 9 for two. Turning a rigid part about a hinge is linear in cos θ and sin θ, so this is exact, and its effect at any joint angles is a fixed sum of learned columns.

**Calibrate** (Learning panel) runs about 12 s of test moves while hovering, or about 25 s with servos:
1. **Settle.**
2. **Test each motor on its own.** It steps up then down, by 6% and then by 16%, while every other motor and servo keeps what it had (**Freeze other motors during pulses**). Each test waits until the drone is calm first. Freezing matters because otherwise the controller answers every pulse with the other motors, and inputs that always move together can't be told apart.
3. **Test each servo on its own.** It swings one way, then the other, by 35% of its range, while the motors and every other servo hold still.
4. **Sweep each servo.** The sweep rides on top of what the controller asks for, so it keeps its servo authority; a tricopter's tail servo is its only real yaw control.
5. **Excite everything together.**
6. **Validate** on a fresh signal.

It then scores the learned model and the description on the same validation data, and switches to the learned model only if it predicts better. On the stock presets the description is already near-perfect and is kept. With hidden masses, weak motors or unknown parts, the learned model wins and the drone flies noticeably better.

**Keep learning in flight** continues the identification with 30 s of memory and a 2% dither, to track slow changes such as the battery draining. Run Calibrate again after big changes.

The learning runs at 200 Hz, on each telemetry frame. Between frames the flight core averages what it sends (the inputs, accelerometer, gyro and motor commands), which is the learning's anti-aliasing filter. The learned model reaches the flight core 5 times a second while it learns in flight.

While a calibration runs, the controller keeps flying on the model it had when the calibration started, and switches only when the calibration decides.

On the stock presets in the simulator, a calibration from the description takes 11 to 25 s and ends with the learned model explaining 91 to 99% of the rotation and 69 to 98% of the force on fresh test moves, against 45 to 95% and 53 to 94% for the description (the tilt-rotor quad gains the most); it then flies on the learned model. The main-lifter layout is the exception: its hover already wanders in a slowly growing circle without any learning, and the motor tests push it over.

### Actuator response

The single-actuator tests learn how each actuator responds over time, not just how strong it is. The results appear under **Actuator response** next to the true values.

| Learned | From | Used for |
|---|---|---|
| Motor lag | `identifyMotorResponse`: which spin-up time best explains the response to a step | The effectiveness identification, which used to assume 35 ms for every motor |
| Servo speed and lag | `identifyServoResponse`: the speed limit and lag that best explain how the drone's response traces the servo's real angle, with no angle feedback needed | `servoPredictor` in the flight core (sent with the model), the servo angle the controller uses when a servo has no feedback |
| Throttle-curve bend | `identifyMotorResponse`: whether a step up gives more than the same step down | Shown only. The flight core assumes a typical brushless curve (bend 0.7) |

The tests run on the 200 Hz telemetry, so a motor's pulse is about 50 samples.

In the tests on the stock presets:
- **Motor lag:** reads long. The thrust really follows a step in about 25–30 ms near hover; the tests give 40–65 ms on most motors (30–80 ms across the presets), with fits above 0.99. The throw's fit, which works on every 1 ms step, gets about 30 ms. Something in the 200 Hz path (most likely how the telemetry's averaging and the 25 Hz filter line up with the pulse edges) adds 10–30 ms; until that's found, treat the hover tests' lag as an upper bound.
- **Servo speed and lag:** the measured speed is what the servo really manages on a short step, below its no-load speed (260 against 360°/s on the tilt-rotor quad, 260 against 300°/s on the tricopter's loaded tail, 150 against 300°/s on the helicopter's swashplate). The command delay (20 ms) comes out at 15–30 ms.
- **Throttle-curve bend:** the real curve comes from the motor physics (about 0.8). The tests can't see it: they give anything from −0.5 to 1.5, because the air a pulse pushes through the prop and the battery sagging under load produce effects of the same size. So it's shown but not used. On real hardware this is measured on a thrust stand.

The hardware has traits the controller is never told, marked **hidden** on the part cards, so there's something to learn:
- **Spin-up time** of each motor (sets its prop and rotor inertia).
- **Servo stall torque** (0.8 N·m), **command delay** (20 ms) and **trim error** (0°).
- **Servo feedback** (off by default, like hobby servos). Without it, the controller never sees servo angles and relies on its prediction.


The panel shows how close each actuator's learned effect is to the truth. The truth comes from linearizing the real simulated physics (airflow and battery included) by nudging each input.

## Throw start

The throw start needs the learning task. In the simulator the drone starts in the hand (1.2 m up), is armed with its motors off, and the learning is told a throw is coming; on a real Pi you arm it in your hand and type `throw`. The hand throws it once the navigation (if any) has its position. From there the learning flies it: it notices the free fall (the accelerometer reads nearly nothing), pulses the motors in open loop through the flight core, fits, sends the model and lets the flight core catch it.

**Reset to: Throw** (or **T**) starts the drone the way Blaha, Smeur and Remes (TU Delft, 2024) do: it is held still for a moment, then thrown upward with its motors off and a random tumble. The throw itself takes 0.12 s of hand push, which the IMU feels, so the drone knows it's climbing. It knows its sensors and how many actuators it has, and nothing about its geometry, mass, props or motors.

1. **Climb.** It rides the throw with the motors off.
2. **Pulse over the top of the arc.** Each motor fires on its own at 50% throttle. A pulse ends after 80 ms, or earlier once the drone's rotation has changed by 4 rad/s, which keeps well inside the gyro's range. The flight core cuts a pulse itself the moment that happens, rather than a link's round trip later. A motor on steering joints is pulsed with each of those joints in the middle, at one end and at the other end, so all its columns can be told apart. The pulses are timed to finish just after the top; on the way down it soon needs its height to recover. If it runs out of room (it needs about 0.45 s to spin up and turn upright, then brakes at 0.8 g), it stops pulsing early. A motor it only tried in the middle is then treated as fixed there, and the servos are held in the middle, with the drone leaning to move, until a calibration has measured them.
3. **Fit** (`identifyThrow`). In free fall the accelerometer feels no gravity, only the rotors and its own swing around the center of gravity. One least-squares fit on under a second of data gives:
   - the effectiveness matrix;
   - where the IMU sits relative to the balance point;
   - the gyroscopic coupling between axes;
   - the spin-up reaction (B₂, their G₂): a motor speeding up twists the frame the other way, several times harder than its steady drag torque. Without this term a pulse from standstill looks like a huge yaw effect;
   - the motor lag. The drone doesn't measure prop speed, so it runs a generic brushless motor model (back-EMF, a current limit, prop drag) with its time constant unknown.

   While falling it keeps one running fit per candidate lag (all motors the same), a fixed cost per step, and a compact 250 Hz log. It fits on every control step: while it flies open loop, the flight core's telemetry carries each 1 ms step's sample rather than their average (the averaged 200 Hz frames made the motor lags come out 15 ms long and the effects 20–50% off). At the moment it has to catch itself it picks the best of those fits at once. Then, in the background, it works out each motor's own lag from the log, one motor at a time, and switches to that if it explains the fall better. A big slow rotor and small fast ones can then share a frame. It handles up to 12 inputs (a quad with a tilting motor on each arm); the main-lifter layout and anything bigger can't be thrown.

   Nothing fights the pulses, so each motor's effect comes out clean.
4. **Catch.** The controller takes over on the model it just learned. It gets upright first and turns to the target heading afterwards.
5. **Refine** (optional, on by default). A hover calibration runs, starting from and competing against the throw model.

If the fit is poor, it catches itself on the airframe description instead and says so. The panel suggests a minimum throw height for the current airframe, since more actuators mean more pulses and a longer fall.

Results on the stock presets (thrown to 7 m, the default), with the learning on the Pi:
- **Identification:** the fit explains 99–100% of the rotation and 97–99% of the force; motor lag comes out at 30–45 ms (true 30 ms), the IMU offset about 1 cm from the balance point. The fit explains the fall well, but the effects it gives match the true ones only 50–85% (the panel's bars): turbulence and the prop's own inflow during the fall make it a rougher model than a hover calibration's.
- **Recovery:** the quad, hexacopter, tricopter, tilt-rotor quad and indoor quad all catch themselves, at about 5.5–6 m, then come down to the target at the navigation's 1.5 m/s (lowest point 1.3–1.4 m for a 1.5 m target). The tilt-rotor quad fires 9 of its 12 pulses before it has to stop (all 12 from 10 m, which also works). From 4 m, the old default, there was much less room: fewer pulses, and the lowest point on the way down was 0.7–1.2 m.
- **A fix that came with the higher throw:** a long pulse sequence can leave the drone upside down when it starts to catch itself. The flight core is meant to allow any attitude for 3 s after a throw's open loop ends, but its tilt cut-off ran first on that very step and switched the motors off at once (the tilt-rotor quad, thrown to 6 m or more). It now waits.
- **Afterwards:** the hover calibration brings the learned model to 87–97% of the force and 90–99% of the rotation on fresh moves, and it flies on that.

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
- **Shapes in the air** (`wingAero`, `bluffDrag`): see Wings and blunt parts, below.
- **Battery** (`batteryModel`): a 4-cell 1.3 Ah LiPo. It supplies the current the motors really draw, drains with it and sags under it, so the same throttle gives less thrust over a flight and during hard manoeuvres. Its resting voltage follows a real discharge curve: 4.2 V per cell full, flat around 3.8 V through the middle, 3.6 V at 10%, 3.2 V empty, and collapsing past that; near empty its internal resistance grows too, so it sags more. The flight controller's voltage compensation keeps the thrust up by raising the throttles until they run out; after that the drone can no longer hold its height.
- **ESC low-voltage cutoff** (Battery section, 2.8 V per cell by default, 0 turns it off): once the pack stays under it, under load, for 1.5 s, the ESCs stop their motors; they restart only after the throttle has been at zero. **Charge at take-off** starts a flight on a part-used pack, to try this without waiting.

  On the quad from 25%, with no supervisor: it flies on as the throttles creep up (0.63 to 0.80), and at about 3% left, 3 m up, the ESCs cut out and it falls. With the supervisor (and a voltage sensor), it heads home at 20% and lands with about 18% left.

Prop radius is a motor setting. **Airflow** on the 3D view shows the wake columns.

### Wings and blunt parts

Every solid part has a shape in the air, **prism** or **wing**: the frame (Frame shape, under the frame's mass) and each rigid mass ("In the air" on its card). **+ Wing** adds a rigid mass that is a wing: 70 cm span, 10 cm chord, 6° of incidence.

- **Prism** (`bluffDrag`): drag on the face the part shows the air, by direction (Cd 1.05 on each face of its box; a sphere's or cylinder's own frontal area). A flat battery drags more face-on than edge-on. The frame as a prism keeps its general drag (`bodyDrag`).
- **Wing** (`wingAero`): a box with its chord along X (leading edge forward), span along Y and thickness along Z, tipped by its **incidence** (leading edge up). It meets the wind, its own motion and the rotors' wash: lift across the airflow rising with the angle of attack (2πα, less for a short wing), a stall at about 15° falling off to what a flat plate gives, drag that grows with the lift, acting at the quarter chord (the middle once stalled). Air from below, from behind or along the span all give sensible forces, so it works in a hover, in a side wind or flying backwards. On a servo set by you, a wing is a flap, a tilting wing or an air brake.
- **The body as a wing:** Frame shape → Wing turns the hub into one, with its own span, chord, thickness and incidence.

**The flight computers aren't told about wings.** To them a wing is an oddly shaped body, as it would be bolted onto a real drone running this firmware: the attitude and position integrators take up what it does, as they do an unknown mass. So it shows: on the **Quad with a wing** layout (an 80 × 10 cm wing at 15° over a quad), accelerating forward at sport speed it leans 20–25° and the wing pushes *down*; cruising at 6.5 m/s it leans about 10° and the wing lifts about 1 N (11% of the weight), and the drone balloons up 20–30 cm before the integrators take it back. With the frame as a 70 cm wing, a quad only reaches about 3.5 m/s at sport: the plate drags when it leans. Each wing is drawn as one: a cambered airfoil (2% camber, its thickness from the wing's) stretched along its span. **Wing lift** and **Wing drag** (Show → Forces, each its own switch, in the legend) draw each wing's lift (green, across the air past it) and drag (orange, along it) from where they act, smoothed over a tenth of a second and sized like the thrust arrows; under the torque line, the readout gives the wings' lift (and whether it pulls up or down), their drag, the angle of attack and the airspeed of the biggest wing, and says when it has stalled. A chip under the airframe check gives the wings' pull with its share of the weight.

The learning doesn't model wings either: it measures what the motors and servos do with short test moves and filters out slow, steady forces, so a wing is background to it. The health supervisor compares the forces it expects from the motors with what the IMU feels; a big wing in fast flight can make it think a motor is weaker than it is (and so slow down, or head home). The airframe check is about hovering, where wings do little, so it leaves them out.

### An imperfect world

Real drones never hover perfectly still, even with a good controller. The simulator adds what disturbs them (Target & environment panel):
- **Turbulence** (0 still air, 1 gusty; default 0.3, light). Gusts ride on the steady wind, random and lasting about 2 s, stronger in stronger wind and weaker vertically. They push the drone around and change what each rotor meets.
- **Churned air at each rotor.** Each disc meets small eddies of its own, lasting about a tenth of a second, from the turbulence and from its own wash curling back. The eddies are about three times stronger near the ground or a roof. Each rotor's thrust wobbles by a few percent on its own, and that uneven push is what makes a real hover twitch. Even still air keeps a little of it.
- **Motor and prop differences** (1 typical; 0 identical). No two motors and props are alike: each one's thrust differs from its card by about 3%, its drag by about 5% and its spin-up time by about 10%. Each motor's values stay fixed for that motor. The controller and supervisor aren't told; the firmware's integrators and the learning have to absorb it.

Measured on the quad over 20 s of hover, with the firmware flying (angle mode, no position hold):

| | Roll-rate jitter | Tilt | Drift |
|---|---|---|---|
| Still air, identical motors (before these existed) | 0.15°/s | 0.7 ± 0.17° | 5 m |
| Defaults (turbulence 0.3, differences 1) | 2.9°/s | 1.0 ± 0.4° | 7 m |
| Turbulence 1 | 8°/s | 2.0 ± 1.0° | 16 m |

The simulator's own controller looks shakier: about 6°/s even in still air. That's not the world. It holds position on noisy GPS, and with **Keep learning in flight** on it also adds a small deliberate throttle wiggle (2%, a few Hz) so the learner always has something to learn from. The firmware does neither.

## Servo joints and rods

A **servo joint** is a hinge mounted on the frame, on a rod or on another joint. A **rod / lever** is a rigid stick with a mass; whatever you attach to it rides at its far end. Anything can be attached to either: motors, rigid masses, cable payloads, sensors, rods and further joints. Chaining them builds an arm: shoulder servo → upper-arm rod → elbow servo → forearm rod → hand and camera. **+ Motor on servo** adds a joint with a motor at the same point, the usual tilt-rotor.

**Adding a part asks where it goes.** Each **+** button opens the airframe as a tree (the frame, then each servo, rod and latch, indented the way the parts list indents them). Pick **on Frame**, or **on** a servo, rod or latch, and the part is attached there. A servo, a rod or a latch can also go **between** a part and what it hangs on: with a rod on the frame and a motor on the rod, *between Rod 1 and M1* puts a latch in there, the motor hanging from it. Nothing moves: the latch's hook goes at the motor's top, a servo's pivot at the part, a rod from where the part hung to the part, and the part keeps its place. Esc closes the list.

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

## Cargo: dropping things and picking them up

A **latch** is a part like any other (Attach → **Latch (drop, grab)**): a hook, a gripper or an electromagnet, mounted on the frame, a rod or a servo. Anything can hang from it: a parcel, a cable payload, a whole arm, a motor, the battery. Add a part on it (or **between** a part and what it hangs on) and it hangs from the hook, its top at the hook. Drag a part that's already there onto it in the list (or set its **Attached to**) and it hangs from it where it is: nothing moves.

**Letting go.** A board runs the **Cargo** task (Computers tab) and drives the latches. Adding a latch puts the task on the flight controller if no board has it; a design without it (one saved before this) says so on the latch's button, and pressing it adds the task there (the flight restarts with it). Opened in flight, a latch takes its open/close time (0.15 s by default); half open, everything under it falls away together as one loose body, keeping the drone's speed and spin at that moment. A cable payload falls as a ball of its own, from where it was swinging. The drone is lighter at once; the flight core isn't told (it flies on the airframe it was given until the next reset, as for any change in flight), so its integrators take up the difference. Drop a motor and the flight core still commands it; drop the IMU and it has nothing to fly on.

**Picking up.** Opened again, a latch shows how far the nearest loose thing is: a dashed line from its hook to that thing's grab point (the middle of its top), green within the latch's **reach**, grey further away, and the same distance on its button. Close it with something within reach and that thing snaps under the hook, in the drone's axes, and the drone carries it from then on (the two share their momentum). Closed with nothing in reach, it just closes. Loose things include whatever the drone dropped and the **things to pick up** set in the Cargo section of the right panel (its Flight tab): boxes with a name, a mass, a size and a place (from the start point), resting on whatever is under them. They're in the world only when the airframe has a latch.

**The buttons.** Each latch has a button on the view, above the flight controls: *drop 250 g* when it holds something, *grab · 12 cm* when something is within reach, *fetch · 2.5 m* when the nearest thing is further (up to 15 m), with a bar that fills as you get closer. **G** works the chosen one (the last one pressed). With a radio the press goes up it as a **LATCH** command (the command module queues it; it goes once the link is up), and the Ground tab's Cargo section has Open and Close buttons too; without one it goes straight to the cargo task's board, as over a cable.

**Power.** A rigid mass can be marked as a battery ("It's a battery" on its card; the layouts' Battery is one, and so is a mass named Battery in a design saved before this). Drop the last one and the drone has no power: the motors stop, every board goes dark (its latches stay where they are), the receiver goes quiet and the command module raises its "no telemetry" alarm, until the next reset. A design with no mass marked as a battery is always powered.

**What's solid.** A loose thing falls under gravity and air drag and bounces to rest on the ground, roofs and other things already at rest, through contact springs at its corners. Once at rest it is solid to the drone: it can stand on it, and its spinning props break on it like on anything else. It doesn't push a resting thing about, and a falling one passes through the drone.

**Fetching.** Press *fetch* and the drone picks the thing up by itself (`runner/fc/pickup_core.c`, part of the navigation's pilot): it flies over it, 0.8 m up, facing the way it faces; comes down slowly (0.35 m/s, so it doesn't dip under the spot near the ground and clip a prop), below the usual 0.15 m floor if it has to; waits until it holds the spot within 5 cm, nearly still, for 0.6 s; has the cargo task close the latch; and climbs back. The button shows which step it's on; press it again, or move the sticks, to stop. If it can't hold still over the spot within 25 s, it gives up and climbs back without closing anything.

The drone doesn't know where things are, or where its own hook is: whoever asks for the pickup does. In the simulator that's the nearest loose thing and the latch's place on the airframe. On the drone it's the pilot: `pickup X Y Z` on the command module (`dfb_ground`, with `--hook DX,DY,DZ`, where the hook sits from the hub) or on the Pi (`dfb_pi`, the same). X Y Z is where the thing's top is, from home, as the pilot knows it (a map, a measured spot, a camera); the command module works out where the hub must go from the hook's offset and the heading in the drone's telemetry, and sends that up as a **PICKUP** command. How well it lands the hook on the thing depends on how well the drone knows where it is: with RTK or optical flow, within a few centimetres; on a plain GPS, the hook can come down half a metre off and close on nothing.

**The Cargo quad** (Layouts) has a hook beside the battery carrying a 250 g bag on a 35 cm line, an RTK GPS (to line the hook up on a parcel: a plain GPS wanders half a metre) and the Cargo task on the flight controller; a 250 g parcel waits 1.3 m away. Drop the bag, then press *fetch* to have it pick up the parcel (or fly over it yourself, come down slowly to the lowest the navigation allows, 0.15 m above where it took off, and grab it: descending in one go from a few metres, it can dip lower and clip a prop).

**The cargo task** (`runner/fc/cargo_core.c`, the same C on every board) takes the pilot's commands (the radio's LATCH command, text commands, or another board's), drives each latch open or closed and gives it its travel time. With a load switch (a microswitch in the hook; on by default) it knows whether something hangs from it, and says so: "latch 1 open: load released", "open but still loaded: stuck?", "closed: holding a load", "closed: nothing in it". It never lets go on its own; a board that restarts drives its latches as they were set up (closed, normally), so a reset in flight doesn't drop the load. Its state goes down the radio as the **cargo** telemetry item. On a Raspberry Pi, `dfb_pi --latch pwm0,gpio17` drives a servo on a hardware PWM channel or an on/off line (below).

## AI agent

The **AI** tab (left panel) connects an AI model that can do what you do here: read the state, fly, rebuild the airframe, move tasks between boards, rewrite formulas, change the wind, break parts, and run the simulation to see what happened. It works on the simulator only: the page has no link to a real drone.

**Connecting.** Until a model is connected the tab is one card: pick the provider (OpenAI, OpenRouter, Ollama or LM Studio on this computer, or another OpenAI-compatible API), the base URL, the API key and the model (**Load models** lists what the endpoint has), and **Connect** (it checks the connection first; a server that doesn't list its models can be saved anyway). Use a model that can call tools. The page sends requests straight from your browser to that endpoint; the key goes nowhere else. It's kept until the tab closes, or in this browser if you tick **Remember**. A server on this computer must allow requests from a web page (CORS; for Ollama, start it with `OLLAMA_ORIGINS=*`). **⚙ Settings** shows the connection, to change or remove (your chats and triggers stay).

**Threads.** Once connected, the tab is the chat. Its list has your **chats** (**+ New chat**; each titled by its first message, with the tokens it used) and the **triggers**, each with its own thread. A thread's header shows the model and the tokens and requests it used; the list's header, the session's against its budget. Chats are kept in this browser (the newest 30, and each trigger's).

**The chat.** Ask in words: "fly a 2 m square at 2 m height and say how well it held the corners", "add a wing and see how it flies in 5 m/s wind", "make the position control softer", "find out why it crashes when M2 stops". Each tool it uses shows as a line you can open to see what it sent and got back. Its tools:

- **Reading:** the state (position, attitude, target, battery, wind, the airframe check, edited or stopped formulas, latches, recent events), the last 120 s of telemetry (10 samples a second), the airframe and any part, the computers, the formulas and their code.
- **Building:** add a part (on the frame, on a servo, rod or latch, or between a part and what it hangs on, as the + buttons do), change a part's fields, attach, remove, set the frame mass, steering and battery, load a layout, set the boards and their tasks.
- **Formulas:** replace one (it's test-called first; an error goes back to the model and nothing changes) or reset it.
- **Flying and the world:** reset, pause, run, speed, throw, calibrate; go to a point, hold, home; open, close or fetch with a latch; wind, turbulence, light, terrain; break a motor, servo or the battery, repair all.
- **Waiting:** run the simulation for a number of seconds, in real time or as fast as it goes, optionally until a condition holds (`err < 0.1`), and report the range of height, distance to target and tilt, and what happened.

**Everything else.** It reaches whatever the page shows or does: every part's true state and what the sensors and the supervisor make of it (a broken prop, a stopped motor, a jammed servo, the battery), each motor's thrust, throttle, speed and current and each servo's commanded, believed and real angle, the estimate against the truth and each sensor's reading, the learning and its settings, the boards' load and messages, the airframe check in full with the mass properties, the radio and the command module, the logs; it holds the flight keys and pokes, sets the camera and what the view draws, the allocation preferences, the radio's settings and the frame's shape, saves and opens designs and undoes, places the things to pick up, and sets up its own triggers. Two more are off until **Settings** turn them on: **looking** at the 3D view (a picture, for a model that can see images; it's dropped from the conversation once seen) and **running its own JavaScript** in the page for anything the tools don't reach (it has the whole page, your API key included, so only for a model and endpoint you trust; it asks first unless you untick that).

**Undoing it.** Airframe and computer changes go into Undo like your own. A formula change has its own **Undo** under its line in the chat. Settings can make it ask before every formula change (Apply / Don't, in the chat).

**While it thinks** the simulation pauses (it picks up where it was when the answer comes); untick it in Settings to keep it running. A turn takes at most 16 rounds of tools. Every request counts against the session's budget (60 by default, in Settings, with the tokens used so far), so a loop can't run away with your API credit.

**Triggers** (**+ New trigger** in the list; its settings at the top of its thread, what happened below, where you can carry on the conversation) ask the AI on their own when something happens: it crashes, the battery falls below a level, it's further from the target than a distance, it tilts more than an angle, a formula stops with an error, every N seconds while flying, or an expression of your own over the telemetry (`alt < 0.5 && flying`, `err > 1 && batt < 40`). They're checked 10 times a simulated second; one fires when its condition turns true, with its message and the state at that moment, and not again within its gap. **Keep flying** lets the simulation run while the AI thinks about it, as a real drone would have to, instead of pausing.

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

The flight code doesn't see the true state. It flies on what its sensors report, through two estimators: the attitude estimator in the flight core, the position estimator in the navigation. Sensors are parts you attach on the Airframe tab, each with a position and mount angle you can set:

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

The **State estimate** panel shows the boards' estimate minus the truth, and warns about missing references. The dashed outline in the 3D view is where the flight software thinks the drone is. (There is no flying on ground truth any more: the boards only ever see their sensors, as on the drone.)

Sensor noise comes from a seeded generator, so every reset replays the same noise.

## Heat, failures and the supervisor

Every motor, servo and the battery has a temperature and a health, and each can fail. The **health supervisor**, a task on the Pi (`runner/fc/super_core.c`, the same C as `dfb_pi`), watches them and rewrites the flight core's settings when something goes wrong. The flight core itself doesn't change: it keeps running the same allocation, just on an edited table.

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

**Repair all** undoes every failure without resetting the flight. The boards keep what they decided (a motor taken out stays out) until the next reset.

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

### The supervisor

It runs at 10 Hz on the Pi. The health sensors are wired to its board; from the flight core's telemetry it takes 50 samples a second: for each motor its column (as the flight core flies it: described or learned) × thrust, for each steering servo how its column changes with angle, and the measured force and rotation. It runs four formulas in a chain:

1. **`actuatorHealth`** compares what the table predicts with what the IMU measures. From how the error changes, it fits one explanation per part: "motor *i* makes only η of its thrust" or "servo *j* is δ away from where it's told". Each fit gets a confidence from how much of the error it explains and how much that part has been moving. It doesn't judge until it has about 3 s of flying, and it ignores the ground, the first 1.5 s and throws.
2. **`faultDecision`** turns that into settings:
   - **Failed motor** (ESC rpm under 30% of what the command should give for 0.2 s, or η < 0.25 with high confidence for 0.5 s): removed from the table.
   - **Weakened motor** (η < 0.88 for 1.5 s): its column in the table, learned or described, is scaled by η.
   - **Hot motor:** its throttle is capped, from 100% at 20 K under its limit to 55% at the limit, so the others take the load before it's damaged.
   - **Stuck servo** (δ confidently above 3°, or its feedback disagrees by 5°, for 0.5 s): taken out of the steering and held at the angle the supervisor believes it's really at, so the motors on it are modelled where they really point.
3. **`liftMargin`** works out, with the table as the flight core now flies it (parts out, ceilings on, steering servos across what's left of their travel), how much lift there is and whether roll, pitch and yaw can still be held at hover.
4. **`flightPolicy`** decides how to fly on what's left, from that margin, the battery and the temperatures:

| Mode | When | Limits |
|---|---|---|
| Careful | a motor over 85% of its limit, a hot battery, margin < 1.6× | slower, less lean |
| Return home | a motor or a battery cell failed, margin < 1.35×, battery < 20% or < 3.3 V per cell for a second | flies home at 1.5 m/s, then lands |
| Land | roll or pitch lost for 1 s, margin < 1.08×, battery < 8%, overheating | straight down, motors stop on the ground |

It only steps up, never back down. A sudden voltage drop of about a cell's worth is read as a lost cell: the supervisor counts one cell fewer when it works out the charge, and heads home. The low-voltage rule waits a second so a helicopter's spool-up dip doesn't trigger it.

Its settings go to the flight core (parts out, columns scaled, ceilings, lean and acceleration limits), to the navigation, which flies home and lands by itself (the pilot's keys are ignored meanwhile; once down, it disarms), and to the learning, which rescales what it has learned for a weakened motor. Without navigation the flight core lands where it is, with its failsafe descent.

### What it can and can't save (supervisor on vs off)

| Failure | Supervisor on | Supervisor off |
|---|---|---|
| Hex, one motor stops | Spotted in 0.4 s from ESC telemetry (0.7 s from the IMU alone), lands | Crashes |
| Quad, one motor stops | Crashes | Crashes |
| Quad or hex, a motor loses 50% | Table scaled, flies on | Flies, less precisely |
| Battery loses a cell | Goes home and lands | Keeps flying on a weaker pack |
| Battery cuts out | Falls | Falls |
| Battery runs flat | Goes home at 20% and lands | Flies on until the ESCs cut out (about 3% left) and falls |
| Tilt-rotor quad, a servo jams | Spotted, taken out of the steering, flies on | Flies, fighting the stuck servo |
| Overheating motor (with a sensor or ESC current) | Caps it, flies carefully, goes home before damage | Burns out |

A quad that loses a motor needs a controller that lets the body spin and flies on three (as in the Delft and ETH work); that isn't modelled.

## Flying it

**Typing numbers.** Every slider has a box beside it, and what you type there isn't held to the slider's range: a 200 kg payload, a 3 m rod, 12 cells. The box only refuses zero or less for something that must be positive (a mass, a size), keeps fractions and percentages within 0–100%, and rounds whole-number fields; a value past the slider says so for a moment.

**Steering** (Airframe tab) is part of the design: how the drone makes a sideways force. **Tilt body** leans the whole drone, as any multirotor does. **Stay level** keeps the body flat and swings the rotors on their servos to push sideways (thrust vectoring). **Mixed** has the servos make a share of it (the slider under it) and leans for the rest. It's written into the flight controller's airframe file (`fc-export.js` → `fc_core.c`, read at start), and the flight core hands it to `thrustAxisTarget`, `forceDemand` and `allocation`; so a change starts the flight again. Mixed and Stay level need a servo that tilts a rotor; on other airframes only Tilt body can be picked. The supervisor falls back to Tilt body by itself when it holds the servos (a failing one).

The pads on the 3D view and the keyboard steer the drone. With navigation they move the target it holds, at a commanded velocity that is also fed forward to the position law, so every airframe you build flies with the same controls. Without navigation (angle mode) they are the sticks: the arrows lean the drone (Gentle, Normal and Sport set how far), A/D turn it, W/S climb and sink around the hover throttle; Hold and Home are hidden, since nothing knows where the drone is.

| Key | Action |
|---|---|
| W / S | Climb / descend |
| A / D | Turn left / right |
| ↑ / ↓ | Forward / back, relative to the heading |
| ← / → | Left / right, relative to the heading |
| Space | Stop and hold the current position (navigation only) |
| H | Fly back to where it took off (navigation only) |
| 1 / 2 / 3 | Gentle (1 m/s) / Normal (3 m/s) / Sport (6 m/s) |
| C | Chase camera: keep the view behind the drone |
| G | The chosen latch: drop what it holds, or close on what's within its reach (see Cargo) |
| E | Edit mode: select and drag parts in the 3D view |
| P (hold) | Charge a poke; release to hit. Tap for a nudge, hold 1.5 s for the strongest. The Poke button on the view works the same way |
| K | Pause / run |
| R | Reset |

With a radio (the Telemetry & radio task), the keys and pads are the buttons of the command module, the pilot's side of the radio (see The command module): its code turns them into channels that go up the simulated link, so a weak link makes the drone slow to answer and a lost one makes it fly home. Held keys ease the sticks in over a quarter of a second (its stickInput formula).

Keys are ignored while you type in a text field or the formula editor. In the published page, click the 3D view first so the page receives the keys.

## Where each number comes from

**Reading the panels.** Both side panels use the same tabs with icons (left: Airframe, Computers, Ground, AI; right: Flight, Health, Control, World). A section's longer explanation sits behind the **ⓘ** by its heading, and its source tags are coloured dots there (hover one, or open **Where it comes from** for the key). The Airframe tab has the design (name, Save, Undo/Redo, Open, Export, the saved designs folded away), the parts (the + buttons, sensors folded under **+ Sensors**, then the parts tree) and the body (Frame, Steering and Battery, each a row that shows its current values and opens to edit them). In the Control tab, the learning's options and the throw start fold away the same way.

**The right panel** has a status strip on top that never changes height: four tiles with icons (**Airframe**: the airframe check's verdict, its explanation on hover and in the Control tab; **Flight**: the phase and how close it holds its target, or why it crashed; **Battery**: charge, voltage and cells; **Formulas**: how many are edited or stopped, a click opens the Computers tab) and one line of warnings (the supervisor stepping in, a broken part, a motor at its limit, a slack cable, the wings' pull). Below it, tabs with icons: **Flight** (the actuator commands, the traces, the cargo), **Health** (the parts grouped as motors, servos and power, each with a dot for its true state, its temperature and a Break menu, and a line under it only when the drone's sensors or the supervisor have something to add; the supervisor and how many parts are in trouble on top; a red dot on the tab while something is broken or failing, or the supervisor has stepped in; then the state estimate), **Control** (the learning, the allocation preferences, the control headroom, the mass properties) and **World** (the target and the environment: wind, turbulence, light, temperature). The tab you leave it on is kept.

A simulator shows much that no drone could know, next to what the drone's own code works out, so every panel is tagged with where its numbers come from (open **Where it comes from** at the top of the right panel for the key; hover a tag or dot for its meaning):

- **Simulated:** the simulated world itself: the true position, thrust, temperatures, battery charge. The 3D view and its readouts are this.
- **Sensor:** what the drone's simulated sensors report, with their noise, delay and limits.
- **On board:** what the drone's flight code worked out (its state, its commands, the supervisor's and the learning's decisions), read straight from the boards as with a cable, not over the radio.
- **Telemetry:** what came down the radio link, as the command module decoded it (the Ground station tab).
- **Command module:** what the command module's own code worked out: the channels it sends, its alerts.
- **Vs truth:** the drone's belief against the truth (the state estimate's errors, how close the learned model is).
- **Calculated:** the simulator's analysis of the design (the airframe check, control headroom, mass properties, board loads).
- **You:** settings, targets and the sticks.

Sections with more than one kind tag each line with a coloured dot: in Health, a part's true state is simulated, the line under it shows the sensor reading and what the supervisor did.

## Telemetry and the radio

The drone has a pilot's radio: an ExpressLRS 2.4 GHz receiver, wired by CRSF to the board that runs the **Telemetry & radio** task. Everything the pilot does goes up it as channels, and everything the **Ground** tab (the ground station) shows came down it. Nothing on that tab reads the simulator or the boards directly: if the link can't carry it, it isn't there. (The right-hand panels are the simulator's view and stay as they are.)

**One interface, any radio.** The tasks don't know there's a radio. Each publishes what it has into a telemetry store after its step (`runner/fc/tlm_sources.c`), the same way on every board:

| Item | From | Every | What |
|---|---|---|---|
| attitude | flight core | 0.1 s | roll, pitch, yaw |
| height | flight core | 0.2 s | barometer height and climb |
| battery | flight core, supervisor | 0.5 s | voltage; current, mAh and charge from the supervisor |
| state | flight core | 1 s, and on change | armed, failsafe, crashed…; which sensors it trusts |
| motors | flight core | 0.5 s | each motor's throttle |
| GPS | navigation | 0.5 s | latitude, longitude to 1e-7°, height, speed, course, satellites |
| position, navigation | navigation | 0.25 s, 0.5 s | position and velocity from home; target, heading, mode bits, speed level |
| learning | learning | 1 s | calibrating, progress, which model, the throw's phase, the fit |
| supervisor, parts | supervisor | 1 s, 2 s | mode, why, lift margin, charge; each motor's state |
| link | radio | 1 s | what the receiver reports |
| cargo | cargo | 2 s, and on change | each latch: closed or open, moving, and with a load switch, loaded or empty |
| messages | every task | when they change | each task's message (the flight core's why, the navigation's, the learning's, the supervisor's events) |

A store on a board without the radio is packed into `RN_LINK_TLM` frames and sent to the radio's board when it asks (`RN_LINK_WANT` bit 2). There a scheduler (`tlm_core.c`) decides what goes next within what the link carries: the flight mode on a change and every second, then messages, then anything that changed state, then whichever item is most overdue. The bytes allowed per second come from the radio's settings (`tlm_crsf_budget`: rate ÷ ratio × 5 bytes, less 10%), scaled by the telemetry link quality the receiver reports, and nothing while it reports nothing for a second (`tlm_crsf_budget_now`). So a slow link sends everything less often rather than falling behind, and a lost one doesn't fill the receiver's queue (a receiver has no flow control: it would drop frames, the newest messages among them); the messages wait in the store and go first when the link is back. The transport is a small table of functions (`tlm_transport`: send an item, the flight mode, a message); `tlm_crsf` is the one there is. Another radio (MAVLink over a serial modem, say) would be another table; the tasks and the store wouldn't change.

**CRSF, as a receiver speaks it** (`runner/fc/crsf.c`, 420000 baud, frames of address, length, type, payload and a CRC-8). What a standard handset or app already understands goes in the standard frames: attitude (0x1E), battery (0x08), GPS (0x02), barometer height (0x09, with the climb), flight mode (0x21). The rest goes in extended frames (0x80) with a sub-type: 0xF1 a text message (as ArduPilot sends them), 0xD0 a telemetry item (its number and its values as scaled 16-bit integers). Coming up: channels (0x16: 16 channels of 11 bits), link statistics (0x14) and 0xD1 a ground-station command (GOTO x y z in centimetres from home, heading in milliradians; LEARN; LATCH latch, action; PICKUP x y z, heading, latch; numbered, so a repeat is done once).

**The handset's channels** (`runner/fc/rc_core.c`):

| Ch | Stick or switch | With navigation | In angle mode |
|---|---|---|---|
| 1, 2 | Right stick | move left/right, forward/back (relative to the heading) | lean |
| 3 | Left stick up/down | climb, sink | throttle |
| 4 | Left stick left/right | turn | turn rate |
| 5 | Arm | arm | arm |
| 6 | Three-way | Gentle, Normal, Sport (1, 3, 6 m/s) | — |
| 7 | Fly | take off; off lands | — |
| 8, 9 | Momentary | Hold here, Fly home | — |

With navigation the sticks move the target at the chosen speed, ramped and kept inside a box 25 m around home and 0.15–15 m up (low enough to hover a hook over a parcel on the ground), and the ground station's go-to moves it straight there. **Radio lost** (1 s without channels): with navigation it flies home and lands, and if the link comes back before it's down, it holds where it is and you have it again. In angle mode the board stops sending stick commands, so the flight core's own failsafe levels it and lands it.

**The simulated link** (`js/elrs.js`). Packets go at the radio's rate (50, 150, 250 or 500 Hz); one in `ratio` comes down carrying 5 bytes of the receiver's queued frames, the rest go up with the channels (now and then 5 bytes of a ground-station command instead). Whether a packet gets through depends on the margin over the receiver's sensitivity at that rate (−115 dBm at 50 Hz to −105 dBm at 500 Hz): transmit power, free-space loss from the handset at the launch point, 18 dB for each building in the line of sight in the city worlds, and an **extra loss** setting for the distance and walls the small world can't have. A lost telemetry chunk is sent again (ExpressLRS's "stubborn sender"), so frames arrive whole but late on a weak link; the receiver's queue holds 512 bytes (a length byte and the frame each), as ExpressLRS's does: a newer frame of the same kind (types below 0x28, and status texts) replaces one still waiting, and when full the oldest go. (The drone therefore spaces its messages so one never waits behind another.) The transmitter module drops a command that arrives while its link isn't connected (no telemetry for five telemetry periods, at least half a second); the command module holds its commands while the module reports the link down. The receiver reports link statistics to the drone ten times a second. At the default 250 Hz and 1:4 that's about 310 bytes a second, which the scheduler fills with roughly 280.

**The Ground station tab** shows what the command module decoded (below) with a small set of displays (`js/gs-widgets.js`: value, bar, badge, artificial horizon, map, columns, message log, sparkline), each greyed once its data is 1.5 s old and struck through at 5 s: the link (quality up and down, RSSI, SNR, throughput against the room it has, the channels being sent, and what the extra loss is equivalent to), the flight (mode, horizon, height, climb, speed, distance from home, GPS), the battery, a map of the track from home, the motors and the Pi's tasks, and the messages; at the top, the command module's alert. Under the radio settings, the **link log** shows what passes through the two simulated modules, which a real ground station can't see, one line each: ↑ the sticks as the drone's receiver passes them on (when they move, and back to centre), switch changes, and commands (`GOTO x 3.0 y 2.0 z 2.5 hdg 0° #2 · 20 ms · 3 pk`); ↓ the drone's messages and flight-mode changes reaching the command module, with how long they took; frames the receiver drops; the link going quiet or coming back. **Every frame** adds all the telemetry. The sticks are channels, in every uplink packet, not commands: the command module sends intent, and only one-off actions are commands. Click a line, or press Enter or Space on it, for its bytes, field by field (address, length, type, payload, CRC); holding the mouse on the list holds new lines back until you let go. **Pause** holds the list still (the button counts what came since), **Clear** starts it afresh, and the toggles above choose what it shows. Above it, the link's numbers over the last 5 s: channel frames made and carried (the command module makes one every 4 ms, each uplink packet carries the newest) and their latency, commands and theirs, telemetry frames written by the drone, delivered and superseded (a newer one of the same kind replaced it in the receiver's queue), their latency, packets lost, frames dropped, and commands dropped while the link was down. Each packet arrives after its time on air (about 13 ms at 50 Hz, 3 ms at 250 Hz).

Everything the pilot does reaches the drone only through the link: the keys and pads through the command module's channels, the go-to (the target fields, the map), Hold and Home, and the Learning panel's buttons (calibrate, which model, keep learning, the throw) as LEARN commands, which the navigation passes on to the learning. Cut the link (extra loss 200 dB) and the keys do nothing: the drone flies home and lands. After landing by itself (the link lost, or the supervisor) it stays disarmed until the arm switch goes off and on again (text: `arm`), and says so. The link also counts as lost when the receiver's link statistics report uplink LQ 0, for receivers that keep sending channels in failsafe. Turning the **fly** switch off in the air lands where it is, then idles, armed; on the ground it stays down. When channels stop coming for 0.1 s the drone counts the sticks as centred (it brakes), as receivers' failsafe hold stage does; after 1 s the link is lost. The radio's settings are there too; the boards take them at once, in flight too (on a real drone they're set on both ends). To show a new telemetry item, add it on the drone (tlm_core.h, tlm_sources.c) and give its values names in `js/crsf.js` for the tab: the command module decodes any item with the drone's own table, so there is no second copy of its scaling.

## The command module

The pilot's side of the radio is real code too: `runner/ground/ground_core.c`, the command module, wired to an ExpressLRS transmitter module by CRSF. The same C runs:
- **in the simulator**, built into the same WebAssembly as the drone's boards, as its own instance on the far side of the simulated link. Your keys and the simulator's pilot (arm, take off) are its buttons;
- **on an ESP32** with buttons, sticks, a buzzer and an LED wired to it (`runner/ground/esp32`);
- **on a Mac, a Pi or any Linux computer** (`runner/ground/dfb_ground.c`), with the terminal's keys, a gamepad, or your own code over UDP.

It sends **intent, not control**. What goes up is stick positions and switches, every 4 ms, and one-off commands (go to, calibrate); what a stick position makes the drone do, its limits, the 25 m box, and what happens if the link drops all stay on the drone, so it's safe whatever is on this side.

Each step it:
1. **Shapes the sticks** with its `stickInput` formula: a real stick gets a 4% deadband and expo on roll, pitch and yaw; a button or key, which is only on or off, eases the stick to full over a quarter of a second and back faster, so a tap nudges and holding is full. Buttons can **latch** (a push button as the arm or fly switch: press on, press off); a switch that holds its own state (a toggle, a `press arm` from a script, a key in the simulator) is taken as it is and never latches. Commands go one at a time, each in place of a channel frame (one frame per beat, so on a single-wire module bay a command never collides with the module's reply); a new go-to replaces one still waiting, so dragging a target sends the latest, not a backlog. Commands wait while the module reports the link down and are dropped after 10 s. **Switch warning:** arm and fly aren't sent on until each has been seen off since the start, so a switch left on, a stuck button or a floating pin doesn't arm the drone when the link comes up.
2. **Sends the channels** (rc_core.h's order) as CRSF frames for the transmitter module, and **commands** from a small queue, one every 0.15 s, so the drone (which keeps the latest) acts on each. The calibrate button sends a learning command; the drone's navigation passes it to the learning.
3. **Decodes what comes back**: the drone's telemetry (standard frames, its own items with the drone's scale table, messages) and the transmitter module's link statistics, into the view the Ground station shows.
4. **Warns** with its `groundAlerts` formula, ten times a second: an alarm when it crashed, telemetry stopped for 1.5 s, a failsafe, the drone hears no radio, or the battery is under 10% (3.3 V a cell); a warning when it lands or returns home, the battery is under 25% (3.5 V), or the uplink is under 60%. A warning stays up 2 s after it clears. The ESP32 beeps and blinks for it; `dfb_ground` prints it and rings the terminal's bell.

If its formulas fail and even its built-in program can't answer, the raw sticks go up unshaped: the pilot keeps control. (In the simulator, a ground program that doesn't compile, fit or load means none runs: the raw sticks go up, groundAlerts doesn't run, and the Ground tab says so.) Its formulas are in the Computers tab under **On the ground**, editable like the drone's: an edit goes through the command module's own loader (checks, a second in the background, then the swap). **Download its program** gives `dfb_ground --program FILE.rnp`. Which computer it is (ESP32, a Pi, a Mac or PC) only sets the load shown; its built-in program is `runner/ground/rn_builtin_ground.c` (`node tools/export_program.js --tasks ground --sym rn_builtin_ground --c runner/ground/rn_builtin_ground.c`).

**On a Mac or a Pi:** `sh runner/ground/build.sh`, then
- `./runner/ground/dfb_ground --tx /dev/tty.usbserial-XXXX --keys` flies from the terminal: W/S climb and sink, A/D turn, the arrows move, Space holds, H flies home, 1/2/3 set the speed, R arms, T takes off and lands, C calibrates. (A terminal reports presses but not releases, so a key counts as held until half a second after its last repeat.)
- `--joystick /dev/input/js0` (Linux) reads a gamepad, Mode 2 by default (`--axes`, `--buttons` to change it). On a Mac, `runner/ground/examples/gamepad.py` (pygame) sends a pad's sticks over UDP.
- Your own code sends text commands over UDP (port 14561, from this computer only; `--listen-all` takes them from the network, where anyone on it could arm the drone) or on the terminal: `press`/`release`/`tap NAME` (right left fwd back up down yawr yawl arm fly hold home gentle normal sport cal), `stick roll|pitch|throttle|yaw V`, `goto X Y Z [HEADING]`, `calibrate`, `latch N|all open|close|toggle`, `pickup X Y Z [LATCH]` (the thing's top from home; `--hook DX,DY,DZ` says where the hook is), `cmd ID V…`, `status`, `messages` (and **G** on the keys: latch 1). Sticks and stick buttons sent this way lapse a second after they were last sent, like a radio's channels: if a script stops, the sticks centre. `press` and `release` set a latching button on or off, `tap` toggles it; `press hold|home|cal|gentle|normal|sport` are taps; the other buttons hold until `release`. Values are checked: sticks within ±1, `goto` within ±327 m, `cmd` ids 1–255; anything else is refused with the reason. `examples/fly_square.py` flies a square; `examples/buttons_pi.py` turns push buttons on a Pi's GPIO into presses.
- `--baud` sets the CRSF speed (400000 by default). macOS takes non-standard speeds through IOSSIOSPEED, Linux through termios2.

**On an ESP32:** flash `firmware/dfb_command_module_esp32_v5.bin` at offset 0 (or build `runner/ground/esp32` with ESP-IDF), then over USB at 115200 baud: `set tx=17,16` (to the module's CRSF input, from its output; one pin for a module bay's single wire: `set tx=17,17`: inverted, half duplex, as ExpressLRS expects there; pins 1 and 3 are the console and are refused, 12 is refused for the module, and 16/17 are PSRAM on WROVER modules), `set arm=25` and so on for each button (to ground), `set roll=34` for an analog stick on an ADC pin (centred at power-on; `34i` inverts), `set buzzer=26`, `set led=2`, `set latch=arm,fly`, then `save` and `reboot` (a `set` takes effect after the reboot: `show` marks wiring changed since power-on). The same text commands work on that port. Buttons are debounced (20 ms). `reboot` is refused while the drone reports itself armed or flying (`reboot force` overrides); after a crash or watchdog restart the switches come back as they were. (dfb_ground on a Mac has no such memory: restarted in flight, it holds arm off and the drone disarms when the link returns, so don't.)

**What's untested:** this has run against the simulator and, on a PC, end to end against the drone's code through pseudo-terminals, not yet against real ExpressLRS hardware. Check, with the props off: the speed your transmitter module expects on its CRSF input (400000 here; modules differ and some detect it), the one-wire wiring of a module bay, and that the module passes the command frames up (the channels always go; the custom 0x80 frames depend on the ExpressLRS version, and MSP over CRSF would be the fallback).

**Where to run it.** Measured on the ESP32 firmware build: the radio task adds 8.1 KB of static memory, a 6 KB task stack and 2 KB of UART buffers (about 16 KB of RAM), and 13 KB of flash, and costs well under 1% of a core. That fits beside the flight core with room to spare, and the receiver then flies the drone even if the Pi stops, so the default puts it on the ESP32. On the Pi it works the same (`dfb_pi --crsf`); the flight core's items then come over the serial link.

## Layout

| File | What it holds |
|---|---|
| `js/laws.js` | **The governing formulas**: 41 functions for the physics, airflow, sensors, estimators, identification, controller and supervisor, plus the text shown for each in the Computers tab |
| `js/runtime.js` | Law registry: compiles edits, validates what each formula returns, falls back to the default when an edit fails |
| `js/budget.js` | Flight computer budget: counts what the flight code costs per control step and keeps in memory, for an ESP32 |
| `js/math.js` | Vector, matrix and quaternion helpers and the bounded least-squares solver. Everything here can be used inside formulas |
| `js/terrain.js` | The world: the city generator (seeded), and the contact, ray and surface-below queries the physics and sensors use |
| `js/sim.js` | Airframe presets, mass properties, controller plumbing, physics stepping and the flight-envelope check |
| `js/multibody.js` | Articulated-body dynamics: the frame and every servo joint solved together (recursive Newton–Euler) |
| `js/joints.js` | Servo joints and rods: the attachment tree, poses from the joint angles (true and believed), carrying parts along, servo state |
| `js/learn.js` | Controller model: described vs. learned effectiveness, the calibration cycle and learning in flight |
| `js/health.js` | Heat, failures and health sensors for every part; the flight controller's settings the supervisor can change; the supervisor itself |
| `js/health-ui.js` | The Battery section and the Health panel |
| `js/sensors.js` | Sensor parts, sampling at each sensor's rate with delay, vibration and magnetic interference, and the drivers' part: readings in body axes for the boards |
| `js/view3d.js` | three.js scene and camera |
| `js/pilot.js` | Keyboard and on-screen flight controls |
| `js/sources.js` | The tags that say where each readout comes from (simulated, sensor, on board, telemetry, command module, vs truth, calculated, you) |
| `js/cargo.js` | Latches, loose bodies in the world, dropping and picking up, power from the battery: what's on the drone now, as an overlay on the design |
| `js/place-ui.js` | Where a new part goes: the list each + button opens, putting a part on a holder or between two |
| `js/agent.js` | The AI agent: the connection, the conversation with the model and its tools, the telemetry it reads, the triggers |
| `js/agent-tools.js` | The agent's tools: each a description for the model and a call into the simulator |
| `js/agent-ui.js` | The AI tab: the chat, the triggers, the connection and settings |
| `js/cargo-ui.js` | The latch buttons on the view (G), how far the nearest loose thing is, the Cargo section |
| `js/crsf.js` | CRSF as the simulated radio modules handle it: frames, the parser, channel and link-statistics frames; the names of the drone's telemetry items |
| `js/elrs.js` | The simulated ExpressLRS link (packets, signal, telemetry slots, the two modules), the command module's inputs, and the Ground station's copy of its view |
| `js/gs-widgets.js` | The data displays: value, bar, badge, horizon, map, columns, log, sparkline |
| `js/gs-ui.js` | The Ground station tab |
| `js/editor.js` | Edit mode: picking parts and the move and rotate handles |
| `js/computers-ui.js` | The Computers tab: boards, tasks, the formulas under the task that runs them, the world's models |
| `js/ui.js` | Airframe editor, telemetry, traces, header controls, persistence and the boot loop |
| `js/designs.js` | Undo and redo, saved designs (your account or this browser) and design files |
| `js/rn-parse.js` | The step compiler's parser for the formula subset |
| `js/rn-compile.js` | The step compiler: types, fixed places in the arena, fused list steps, linking, the steps listing |
| `js/rn-sigs.js` | What each flight formula takes and returns (the drone passes exactly these) |
| `js/rn-ops.js` | The runner's instruction set, shared by the compiler, both runners and the C header |
| `js/rn-vm.js` | The JavaScript runner, the program image with its self-tests, and the WebAssembly runner's wrapper |
| `js/rn-wasm.js` | The C runner built to WebAssembly (generated by `runner/build_wasm.sh`) |
| `js/rn-bridge.js` | Compiling the flight formulas into the program every board loads |
| `js/boards.js` | The flight computers: boards, tasks, wiring, links; one WebAssembly instance of the flight code per board; the simulator's pilot (arm, take off) |
| `js/board-wasm.js` | The flight code (`fc_core.c`, `nav_core.c`, …, the runner) and the command module (`ground_core.c`) built to WebAssembly by `runner/fc/build_wasm.sh` |
| `js/fc-export.js` | The airframe file for the flight core (`.dfa`) |
| `runner/fc/` | The flight code: the flight core (`fc_core.c`), the navigation (`nav_core.c`), the learning (`learn_core.c`), the health supervisor (`super_core.c`), the telemetry store and scheduler (`tlm_core.c`, `tlm_sources.c`), CRSF (`crsf.c`, `tlm_crsf.c`), the radio pilot (`rc_core.c`), the cargo task (`cargo_core.c`) and the pickup (`pickup_core.c`), their tests, the WebAssembly build, and the ESP32 flight firmware (`esp32/`) |
| `runner/ground/` | The command module: its core (`ground_core.c`), its text commands (`ground_text.c`), the program for a Mac or a Pi (`dfb_ground.c`, `build.sh`), the ESP32 firmware (`esp32/`), its built-in program, examples, its tests (`test_ground.c`, and `test_ground_e2e.c` with `dfb_pi`) |
| `runner/pi/` | The Pi's side: `dfb_pi.c` (the navigation, the learning and the supervisor, with GPS and the serial link, and the latches' outputs, `latch_hw.c`; `build.sh`), its built-in program (`rn_builtin_pi.c`), its end-to-end test, `fly.py` |
| `runner/` | The runner in C (`rn.c`), the drone's program slots (`rn_host.c`), the Pi link (`rn_link.c`, `pi/send_program.py`), the built-in program, the ESP-IDF example (`esp32/`) and the tests |
| `tools/` | Node tools: program export, the formula check against recorded flights, test data |
| `css/style.css` | Styles, light and dark |

## The formulas

**Physics (the plant):** `rigidBody`, `gravity`, `rotorWrench`, `jointRotation`, `motorDynamics`, `servoTorque`, `bodyDrag`, `cableTension`, `payloadDrag`, `groundContact`.

**Sensors:** `imuModel`, `magModel`, `baroModel`, `posFixModel`, `flowModel`, `rangeModel`.

**Airflow, battery and heat (physics):** `wakeVelocity`, `rotorAero`, `wakeLoad`, `wingAero`, `bluffDrag`, `batteryModel`, `thermalModel`.

**Estimation:** `attitudeEstimator`, `flowVelocity`, `servoPredictor`, `positionEstimator`.

**Identification:** `identifyThrow`, `identifyMotorResponse`, `identifyServoResponse`, `identifyEffectiveness`.

**Controller:** `positionControl`, `thrustAxisTarget`, `attitudeError`, `attitudeControl`, `forceDemand`, `allocationPreferences`, `allocation`, `thrustLinearization`, `voltageCompensation`.

**Supervisor:** `actuatorHealth`, `faultDecision`, `liftMargin`, `flightPolicy` (and `thermalModel`, for a motor's temperature from its ESC's current).

The simulator only calls these by name through `run(key, …)`. To change a default, edit the function in `js/laws.js`.

Edits made in the Computers tab:
- take effect on the next time step, mid-flight;
- are test-called with sample inputs before they're accepted, so syntax errors and wrong return shapes are rejected with a message;
- are switched off automatically if they throw or return something unusable during flight, and the default takes over;
- are saved in your browser's local storage. **Copy edited formulas** gives you text to paste back into `js/laws.js`.

`rotorWrench` and `jointRotation` are shared by the plant, the controller's effectiveness matrix and the envelope check. The controller evaluates `rotorWrench` at T = 1 N, so it assumes the law is linear in T.

## Flight code on the drone: the step runner

The flight formulas (estimation, control, allocation, navigation, learning, supervision) don't have to be ported to C by hand. The simulator compiles them into a list of steps for a small runner written in C. The same runner is built three ways:
- for the ESP32, as an ESP-IDF component (`runner/`);
- for a PC, where the tests run;
- as WebAssembly, inside each simulated board (with the flight core and the navigation around it).

A formula edited in the Computers tab therefore flies in the simulator exactly as the drone would run it, and **Download the program** gives the file to send it.

### What a program is

- **Steps:** each step is an opcode and its operands. Operands are addresses in one array of 32-bit floats, the arena, fixed when the program is built, so a step looks nothing up by name.
- **Built-in math:** the heavy parts are native in the runner: 3×3 matrices, quaternions, the bounded least-squares allocation, and arithmetic over whole lists. The formulas are the steps between them.
- **The compiler** (`js/rn-compile.js`) takes a subset of JavaScript: numbers, arrays and small records, the math helpers, `if`, `for` loops with a bound it can work out, `.map`, `.reduce`, `.slice` and `.push`. It uses each formula's signature (`js/rn-sigs.js`) for the types of its inputs.
- **Lists of actuator inputs** have room for `RN_IN` = 24. A drone that gains parts in flight uses the spare places, and every loop's worst case is known.
- **Memory:** a formula's memory (the `st` argument) becomes named fields in the arena.
- **Size:** each board loads only its tasks' formulas. The ESP32's built-in program (flight core and navigation, 13 formulas) is 14 KB of steps and 24 KB of working memory; the Pi's (navigation, learning, supervisor) is 173 KB of steps and 370 KB of working memory, most of it the throw's fit.

Each flight formula's card in the Computers tab shows its compiled steps and how many run per call.

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

Nothing that flies changes before step 4. The Computers tab shows where an edit is and keeps a log.

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

Memory on that board: the built-in program's arena (65 KB) and one slot for programs sent over the link (arena 65 KB, steps in IRAM, 40 KB receive buffer) fit, with 54 KB left; a third slot doesn't, so the board runs in two-slot mode. At 921600 baud (the firmware's default), a 40 KB program arrives in about 0.5 s (4 s at 115200) and is checked and self-tested in 33 ms.

### Checks

`runner/test.sh` builds the runner and runs:
- **`tools/check_formulas.js`:** every flight formula compiled and run on the JavaScript runner and on the C runner, against about 180 calls recorded in simulated flights. They match within 32-bit float rounding. For the allocation, the forces and torques it makes must match, since several motor splits can be equally good.
- **`runner/test_rnhost.c`:** the drone's loading steps, covering a good edit, one that gives NaN in the background, one that traps after taking over, a corrupted image, and one whose formula takes other inputs.
- **`runner/test_link.c`:** the framing, including frames made by the Pi's Python.

## The flight controller firmware

`runner/fc/` is the drone's flight code around the formulas: the flight core (`fc_core.c`), the navigation (`nav_core.c`), the learning (`learn_core.c`) and the health supervisor (`super_core.c`). `runner/fc/esp32/` is the firmware for an ESP32 that runs the flight core, and `runner/pi/dfb_pi.c` the program for the Pi that runs the other three. The simulator flies exactly this code (see Flight computers): the IMU, compass, barometer and battery readings go in, and the throttles and servo angles it returns drive the simulated ESCs and servos.

**Guided commands.** Besides the sticks, the flight core takes a guided command: the world acceleration wanted and the heading. The navigation sends one 100 times a second; the throttle field still says whether to fly (below 0.05 the motors idle). Over the link it's a 12-float `RN_LINK_CMD` (the 7 stick floats, then guided, the acceleration and the heading). While they come, the ESP32 sends `RN_LINK_NAV` 100 times a second (attitude, rates, accelerometer, barometer height) and its full telemetry only twice a second. If they stop for 0.5 s, the usual failsafe: it levels and lands.

**The learning's and the supervisor's frames.** While the Pi asks for it (`RN_LINK_WANT`, twice a second), the ESP32 also sends `RN_LINK_LTEL`: attitude, the averaged accelerometer and gyro, battery, height, each motor's command and each servo's command and believed angle, and in open loop (a throw) every 1 ms step since the last frame. 200 times a second at 921600 baud, 100 at 460800, 50 slower. The Pi answers with test moves (`RN_LINK_EXC`, which lapse 0.1 s after the last one), the model to fly on (`RN_LINK_MODEL`) and the supervisor's settings (`RN_LINK_SET`: its mode, the lean, acceleration and speed limits, and each motor's and servo's state). The link runs at 921600 baud by default (the `baud` setting; boards set up with firmware v2 keep 115200 until it's changed with `fly.py PORT set baud=921600`, `save`, `reboot`: 115200 is too slow for the learning).

**Telemetry and the radio's channels.** The board with the radio asks the other for its telemetry items (`RN_LINK_WANT` bit 2, twice a second) and gets them as `RN_LINK_TLM` frames. When the radio is on the ESP32 and the navigation on the Pi, the ESP32 passes what the receiver got (channels, link statistics, the ground station's last command, with its age) to the Pi as `RN_LINK_RC` 50 times a second, and the Pi's navigation flies on it (a command over a second old isn't acted on, so a board that restarts doesn't replay an old go-to); in angle mode the ESP32 flies on the sticks itself.

**The compass** goes into the attitude estimator when there is one, so the heading doesn't drift; GPS navigation needs it. (The ESP32 firmware has no compass driver yet: on the drone, the heading drifts until it does.)

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
- **The learning and the supervisor** run on the Pi (`dfb_pi`, below). The flight core only takes their frames: it adds the test moves to what it flies (or, for a throw, sets the motors itself in open loop), flies on the model it's sent, and applies the supervisor's limits and its motor and servo states. Without a Pi, it flies the exported table.

### Safety

- **At power-on** every ESC gets its minimum pulse.
- **Arming** needs all of these: the arm switch seen off since the last disarm (so nothing re-arms by itself), an airframe loaded, a working output for each of its motors and servos, a gyro, a settled attitude, less than 15° of tilt, the throttle stick at the bottom, and, when a battery sense wire is set, a reading that fits the pack.
- **Disarmed**, the motors get the minimum pulse and the servos go back to their set angles (a helicopter's swashplate is levelled while its rotor runs down: left tilted on the ground, the spinning disc can roll it over). The motor test spins one motor, at most at 30%, for 3 s from when it starts (it stops sooner if commands stop); another test needs the test switched off first.
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

**ExpressLRS receiver (optional):** set with `crsf=RX,TX`: the GPIO its TX goes to (an input-only pin, 34–39, works), then the one its RX goes to (`crsf=-1` for none). By default every output pin has a motor or servo, so free one first: on a quad, `motors=25,26,27,14` frees 32, 33, 4 and 13, then `crsf=35,33`. `elrs=250,4` must match the packet rate and telemetry ratio set on the radio, so the telemetry fits. Bind the receiver and set it to CRSF output as usual. Wiring saved by older firmware is kept, with no receiver.

**Wiring settings:** the wiring is kept in flash. Change it with `fly.py PORT set …`, then `save` and `reboot`.

**Tasks**

| Core | Task | What it does |
|---|---|---|
| Core 1 | Control loop (1 kHz) | Runs the flight code |
| Core 0 | Sensor task | Reads the IMU every step and the barometer at 25 Hz |
| Core 0 | Link task | The Pi's commands, programs, airframe, settings and the learning's and supervisor's frames; telemetry at 20 Hz, and LTEL at up to 200 Hz while asked |
| Core 0 | Radio task | With a receiver: its channels (sticks when flying in angle mode, otherwise passed to the Pi), the telemetry store and the CRSF telemetry, 200 times a second |

### The Pi's program: `runner/pi/dfb_pi.c`

The same navigation, learning and supervisor code the simulator runs on its Pi board, with the step runner and the Pi's built-in program (`rn_builtin_pi.c`: the formulas of those three tasks, made with `node tools/export_program.js --tasks nav,learn,super --c runner/pi/rn_builtin_pi.c`), plus what a Pi needs around it:
- **The link** to the ESP32 (`/dev/serial0` at 921600 baud by default; `--link`, `--baud`): it reads `RN_LINK_NAV`, runs a navigation step on each one, and answers with a guided command. With the learning or the supervisor it also asks for `RN_LINK_LTEL` and runs them on every frame, answering with their frames (see Guided commands above). The supervisor's settings reach the navigation too (it flies home or lands) and the learning (it rescales what it learned).
- **A GPS** on its own serial port (`--gps /dev/ttyUSB0`): NMEA `GGA` and `RMC`, as the NEO-6M sends at 9600 baud; sentences with a missing or wrong checksum are ignored. Positions are metres north and west of the first fix. If the Pi stops commanding in flight (its navigation fails, or it restarts), it goes silent rather than sending a disarm, and the ESP32's failsafe lands the drone; restarted while the drone flies, it waits for the radio's channels or a text command before commanding again. A missed IMU sample or two doesn't unsettle the attitude (only 50 ms without one does).
- **The pilot's commands**, as lines of text on its input or over UDP (port 14560):
  - flying: `arm`, `disarm`, `takeoff [height]`, `land`, `goto X Y Z`, `move VX VY VZ`, `heading DEG`, `hold`, `home`, `status`. It takes off only once its position estimate has settled; home is where it took off;
  - learning: `calibrate` (while hovering) and `stop`, `learned` or `description` (which model to fly on), `keep on` / `keep off` (in-flight learning), `throw` (arm it in the hand, held level, then throw it upward: it catches itself and holds where it did), `learning` (where it is);
  - `health`: the supervisor's mode, why, the lift margin and its latest events;
  - cargo (with `--latch`): `latch N open`, `latch N close`, `latch N toggle`, `latch all open`, `latches`; `pickup X Y Z [N]` (fly the hook onto a thing whose top is at X Y Z from home and close latch N; `--hook DX,DY,DZ`, where the hook is from the hub, 0,0,-0.06 by default). A PICKUP from the radio closes its latches too.
- **Latches** (optional): `--latch pwm0,gpio17`, up to 8: `pwmN` a servo on hardware PWM channel N (`/sys/class/pwm/pwmchip0`; GPIO 18 or 12 for channel 0, 19 or 13 for 1, with `dtoverlay=pwm-2chan` in `config.txt`), 50 Hz, 1000 µs closed and 2000 µs open (`--latch-us 1000,2000`); `gpioN` an on/off line (`/dev/gpiochip0`), high for closed, for the driver of a solenoid or an electromagnet; `dry` drives nothing, to try the commands. They start closed. The radio's LATCH commands work them too, and their state goes down the radio. The Pi has no load-switch input yet, so its latches report closed or open only.
- **A radio** (optional): the ExpressLRS receiver on a serial port of the Pi's own (`--crsf /dev/ttyAMA1 --elrs 250,4`) instead of on the ESP32. Its channels then fly the drone through the navigation (they take over from the text commands while the link is up, and if it's lost it flies home), and the telemetry, the flight core's items included, goes down it. Without `--crsf`, when the ESP32 has the receiver, the Pi sends it the navigation's, learning's and supervisor's items and flies on the channels the ESP32 passes on.
- **Its files** from the simulator: Computers tab → the Pi's board → **Export**: the navigation config (`.dnc`), and with the learning or the supervisor the airframe (`.dfa`) and the Pi config (`.dlc`). `--no-learning` or `--no-supervisor` leaves one out. This Pi has no health sensor drivers yet (motor temperatures, ESC currents): the supervisor works from the flight core's data stream.

Build and run it on the Pi: `sh runner/pi/build.sh`, then
`./runner/pi/dfb_pi --nav drone.dnc --airframe drone.dfa --pi drone.dlc --gps /dev/ttyUSB0`.

It's tested end to end on a PC: `runner/pi/test_dfb_pi.c` puts a fake ESP32 (the real flight core flying a simple plant, sending LTEL 200 times a second while asked) and a fake GPS (NMEA at 5 Hz) behind two pseudo-terminals. It starts the real `dfb_pi` on them and types `arm`, `takeoff 1.5`, `calibrate`, `health` and `goto 2 1 2`. It checks the calibration runs over the link (about 13 s of tests, the height kept within 1.3–1.85 m; the plant is the description 4% stronger, so it rightly keeps flying on the description), that the supervisor answers, and that the drone gets there; then it sends the radio's channels as the ESP32 would (arm, fly, and full forward on the right stick for 2 s), checks the stick moved the drone and that telemetry items come back, and stops the channels: the drone must fly home and land by itself. `runner/fc/test_nav.c` tests the navigation with the flight core directly, through a delaying link:
- take-off, hold and goto with GPS and barometer;
- holding against wind and following a moving target;
- link loss, landing in the failsafe;
- optical flow alone indoors;
- the tilt-rotor quad.

### The Pi side for manual flying: `runner/pi/fly.py`

| Command | What it does |
|---|---|
| `python3 fly.py PORT status` | Show what it's doing and its wiring |
| `python3 fly.py PORT airframe my-drone.dfa` | Send the airframe; it's kept in flash |
| `python3 fly.py PORT watch` | Show telemetry and messages |
| `python3 fly.py PORT test 1 0.1` | Spin motor 1 at 10% for 2 s (**props off**) |
| `python3 fly.py PORT fly --gamepad` | Fly with a gamepad (`pip install pygame`), or with the keyboard for tethered tests |
| `python3 fly.py PORT program formulas.rnp` | Send edited formulas; they reload in flight as before |

- **Commands** go out 50 times a second while `fly.py` runs. If it stops, the drone goes to its failsafe.
- **While a program is arriving** (under a second at 921600 baud), the firmware keeps flying on the last command, only as long as a frame that size takes and never out of a failsafe. In `fly`, start with `--program FILE.rnp` and press P to send it; don't run a second `fly.py` on the same port (it opens the port exclusively, and without the DTR/RTS toggle that resets most ESP32 boards).
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
- **`runner/fc/test_cargo.c`** checks the cargo task: commands (one latch, all, toggle, and what's refused), the travel time, the load switch's verdicts (released, stuck, caught, empty), the LATCH command through CRSF as the ground sends it and the receiver reads it (taken once, not when stale, passed on to another board), the cargo telemetry item, and the pickup: its steps on a drone that follows its target (never below the spot, one request to close the latch, back up), giving up when it can't hold still, a spot out of the box refused, and a PICKUP through CRSF that the sticks stop. `test_ground.c` checks the command module's `latch` command.
- **`runner/fc/test_tlm.c`** checks CRSF (round trips, the CRC), the telemetry scheduler within its budget, the GPS's precision, the battery frame put together from two tasks, items packed on one board and sent from another, and the radio pilot: angle-mode sticks, the go-to, the speed along the heading, the link lost (it flies home) and back (it holds).
- **`runner/ground/test_ground.c`** checks the command module: buttons easing the sticks in, a real stick's deadband and expo, latching switches (and switch states that don't latch), commands one at a time (a waiting go-to replaced by the next), the module's own frames not counted as telemetry, the drone's telemetry decoded (GPS to 1e-7°, the battery, the flight mode, items unscaled, messages), the alerts and their 2 s hold, and the raw sticks when the formulas can't answer. **`test_ground_e2e.c`** runs the real `dfb_ground` and the real `dfb_pi` (with a fake ESP32 and GPS) on a PC with pseudo-terminals, playing the two radio modules between them, and flies them with a script over UDP: arm, take off, go to, the stick, a calibration started from the radio, telemetry back, and the radio going quiet (the drone flies home and lands; the command module raises its alarm).
- **`runner/test.sh`** runs these with the rest, `test_nav.c` and the Pi's end-to-end test.

**Known limit: drift after fast flight.** The default attitude estimator trusts the accelerometer's "up", and while the drone speeds up or slows down that "up" is off. Two effects follow:
- after a fast pass the drone levels out slowly and drifts on for a while;
- a hard lean reads a few degrees less than it is.

The firmware tracks what it believes exactly. The belief is the formula's, and you can improve it in the Computers tab (a lower accelerometer gain, or a model of rotor drag) and send it to the drone.

## What it models

The physics isn't simplified for speed; every step (2 kHz) does the full version. The controller runs at 1 kHz.

- **Articulated rigid bodies** (`js/multibody.js`, `rigidBody`). The frame is a free-floating body and every servo joint adds another: the servo's output and everything rigidly on it, including rods, further joints and what they carry. The equations of motion for the frame's 6 degrees of freedom plus every joint angle are built each step with the recursive Newton–Euler algorithm (Featherstone) and solved together. So a swinging arm pushes the frame the other way, a load drags its servo, the whole thing conserves momentum in free fall (checked: angular momentum to 0.2%, linear to 0.01% while an arm swings ±60°), and a sensor on an arm feels the arm's own acceleration.
- **Motors** (`motorDynamics`). Each motor is a brushless motor, ESC and prop sized from its card: throttle sets the voltage, back-EMF and winding resistance set the current, current sets the torque, the prop's inertia sets how fast it spins up, and thrust and drag torque grow with speed squared. From that come a throttle curve that bends upward, spin-up faster than spin-down, a current-limited start from standstill, thrust that fades with the battery, the frame feeling each motor's torque while it speeds up (the spin-up reaction), and the gyroscopic torque of a spinning prop when the frame or its servo turns it. Health scales the thrust. Hover hints (rpm and amps) show on each motor's bar. The prop is sized by momentum theory from its card: its rated thrust and diameter set how fast it spins at full throttle (thrust = Ct·ρ·n²·D⁴, Ct = 0.10, a typical multirotor prop), and its figure of merit sets the power it takes (the ideal power to push the air down, T^1.5/√(2ρA), over the figure of merit). That fixes its drag torque ratio κ = √Ct·D/(2π·FM·√(π/2)): about 0.011 m for a 7″ prop at 0.6. κ is also the yaw the props make, so the controller's description and the airframe check use the same value. The motor card shows it, with the full-thrust rpm and the air power at half thrust. A bigger prop spins slower and draws less current for the same thrust; a better prop (higher figure of merit) less again. The Quad X layout (0.9 kg, 4S 1.3 Ah) hovers at about 8,400 rpm on 7.9 A, around 8 minutes to 20% charge.
- **Pullers and pushers** (a motor's **Prop** setting). A motor is mounted along its shaft, which points from the motor to the prop (the shaft tilt and azimuth). A puller (tractor) makes thrust along the shaft, toward the prop, and blows air back past the motor. A pusher's prop is pitched the other way: its thrust points back along the shaft, toward the motor, and it blows air away past the prop. Spin is always seen facing the prop, so the drag torque and gyroscopic torque follow the prop's real rotation either way; a pusher's spin about its thrust axis is the reverse of its card. A quad of pushers hung under the arms (shafts down) flies the same as an ordinary quad. In edit mode the selected motor shows its thrust arrow. Two faint arcs with arrowheads on each prop disc show which way it turns, seen facing the prop.
- **Collective-pitch rotors** (a motor's **Blade pitch** setting), as on a helicopter. The ESC's governor holds the rotor at a set speed and the blade pitch sets the thrust, so thrust follows the command after the pitch servo's 30 ms lag. The rotor never speeds up or slows down, so there's no spin-up twist, but more pitch means more drag torque, which twists the frame. The blades flap: the disc follows the mast a few milliseconds behind (flapping time constant 16/(γΩ), Lock number γ ≈ 4), so turning the airframe doesn't meet the rotor's gyroscopic stiffness the way a rigid prop does. With rigid blades, a helicopter-sized rotor couples the axes so strongly the controller can't hold it.
- **Servo joints** (`servoTorque`). A hobby servo is a geared motor with a position loop: full stall torque when stopped, none at its no-load speed, a 3° proportional band, the gearbox's reflected inertia, a command delay, and hard stops just past its travel. It moves by the multibody dynamics, so a light arm snaps to its target, a heavy one lags and overshoots, and thrust or weight on an arm holds it slightly off target (checked: a 100 g weight on a 15 cm arm sags it 0.5°).
- **Rigid mass:** box, sphere or vertical cylinder, riding on whichever body it's attached to; in the air a prism (drag) or, as a box, a wing (lift and drag), turned by its incidence.
- **Mass on cable:** a point mass on a tension-only spring-damper cable that can swing, go slack and touch the ground.
- **Sensors:** each rides on its body. The IMU has scale errors and axis misalignment as well as noise, bias and drift; the magnetometer has soft-iron distortion as well as hard iron and motor interference.
- Masses and cables can be hidden from the controller ("Controller knows" off), so it must absorb them with integral action.

## Control and allocation

- Position PID produces a desired force. Far from the target, the position error asks for a velocity instead, capped at the speed limit sideways, 3 m/s up and 1.5 m/s down (sinking faster, the drone falls into its own downwash and can't brake in time), and the integral only builds up within a metre of the target (on the way to a far one it would wind up and carry the drone past it: a 4.5 m descent used to end over a metre too low, now 0.3 m). Attitude uses geometric control on SO(3) with integral action.
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

Each board in the Computers tab shows roughly what its tasks cost it: the formulas' costs (below) at each task's rate, plus the code around them, against the board's speed; and on a microcontroller, the flight program's memory. The table below was measured with the simulator's old controller, which included the in-flight learning; that now belongs to the Learning task on the Pi.

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
- A wing is one panel with one angle of attack, taken at its middle (a long wing's tips don't see different air), with the rotors' wash where its middle is. Wings and blunt parts don't shade each other or the rotors, and a prop's disc can sit inside a wing without touching it. Nothing on board knows about wings: there is no airspeed sensor and no control through them.
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
- Cargo: a loose thing is a rigid body with corner springs: it doesn't push the drone (a falling one passes through it), and the drone doesn't push one at rest. Loose things stack, but only on ones already at rest. Picking up snaps the thing under the hook in the drone's axes. The ESP32 flight firmware has no latch outputs yet: in the simulator any board can run the Cargo task, but on hardware only `dfb_pi --latch` drives latches so far.
- The runner has run on an ESP32 bench (its self-tests, timings, and loading, rejecting and falling back from programs sent over USB), not in a drone yet.
- In 32-bit floats the allocation can pick a different split between motors where several are equally good; the forces and torques it makes match.
