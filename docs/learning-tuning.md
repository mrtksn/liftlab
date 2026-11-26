# Airframe learning and measured tuning

Airframe identification answers what each actuator does. PID tuning answers how quickly the controller can
ask it to respond, with enough damping and room for delay/noise. The gains use acceleration units, with mass
and inertia normalized out; Betaflight PID numbers are not interchangeable with them.

## Flight models

The estimator (`B`) and accepted flight model (`flyB`) are separate. Calibration flies on the accepted model,
tests motors/servos and effectiveness, freezes a candidate, and compares it with the baseline on a fresh final
test segment. A better candidate must explain force and rotation sufficiently well before it is accepted.
Stopping a calibration keeps the accepted model. Calibration preserves the adaptation preference.

Adapt from ordinary flight is off by default. When enabled, it observes without adding motor dither. Candidate
updates are limited to 5% of each motor's force/rotation column groups, then held fixed for three seconds of
fresh observations. Insufficient signal, correlated inputs, saturation, large motion and poor prediction prevent
acceptance. An accepted update has two seconds of probation against its predecessor; worsened prediction or
unsafe motion or missing informative probation data restores the predecessor. The health supervisor's diagnosed actuator corrections remain separate.
With little independent excitation (especially coupled servo bases), observation can legitimately produce no
update. This is conservative adaptation rather than a promise of continuous improvement.

The RLS transient terms participate in the entire covariance matrix, including cross covariance with thrust
terms. Numerical regression checks compare this with an independent information-matrix implementation and
the JavaScript and native compiled runners.

## Scale thrust test

Tune → Motor thrust (`js/thrust-test.js`, `js/thrust-test-ui.js`) measures what each motor really lifts, on the
ground, so a drone that can't hover yet still gets a model of its motors. Props on, the drone taped to a kitchen
scale; the motors-off reading is the zero, then each motor alone at 20–100% throttle (the highest is a setting).
A motor test sends the throttle straight to the ESC, so a reading at throttle u and pack voltage V is fitted as
T = Tmax·((1 − k)·e + k·e²) with e = u·V/Vref (least squares; readings under 1 g, an ESC's dead band, are left out).
A fit that misses its readings by more than 8% of Tmax, or bends outside −0.3…1.3, is flagged.

Apply writes `thrustTest` on each motor (readings, Tmax at Vref, bend, the fitted coefficient). A fixed brushless or
brushed motor gets the prop thrust coefficient (`propPhysics.ct`) that makes `FlightPhysics.equilibrium` give the
readings; a generic motor gets `tmax`. A motor with a measured prop table keeps its table. While the card still
holds what was fitted (`motorMeasured`), `curveHat` tells the flight controller the measured bend and the simulator
drops that motor's hidden thrust spread. Editing the card's coefficient, thrust or table, or the pack's cell count,
retires the record. The drone needs Upload design for the new airframe.

On the drone the page sends `motors 0 0.5 0 0` every 0.25 s; dfb_pi turns it into RN_LINK_BENCH_TEST frames for
0.5 s after each, renewals of the same line say nothing back. The test also goes while the radio's channels come,
as long as their arm switch is off (LiftLab connected sends its channels all the time); arming takes the sticks
back and ends it. The ESP32's motor test is disarmed-only and lapses on its own command timeout. The simulator holds
the drone still on its scale (`simBenchHold`), runs the same flight-core motor test (`simBenchSet`), and reads the
scale from the rotors' still-air thrust once the throttle has run half a simulated second. The ground tests can't
see the props' extra inflow in flight, ground effect or the pack's sag under a full hover load.

`node tools/test_thrust_test.js` checks the fit, the physical coefficient and the refusals;
`test_ground_angle_e2e` (in `tools/test_radio_e2e.sh`) checks the motor test through dfb_pi with the radio's
channels coming.

## Gains from the motors

Tune → Gains from the motors (`js/tune-suggest.js`, `js/tune-suggest-ui.js`) rates and suggests attitude gains with
no flight data. Each axis is the measured-autotune plant `strength·e^(−s·delay)/(s²(1+s·lag))` (`atPlant`,
`atMargins`). lag: the slowest motor's `motorSpinUp` near hover (flight-model.js; also the Tune step prediction's
lag now, instead of the card's `tau`); delay: 12 ms assumed (a loop step, the ESC pulse, the MPU-6050's 98 Hz
filter); strength: 0.85–1.15 when every motor is `motorMeasured`, else 0.7–1.5.

Limits: phase margin ≥ 30° at strength 1 with peak ≤ 1.35 and sensitivity ≤ 2.5, ≥ 20° at the range's ends, the
integral inside `atMargins`' bound. Gains within them are kept (never sped up). Otherwise the quickest response
(0.025 Hz steps, damping 0.8–1.1, integral at most the current one and 0.8 /s, lowered to 0.5/0.3/0.15/0 where
needed) inside them. Roll and pitch also have a floor, `firmHz`: kR ≥ α/15°, α the push of one motor lifting 6%
(2% measured) more than its share at its lever over the inertia; below it the suggestion is the firm one with
the most margin, marked as a compromise. The rating of each option includes the tilt that push causes.

The simulator checks (toy-drone preset): lag predicted 24 ms, step-measured 29 ms; with rotor inertia ×3.3,
80 ms against 85 ms. On those slow motors the defaults (rated 9°/5° at the assumed 12 ms) fly at 1.7° RMS after
take-off, the suggestion (0.88 Hz, 30°/24°) at 5.4° RMS with an 18° peak; 0.4–0.6 Hz with the integral tips over
at lift-off (which is what the floor is for). The simulator's own delay is about 2 ms and its motors match their
cards, so it can't show the failure the margins guard against; a real drone that overshoots and diverges has
less margin than its model. `node tools/test_tune_suggest.js` checks the ratings, the suggestions and the floor.

## Rope spin test

Tune → Rope hang → Spin test (`runner/fc/spin_core.c`, `js/spin-test-ui.js`). The drone hangs on a rope, free to turn
in yaw; disarmed, the flight core's motor test runs every motor at `base`, then steps each motor by `±step`
signed by its yaw column in the airframe (`cols[0][5]`), in the doublet +T, −2T, +2T, −2T, +2T, −2T, +2T, −T
(T = `half`, 0.25 s by default), between 1.5 s of settling and 0.9 s after.

The samples are the flight core's LTEL frames (time, state, gyro, each motor's averaged throttle), timed by the
flight core's own clock; frames older than the last are dropped. The differential throttle
D = mean of sign·(u − base) and the yaw rate ω are fitted as
ω̇ = p·D(t − d) + q·x − k·θ − c·ω + b, x = D(t − d) through 1/(1 + τs), every signal averaged over the
telemetry windows and smoothed alike. For each (d, τ) on a grid (d 0–80 ms, τ 5–300 ms, coarse then fine) the
coefficients are least squares on the yaw rate itself (the model integrated over time: ω = p·∫a + q·∫x − k·∫θ −
c·θ + b·t + ω₀, so the gyro's noise isn't differentiated, and b takes a steady torque from motors that don't quite
cancel at the base throttle); the best residual wins, and the fit is graded on what's left after that steady
torque, the rope and the air. On the first real drone the derivative form explained 4% and this one 90%. g = p + q is the yaw acceleration per unit of
differential throttle and r = p/g the kick: a voltage-driven brushless motor's reaction torque jumps with its
current when the throttle steps, then settles to the drag at the new speed, with the same pole τ as its thrust
(the simulator's toy motors: r ≈ 6–9). r is kept ≥ 0 (a negative kick would stand in for delay). The fit is
kept when it explains at least 60% of ω̇ and τ, d aren't at the grid's edge. Apply stores
`cfg.motorTiming = { lag: τ, delay: d + 1 ms }` (the flight core's own step, which the test can't see) in the
design (undo, files, library); Gains from the motors and `tuneMotorLag` use it until it's forgotten.

dfb_pi: `spintest BASE STEP [HALF]` starts it (angle mode, an airframe, disarmed, the learning or supervisor
running so the ESP32 sends LTEL; the radio's channels may come while their arm switch is off), sending
RN_LINK_BENCH_TEST every 20 ms; `spintest` says how it goes and, at the end,
`... ; result DELAY_MS LAG_MS GAIN R2 OK PEAK KICK`; `spintest stop`, any motor or stick text command, or the
radio's pilot arming stop it. The simulator runs the same C on the learning's board (`simSpinStart`), holds the
drone as on a rope (`simBenchHold('rope')`: yaw from the rotors' torques alone, a weak twist and drag) and lets go
when it ends. `runner/fc/test_spin.c` (CI and `runner/test.sh`) fits made-up ropes with delays of 3–25 ms,
spin-ups of 28–140 ms and kicks of 0–6 through a link with 3–25 ms of varying, in-order delay.

## Rope lift test

Tune → Rope hang → Lift test (`spin_core.c` `lift_*`, `js/spin-test-ui.js`). Disarmed, every motor at the same
throttle: 1.5 s at `base` (the accelerometer's mean: its reading and direction at rest), then rising at `rate` per
second. The accelerometer along that direction, less its reading at rest, is integrated into a vertical speed that
leaks away over 0.7 s; lift-off is that speed passing 0.35 m/s. A springy rope's bounce averages out (±6% of g at
1.4 Hz on the first real drone gave ±0.13 m/s), a lift keeps climbing. The barometer is not used: the props' wash
moved it ±20–40 cm with the drone still. At lift-off the throttle drops to `base` for 0.5 s (a brake) and, with
`step` 0 (the default), it ends. Rising stops if that speed passes 1.2 m/s.

Lift-off comes about 7% over the hover throttle (the excess the leaky speed needs, for a moment), steady across
fast and slow (190 ms) motors and a bouncing rope in `test_spin.c`; LiftLab takes hover = lift-off / 1.07 and the
strength = weight / what the flight controller believes the motors give there (`tmax`, `curveHat`, the pack's
voltage). Apply refuses a strength outside 0.6–1.8 (the first real run read 3.39 off a barometer jump), fits each
card (a fixed motor's prop coefficient by bisection, a generic one's `tmax`) to hold its share of the weight at
hover, and keeps its old numbers on `thrustTest.prev`; **Forget the rope fit** puts them back.

With `step` > 0 it then holds 85% of lift-off (after the brake) and gives pulses at lift-off, + step, + 2 step:
up to 0.6 s each, cut once the accelerometer says it gained 0.7 m/s, read over their last quarter second, each
waiting for 0.3 s of calm first, braking after. They climb more and, with slow motors, read a little low.

dfb_pi: `lifttest BASE TOP [RATE STEP]` (0.04 a second, step 0), `lifttest`, `lifttest stop`; result line
`... ; result LIFT_U VOLTS NP [U F V ...]`. Each test's samples go to `rope-lift.csv` / `rope-spin.csv` beside the
design. The simulator's rope (`simBenchHold('rope')`) holds the drone level and lets it rise: the rotors' force
against its weight, caught where it hung; the sensors read that motion.

## Simulator autotune workflow

1. Take off, release the controls and run an airframe calibration under Control → Learning.
2. Open Tune → Measured autotune and Measure attitude. Bounded sine sweeps test roll, pitch and yaw separately.
3. Review the proposed response, damping and integral settings. Effective gain/lag/delay are estimated from
   synchronized commanded-angular-acceleration and gyro telemetry, so the actuator fit operates inside the
   closed loop instead of confusing navigation feedback with actuator lag. Rotor-transient lead is included.
   The fit must have adequate signal and match the measured response. Recommendations require predicted phase margin,
   limited resonant response and sensitivity, and a conservative integral bound.
4. Apply & verify stages provisional constants through the existing board program loader and repeats the
   actual tests. Trial constants are runtime state, outside the saved design. Failure or Stop restores the
   original program constants; board fallback also cancels verification. Passing verification saves one undoable tuning edit.
5. Measure position is available after attitude verification. It tests a small horizontal target offset using
   the navigation board's velocity estimate and the accepted attitude response model. Position recommendations remain several times slower than
   attitude and are also verified before saving. Position gains are shared across axes; this initial horizontal
   test does not establish independent vertical-axis performance.

Model adaptation stays suspended through measurement/review/verification. Stop or completion restores the
previous preference. The ordinary flight target is not edited. Pilot input, reset, formula/tuning changes,
loss of navigation/flight state, telemetry gaps and excessive motion/load stop the test. Recommendations and
attitude verification belong to the accepted flight model; changing it requires fresh measurement. Each drone owns its
measurement and provisional state, so switching selection does not apply a result to another drone.

Initial support is fixed-motor aircraft in tilt mode with the standard control formulas, Learning and Navigation
tasks. Custom/vectoring controllers need a matching identification model. The browser workflow controls
the simulator; the Pi + ESP32 path now has a native coordinator and command interface documented in
[hardware-tuning.md](hardware-tuning.md). Neither path claims real-flight validation or automatically uploads firmware. Native core attitude test frames
(EXC mode 3, at most 4°) and navigation test targets (at most 0.2 m) expire after 100 ms without refresh and
yield to supervision/failsafe. The browser performs simulator analysis; `autotune_core.c` performs hardware analysis on the Pi,
using matching command/gyro samples and leased native gain transactions. Existing Tune plots remain simplified predictions.

## Regression commands

- `node tools/test_learning.js`: reference covariance, compiled parity, measured-response fitting and poor-signal rejection.
- `node tools/test_learning_browser.cjs`: real simulated-flight workflow, provisional settings, verification and undo.
- Native gates: compile `tools/test_learning_native.c` with `runner/fc/fc_core.c`, `runner/fc/nav_core.c`,
  `runner/rn_host.c`, `runner/rn.c`, and `runner/pi/rn_builtin_pi.c` (include `runner` and `runner/fc`, link `-lm`).
- `node tools/check_formulas.js`, existing flight, tuning and multi-drone browser regressions.

`tools/rebaseline_learning.js` updates only RLS expected output/state for the corrected covariance recurrence;
it preserves recorded inputs and other formula fixtures. Run the independent numerical test before accepting
that fixture update.

## Agent integration and cable loads

The in-app agent reads/edits PID gains with `get_tuning`/`set_tuning`, and uses `autotune` for status,
attitude/position measurement, apply-and-verify and Stop. These use the same staging, rollback and undo
paths as Tune. `get_learning` distinguishes accepted/candidate models and passive adaptation from tuning.
`get_runtime` inspects board runtimes, custom programs/apps and source/interface chunks; `get_bus` reads
topic layouts, values and freshness. Native/Python apps are not simulated; their managers handle editing.

A cable under tension is an external disturbance outside the current identification/autotune models.
The simulator pauses passive learning, stops calibration/throw identification while retaining the accepted
model, and rejects/stops measured autotune until the cable is unloaded. Supervisor actuator-effectiveness
inference also waits under cable tension; explicit RPM/current/temperature fault checks and lift checks
continue. Cable swing identification and inference with a changing external load remain future work.
