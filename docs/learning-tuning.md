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
tasks. Custom/vectoring controllers need a matching identification model. The workflow is simulator-only;
it does not claim real-flight validation or automatically upload firmware. Native core attitude test frames
(EXC mode 3, at most 4°) and navigation test targets (at most 0.2 m) expire after 100 ms without refresh and
yield to supervision/failsafe. The browser performs analysis and recommendation; this is not an onboard PID
autotuner in the Pi firmware. Existing Tune plots remain simplified predictions.

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
