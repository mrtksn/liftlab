# Simulation fidelity and performance cost — 2026-10-06

Most identified fidelity improvements can use cached parameters, constant-size lookup tables and inexpensive arithmetic. Detailed unsteady wakes, flexible-body dynamics and image-based sensing require separate performance evaluation. The original assessment below describes the pre-change baseline; the implementation and measured results are recorded at the end and in [simulation-physics.md](simulation-physics.md).

Decision impact, implementation effort and recommended order are assessed in [simulation-impact.md](simulation-impact.md).

## Measured baseline

Isolated local Headless Chrome 154, 1440×1000 viewport, default preset boards and parkour terrain. Software WebGL was enabled for browser compatibility, but the timed sections exclude rendering, scene updates and periodic UI work. Analytics/fonts were blocked. Measurements are specific to this host and browser, not a mobile or total-FPS guarantee.

Each preset warmed for 6,000 physics steps. Then 180 batches of 34 production `physStep()` calls plus one `pilotStep()` were timed. Physics uses a 0.5 ms step; 34 steps represent approximately one 60 FPS frame at normal speed. Batches ran consecutively, yielding every 30 batches; this is a CPU microbenchmark rather than a sustained real-time frame trace.

| Preset | Motors / joints | Median batch | p95 batch |
| --- | --- | --- | --- |
| Quad X | 4 / 0 | 1.4 ms | 1.8 ms |
| Hexacopter | 6 / 0 | 1.7 ms | 2.0 ms |
| Tilt-rotor quad | 4 / 4 | 3.4 ms | 3.9 ms |
| Quad with wing | 4 / 0 | 1.5 ms | 2.0 ms |

All sampled presets remained uncrashed with flight state 1; no page errors occurred. Timing wrappers were installed only in the isolated page and restored afterward. A separate instrumented 2,000-step sample showed `mbSolve()` taking about 78 ms for the tilt quad, versus 16 ms for Quad X; control took about 43 ms versus 20 ms. Instrumentation adds overhead, and nested timings overlap: do not sum them or treat them as uninstrumented percentages.

At 60 FPS the entire frame has 16.67 ms. At 120 FPS it has 8.33 ms, with approximately half as many physics steps. GPU/rendering, UI refreshes, garbage collection, learning/refinement and other scenarios consume additional time. The current loop runs physics on the rendering thread, clamps elapsed time to 50 ms and caps work at 200 steps per frame; these limits do not guarantee smooth rendering or real-time simulation when overloaded. Higher simulation-speed settings multiply the physics work.

## Cost of proposed corrections

Costs below are architectural estimates; none of the proposed physics replacements was implemented or benchmarked.

| Correction | Efficient implementation | Expected incremental CPU cost | Main tradeoff |
| --- | --- | --- | --- |
| Battery mass consistency | Explicitly link a battery mass part to battery settings; recompute inertia when edited | Negligible during flight | Preserve manual mass overrides and cargo attachment/detachment |
| Duty-aware hover/headroom | Apply actual output ceilings when building feasibility/allocation limits | Negligible to low | Keep requested and delivered actuation consistent across browser and native code |
| Explicit motor KV, resistance, current limits | Cache fixed electrical parameters; retain the existing motor equations | Negligible to low | Migration and calibration are more work than runtime computation |
| Prop-specific thrust/torque and efficiency | Precompute curves or small tables; use constant-size interpolation per motor | Low, O(motors) | Accuracy depends on supplied data and table coverage; no blade-by-blade simulation needed |
| Tip Mach / compressibility | Compute tip speed and a smooth bounded coefficient correction per rotor | Low, O(motors) | A generic correction remains approximate; manufacturer RPM limits are separate |
| Airflow-to-load coupling | Reuse computed rotor airflow for both thrust and prop drag torque; use previous-step inflow or a bounded predictor/corrector | Low to moderate | Lagged coupling needs numerical/convergence checks; avoid unlimited global iterations |
| Servo, ESC, BEC and device consumption | Simple servo/load/loss equations; sum device electrical demand | Low, O(devices) | Preserve peak loads and conversion losses, rather than only idle watts |
| Battery/thermal calibration | Keep a small equivalent-circuit model and calibrated thermal coefficients; add one or two scalar states if useful | Low | At slower thermal rates, integrate I and I² over fast steps to retain charge and heating accurately |
| Variable air density | Cache density from environment pressure/temperature or elevation, refresh when conditions change | Negligible | Feed one consistent density into thrust, torque, wakes, wings and drag |
| Better body drag / wing polars | Replace generic coefficients with fitted curves or table interpolation | Low, O(aero parts) | Dynamic stall states add work but can remain bounded; measured geometry/data matter |
| Partial wake overlap / better descent model | Cache pair geometry where possible; use a small fixed set of disk samples and bounded rotor interactions | Moderate, approximately O(rotors² × samples) | Five samples can multiply the wake kernel cost; it does not multiply the entire simulation cost |
| Sensor/environment realism | Add scalar dropout, obstruction and pressure-disturbance models at the sensor's existing sample rate | Low to moderate | Full camera rendering and optical-flow processing are a much larger feature |
| Structural flexibility / resonance | Optional small modal spring/damper model with a fixed mode count | Moderate, model dependent | Full flexible-body meshes require many states and stiff integration; not suitable as a default without profiling |
| Better contact friction / collision behavior | Load-bounded friction per existing contact; broad-phase checks and analytic geometry where appropriate | Low for friction; geometry dependent for collision detail | Adding dense contact samples increases work at the existing 2 kHz rate |
| Detailed time-dependent wakes, blade-element solvers, CFD | Precompute offline and export compact tables when possible | High if run directly in the browser at 2 kHz | Reserve online high-fidelity solvers for an optional mode after measuring them |

## Performance boundaries and priorities

- First: battery mass, duty limits, cached motor parameters, prop tables, tip speed and consistent density. These correct important comparisons without adding new global solvers.
- Next: bounded airflow/load coupling and electrical/thermal calibration. Benchmark each change against the unchanged baseline; verify conservation, transients and simulation-step convergence as well as speed.
- Keep detailed wakes, flexible bodies and camera sensing optional until their incremental costs are measured.
- Cache static motor/thermal coefficients: current code rebuilds motor parameters twice per motor per physics step, including through `motorThermal()` (16,000 calls per simulated second for Quad X). Invalidate on edits, not simply on every step.
- Existing joint scaling deserves attention before adding flexible degrees of freedom. `mbSolve()` rebuilds a dense (6 + joints) mass matrix using repeated inverse-dynamics passes, then performs a dense solve. Work increases quadratically for matrix construction and cubically for the solve as joints increase.
- Avoid new temporary arrays and objects in hot loops. More allocations can create intermittent stalls even when median computation time is small.
- Proposed acceptance budget on this reference host: keep the combined ordinary-fidelity additions within roughly 1 ms per 34-step batch, and tilt-quad physics/control p95 within 5 ms. These are targets, not measured upgrade costs or an FPS promise. Measure total frame p95/p99, long frames, real-time simulation speed and GPU cost on desktop and a slower target device before release; establish a baseline if a target already misses 60 FPS.
- Do not reduce the fast motor/control rates without checking stability and transient accuracy. Thermal/device updates may run slower with correct aggregation; rendering can interpolate independent of the physics rate.

## Evidence and remaining checks

- `js/sim.js`: `PDT`, `physStep`, motor parameters, current accumulation and rotor airflow.
- `js/ui.js`: `boot()` animation loop and periodic UI work.
- `js/multibody.js`: `mbSolve()` matrix construction and dense solve.
- `js/health.js`: thermal coefficients and repeated motor-parameter calculations.
- Follow-up: benchmark actual implementations, larger custom airframes, active learning/calibration, contact-heavy scenes, 2×/4× simulation speed and lower-performance devices. The present hover microbenchmark does not cover these cases.

## Implemented models and hardware-rendered comparison

The final implementation caches motor/thermal/pack sizing, avoids unchanged controller SET calls, uses bounded tables/formulas, and samples overlapping wakes only where geometry warrants it. The animation loop now estimates an 11 ms physics budget per frame; it preserves the fixed integration step and slows requested playback instead of amplifying catch-up work after late frames. Model details and limitations: [simulation-physics.md](simulation-physics.md).

The same local Chrome 154 harness ran against tracked HEAD and the updated working tree with **ANGLE Metal / Apple M1 hardware rendering**, at 1440×1000. This is a short controlled local comparison, not sustained profiling or mobile coverage. Raw results: [baseline](benchmarks/flight-physics/baseline-m1.json), [updated](benchmarks/flight-physics/updated-m1.json). Run `node tools/benchmark_flight.cjs --hardware-gpu`; add `--baseline` for HEAD. Use `BENCH_OUTPUT` to save JSON.

Production 34-step batches, 180 samples per case after 6,000 warm-up steps:

| Preset | Baseline median / p95 | Updated median / p95 |
| --- | --- | --- |
| Quad X | 1.3 / 1.7 ms | 1.3 / 1.7 ms |
| Hex | 1.6 / 2.0 ms | 1.6 / 2.8 ms |
| Tilt quad | 3.2 / 3.6 ms | 3.1 / 3.5 ms |
| Wing quad | 1.4 / 1.7 ms | 1.4 / 1.6 ms |
| Quad, fixed motors + 64-row prop tables | — | 1.3 / 1.6 ms |
| Eight rotors in four overlapping pairs, five-point wake sampling | — | 2.2 / 2.6 ms |

All cases flew without crash/page errors. Ordinary cases met the proposed ≤1 ms additional batch-p95 target and the tilt-quad ≤5 ms batch-p95 target in this run. Browser timers are coarse and results vary between runs; an earlier software-rendered run had larger outliers. Synthetic prop curves exercise runtime cost; they do not establish hardware accuracy.

Actual animation frames, 120-frame windows per case, including scene submission/UI (asynchronous GPU execution is outside the CPU timer):

| Craft / requested speed | Baseline FPS / actual rate | Updated FPS / actual rate | Baseline / updated CPU p95 |
| --- | --- | --- | --- |
| Quad / 1× | 60.0 / 0.99× | 60.0 / 0.99× | 5.4 / 6.2 ms |
| Quad / 2× | 60.0 / 2.00× | 60.0 / 2.00× | 5.5 / 7.0 ms |
| Quad / 4× | 60.0 / 4.01× | 60.0 / 4.01× | 8.2 / 9.8 ms |
| Tilt quad / 1× | 60.0 / 0.99× | 60.0 / 0.99× | 6.4 / 8.2 ms |
| Tilt quad / 2× | 60.0 / 2.00× | 60.0 / 1.99× | 9.5 / 15.7 ms |
| Tilt quad / 4× | 59.5 / 3.99× | 60.0 / 2.58× | 14.8 / 13.9 ms |

Updated frame-interval p95 stayed about 16.7–16.8 ms. At 4× on the tilt quad, CPU budgeting deliberately trades simulation throughput for smooth rendering. Before budgeting, an updated run dropped to about 38 FPS / 3.12×, and after cache optimizations to about 55 FPS / 3.82×; avoiding the catch-up feedback loop is the significant performance safeguard. The budget is estimated, so isolated frames can exceed it. The stock quad still achieved requested 4×.

Software WebGL produced only roughly 13–17 FPS with long frame gaps and could not establish user-facing FPS. The hardware-rendered measurements above are the relevant local evidence. They do not guarantee 60 FPS for large/contact-heavy designs, lower-end hardware, every open panel, long runs, or sustained learning/calibration. Keep live Frame timing available and validate those targets separately before a release claim.
