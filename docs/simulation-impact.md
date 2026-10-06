# Simulation improvement impact — 2026-10-06

## Basis

Priorities assume smooth interactive airframe editing, ordinary hover/control experiments and more useful motor/prop/battery comparisons. Ratings describe expected decision impact, not measured improvement against real flight. Existing CPU measurements and implementation-cost estimates are in [simulation-performance.md](simulation-performance.md).

Prioritize corrections that can change a hover verdict, component choice or endurance estimate. Additional physical detail needs evidence that it changes the intended experiment. A displayed quantity is useful without improving the underlying physics, and these benefits should be described separately.

## Ranked assessment

| Priority | Change | Expected impact / affected use case | CPU cost | Implementation effort | Recommendation |
| --- | --- | --- | --- | --- | --- |
| First | Battery mass consistency | High when comparing capacity, payload and endurance; also changes balance/inertia | Negligible | Small–medium | Link battery settings to an explicitly identified mass part, preserving manual mass overrides and detachable batteries |
| First | Actual motor limits in feasibility/allocation | High for capped/brushed outputs; can change hover verdict and control headroom | Negligible–low | Medium | Derive available thrust from actual duty, voltage and motor model; align checks, allocation and delivered commands |
| First | Useful readouts | High explanatory value; exposes disk loading, inferred KV, tip speed, electrical watts and hover margin | Negligible at existing UI cadence | Small | Show the model's assumptions and quantities; do not describe this as a physics accuracy improvement |
| First, enabling work | Baseline full-frame profiling, cached parameters and performance checks | High for the user's smoothness requirement; no direct fidelity gain | Should reduce overhead | Small–medium | Cache static coefficients with correct invalidation; benchmark before considering a solver rewrite |
| Next | Fixed motor KV/resistance/current plus calibrated prop thrust/torque curves | Very high for real component comparisons; separates motor choice from prop choice | Low | Large, including data acquisition | Build one coherent propulsion feature; derive achievable thrust consistently instead of retaining unrelated arbitrary ratings |
| Next | Electrical load accounting | Medium generally; high for servo-heavy or electronics-heavy craft | Low | Medium | Include device power, servo load and ESC/BEC conversion losses; verify power accounting |
| Next | Airflow-to-prop-load coupling | Medium in hover, high for climb/descent, wing-assisted flight and rotor interaction | Low–moderate with bounded formulas | Medium–large | Reuse existing inflow for thrust and torque; compare equilibrium, transient and step-size behavior |
| Conditional | Separate brushed-motor dynamics | High for small brushed craft and matching physical MOSFET builds; low for brushless presets | Low | Medium | Implement as a motor profile using the electrical model; independent of the output-driver choice |
| Conditional | Air density | High when exploring hot/high-altitude conditions; low for the current fixed environment | Negligible | Small–medium | Add a consistent environment density after fixed hardware parameters exist |
| Conditional | Tip Mach, compressibility and prop RPM limits | Low for the stock quad, high for unusually fast/small/high-voltage configurations | Low | Small for readouts/limits, medium for calibrated losses | Add visibility first; apply model-specific loss/limit data when available |
| Conditional | Measured drag coefficients and wing polars | Low in hover, high for cruise speed and wing layouts | Low | Medium, including calibration | Prioritize if wing-assisted flight/cruise becomes a main use case |
| Conditional | Battery/thermal calibration | Medium for endurance and burst duty; high for overheating/failure experiments | Low | Medium–large, including measurements | Fix energy and mass accounting first; calibrated coefficients matter more than extra states |
| Conditional | Wake overlap and improved descent response | High for coaxial/closely spaced rotors or descent-recovery experiments; lower for ordinary separated rotors | Moderate | Large | Use a bounded sample model and scenario benchmarks before detailed wake solvers |
| Conditional | Environment-aware sensor errors | High for GPS-denied flight, obstacle-adjacent navigation, estimator/learning robustness | Low–moderate for scalar models | Medium | Add obstruction/dropout and pressure disturbances at existing sensor rates; validate each failure scenario |
| Later | Contact/friction improvements | Medium for landing, cargo and ground movement; low for flight away from surfaces | Low–moderate | Medium | Correct force/friction behavior before adding denser collision geometry |
| Later / optional | Flexibility, structural resonance, detailed wakes and image-based sensing | Potentially high for narrow vibration/vision/aerodynamics studies; uncertain general benefit | Moderate–high | Large–very large | Keep optional; require a concrete experiment, validation reference and frame budget |

## Concrete examples from the current model

### Battery sizing can reverse an apparent benefit

Illustration: take a 0.9 kg craft containing a 0.18 kg battery. Doubling capacity and assuming the battery also doubles in mass makes the craft 1.08 kg. At unchanged rotor area/efficiency, the simulator's ideal hover-power law scales as weight^1.5:

- Hover power multiplier: `(1.08 / 0.9)^1.5 = 1.315`, about 31% more.
- Ideal endurance multiplier: `2 / 1.315 = 1.52`, about 52% more, versus doubling if the mass is left unchanged.

This is an illustrative calculation, not a measured prediction for the preset: electrical losses, reserve, pack resistance, flight conditions and actual battery mass matter. It establishes that the missing weight penalty can be consequential without any expensive new solver.

### Duty ceilings can change the hover verdict

At the current model's nominal 16 V, the fixed-pitch steady speed fraction is `x = -2 + sqrt(4 + 5u)`. At a 60% duty ceiling:

- `x = 0.646`; thrust fraction `x² = 0.417`.
- A 6 N motor therefore supplies about 2.50 N before health/airflow corrections, not 3.6 N and not 6 N.

`hardwareMotorThrottle()` applies that ceiling to actuation; `envelopeCalc()` still uses rated maximum thrust times health. Correcting the ceiling must include the nonlinear motor model, and meaningful voltage handling, rather than multiplying the thrust rating directly by the duty percentage. Allocation/exported model behavior must remain consistent across browser and native code.

### Tip-speed modeling is not the first accuracy investment for stock hover

The default quad motor's approximately 0.172 m prop, 6 N rating, Ct=0.10 and density 1.225 kg/m³ imply roughly 14,200 RPM and 128 m/s tip speed at rated thrust, approximately Mach 0.37 if sound speed is taken as 343 m/s. Hover is slower still. High-Mach loss modeling has limited impact on this particular operating range; the readout can cheaply reveal which custom designs need it.

This does not establish a permitted RPM for an actual prop. Structural RPM ratings and aerodynamic tip Mach are different constraints. The current synthesized motor parameters can also change when a prop is changed; fixing hardware parameters is a prerequisite for interpreting prop swaps as real component experiments.

## Dependencies and validation

- Preserve a generic mode for existing designs. A calibrated hardware profile needs fixed motor data, prop curves with voltage/RPM coverage and compatible controller model/export updates. More editable numbers alone do not establish accuracy.
- Static thrust/torque data primarily establish static operation. Forward-flight/climb/descent predictions need appropriate inflow data or a validated aerodynamic extension.
- Battery mass changes need undo/save/export coverage and correct CoG/inertia for movable/detachable battery parts; identify the physical battery explicitly and avoid double counting.
- Duty-aware checks need a near-hover-threshold regression where full duty can hover and a lowered ceiling cannot. Exercise mixed driver outputs, restored settings and native/browser consistency.
- Propulsion validation needs reference points for thrust, RPM, current, voltage and transient response; include operating points not used to fit the coefficients. Set error targets after reference-data quality is known.
- Electrical/thermal checks should test power balance, charge integration and heating from mean I², including burst loads. Changing a configured failure threshold should not silently change calibrated cooling.
- Density changes should leave actual motor electrical parameters fixed and affect every aerodynamic path consistently. Avoid an implicit motor resize when environment conditions change.
- Compare each implementation's total-frame p95/p99 and simulated-time progress against baseline, including slower hardware, active learning, contacts and accelerated playback. The proposed 1 ms additional CPU budget is a target, not a measured guarantee.

## Recommended sequence

1. Battery mass consistency, output-aware feasibility, readouts and frame-performance instrumentation.
2. Fixed motor profiles plus calibrated prop curves, followed by complete electrical accounting and bounded airflow/load coupling.
3. Add density, brushed physics, wing/sensor/thermal refinements according to the next concrete experiment.
4. Keep structural flexibility, detailed unsteady wakes and image-processing simulation out of the default workload until measured benefit justifies their cost.

This assessment preceded implementation. The low through moderate CPU-cost models are now implemented as described in [simulation-physics.md](simulation-physics.md), with measured costs in [simulation-performance.md](simulation-performance.md). Expected accuracy gains remain hypotheses until tested against suitable external measurements; the battery and duty examples are calculations from the original equations.
