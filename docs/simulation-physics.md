# Flight physics profiles

The simulator now supports the inexpensive and bounded models from the [impact assessment](simulation-impact.md). They improve internal consistency and component experiments; measured accuracy against real flight remains unverified.

## Design readouts

In **Edit** mode, the viewport replaces flight time, position and speed with design mass, rotor count/summed disk area, disk loading, upward static thrust-to-weight, rigid center of gravity, nominal battery energy and estimated maximum RPM/tip Mach. **Show → Readouts** controls their visibility. Leaving Edit restores the flight HUD.

These values use the saved components at joint rest angles, including unknown masses and cable payloads in total weight. They ignore dropped cargo, simulated failures, battery depletion and live joint poses. Disk loading is weight divided by summed disk area, without correcting overlaps or rotor tilt. Thrust-to-weight sums upward thrust at rest with configured wiring/duty limits, current air density and full-pack resting voltage; it omits voltage sag and is not a hover feasibility verdict. Battery energy uses 3.7 V per LiPo cell and the configured Ah capacity. Center of gravity describes only rigid parts, in millimeters from the hub. RPM/tip Mach is a static motor estimate, not a manufacturer speed limit or efficiency target.

## Motor and propeller

Open a motor’s **Physical model** section. Generic mode preserves the existing inferred motor: changing its thrust/prop settings also changes its inferred KV, resistance and inertia. Fixed brushless or brushed mode freezes those traits, so a prop swap changes the load on the same motor. Initial fixed values are estimates, not specifications of an identified product.

The electrical model uses `Ke = 60 / (2π KV)`, equivalent winding resistance, current limit and combined motor/prop inertia. Brushed mode adds brush voltage drop and mechanical friction. The existing editable motor formula remains the plant entry point. Semi-implicit damping keeps stiff, small-inertia inputs bounded at the normal 0.5 ms step. This is an averaged equivalent circuit, not a phase-resolved ESC or brush-commutation simulation. Prop inertia must be updated when a real replacement changes it.

The controller’s reference rating stays at **16 V and 1.225 kg/m³**, matching its existing file/API convention. A fixed profile derives thrust at this reference point; actual RPM/current/thrust depend on pack voltage, density, temperature and airflow. Generic prop coefficients use Ct and figure of merit to estimate torque; changing the prop does not identify its pitch, blade shape or Reynolds dependence.

Static measured prop data accepts 2–64 CSV rows:

```text
RPM, thrust N, torque Nm
6000, 1.0, 0.01
12000, 4.0, 0.04
```

These rows are an illustrative format, not measurements. Enter the test density and source/conditions. RPM must increase; thrust must be positive and nondecreasing; torque must be positive. Curves interpolate inside coverage, extrapolate quadratically outside, and warn above the highest measured RPM. They belong to the saved prop radius; changing radius clears the curve. Importing a curve from generic mode freezes an estimated motor profile, which still needs real motor data. Collective pitch requires a pitch-dependent map and does not accept this static fixed-pitch import.

Tip Mach includes rotational speed, estimated axial flow and advancing-blade crossflow. A bounded generic loss above Mach 0.7 reduces thrust and increases torque. Measured static curves already contain their rotational losses, so those are not applied twice; generic corrections extend additional inflow/extrapolation. Sonic-tip RPM is a diagnostic, not a structural or recommended operating limit. An entered manufacturer RPM limit produces an overspeed warning, not a speed governor. There is no assumed efficiency optimum at Mach 0.6 or 60% throttle.

## Battery, power and temperature

Battery mass parts retain their initial mass as an anchor at the design’s saved capacity and cell count. Capacity/cell edits scale their mass and rebuild true/believed mass, CoG and inertia. Editing a battery part’s mass selects manual weight; its Physical model checkbox can re-enable scaling with a new anchor. Scaling assumes unchanged pack construction/specific energy. Pack resistance remains an independent setting. Multiple flagged masses scale together; designers must enter their intended combined weight. A design without a battery part retains the existing always-powered behavior and an estimated thermal mass.

Battery input includes duty-weighted motor winding current, ESC efficiency/idle draw, configured avionics power, extra per-part device draw, and servo mechanical work/load losses through the BEC. Latches draw additional power while moving. Defaults are generic estimates. Braking dissipates rotor energy; regeneration into the battery is not modeled. Avionics is a configured total, not a measured sum for the selected board types.

Motor and battery temperatures use I²R and lumped heat capacity/conductance. Explicit thermal parameters override estimates. Generic motor cooling uses a fixed reference rise independent of the failure threshold; raising a failure threshold no longer changes cooling. Battery thermal mass follows its actual battery-part weight. Existing temperature effects, wear, supervisor sensors and failures remain active. ESC/servo temperatures and detailed convection are not modeled.

## Airflow, aerodynamic data and sensors

Air pressure and temperature set density and sound speed without resizing motor hardware. Rotor loads/wakes, wings, blunt-part drag, body/payload damping and downwash forces use that density.

The existing inflow/ground-effect/descent model also produces a bounded torque correction. The following motor step uses it, coupling aerodynamic thrust changes to RPM, current and heat with one 0.5 ms delay. Its generic 20% profile / 80% induced-power split is an estimate, not a blade-element calculation. Where a wake can intersect another disk, five fixed samples estimate disk-average inflow; separated disks keep one query. Environment settings can select one-point sampling. The sampling approximates partial/coaxial overlap; it is not wake CFD or validated vortex-ring recovery.

Rigid masses accept drag coefficients. Wing parts and wing-shaped frames accept 2–64 angle-degrees/Cl/Cd rows, with source notes. Interpolation uses the supplied coefficients, blending to the generic stall model over 10° outside coverage. Center of pressure remains generic; Reynolds/Mach-dependent polars and compressibility are not inferred.

Optional GPS/RTK obstruction uses five sky rays, cached for at least 0.1 simulated seconds. Poor visibility increases noise or suppresses fixes; stale fixes expire through the existing sensor path. Barometer wash adds a bounded generic pressure/altitude bias at its existing sampling rate. These effects default off and do not represent satellite geometry, multipath, antenna shielding or measured pressure-port behavior.

Contact sliding force is bounded by normal load. Contacts with zero normal force no longer apply tangential drag. Existing sphere/spring collision geometry and rigid multibody dynamics remain approximations; flexible structures, blade-element/CFD solvers and camera image processing were deferred.

## Allocation, diagnostics and smooth playback

The hover envelope and thrust/weight display use driver ceilings, actual voltage/density and motor temperature. The browser flight core receives ceilings through its existing settings API in reference-voltage thrust coordinates, preserving supervisor limits, motor removals and servo fault settings. Unchanged caps are not repeatedly submitted. The core retains its existing throttle-curve prior/learned effectiveness; these bounds are not a new physically exact allocation map. Collective feasibility accounts for governor speed, motor load and available duty.

**This allocator integration is browser-side.** Native firmware, radio framing and schemas are unchanged; allocation-aware physical MOSFET ceilings remain a native follow-up. Exported descriptions use the new reference rating/torque/power; they do not embed the browser plant profiles or replace hardware calibration. Changing the plant in flight still requires a reset to give the core a new airframe description, as before.

Airframe → **Flight physics** displays battery amps/power, motor electrical/shaft power, device draw, thrust per battery watt, weight/disk area, density, RPM/Mach and warnings. Thrust per watt is a lifting metric, not thermodynamic propulsive efficiency; shaft power includes acceleration work. Summed disk area does not correct overlap or tilt. **Frame timing** reports rolling 240-frame FPS, actual simulation rate and CPU/frame-interval percentiles; CPU timers do not measure asynchronous GPU execution.

The animation loop estimates an **11 ms physics CPU budget per frame** and caps catch-up steps to avoid a feedback loop between slow rendering and more physics work. The fixed 0.5 ms integration and board/sensor rates do not change. Requested fast playback can run slower than its label on a busy device; Frame timing shows the actual rate. This is an estimated budget, not a hard deadline or universal 60 FPS guarantee. See [measured performance](simulation-performance.md).

## Validation

`node tools/test_flight_physics.js` checks reference equations, fixed-hardware prop changes, curves, density, mass scaling, thermal independence, electrical power/charge balance, stiff-step convergence, tip-loss accounting, friction and polar forces. `node tools/test_flight_browser.cjs` checks UI/profile persistence, undo/redo, saved-file validation, browser ceilings/supervisor preservation, GPS/barometer behavior, overlap sampling and generic/brushed/table/collective flight. Existing formula/WASM and hardware integration regressions also pass. These are consistency/integration checks, not independent hardware validation.

`node tools/benchmark_flight.cjs --hardware-gpu` measures production physics batches and full frames. `--baseline` serves tracked files from HEAD. `PLAYWRIGHT_PATH`, `CHROME_PATH` and `BENCH_OUTPUT` customize the local harness. External Three.js must be reachable. Larger/contact-heavy designs, open code/learning panels, sustained learning/calibration, slower devices and hardware measurements need separate coverage.
