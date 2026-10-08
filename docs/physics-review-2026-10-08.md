# Crash and cable physics review — 2026-10-08

Scope: the reported crashed aircraft perched awkwardly on a roof with its hanging mass seemingly not pulling it down. This is a source review and browser reproduction, not a physics fix or a full certification of the simulator.

## Findings

- **High priority: the cable ignores obstacles.** `js/sim.js` computes tension along the straight attachment-to-ball vector; only the ball receives terrain contacts. There is no rope segment contact, wrapping or routing around a roof edge. In a fixed-anchor fixture, a 2 m cable crossed a solid slab at z=2.8–3.2 m while its ball remained below the slab at z=1.99938 m and tension remained 2.4525 N. Crashed roof fixtures also settled with the ball below the roof and the cable intersecting it. The resulting force direction can be wrong near buildings even though equal/opposite tension is applied correctly.
- **Contact damping is tied to tick count.** The ball's ground branch multiplies horizontal velocity by 0.995 per step; each building contact multiplies its entire velocity by 0.995. With a slack rope, a 0.25 kg ball starting at 1 m/s retained 0.3558 / 0.1306 / 0.0176 m/s after 0.2 s at steps of 1 / 0.5 / 0.25 ms respectively. Distance travelled was 0.1250 / 0.0856 / 0.0487 m. This is strongly dependent on integration rate and contact count, rather than a material friction law. Current production steps are fixed at 0.5 ms; playback speed changes the number of steps, not this step size.
- **Slack rendering suggests tension that may not exist.** `js/view3d.js` always draws two endpoints joined by a straight line, irrespective of rest length or tension. A bag resting on a surface can look suspended on a shortened, taut rope. The view has no cable-tension force arrow or magnitude to distinguish this from actual support.

## What is working

- `crash()` zeros motor commands and records the crash. The animation loop, aircraft dynamics, payload gravity and cable reaction continue while the simulation is running. A live browser check advanced simulation time and both aircraft and ball fell after the crash flag was set.
- A crashed, unsupported aircraft with a 1.06 kg suspended load received 10.3986 N of cable tension at an attachment 32 mm off-centre, with a clear angular acceleration. The code applies the opposite force to the ball and applies the aircraft reaction at the attachment, including its moment.
- Roof contact can support the aircraft and its payload. It is not inherently wrong for a crashed craft to stay on a roof. In a controlled ledge comparison (20 degree initial pitch, x=-0.05 m from the edge, 1.83 m rope), the 0.05 kg load stayed perched while the 1.06 kg load tipped the craft off. A craft farther onto the roof stayed supported under the heavy load.
- The first hard-impact reproduction (30 degree pitch, 4 m/s downward impact, heavy load) fell off the ledge normally. The exact awkward pose in the user's screenshot has not been reproduced, so a general aircraft-contact trapping bug is **not confirmed**.

## Proposed repair order and regression checklist

1. Add cable/terrain contact with a consistent routed or segmented cable constraint. Preserve slack behavior and transmit reactions at the actual contact/attachment directions. A cosmetic bend alone would leave the physics wrong. Cover a roof crossing, an edge drape, an unobstructed swing, slack pickup and release; retain the existing known-load and learning/autotune guards.
2. Replace the payload's per-tick velocity multipliers with time- and load-consistent contact friction. Check sliding, a ball hitting a wall obliquely, multiple contacts, rest without jitter, and convergence at 1 / 0.5 / 0.25 ms.
3. Render slack cable length consistently and expose tension/support diagnostics after a crash. Check a ground-resting bag, a suspended bag and an aircraft supported by a roof. Keep an explicit distinction between a visual slack curve and any physically simulated cable segments.
4. Capture/reproduce the exact roof pose before changing general aircraft contacts. Test supported rest versus an unstable overhang and verify that removing support or increasing an off-centre load causes motion rather than pinning.

## Validation boundary

Temporary headless Chrome fixtures exercised the actual loaded simulator's `dynamics`, terrain contacts, cable state, view geometry and the running animation clock. Fixtures used the Cargo preset, zero wind/turbulence and stopped motors; direct dynamics comparisons isolated the load/contact behavior. The normal flight-physics reference test (`node tools/test_flight_physics.js`) also passed. These checks do not establish real-flight fidelity, arbitrary crash-contact correctness or physical rope material properties. Production physics and rendering are unchanged by this review.
