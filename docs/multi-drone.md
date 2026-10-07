# Multiple drones in one world

Choose **Add airframe** to add a layout or saved library design. Select a drone with the top dropdown or a short click/tap on its visible body. Orbit drags keep their existing behavior. Both sidebars, pilot controls, charts, edit HUD, formulas, hardware drafts, health and AI chats follow selection. Instance names belong to the world; saved-design names belong to the design library. The remove button keeps library designs.

Play/pause, simulation speed, Edit mode, terrain and atmospheric settings are shared. Reset resets the selected drone. Changing terrain resets every drone. Other drones hold their own targets when selection changes; held manual inputs are released. Sound follows the selected drone. Importing a file/shared link or opening a design from the sidebar replaces the selected design; the header layout/library menu adds another instance.

## Ownership boundary

The existing feature modules use drone-local bindings. `js/drone-context.js` explicitly captures and installs them, and `js/fleet.js` owns the records and scopes synchronous engine calls. This avoids converting the physics, controller, radio, installer and editor into a new framework simultaneously.

| Owner | State |
| --- | --- |
| Each drone | Configuration, component IDs, control laws and runner arenas, board/ground WASM instances, physics/multibody/joints, motors, sensors/estimator, battery/health, learning, cargo, radios/packet state, target/pilot, history/trail, undo/design metadata, formula/hardware drafts, chats/triggers and render groups |
| Shared world | Terrain, atmosphere/wind, simulation clock/rate, renderer/scene/camera, view preferences and frame profiler |
| Shared services | Immutable compiled WASM modules, saved-design library/account connection, AI provider credentials/request budget and one real-device session |

`withDrone(drone, fn)` installs a drone's **engine** state and restores the previous state in `finally`. It only accepts synchronous functions. It does not switch graphics or rebuild DOM. UI changes run against the selected drone; renderer operations additionally use `droneGraphicsSwitch`. `fleetScene` restores selected graphics and engine state after drawing every craft. Add any new mutable drone-local binding to both capture/install functions.

Board and formula WASM modules compile once; memories, runners, board instances and asynchronous instance pools remain separate. Delayed formula staging and loader callbacks carry the owning RN/board-runtime object through `runDroneCallback`, including during saved-world startup. Removed owners' callbacks are ignored. UI listeners skip background physics.

Switching rebuilds controls that close over model objects, rather than leaving sliders bound to the previous drone. It preserves uncommitted formula/hardware drafts and releases held controls. Selection/add/remove waits for an active AI turn, pending file/account design mutation or real-device connection to finish. Installer/edit dialogs and active drags also retain their selected context. AI tools operate on the selected drone; automatic triggers currently run only for that drone. Arbitrary enabled `run_js` retains its existing full-page privileges.

## No selection and audio

Selection may be `null`, saved explicitly in the world record. The dropdown's World option, World settings button and an empty viewport click deselect. Drone sidebars, flight HUD values and pilot/edit/reset/share/live controls hide; target inputs remain in the selected drone's Control pane. A collapsible right overlay contains shared terrain/environment settings. Collapse preference is stored separately. Adding an airframe or opening a shared code from world view adds and selects a new drone.

`fleet.active` and `fleet.visual` retain a valid internal owner when selection is empty. Physics, async owner callbacks and rendering continue; user keyboard/undo/AI actions cannot operate on that retained owner. The camera keeps its orbit centre rather than following the retained drone. Hover uses solid visible craft geometry, a pointer label and an outline; click recognition tracks the entire drag/pinch/cancel gesture and defers to editor handles. Ctrl/Command drag and two-finger pan/pinch are camera-only gestures; they do not pick drones or edit parts. Panning retains a follow/edit offset until Show → Center view, selecting another craft or selecting an editor part. Camera control regressions are in tools/test_camera_controls.cjs.

Audio has one context/compressor and distance/pan mixing. With selection it emits that craft's voices; with none it emits all craft. Voice keys include drone and part IDs; crash/prop/latch/arming/battery event state is per craft. Selection changes preserve matching voices, fade departing ones and seed event history to avoid replaying old strikes or immediate low-battery reminders. New motor oscillators start at zero gain and the actual rotor frequency, so selecting a craft adds no chirp. Motor/servo pitch and wash playback follow the shared speed setting without recreating voices or changing their volume/pan. One-off effects use the speed at their trigger for pitch, duration and spacing; battery reminders follow each flight clock, including pause/reset. Real-device telemetry stays at 1x. The existing limit of eight motors and six servos applies per craft; cost grows with audible fleet size. Pause, edit, hidden page and mute retain their existing behavior.

## Shared stepping and performance

Every fleet tick advances all drones by the existing 0.5 ms physics step and then resolves inter-drone contacts. The world clock advances once per tick. Selecting, adding or individually resetting a drone does not reset this clock. The World reset icon or holding R for 650 ms resets every flight and the shared clock in one synchronous pass, preserving selection, designs, targets, environment and pause/Edit state. A tap of R resets only the selected flight on key release; focus/selection changes cancel an unfinished hold. Individual flight clocks retain the existing reset behavior.

The existing adaptive **11 ms physics budget per rendered frame** bounds the entire fleet. When work grows, all drones advance fewer equal ticks and simulated time slows together. Rendering and selected-sidebar work are additional costs; the budget is not a universal FPS guarantee. Collision broad phase checks craft radii; component geometry and detailed contacts are evaluated only for nearby pairs. Broad-phase pair count is O(drones²), suitable for small fleets; large swarms should add a spatial index and consider physics workers before raising the budget.

Run `node tools/benchmark_fleet.cjs --hardware-gpu` for production-loop 1/2/4/8/16-quad and two-tilt-quad measurements. `BENCH_OUTPUT` saves raw JSON. Results and environment are recorded in [simulation performance](simulation-performance.md).

## Collisions

Nearby drones use oriented boxes for the frame, connecting arms, motors, joints, masses, rods and sensors. Sphere/cylinder parts use conservative boxes. Box contacts use separating axes, equal/opposite impulses with whole-body mass/inertia, restitution/friction and mass-weighted overlap correction. Hub velocity is adjusted so impulses preserve center-of-mass linear momentum even for off-center loads.

Spinning prop disks intersect component box edges/interiors and sampled prop rims; strikes damage the corresponding prop. Hard body impacts mark both drones crashed. One deepest body contact per pair bounds cost. Joint response holds articulation during the impulse rather than solving a coupled contact Jacobian; existing articulated flight dynamics resume on the next tick. Contacts are discrete at 0.5 ms, not swept high-speed collision detection.

Inter-drone aerodynamic wakes, shared RF interference, loose-cargo exchange and flexible cables connecting different drones are follow-ups. Existing per-drone airflow, radios and cargo remain independent while using the shared terrain/environment.

## Persistence and checks

`liftlab-world-v1` stores drone identities, selection, designs/formulas, targets, radio/learning/launch settings and trigger/chat associations plus shared terrain/environment. It writes on edits, add/remove/selection, target changes and page hide. Reload restores each design with fresh independent runtimes; it restarts flights rather than snapshotting live WASM memory. Legacy single-design storage remains supported and seeds the first world. Individual imports/exports retain the existing format.

`node tools/test_multi_drone.cjs` checks isolated edits, undo, slider bindings, drafts/chats, boards/runners, delayed formula staging, held controls, synchronous exception restoration, shared flight stepping, reset, viewport/dropdown/add selection, collisions/center-of-mass momentum/prop damage, removal, persistence and phone themes. Existing flight/radio, Markdown/design-HUD and mock BLE suites cover compatibility. `node tools/test_world_view.cjs` covers nullable selection, world/target ownership, hover/click/gesture preservation, audio voices/events, reload/add/shared-code behavior and responsive themes. Browser tests do not establish real multi-drone hardware operation.
