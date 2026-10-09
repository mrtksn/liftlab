# World animation editor

Select **World → World editor**, select an imported world object, then **Add animation** (or **Edit animation**). This editor uses the same viewport, properties panel, slider/number fields, solid-shape controls and camera framing as object import. On phones the controls sit below the uncovered preview. **Frame motion** fits the full motion; orbit, pan and projection controls remain available.

The object's normal placement is the **Closed** keyframe. Configure the **Open** keyframe relative to it:

- **Hinge:** choose local X, Y or Z, set the hinge point in local metres or pick it on the object's surface, and set the open angle. The orange marker/line shows the hinge.
- **Slide:** set an XYZ offset in local metres. The blue line shows the straight path from Closed to Open.

Local axes follow the object's placement yaw; coordinates are measured from its centered base and already use the placed size. Set travel time and constant speed or smooth start/stop. The timeline scrubs between both poses. **Play / pause**, **Closed** and **Open** preview the motion while drone physics remains paused. **Done** returns the object to Closed. Changes save with the world; Remove animation retains its normal placement and collider.

## Activation

- **Button:** an Open / Close control appears over the simulation, including while a drone is selected. It changes the target pose; Run advances the movement. Changing direction halfway through reverses from the current pose.
- **Loop:** wait Closed, travel to Open, wait Open, travel back, repeat. Both waits are configurable.
- **Drone proximity:** the green sphere is a fixed detection zone relative to the object's Closed placement. Any drone hub inside opens it. It closes only after the final drone leaves and the configurable empty-zone delay passes. A 10 cm exit margin prevents boundary chatter.

Playback advances once per shared physics step, independent of frame rate or the number of drones. Pause freezes runtime motion. Reset and reload return to Closed. World button actions are global recording events; reset/replay restarts motion state and preserves recorded motion definitions. Runtime collider updates do not invalidate recordings. As with existing replay, placed objects must remain available in the world.

## Moving solids

Both **Follows model** and **Filled boxes** use the same rigid transform as graphics. Triangle surfaces and voxel boxes are retained and transformed; playback does not regenerate mesh geometry or voxelize each frame. Contact normals, rays, surfaces below, radio obstruction and blue indicators follow the current pose. The physics response uses relative velocity against the moving surface, including hinge rotation. Nearby resting cargo wakes when a moving solid reaches it.

**When blocked** selects Stop until clear, Reverse, or Continue and push. Stop/reverse predict obstruction using articulated drone contact points, cable payloads, loose cargo and moving surface samples against scenery/ground. Pre-existing overlaps can move out rather than locking the object. Proximity reversal temporarily overrides the trigger so it can retreat before trying again. A blocked button-controlled object shows its status on its control.

This is prescribed world motion, equivalent to an external actuator holding the configured trajectory. Drone parts never receive animation transforms: their motion remains under the physics/joint/actuator systems. An animated gate's moving leaf should be a separate library object/world copy from its stationary frame.

## Limits and validation

This first editor has two keyframes, rigid hinge/straight-slide motion and the three activation modes. Multi-keyframe curves, object grouping, scripts, key bindings and imported skeletal/morph/glTF animation clips are follow-up features. Motion settings belong to each world copy; duplicate copies keep independent definitions, and saved/portable maps include them.

Contact and obstruction remain sampled, not continuous collision detection or a motor/force-limited hinge simulation. Extreme speeds, very thin features and complex interlocking meshes can miss contact between samples. Motion vectors/hinge points are bounded to ±100 m, angle to ±360°, travel time to 0.1–120 s and waits/delay to 0–120 s. Portable validation rejects invalid/nonfinite definitions before applying a map.

`tools/test_world_animation.cjs` covers actual hinge picking, timeline previews, shared mesh/box transforms and normals, rays/radio/height queries, moving-surface velocity and force response, loops and fleet timing, global button recording/replay, multi-drone proximity and close delay, obstruction/reversal/ground, independent duplicates, map validation, reload and phone framing. Existing world/map, object-library, camera and payload suites cover compatibility.

## Animation sounds

Open **Sound cues** in the animation editor. Set a cue independently for **Motion starts**, **While moving**, **Motion stops**, **Reached Open**, **Reached Closed** or **Blocked**. Start and stop cues also fire when movement reverses; blockage fires once when the object becomes obstructed, rather than repeating each simulation step.

Choose **Generated** for a procedural motor whirr, metal clunk, click, beep or whoosh. Choose **Upload sound…** to use an audio file; uploaded sounds can be selected again by other cues and objects. Volume and pitch use the same slider/number controls as the rest of the editor. One-shot cues also have a sound-length control. File pitch changes playback speed, as well as pitch. **While moving** repeats the whole uploaded clip or keeps the generated sound running until motion stops.

**Listen** enables the speaker and auditions at most five seconds without advancing physics. **Play / pause** previews the movement with its sounds. Timeline scrubbing is silent. Leaving the editor cancels its audio. Preview audio has its own bus so paused drone motors stay silent.

World sounds share the speaker button/**M** mute setting and the existing camera-based stereo position and distance attenuation. Simulation speed changes continuous sound pitch/playback rate. Runtime pause, mute, reset, removal or map replacement cancel playing and pending sounds; continuing a paused movement resumes its loop at the simulated elapsed position without replaying old event cues. World sounds are independent of which drone is selected. They do not run in a real-board view.

Original audio bytes are retained in browser object storage and included in saved and exported maps, including disabled cues retaining a selected file. Duplicated world copies own independent cue settings. Files are decoded and validated before assignment or portable map import: maximum **16 MB and 120 seconds per file**, within the map's existing 128 MB limit. WAV, MP3 and Ogg are useful choices; support for other codecs depends on the browser. Missing files must be replaced before saving/exporting. Source sounds are kept when copies are removed so saved maps and other cues can still reuse them.

`tools/test_world_animation_audio.cjs` exercises real Web Audio synthesis and measured PCM output, uploaded WAV decoding, start/stop/endpoint/block events, moving-loop rates, pause/mute/reset and delayed-decoder cancellation, independent copies, failed/quota-limited uploads, sustained file volume, byte-exact portable files, saved-map audio restoration, cold-browser import, reload, migration of the existing object library and desktop/phone UI.
