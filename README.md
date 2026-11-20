# Handheld room survey

Open **Room Survey.command** on the Mac, then **Connect camera**. The local dashboard uses SSH to run the camera sender on the Pi. The default host is `pi@192.168.2.127`; the Pi needs Picamera2 and its hardware MJPEG encoder. A cold start on the original Zero can take a minute. The Pi stamps and compresses 640 × 480 frames at a target of 8 fps. The Mac saves the received JPEGs and camera-relative microsecond timestamps under `captures/room-surveys/<walk>/`. A network interruption ends the recording and schedules a final reconstruction of the received frames.

The Pi rotates the image 180° before JPEG encoding to compensate for the inverted camera mount. Preview, calibration and new recordings share that orientation.

1. Check the preview for focus and useful texture. For a measured lens model, open **Fisheye calibration** and its checkerboard on a flat display or paper. Collect at least 12 different board poses, cover the centre and edges, and click **Fit lens**. The fit applies to this lens, crop, orientation and resolution. Without it, COLMAP estimates a shared fisheye model from the walk. Rebuild a walk recorded in the same orientation after calibration if needed.
2. Carry the drone by hand with the motors off. Start level so the first registered camera gives the map an approximate up direction. Click **Start walk**, move sideways in a slow arc, keep overlapping objects visible and revisit the starting view. Avoid pointing exclusively at a blank wall or rotating on the spot.
3. For metric scale, mark A and B while the camera centre rests at each of two measured positions, then enter their separation in metres before pressing **Mark B**. A second person can click the buttons or use two fixed resting spots. The dashboard reports metres when both exact frames register together. Check a second measured distance independently. Without scale, the path and landmarks have relative units.
4. Watch the raw frame count and reconstruction stages, then **Finish walk**. The first reconstruction starts after 12 candidate views, normally around six seconds of capture. Each update is a batch estimate and can revise earlier positions. The Mac samples approximately two views per second, selecting up to 160 distributed views plus the marked frames. For longer runs it also proposes visually similar distant frames and uses available attitude, range and barometric height to propose additional matches. Each match must pass visual geometric verification. Drag the plot to orbit and scroll to zoom.
5. **Map JSON for LiftLab** exports the walk’s single map. Gaps in registered views are shown as breaks in the path. In LiftLab, select **World → World settings → Room survey → Open camera survey**. The overlay displays the trajectory and coloured landmarks. It uses right, initial forward and initial up axes. Collision and free-space estimation require additional geometry. The overlay is session-local. Raw frames and exported results remain on disk.

Use **Download walk ZIP** to export the walk selected in the dropdown, including the original JPEGs and all files in its folder. **Delete walk…** moves that folder into `.trash` under the survey output directory after confirmation. To recover it, move the folder back into the survey output directory and give it its original name.

A failed solve leaves the last successful snapshot available and reports the failure. Frame filenames in `survey.json` correspond to `timestamps.csv`. The reprojection error is a pixel fit diagnostic. Ground-truth accuracy requires independent measurements. The current prototype runs structure from motion on the Mac. The Pi reads camera and ToF data and listens on loopback UDP port 14681 for flight telemetry.

## Range measurements

The mounted TOF250 responds at 7-bit address 0x52. Start walk saves one row per JPEG in timestamps.csv, including camera-relative microseconds, the Pi monotonic time of the encoder callback, the time the range register read completed, and the raw reading. A blank range field means the read failed. New walks also save range.csv, with register reads polled independently at a target of 60 Hz, timestamped before and after each transaction on the Pi. The camera CSV pairs each encoder callback with the most recent completed read within 80 ms. Older walks sampled at JPEG output time. Read timing does not establish the sensor’s internal measurement cadence or hardware synchronization with camera exposure. Median readings were 52 at 50 cm and 104 at 100 cm. Values are kept raw for replay. The tilted sensor measures along its viewing direction. The mapper estimates metric scale from compact, depth-consistent landmark patches near the optical axis when the available ranges span multiple distances and withheld views agree. This estimate assumes a nearly parallel camera and range beam. The map uses relative scale when the range check fails.

With the survey-enabled Linux companion running, the Mac saves the ESP32 attitude quaternion, body-frame specific force and angular rate, height, vertical speed and validity in motion.csv. Times are Pi UART receipt times. Reconstruction associates observations with encoder callbacks, and can orient the map to gravity from consistent rotations about multiple axes. A held-out rotation check rejects weak fits. Raw inertial samples remain available for replay. These steps do not integrate acceleration to fill translation gaps.

## Terminal launch and regression

From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python server.py --pi pi@192.168.2.127
.venv/bin/python -m unittest discover
```

The dashboard binds to loopback on port 8779. SSH authentication must already work without an interactive password prompt. Camera diagnostics are in `captures/room-surveys/camera.log`. **Disconnect** releases the camera.

To compare the same recorded frames on the two computers, run `.venv/bin/python track_bench.py captures/room-surveys/<walk>`. This streams the first 48 downsampled grayscale frames to a temporary NumPy process over SSH. It measures the compute time of spatially distributed 7 × 7 patch matches at 160 × 120. Capture, decoding, transport, pose estimation and reconstruction are outside this timing measurement. Compare the patch counts and the 95th-percentile time with the camera interval.

## Second handheld pass

Open the indexed walk and use **Localize in this walk → Start tracking**. An estimate matches SIFT features to triangulated landmarks and solves the fisheye camera pose on the Mac. The match image and counts refer to the same sampled frame. The blue map marker and direction line appear when the solver has enough geometrically consistent matches spread across the image. Choose **Recorded frame** to scrub the original sequence, change **Minimum inliers** to compare rejection rates, and switch the map layers or top view to inspect the result. A new walk records a separate map. Use tracking on the saved map for a second-pass localization experiment.

The HCJ six-pin TOF250 probe uses the Pi Zero header I²C bus 1, with SDA on physical pin 3 (BCM2), SCL on pin 5 (BCM3), and a shared ground. Confirm the sensor variant, pin numbering and logic levels before connecting. The PDF lists 5 V communication levels. Use a bidirectional I²C level shifter when using 5 V sensor-side pull-ups. Mount the optical window beside the camera facing forward and measure their offset. **Read sensor** shows the current paired reading when streaming or probes the register when offline. Compare raw readings with measured ranges before assigning units. The probe is separate from the pose solver and occupancy mapping.

## LiftLab integration

The standalone dashboard and camera stream use SSH. The optional Pi flight telemetry tap is provided as `liftlab/telemetry-tap.patch`, extracted from drone-force-bench at commit `c57ff1ff0043d39b8ffc9d9facace10dd548561a`. Apply it to a compatible LiftLab checkout to build the companion with the survey telemetry output. The existing LiftLab room survey viewer is included in `liftlab/room-survey.js` as an integration reference.

Pass `--output /path/to/captures/room-surveys` to open an existing capture directory. Captures are local and excluded from Git.

## Surface growth

Enable **Surfaces** to inspect the mesh grown from the saved landmark observations. Select **Follow replay** to scrub through the faces as they form and change. Each registered view supplies a camera origin and landmark IDs. Within one frame the nearest observations are processed first. An isolated point starts a zero-area seed. A second point can form an edge. A third non-collinear point can form a triangle. An uncovered landmark can extend the nearest eligible boundary when its 3D distance to the closed edge segment fits the growth distance. A landmark near the middle of an edge can attach even when both endpoints are farther away. Unattached seeds are reconsidered as edges and faces change around them. A ray hitting an existing face before its target defers that observation. Later views can retry the same landmark.

Change **Growth distance (cm)** and click **Rebuild surfaces** to regenerate the mesh from the saved landmarks. The default is 20 cm and each walk saves its own setting. A resulting triangle side can be longer than the growth distance. The limit uses the walk’s measured baseline or validated ToF scale. Hold Command or Option while dragging the reconstruction to pan. For a relative-scale walk, use **Metric scale** to mark two mapped replay frames with a known camera separation. The surface readout counts triangles, single seeds, open pairs and landmarks still occluded. The landmark layer and walk export include deferred points.

### Gaussian mapping

Open a finished walk and choose **Reconstruction method → Gaussian mapping**. **Build Gaussian map** trains locally on the Mac. Use **Display** to switch between colored triangulated points and Gaussian splats. **Compare withheld images** shows the rectified camera image and the trained rendering at its registered pose. The PLY download follows the selected display.

The trainer uses the saved calibration to rectify the fisheye images. It selects registered cameras with full rotations, reserves every tenth view starting at the fifth for evaluation, and triangulates tracks independently in the two neighboring training views. Tracks must agree on depth and pass forward/backward image tracking, parallax and reprojection checks. These points and the original landmarks initialize 1,500 steps of Brush training. Scale and alignment come from the saved map. This experiment holds images out of Gaussian training. The underlying camera reconstruction used the original walk. The model describes appearance in observed regions and does not measure obstacle clearance.

Use Brush v0.3.0 from the [official releases](https://github.com/ArthurBrussee/brush/releases/tag/v0.3.0). Set `ROOM_SURVEY_BRUSH` to the executable, install it on PATH as `brush_app`, or place it in `~/Library/Caches/RoomSurvey/brush-v0.3.0/brush_app`. Trained models remain local to each walk and camera reconstruction. To reproduce the sparse-landmark control, use `python gaussian_map.py brush WALK NEW_OUTPUT --sparse-seed` with the same Brush binary.

### Original DROID-W backend

`python gaussian_map.py prepare-droid WALK NEW_OUTPUT` exports every saved image, including frames without existing poses. It saves the capture timestamp table and a calibrated config for the authors’ RGB loader with Gaussian mapping enabled. The local Brush option uses saved camera poses. DROID-W estimates a new trajectory and depth using its learned tracker.

For an installation of [the authors’ code](https://github.com/MoyangLi00/DROID-W) at revision `c3414af6047d06bafc1a6645d09f23247f3b2cdc`, follow its CUDA and pretrained-weight setup. From that checkout, run `python run.py --config /absolute/path/to/export/walk.yaml`. If the prepared folder moves, update `data.input_folder` in its config. The upstream run writes keyframe depths and poses to `video.npz` and the Gaussian model to `final_gs.ply`. Preparing the export requires no GPU or network.
