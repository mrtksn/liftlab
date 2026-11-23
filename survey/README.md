# Handheld room survey

Open **Room Survey.command** on the Mac, then **Connect camera**. The local dashboard uses SSH to run the camera sender on the Pi. The default host is `pi@192.168.2.127`; the Pi needs Picamera2 and its hardware MJPEG encoder. A cold start on the original Zero can take a minute. The Pi stamps and compresses 640 × 480 frames at a target of 8 fps. The Mac saves the received JPEGs and camera-relative microsecond timestamps under LiftLab's `captures/room-surveys/<walk>/` (the repository root's `captures/`, not inside `survey/`). A network interruption ends the recording and schedules a final reconstruction of the received frames.

The Pi rotates the image 180° before JPEG encoding to compensate for the inverted camera mount. Preview, calibration and new recordings share that orientation.

1. Check the preview for focus and useful texture. For a measured lens model, open **Fisheye calibration** and its checkerboard on a flat display or paper. Collect at least 12 different board poses, cover the centre and edges, and click **Fit lens**. The fit applies to this lens, crop, orientation and resolution. Without it, COLMAP estimates a shared fisheye model from the walk. Rebuild a walk recorded in the same orientation after calibration if needed.
2. Carry the drone by hand with the motors off. Start level so the first registered camera gives the map an approximate up direction. Click **Start walk**, move sideways in a slow arc, keep overlapping objects visible and revisit the starting view. Avoid pointing exclusively at a blank wall or rotating on the spot.
3. For metric scale, mark A and B while the camera centre rests at each of two measured positions, then enter their separation in metres before pressing **Mark B**. A second person can click the buttons or use two fixed resting spots. The dashboard reports metres when both exact frames register together. Check a second measured distance independently. Without scale, the path and landmarks have relative units.
4. Watch the raw frame count and reconstruction stages, then **Finish walk**. The first reconstruction starts after 12 candidate views, normally around six seconds of capture. Each update is a batch estimate and can revise earlier positions. The Mac samples approximately two views per second, selecting up to 160 distributed views plus the marked frames. For longer runs it also proposes visually similar distant frames and uses available attitude, range and barometric height to propose additional matches. Each match must pass visual geometric verification. Drag the plot to orbit and scroll to zoom.
5. **Map JSON for LiftLab** exports the walk’s single map. Gaps in registered views are shown as breaks in the path. In LiftLab, select **World → World settings → Room survey → Open camera survey**. The overlay displays the trajectory and coloured landmarks. It uses right, initial forward and initial up axes. Collision and free-space estimation require additional geometry. The overlay is session-local. Raw frames and exported results remain on disk.

Use **Download walk ZIP** to export the walk selected in the dropdown, including the original JPEGs and all files in its folder. **Delete walk…** moves that folder into `.trash` under the survey output directory after confirmation. To recover it, move the folder back into the survey output directory and give it its original name.

A failed solve leaves the last successful snapshot available and reports the failure. Frame filenames in `survey.json` correspond to `timestamps.csv`. The reprojection error is a pixel fit diagnostic. Ground-truth accuracy requires independent measurements. The current prototype runs structure from motion on the Mac. The Pi reads camera and ToF data and listens on loopback UDP port 14681 for flight telemetry.

## Processing pipeline

The pipeline panel draws and runs a graph described entirely as data. Nothing in the page names a method.

- **Operators** (`operators.py`) are the processing steps the Mac can run: camera map, depth models, backprojection, landmark and depth surfaces, AI depth planes, Gaussian map, Gaussian mesh, walk clearance and LingBot-Map. Each declares typed inputs and outputs and its parameters, with the control to draw for each (number, whole number, checkbox, choice), its range, default and help. Choice options come from the modules that implement them. `GET /operators` returns the catalogue. `POST /operator` runs one: it returns outputs, reports a background job as pending, or asks the browser for work it does there (depth predictions for AI depth planes). Results built with non-default settings, such as Gaussian training steps, are kept beside the default ones.
- **Nodes** are an operator, its input connections and its settings. A node can override its operator's controls by key (label, default, range, options, help, hidden). Code nodes declare their own ports, output type and controls, and read the values as `settings`. Any operator node can carry code that calls `runNative(settings)` and edits its first output.
- **Presets** are JSON files: built-ins in `presets/`, yours in `presets/user/` (Save preset). A preset can define its own controls (checkbox, choice, number, button). A node setting follows one when it holds `{"var": "key"}`; in the inspector, choose “New preset control” beside any setting to create and bind one. A button control runs the pipeline to the node named in `run`. Choice controls can take their options from an operator parameter with `options_from`.
- **Viewer** nodes choose which outputs the 3D view and the frame panel show. Panels for Gaussian maps, clearance and LingBot results appear when the viewer shows that data type.

Opening a preset collects outputs that already exist and stops at the first node that would start new work. Run to node builds the selected node and everything it needs.

Data types (`TYPES` in `operators.py`) declare how they are displayed (map, points, polygons, splats, frames, table, …) and which tool panel goes with them, so a new operator that produces an existing kind of data needs no dashboard change. Thresholds the processing used to fix — ToF scale checks, up-axis limits, frame recovery and gap repair, cloud-plane support, clearance grid and pose gaps — are operator settings with the previous values as defaults.

**Device and recording** (beside Survey leveling) holds how this Mac drives the hardware, saved as `device.json` in the survey folder: Pi SSH destination, image size, frame rate, JPEG bitrate, 180° rotation, lens calibration file, ToF I²C address and poll rate, telemetry UDP port, live-map view spacing and limits, and the calibration checkerboard size (the printable target follows it). `--pi` on the command line still overrides the destination. Camera and sensor changes apply on the next Connect camera.

## Frame depth replay

Open a saved walk and use **Frame view** above the recorded image to switch between the original photo and four depth sources. The shared timeline selects the same frame in both views. The original photo includes the full fisheye field of view. All four depth previews use the central rectified camera view. In **Reconstruction**, select a method to see its build and display controls.

| Frame source | What the colours show |
| --- | --- |
| Depth Anything V2 Small | Relative inverse depth from each image. Yellow is near, purple is far. |
| Apple Depth Pro 2 Small | Video depth using overlapping eight-frame windows and the rectified focal length. Estimated metres use a fixed 0–8 m colour scale. |
| LingBot-Map | Relative depth for each image in a 48-frame group with up to four preceding context frames. Scale can vary between groups. |
| Gaussian rendered depth | Visible surface depth rendered from the saved splats. The selected frame indicates a saved, interpolated or nearest camera pose. Black pixels have no usable surface depth. |

In **Recorded photo**, **Generate missing depth maps** queues all available methods that still need processing. **Generate all frames** computes the selected method for every recorded JPEG in a finished walk with a camera map. Gaussian depth also requires a trained Gaussian map. Relative previews stretch the colour scale for each frame. The numeric arrays retain the model values. The walk ZIP includes those arrays under `frame-depth-v1-<camera index>/<method>/<frame>.npy`. Models and inference stay on the Mac. This setup uses the existing LingBot Python environment and model caches under `~/Library/Caches/RoomSurvey`.

**Compare** gives the left and right sides independent sources. The divider supports dragging, arrow keys, Home and End. When a photo is compared, replay uses the rectified crop to align it with the depth pixels.

Choose an **Image set** under **Reconstruction**. Recorded photos use the existing reconstruction methods. A depth source backprojects the numerical arrays into the saved camera poses and uses the recorded colours. Depth Anything fits inverse-depth scale and offset against tracked landmarks. LingBot aligns its scale with landmark depth, as does Apple when the saved walk has relative scale. Only oriented frames with usable depth and points supported by at least two views contribute. Gaussian depth reprojects the trained splats. This point cloud is a visualization in the existing camera coordinate system.

**Processing times** shows the machine's measured wall time, average time per processed frame and throughput. New builds save stage timings with the depth archive. For the original walks the dashboard labels the 24-frame benchmark separately. The depth-cloud report times reading, projection and fusion after depth maps are available.

### Surfaces from depth points

In **Reconstruction**, choose a depth **Image set**, select a **Surface method**, then click **Generate surfaces**. The **Surfaces** and **Depth points** switches control their visibility independently. **Planar regions** fits bounded polygons for walls and other flat areas. **Voxel envelope** wraps clusters of observed points in a simplified triangle mesh. Its thickness depends on the voxel size.

**Include ToF returns** is available in metric walks. It pairs range readings with oriented camera poses, uses the measured sensor mount when present, and adds endpoints within two voxel widths of the depth cloud. The result reports accepted and paired returns and identifies a camera-axis approximation. Gaps without supporting observations remain uncertain. Each source, method and ToF setting has a separate cached result in the walk ZIP. Surface method labels and builders are registered in `depth_surfaces.METHODS`.

## Range measurements

The mounted TOF250 responds at 7-bit address 0x52. Start walk saves one row per JPEG in timestamps.csv, including camera-relative microseconds, the Pi monotonic time of the encoder callback, the time the range register read completed, and the raw reading. A blank range field means the read failed. New walks also save range.csv, with register reads polled independently at a target of 60 Hz, timestamped before and after each transaction on the Pi. The camera CSV pairs each encoder callback with the most recent completed read within 80 ms. Older walks sampled at JPEG output time. Read timing does not establish the sensor’s internal measurement cadence or hardware synchronization with camera exposure. Median readings were 52 at 50 cm and 104 at 100 cm. Values are kept raw for replay. The tilted sensor measures along its viewing direction. The mapper estimates metric scale from compact, depth-consistent landmark patches near the optical axis when the available ranges span multiple distances and withheld views agree. This estimate assumes a nearly parallel camera and range beam. The map uses relative scale when the range check fails.

With the survey-enabled Linux companion running, the Mac saves the ESP32 attitude quaternion, body-frame specific force and angular rate, height, vertical speed and validity in motion.csv. Times are Pi UART receipt times. Reconstruction associates observations with encoder callbacks, and can orient the map to gravity from consistent rotations about multiple axes. A held-out rotation check rejects weak fits. Walks that turn mostly about the vertical are levelled instead from the IMU up direction of each mapped view: the fit also estimates the camera-to-IMU mount, so a remounted board needs no setting. It keeps the first camera's heading as +Y, and reports its uncertainty, held-out error and the fitted camera pitch under up_fit in survey.json. It rejects fits that withheld views contradict or that leave up uncertain by more than 10°. A recorded level start takes precedence over both. Raw inertial samples remain available for replay. These steps do not integrate acceleration to fill translation gaps.

## Terminal launch and regression

From LiftLab's repository root, which holds the Python environment and `captures/`:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r survey/requirements.txt
cd survey
../.venv/bin/python server.py --pi pi@192.168.2.127
../.venv/bin/python -m unittest discover
```

The dashboard binds to loopback on port 8779. SSH authentication must already work without an interactive password prompt. Camera diagnostics are in `../captures/room-surveys/camera.log`. **Disconnect** releases the camera.

To compare the same recorded frames on the two computers, run `../.venv/bin/python track_bench.py ../captures/room-surveys/<walk>` from `survey/`. This streams the first 48 downsampled grayscale frames to a temporary NumPy process over SSH. It measures the compute time of spatially distributed 7 × 7 patch matches at 160 × 120. Capture, decoding, transport, pose estimation and reconstruction are outside this timing measurement. Compare the patch counts and the 95th-percentile time with the camera interval.

## Second handheld pass

Open the indexed walk and use **Localize in this walk → Start tracking**. An estimate matches SIFT features to triangulated landmarks and solves the fisheye camera pose on the Mac. The match image and counts refer to the same sampled frame. The blue map marker and direction line appear when the solver has enough geometrically consistent matches spread across the image. Choose **Recorded frame** to scrub the original sequence, change **Minimum inliers** to compare rejection rates, and switch the map layers or top view to inspect the result. A new walk records a separate map. Use tracking on the saved map for a second-pass localization experiment.

The HCJ six-pin TOF250 probe uses the Pi Zero header I²C bus 1, with SDA on physical pin 3 (BCM2), SCL on pin 5 (BCM3), and a shared ground. Confirm the sensor variant, pin numbering and logic levels before connecting. The PDF lists 5 V communication levels. Use a bidirectional I²C level shifter when using 5 V sensor-side pull-ups. Mount the optical window beside the camera facing forward and measure their offset. **Read sensor** shows the current paired reading when streaming or probes the register when offline. Compare raw readings with measured ranges before assigning units. The probe is separate from the pose solver and occupancy mapping.

## LiftLab integration

In LiftLab, **World → Room survey → Room Survey…** opens this page in a full-size dialog. LiftLab serves the page itself (`survey/index.html`, from its site or a local server) and names the companion, this server, with `?companion=http://127.0.0.1:8779`; the dialog's address field changes it, and **Open in a tab** opens the same page on its own. Every request the page makes goes to that address (`api.js`). Opened from the server itself, the page uses its own address.

**Send to World.** In the dialog, any polygon or triangle output (a node's **Output** section: the Gaussian surface mesh, depth or landmark surfaces) has **Send to World**. LiftLab opens it in the World editor's object import as a Z-up model: levelled as the viewer shows it (the map's IMU levelling plus any **Survey leveling** correction), in metres when the map has a ToF scale; otherwise at the output's own ToF estimate when it has one (the Gaussian mesh casts the paired ToF beams into itself: the import note says how well that held on held-out beams), or at a size you set, each triangle facing the walk (the side the camera saw, so the room shows from inside and you can look in from outside). **Save to object library**, then **Place in world**: copies collide with the triangles (**Follows model**). A world object takes up to 100,000 triangles.

The server listens on 127.0.0.1 only and answers pages it serves, LiftLab's site (`https://mrtksn.github.io`) and pages from a local web server (`http://localhost` or `127.0.0.1`, any port). For another page, start it with `--origin URL` (repeatable). Other sites get no CORS headers and their POSTs are refused; requests naming another host are refused, against DNS rebinding. A LiftLab opened as a file can't use it: browsers send the origin `null`, which sandboxed frames on any site also send. Chrome may ask once whether LiftLab's site may reach devices on your local network: allow it.

The standalone dashboard and camera stream use SSH. The optional Pi flight telemetry tap is `liftlab/telemetry-tap.patch`, a change to LiftLab's `runner/pi/dfb_pi.c`; `liftlab/room-survey.js` is the World view's survey overlay. Both move into LiftLab proper as the survey joins the page.

Pass `--output /path/to/room-surveys` to open another capture directory. Captures are local and excluded from Git.

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

### LingBot-Map on Apple Silicon

In a finished camera walk choose **Reconstruction method → LingBot-Map (Apple Metal)**. Build once per saved calibration. Orbit and zoom the point cloud and scrub its walk replay. The blue line connects predicted camera positions. Inference takes rectified RGB images and estimates a trajectory in its own relative coordinates. It uses every eighth image plus the finish, including images without registered camera poses. ToF and inertial streams are available in the recording and are independent of this estimate.

The tested runtime is arm64 Python 3.11, torch 2.8.0 and torchvision 0.23.0. The dashboard expects the following local layout under `~/Library/Caches/RoomSurvey/lingbot`:

- `source/` is [LingBot-Map](https://github.com/Robbyant/lingbot-map) at `b9dd1b1de535e6d36e777fb42b8433c781bfd783`.
- `lingbot-map.pt` is the [balanced pretrained checkpoint](https://huggingface.co/robbyant/lingbot-map).
- `env/` is a Python virtual environment with torch, torchvision, einops, scipy, numpy, pillow, opencv-python, tqdm, safetensors, huggingface_hub and omegaconf.

The runner uses native PyTorch attention at 224 × 168, two initialization frames, a sliding attention window of eight, and two camera refinement iterations. It assembles the complex rotary table on the CPU before passing it to Metal. PyTorch CPU fallback is enabled for unsupported operations. The point cloud keeps positive depths with confidence above 1.5 and samples a four-pixel grid. The standalone HTML viewer displays one in three of those points. The PLY export contains the full filtered cloud. The walk ZIP includes per-pixel depth and confidence, predicted pinhole intrinsics, camera-to-world poses, and source frame names in `predictions.npz`.

## Gaussian surface mesh

Open a finished walk and choose **Gaussian mapping**. Under **Display**, choose **Surface mesh** or **Splats + mesh overlay**. Click **Extract mesh** on the first use. Both views support orbit and drone replay. **Download PLY** in either mesh mode exports indexed triangles in the same coordinates and units as the saved splats.

Extraction runs locally on the Mac CPU and reuses the trained Brush PLY. Up to 120 registered camera poses render visibility-weighted depth. Each accepted return needs agreement from two other views with parallax. Truncated signed-distance fusion, masked Marching Cubes, connected plane fitting and quadric simplification produce the mesh. Openings in the mesh indicate missing surface evidence. Wall and furniture orientation comes from the fitted geometry.

The range check uses valid ToF readings paired within 80 ms of a camera callback, counts first mesh hits, and reports the median and 80th-percentile range differences. For relative maps, alternating eligible frames estimate a provisional scale for the check. Remaining frames measure agreement. The exported coordinates stay in the original map units. A metric map can itself have a ToF-derived scale. For a measured mount, place `tof-extrinsics.json` in the walk directory before extraction with `direction` (a 3-vector) and `origin_m` (a 3-vector in metres), both in rectified camera coordinates, right/down/forward. With no mount file, the check assumes a beam through the camera centre. A nonzero measured offset requires a metric map. The walk's Gaussian artifact folder contains the mesh and its JSON report under `mesh-v1/`.

Under **Gaussian mapping**, enable **Show ToF evidence** to inspect returns on the saved mesh. **Validation misses** isolates reserved frames whose beam misses every triangle. Select a frame or use **Next reading** to see the recorded photo, measured endpoint and first mesh intersection. **All paired readings** includes the independent split used to fit provisional scale on relative walks. The markers show sightlines from registered poses and are drawn over the mesh for inspection.

### Walk clearance

Open a saved walk, choose **Gaussian mapping → Walk clearance**, and build the clearance map after training the splats. Scrub the colored strip or the main replay. Set the camera-centered enclosing radius (including propellers) and an extra margin. The strip reports a result at each frame with a supported pose. Radius changes reuse the saved grid.

The Mac integrates multi-view-consistent rendered depths and timestamped ToF returns in a metric voxel grid around the walk. Ranges use the independent log when present, otherwise the frame-paired measurements. Short gaps between registered poses use bounded position and rotation interpolation. Longer gaps keep their unknown pose. Cells near rays from two viewpoint groups can count as observed free, and range or depth endpoints count as occupied. The report distinguishes overlap with an occupied cell, overlap with unknown space, and a sphere contained in observed free cells. These are recorded-frame estimates. Between-frame clearance and current room conditions require separate checks.

The grid download is a NumPy archive with XYZ-ordered cells (0 unknown, 1 observed free, 2 occupied), origin in metres at the first cell's lower corner, voxel size, and metres per map unit. Relative maps need a validated provisional ToF scale or a measured baseline. The report identifies the scale and whether the beam uses measured extrinsics or the camera-axis approximation.
