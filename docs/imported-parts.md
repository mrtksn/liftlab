# Imported drone parts

In **Airframe → Add a part → Payloads → Import 3D object…**, choose a GLB, glTF, OBJ or STL. Pick a glTF's buffers and textures together with its main file. OBJ uses the same plain material as the world importer. glTF/OBJ are Y-up and STL is Z-up.

The simulation pauses. The model appears beside the drone, with the import form in the right panel (below the viewport on a phone). Set its name, category, longest side in metres and weight in kilograms. The default category is Payload; choosing another category labels a rigid mass, without adding a motor, sensor or battery function.

The initial longest side is 20 cm regardless of file units. Starting weight estimates a uniform density of 200 kg/m³ over the voxel solid, bounded to 5 g–20 kg. It is an editable estimate, not a measurement. The initial center of mass is the volume centroid of that solid. Resizing before an explicit weight edit updates this estimate; subsequent size edits keep the chosen weight.

Orange marks the center of mass and blue marks attachment points. Use **Pick center on object** / **Pick on object** to choose a surface, or XYZ fields to place an interior point. Add, name and remove mounting points as needed. Choose **Attach to** and **Mount object by** to align the object's own point with a parent point. **Attach object** adds it to the drone and keeps a reusable copy in its selected Add a part category. **Cancel** leaves the design unchanged and restores the previous editing/running state.

## Editing and attachment

Imported parts expose weight, size, category, rotation and **Center of mass & attachment points** in their part editor. The same physical controls are available on existing rigid parts; frame offsets and points are under **Tune → Frame**. Surface picking is also available after import. Part selection in Edit mode shows the markers.

Coordinates are local metres from the drawing origin. Imported models are centered on their bounding box; rods start at their base, with X along the rod, and their CoM offset is measured from their middle. Frame offsets/points use the drone's body axes from the hub. Moving the CoM changes the drone's mass properties without moving the drawing or collision shape. Scaling an imported part scales its shape, CoM offset and mounting points together; rotations keep its chosen own mounting point aligned with the parent. Children follow their mounting points. Each part has one parent; several children can use different points. This is a tree, not a loop of physical constraints. Existing rods expose Base and Tip; custom coordinates are fixed local mounting sites.

Cable payloads retain their existing point-mass behavior: the hanging ball is the CoM, while position fields place the cable attachment. Additional rigid parts mount on rigid holders.

**Save to part library** updates the reusable template from an edited imported part. Templates are shared across this browser's drones. The × beside a library entry removes that template without removing copies already attached to drones.

## Solidity, physics and storage

The importer reuses the world's readers and voxel algorithm: intersected surface cells and enclosed cells become solid, then merge into boxes. Medium detail starts at 32 cells on the longest side and falls back to coarser detail above 6000 boxes. Open surfaces become thin solid shells; holes remain open when the voxel resolution resolves them. Original meshes and textures draw the object; voxel boxes drive ground contacts, fleet collisions and loose cargo geometry. Mass/inertia estimates assume volume-weighted boxes about the selected CoM; aerodynamic drag uses the part's bounding dimensions. Complex models can increase simulation cost.

Model source files live in browser IndexedDB; the current design, undo snapshots and reusable templates keep the shape and settings. Saving a design, exporting a design JSON and sharing a design code embed its original model files, buffers and textures. Undo snapshots omit source bytes. A fresh browser can restore the embedded files. Large designs may exceed the browser/account library limit; use an exported file when saving reports failure. Missing originals still have their solid-box drawing and collision geometry. Imports accept at most 128 MB of source files and 1024 named points per part.

Regression coverage: `tools/test_part_models.cjs`, plus the sidebar, world-view, multi-drone and payload-collision suites. These verify simulator behavior; they do not measure the real object's material properties or physical flight performance.
