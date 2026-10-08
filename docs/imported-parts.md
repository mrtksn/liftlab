# Imported drone parts

In **Airframe → Add a part → Import 3D object…**, use the independent button above the part categories to choose a GLB, glTF, OBJ or STL. Pick a glTF's buffers and textures together with its main file. OBJ uses the same plain material as the world importer. glTF/OBJ are Y-up and STL is Z-up.

Import now creates a shared asset in **Object library**. Its overlay sets name, file units, up axis and default scale. **Save to object library** does not place a copy. Select **Use on drone** to open the physical-properties overlay beside the drone. Choose Payload (default), Battery or Wing, category, size, weight, center of mass, mounting points and aerodynamic properties. Choose **Follows model** or **Filled boxes** (Coarse / Medium / Fine) for collisions. Both this overlay and the world editor share the scale slider, collision controls and indicators. See [Object library](object-library.md) for workflow and collision details.

Starting weight estimates 200 kg/m³ over the voxel volume, bounded to 5 g–20 kg. The initial center of mass is that volume's centroid. Explicit weight edits keep the chosen mass through later resizing. Orange marks CoM and blue marks mounting points; surface picking and XYZ fields place them. Choose a parent and own mounting point, then **Attach object**. Cancel leaves the design unchanged; its saved asset remains reusable.

## Editing and attachment

Imported parts offer **Edit object copy…** for the same overlay, plus weight, size, category, rotation and **Center of mass & attachment points** in their part editor. The same physical controls are available on existing rigid parts; frame offsets and points are under **Tune → Frame**. Surface picking is also available after import. Part selection in Edit mode shows the markers.

Coordinates are local metres from the drawing origin. Imported models are centered on their bounding box; rods start at their base, with X along the rod, and their CoM offset is measured from their middle. Frame offsets/points use the drone's body axes from the hub. Moving the CoM changes the drone's mass properties without moving the drawing or collision shape. Scaling an imported part scales its shape, CoM offset and mounting points together; rotations keep its chosen own mounting point aligned with the parent. Children follow their mounting points. Each part has one parent; several children can use different points. This is a tree, not a loop of physical constraints. Existing rods expose Base and Tip; custom coordinates are fixed local mounting sites.

Cable payloads retain their existing point-mass behavior: the hanging ball is the CoM, while position fields place the cable attachment. Additional rigid parts mount on rigid holders.

**Save drone use defaults** updates defaults in the shared Object library without changing other copies or shared import settings. The library × removes its entry while retaining files for placed copies.

## Solidity, physics and storage

The importer reuses the world's readers and voxel algorithm: intersected surface cells and enclosed cells become solid, then merge into boxes. Medium detail starts at 32 cells on the longest side and falls back to coarser detail above 6000 boxes. Open surfaces become thin solid shells; holes remain open when the voxel resolution resolves them. Original meshes and textures draw the object. The copy’s collision choice selects triangle surfaces or voxel boxes for contacts. Mass/inertia estimates assume volume-weighted boxes about the selected CoM; aerodynamic drag uses the part's bounding dimensions. Complex models can increase simulation cost.

Model source files live in browser IndexedDB; the current design, undo snapshots and reusable templates keep the shape and settings. Saving a design, exporting a design JSON and sharing a design code embed its original model files, buffers and textures. Undo snapshots omit source bytes. A fresh browser can restore the embedded files. Large designs may exceed the browser/account library limit; use an exported file when saving reports failure. Missing originals still have their solid-box drawing and collision geometry. Imports accept at most 128 MB of source files and 1024 named points per part.

Regression coverage: `tools/test_part_models.cjs`, plus the sidebar, world-view, multi-drone and payload-collision suites. These verify simulator behavior; they do not measure the real object's material properties or physical flight performance.
