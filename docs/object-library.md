# Object library

The **Object library** is shared by **Airframe → Add a part** and **World settings → World editor**. Both **Import 3D object** buttons open the same import overlay. GLB, glTF, OBJ and STL use the same readers; select a glTF's buffers and textures with its main file.

Import sets the asset's name, file units, up axis and default size with a live scale slider. It contains no mass, mounting or aerodynamic settings. **Save to object library** saves the source and opens the library; it does not place an object. **Import settings** edits the reusable asset without changing copies already placed.

**Use on drone** opens an independent copy beside the selected drone. Choose Payload (default), Battery or Wing, a category, size, rotation, weight, center of mass, any number of named mounting points, and its parent/own mounting points. Physical model reuses the existing drag/polar editor. Battery copies participate in the existing power model when wired in **Computers → Power**; capacity/cell settings remain in the drone's battery controls. Wing copies use the existing chord-X/span-Y airfoil model on their bounding dimensions; the imported drawing and collider stay intact. Cancel leaves the design unchanged. **Edit object copy** opens this same overlay for existing imported parts.

**Place in world** places an independent fixed copy and opens its existing in-view world editor. Set its name, position, turn, scale and collision geometry. World copies can add a two-pose hinge/slide animation with loop, button or proximity activation; see [World animation editor](world-animation.md). Animation is exclusive to world copies. File units and up axis belong to the library import settings. Both editors share the actual scale field, collision controls, collision renderer and overlay styling.

## Collision choices

- **Follows model** uses the original triangle surfaces for contacts, collision rays and radio obstruction. Closed meshes enclose a volume; an open mesh acts as a surface. The blue indicator draws those triangles, rather than a box approximation. This applies to static world objects, imported drone shapes, cable-ball/fleet contacts and released imported cargo.
- **Filled boxes** uses the existing surface sampling, outside flood fill and merged voxel boxes. **Coarse / Medium / Fine** start with 16 / 32 / 64 cells on the longest side, falling back to coarser geometry above 6000 boxes. Blue indicators draw the boxes used by physics.

Box mode is the compatible default for existing copies. Collision settings belong to the copy. Hiding the indicator leaves physics enabled. Scaling carries collision geometry, center of mass and mounting points together. Mesh geometry retains triangle topology; it cannot restore detail absent from a low-poly source.

The physics solver still samples drone/loose-body contacts and approximates articulated impact response. Mesh-to-mesh contact response is sampled; this is not a replacement for a full continuous collision solver. Mass, inertia and default center of mass remain estimates from uniform-density voxel volume, independent of the chosen collision surface. Drag uses bounding dimensions; wing aerodynamics use the existing polar. Initial weight estimates 200 kg/m³, bounded to 5 g–20 kg, until explicitly edited. Current imports limit source files to 128 MB and geometry to 100,000 triangles. Very detailed geometry increases simulation and storage cost.

## Persistence and compatibility

Library metadata and original model/buffer/texture files use IndexedDB. Copies retain their own physical geometry/settings in designs or maps. Portable designs/maps include original files and mesh collision data; validation rejects malformed meshes before applying them. Undo keeps geometry/settings without duplicating source bytes.

Old imported-part templates migrate into this library with their saved drone-use defaults; the old storage remains as a backup. Existing world/drone model assets are adopted into the common library. **Save drone use defaults** explicitly saves a part's settings for subsequent drone copies; it does not alter existing copies or the asset's import settings. Removing a library entry retains files and existing copies, including saved maps. Removal is remembered so automatic migration does not recreate the deleted entry. Unreferenced retained files are not currently garbage-collected.

Missing original files leave saved collision geometry usable; source editing or new placement reports that the original is missing. Browser storage failures are reported. Export important designs/maps before clearing site data.

## Regression coverage

`tools/test_object_library.cjs` covers one import flow, independent drone/world use, battery/drag/mounts, actual mesh indicators, rotated mesh contacts, sloped normals, empty bounding-box corners, open-surface crossing, triangle rays, cancel, portable mesh validation and deletion/reload retention. Existing imported-part and world/map suites cover real formats/textures, numerical mass/inertia, attachment trees, flight, exports, cold browsers and responsive previews.
