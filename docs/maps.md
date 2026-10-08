# Maps and world editor files

Choose a map from the top bar’s World group. Open Settings → World editor to import models and edit object positions, rotations, scale, orientation and collision detail. The Maps section stays available while editing.

## Saving and switching

- Enter a name and click Save map. The selected map's existing name updates that saved map; a new name creates a separate map.
- Saved maps appear alongside Open field, Parkour city and Full-scale city in the top bar selector.
- A map keeps the base layout/seed, placed objects and solid shapes, random seeds and environment settings. Each saved map owns copies of its model files, so deleting an active object does not damage saved maps.
- Switching maps replaces the objects and resets every flight. Drone designs, selection and pause state stay in place. Clear targets stay unchanged; blocked targets follow the usual reset fallback to the start plaza.
- Built-in maps clear imported objects. The city dice button changes the generated city seed and keeps the current objects.
- Delete saved map removes the saved copy; the active world stays in place. Save seeds keeps only the layout/seeds/environment, as before.

Saved maps live in this browser's IndexedDB. The active world still uses the existing fleet save, including the selected map ID. Clearing site data can remove both; export important maps.

## Export and import

Export map downloads the current edited world, including edits made since the last save, as `.liftlab-map.json`. Import map loads that file and adds a saved map to the selector when storage is available. Use these files to move maps between browsers or computers.

Version 1 files contain `format: "liftlab-map"`, a name, a `world` snapshot (`map`, `objects`, `seeds`, `environment`), and `files`. Each model file group has its original file names and base64-encoded model/buffer/texture bytes. Imported file IDs are regenerated to avoid overwriting other local assets. Drone designs, programs and recordings are not part of the map file.

The complete JSON structure, transforms, collision boxes, seeds, environment and embedded-data encoding are validated before changing the world. Unsupported versions and invalid data produce a message. The file limit is 128 MB; there are limits of 1000 objects, 6000 collision boxes per object and 600000 boxes in total.

When an original model is already missing, saving/exporting keeps its collision boxes and reports the limitation. Those objects can move but cannot turn or resize. Import can still apply a map if browser storage fails, with a message asking you to keep the imported file. A failed Save map leaves the library unchanged.

## Regression checks

`node tools/test_world_objects.cjs` covers the original editor plus map save/update/copy, switching, reload, asset retention, fresh-browser portable imports, glTF buffers/textures, collision shapes, malformed-file rejection, browser quota failures and phone layout. `node tools/test_world_view.cjs` checks existing world selection, controls, defaults, reload and responsive themes. Refresh static asset fingerprints with `python3 tools/stamp_ui_assets.py` after source/CSS edits.
