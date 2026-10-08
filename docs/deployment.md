# GitHub Pages deployment

LiftLab is a static site at https://mrtksn.github.io/liftlab/. Pages is configured to publish the `main` branch from the repository root. There is no website build step. Changes in a local working folder reach Pages only after they are committed and pushed to `main`.

## Publishing site changes

After editing JavaScript or CSS, run `python3 tools/stamp_ui_assets.py` before committing. `python3 tools/stamp_ui_assets.py --check` checks that HTML asset references and CSS imports match the current files. The shared UI needs no framework build step; see [ui-design-system.md](ui-design-system.md).

1. Review `git status` and `git diff`. Stage the intended files explicitly; leave unrelated work and firmware exports out.
2. Commit and push the reviewed changes to `origin main`.
3. In GitHub → Actions, check **pages build and deployment** for the pushed commit. The history contains individual runs, not a separate workflow for every change.
4. Once the deployment succeeds, reload the site. If it still shows old controls, try a hard reload or a private window and confirm the run deployed the intended commit.

For a manual retry, open the Pages run for the intended commit and use **Re-run all jobs**. This redeploys that run's revision; it cannot publish local changes that have not been pushed. Avoid using an older run to verify newer work.

## Firmware is a separate release

**Board firmware and host programs** validates chip/host builds and selected tests, and uploads the built firmware bundles as an artifact. It does not publish a website or replace the checked-in firmware bundles. A successful Pages run does not imply that board checks passed, and neither proves real-flight behavior.

After native firmware changes, the bundles in `firmware/` must be rebuilt and committed; Pages serves those files and does not compile firmware on demand. Two ways:

- **From CI (no local ESP-IDF):** every run of **Board firmware and host programs** builds the six chip/role bundles with `tools/build_firmware.sh` in the ESP-IDF 5.3.2 container and uploads them with their manifest as the artifact `firmware-COMMIT` (30 days). Download it (`gh run download RUN -n firmware-COMMIT -D firmware`), review `git diff --stat firmware`, and commit the bundles and manifest.
- **Locally:** activate ESP-IDF 5.3.2 and run `sh tools/build_firmware.sh`. It stamps every build with this checkout's commit (`+changes` when `runner/` has uncommitted edits); `ONLY=esp32s3-flight` builds one bundle.

Each build reports that stamp to `version` and at power-on, and the manifest records it: the install dialog compares the two when it connects. `sh tools/check_firmware_fresh.sh` says whether the checked-in bundles were built from the current firmware source (exit 1 lists the commits since); CI runs it and leaves a warning when they are stale. For browser C/WebAssembly changes, regenerate the corresponding checked-in artifacts with `runner/fc/build_wasm.sh` or `runner/build_wasm.sh` and validate them separately.

Custom sensor C saved in a design needs a matching rebuilt firmware installation. Saving a design or deploying the website does not compile its driver source. See [hardware wiring](hardware-wiring.md) and [board support](boards.md).
