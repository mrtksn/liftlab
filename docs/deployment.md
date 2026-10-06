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

**Board firmware and host programs** validates chip/host builds and selected tests. It does not publish a website or replace the checked-in firmware bundles. A successful Pages run does not imply that board checks passed, and neither proves real-flight behavior.

After native firmware changes, activate ESP-IDF 5.3.2, run `sh tools/build_firmware.sh`, run the relevant tests, and review the generated chip/role bundles and `firmware/manifest.json`. Commit the intended bundles and manifest along with the source changes. Pages serves those files; it does not compile firmware on demand. For browser C/WebAssembly changes, regenerate the corresponding checked-in artifacts with `runner/fc/build_wasm.sh` or `runner/build_wasm.sh` and validate them separately.

Custom sensor C saved in a design needs a matching rebuilt firmware installation. Saving a design or deploying the website does not compile its driver source. See [hardware wiring](hardware-wiring.md) and [board support](boards.md).
