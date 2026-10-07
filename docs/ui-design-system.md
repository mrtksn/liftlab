# Shared UI and theme

LiftLab uses native DOM components in `js/ui-components.js`. Both tab bars are instances of one component, configured in `js/ui-shell.js`. Feature modules supply content and callbacks; components own labeling and common interaction. CSS components own appearance. Simulation, formulas, board runtime and agent tools remain separate.

No React, Tailwind or bundler is required. Native controls preserve browser keyboard behavior, while live readouts update existing nodes instead of rebuilding a component tree.

## Where to change the design

| File | Responsibility |
|---|---|
| `css/tokens.css` | Both palettes, semantic colors, fonts, type sizes, spacing/density, radii and focus settings |
| `css/base.css` | Document defaults and resets |
| `css/components.css` | Shared tabs, buttons, segmented controls, fields, cards, menus, disclosures and status styles |
| `css/features.css` | Simulator layouts and feature states, including pilot pads, formula editors and wiring overview |
| `css/layout.css` | Responsive arrangements and reduced-motion rules |
| `css/style.css` | Entry point importing these files in order |
| `js/ui-components.js` | Shared factories and interaction helpers |
| `js/ui-shell.js` | Editor/readout tab data and feature callbacks |
| `js/section-nav.js` | Shared section navigation for both scrolling panels |

Change `--palette-accent-light` and `--palette-accent-dark` to recolor selection, primary actions and focus throughout the app. Change radius and font tokens to restyle common elements. `--density` scales spacing; fixed dimensions for the 3D view and flight pads stay in layout rules. Source badges and 3D visualization colors also use theme tokens. Keep palette literals in the token file and reference semantic variables elsewhere.

The system theme is automatic. A host can set `data-theme="light"` or `data-theme="dark"` on the document root to override it; removing the attribute restores system behavior.

## Component patterns

Factories return native elements and accept native attributes/events, preserving the caller's `class`. Use the same factory and variant when controls have the same purpose. Use `el()` for structural markup, not another implementation of a shared control.

```js
const save = UI.button({ variant: 'primary', text: 'Save', onclick: saveDesign });
const board = UI.choice({
  label: 'Board', options: [[0, 'Off'], [1, 'Flight controller']],
  value: 1, commit: true, onChange: assignBoard,
});
const field = UI.field({ label: 'Board', hint: 'Where this device is wired.' }, board);
const card = UI.card({ class: 'hw-device' }, field, save);
```

- `UI.button`, `input`, `select`, `textarea`: native controls with shared styling/focus hooks. Buttons default to `type="button"`; `variant` adds `.btn` chrome (for example `primary`). Classes such as `btn`, `icon-btn`, or a segmented-control parent express the other visual variants.
- `UI.field`: visible label and optional help associated with a direct or grouped input; `layout: 'inline'` selects an inline field.
- `UI.choice`: data-driven options and change callback. `commit: true` defers keyboard stepping until Enter or blur for changes that reset flight; pointer selections apply immediately.
- `numField`: shared slider/number field with unit conversion, typed values beyond the slider when allowed, hard bounds and an optional disabled reason. Returns `{node, refresh, setOff}`.
- `UI.card`, `section`, `status`: shared containers and native status region. Feature classes add layout/state, not duplicate default card chrome.
- `UI.details`: native disclosure with title, initial state and optional `onToggle`. `UI.bindDisclosure` handles existing button/body disclosures, including guarded activation and lazy feature rendering.
- `UI.tabs`: selection, disabled tabs, ARIA relationships, arrow/Home/End navigation, roving focus and optional persistence. Supply tab/panel IDs, labels and optional icons/badges. Construction only renders initial DOM; call `restore()` or `select()` during bootstrap after feature modules load.
- `menuButton`: keyboard action menu with disabled items, grouping, hints and Escape focus restoration.
- `keepFocus`, `setText`, `syncKv`, `syncChips`: preserve focus, readout selection and stable nodes during feature updates.

Static HTML controls receive shared classes at bootstrap. Generated HTML fragments must call `UI.hydrate(fragment)` or use factories. Native checkbox/radio/range behavior and specialized code-editor fonts remain intact.

To add a tab, add panel markup and an entry in `EDITOR_TABS` or `READOUT_TABS`; do not write a new click/keyboard handler. Keep application state, saving, board access and formula application in feature callbacks. Component construction must not silently save an initial value over persisted state.

## Verify and publish

After changing local JavaScript or CSS, refresh content fingerprints:

```sh
python3 tools/stamp_ui_assets.py
python3 tools/stamp_ui_assets.py --check
```

The tool versions HTML asset references and nested CSS imports. It leaves the external GoatCounter and Three.js URLs intact. Pages still serves static files from the repository root, without a new build step. See [deployment.md](deployment.md).

Serve the repository and open `/tools/test_ui_components.html` for nine browser DOM regression checks. Its theme picker previews shared styles. Also run existing agent, hardware, installation and driver-preset checks. Verify desktop/narrow layouts, flight controls/readouts, part editing/undo/save, formula apply/revert, driver drafts, installation guides and AI/ground panels. UI tests do not verify physical flashing or live-model reasoning.

## AI chat and design HUD

Assistant replies render GitHub-flavored Markdown with pinned local Marked/DOMPurify distributions; attribution and versions are in [the vendor note](../js/vendor/markdown-README.md). User messages and tool results remain plain text. Raw HTML is shown as text, images become links, code is inert and link protocols are restricted. Parsed assistant content is cached per feed item; rebuilding the feed clones those nodes instead of reparsing unchanged replies.

The Edit viewport shows saved-design characteristics through `js/design-stats.js`, cached until geometry, environment, battery or output-limit settings change. It reuses mass-property math with saved components/rest angles and does not read live battery/health/cargo state. Only the text in the three existing flight readout spans changes: the badge, legend, font, position, transparent background and pointer behavior use the existing HUD unchanged. [Physics definitions](simulation-physics.md#design-readouts) explain the estimates.

Run `node tools/test_chat_design.cjs` for Markdown safety, design equations, mode/visibility transitions and responsive light/dark checks. Set `LIVE_URL` to verify a deployment or `TEST_SCREENSHOTS` to a path prefix for previews. Use `--hardware-gpu` for hardware-rendered screenshots. This is separate from the full flight/radio regression in `tools/test_flight_browser.cjs`.

## Selected-drone UI

The top dropdown and viewport selection bind both sidebars to one drone. Add airframe creates another instance. Use the same native controls and HUD for every instance; the design HUD retains its transparent flight-HUD styling. Selection rebuilds controls that capture drone-local objects and restores per-drone drafts/undo/chat context. Shared world/view controls remain shared. Ownership and callback rules are in [multi-drone architecture](multi-drone.md).

## Checkpoint and rollback

The complete pre-refactor state is commit `53cf25c`, tagged `ui-before-refactor-2026-10-06`. The refactor is on `codex/ui-components`. Compare with `git diff ui-before-refactor-2026-10-06..codex/ui-components`. With a clean checkout, switching to the tag restores the original version in detached HEAD; switching back to `codex/ui-components` restores this version. Preserve subsequent local work before switching. Avoid a hard reset as a rollback shortcut.
