# `/collab/` design finish record

Date: 2026-10-23. Scope: the new industrial kitting route only. Mode: Operate.

The bounded design finish review and route documentation are complete. The fresh UI review disposition is **ship, limited to the two original findings**: the task overview fits the 1440 × 900 desktop view, and the scene has source-defined keyboard camera controls. This is not a blanket verdict that every surface or runtime behavior was tested. The real 300-second browser run and final continuous-order acceptance remain separate, pending runtime work; this record does not certify either.

## Authority and method

The approved direction is an industrial production variant of the incumbent steel-blue/night laboratory world. The new route extends that world; it does not redesign `/multi/`, the individual robot routes, or the reference route. The product and physical-task truth remain in `PRODUCT.md` and [the collaboration brief](COLLAB-SURFACE-BRIEF.md).

A fresh general-agent documenter used the equivalent finish-documentation discipline because the specialized shipped documenter harness was not loaded. No agent definition files were read. The documenter read Impeccable's complete skill and document reference, the existing product/design context, both route briefs, and the four surface sources. The session's existing context load and single detector run were not repeated. No UI code was changed in this documentation pass.

Checked evidence:

- `dist/collab/index.html`, `style.css`, `view.mjs`, and `app.mjs`: source inspection of layout, semantics, state presentation, rendering, and camera behavior.
- `DESIGN.md` and `.impeccable/design.json`: the incumbent tokens, scope rules, component samples, rendering notes, and legacy preservation rules.
- `.impeccable/review/night-desktop.jpg`: incumbent visual reference, viewed alongside the new route captures.
- [.impeccable/review/collab/desktop.png](../.impeccable/review/collab/desktop.png): 1440 × 900 image, showing the ready state and task overview through ownership and the collapsed task notes.
- [.impeccable/review/collab/mobile.png](../.impeccable/review/collab/mobile.png): 390 × 1499 full-page image, showing the scene followed by the complete ordered control panel without clipped horizontal content. The image height is a full-page capture, not a claim that all controls fit in a phone viewport.
- [The recorded single detector result](validation/collab/design-detect.md) and [fresh finish review](validation/collab/finish-review.md): original findings, one-batch fixes, same-size capture confirmation, and the review's limited disposition. The mobile review viewport was 390 × 844 CSS pixels.

The capture comparison supports the ready-state visual finish at the recorded sizes. It does not prove every device, screen-reader behavior, all dynamic text lengths, runtime contacts, or continuous production. State and keyboard statements below are source observations; runtime claims require their own recorded tests.

## Inherited system and route-local additions

`/collab/` uses local CSS aliases for matching incumbent values. These aliases do not import a theme controller or expand the global token namespaces. The global design files continue to describe `/multi/` and the independent routes; the new route's additions are deliberately recorded here and in its surface briefs.

| Use | Inherited evidence | `/collab/` application |
| --- | --- | --- |
| Night neutral hierarchy | `multi-night-page/panel/surface/field/ink/muted/edge` | Same values: `#152431`, `#1b2e3d`, `#233c4d`, `#142a3b`, `#edf5fa`, `#b3c8d6`, `#3d5669`. |
| Main action and feedback | Night action/ink/hover, success, shared danger | Mint start (`#9ae9df` / `#12363d`, hover `#b9f7ed`), success `#98eac8`, danger `#ffb5bf`. Written labels accompany state color. |
| Type and measurement | Chinese system stack, 14px body, 12px supporting labels, tabular numbers | Same stack and body rhythm (1.55). Supporting copy is at least 12px. Route-local headings are 18px in the header and 20px for the task; elapsed time is 30px. |
| Shapes and controls | 8px controls, 5px badges, 6px small HUD geometry, 10px HUD container | Same quiet corner vocabulary; progress uses a route-local 4px radius. Primary/run actions are at least 44px high; ordinary buttons 40px and the compact view button 36px. |
| Operation hierarchy | Tonal layers, native fields, separators, strong start action | A task panel with thin rules, three native availability checkboxes, BOM table, ownership, disclosure, and event log; no nested ornamental card grid. |
| Lighting and original assets | Legible original robot finishes, neutral task lights, ACES/sRGB | Original robot loaders/material setup is reused. Industrial architecture and lighting are code-native and local to this route. |

Local additions are functional industrial media and surface behavior, not a global recolor:

- The valve, pin, and sleeve use muted physical material colors (`#4d86ae`, `#bd4f49`, `#d4a831`). Their labeled BOM swatches are `#5a96c5`, `#c65b56`, and `#d9b13d`. These are categorical part identifiers for the new BOM, not replacements for `/multi/` red/blue/yellow object tokens; lit 3D materials and flat UI swatches are not color-calibrated equivalents.
- Amber UI `#f3ca86` matches the incumbent hangar accent value, but here it identifies the prescribed mobile route and tray docks. Physical amber (`#d5aa5e`) and mint staging paint (`#6cbbae`) are route-local materials. Written legend labels keep meaning available beyond color.
- Local industrial materials include scene/fog `#344956`, floor `#566a78`, aluminum framing `#8a9ba4`, dark structure `#344d5e`, table tops `#8297a2`, and rear wall `#455d6c`. They belong to the task set, not the interface palette or robot shells.
- Checked participant rows use `#2a4352`; running, complete, and failed badges use local background fills with inherited text/status colors. The source does not offer the `/multi/` night/hangar scene switcher on this route.

## Layout and responsive rules

The desktop page has a 60px header and a remaining-height scene/panel grid. The default panel is 354px wide, becomes 370px at widths of at least 1600px, and 326px at widths up to 1000px. The scene has a 450px minimum height. The panel scrolls independently; its sections use 18px separators and spacing. At 1440 × 900, run controls, availability, BOM, ownership, and the collapsed operation notes are visible. The collaboration log continues below the fold; this is intentional panel scrolling, not a claim that every log item fits initially.

At widths up to 780px, the route changes to normal vertical flow: scene first, then the panel. Scene height is 54svh with a 370px minimum and 620px maximum; controls stop using an independently scrolling panel. Section content is bounded to 600px, with 24px inter-section spacing. At widths up to 370px, header subtitle removal and smaller side insets preserve room for controls. The document minimum width is 280px; only the recorded 390px mobile capture was visually checked in this finish pass.

The operation notes use a native `details` disclosure. Its collapsed default keeps the task overview compact while retaining the task's physical limits. Long status and ownership text can wrap. The event log is bounded to 190px on desktop and 240px in the stacked layout. This route does not inherit `/multi/`'s 900px stacking breakpoint or 340px panel width; those remain scoped to their original route.

## States, interaction, and camera

Loading uses a polite live status, a real progress element, disabled task controls, and a bounded message overlay. Model/physics failure exposes written error feedback and a reload action. A ready snapshot enables native controls. Start, stop, reset, availability, elapsed time, BOM counts, ownership, and events read the engine snapshot; they do not invent successful placements.

The start label becomes “继续本次作业” after stopping. Busy states disable start and reset while permitting stop; availability changes are disabled during safe draining and pending commands. Waiting, awaiting material, safe draining, failure, and completion have distinct written labels/messages. Several busy states share the mint running tone, so color alone is not the state model. The current finite material-batch limitation is explicitly stated in the operation notes and awaiting-material message; waiting for new physical material does not count as continuous-production acceptance.

Buttons, links, checkboxes, the disclosure summary, and the main canvas have a visible 2px mint focus outline with 3px offset. The canvas is focusable and has accessible keyboard instructions. Arrow keys rotate; `+`/`=` and `-`/`_` zoom; Home restores the full workcell. Tab leaves the canvas, and Ctrl/Meta/Alt combinations pass through to the browser. Pointer orbit and wheel zoom remain available. The keyboard step respects orbit limits and clears residual damping before changing the camera.

The overview is fitted once after physical metadata arrives and on the explicit full-view action/Home. Bounds include stations, support tables, the mobile platform, robot extents, and the full prescribed R1 route. Simulation sync updates robot, part, and tray poses without following the moving robot or resetting the camera; resize updates renderer size and aspect only. Neutral task lights and procedural softbox reflections reveal the existing robot finishes. The renderer uses ACES filmic tone mapping (exposure 1.12), sRGB output, and a pixel ratio cap of 1.5. These are source parameters, not optical calibration.

Interface state transitions run for 150ms only when reduced motion is not requested. The physical demonstration remains dynamic; this CSS preference does not claim to pause simulation motion.

## Detector and finish-review disposition

The [recorded single detector output](validation/collab/design-detect.md) contains five `tiny-text` warnings (four 11px, one 10px) and eight advisories. The warnings were addressed in the existing one-batch fix by raising supporting text to 12px. The advisories covered the 3px progress radius, table divider `rgb(49,76,95)`, three BOM material swatches (`rgb(90,150,197)`, `rgb(198,91,86)`, `rgb(217,177,61)`), and 17/11/21px text sizes. Current source has a 4px progress radius and adjusted 18/12/20px text. No detector was rerun, so this is not a zero-finding detector report.

The retained `#314c5f` divider and muted BOM swatches are route-local additions. Their part-label and industrial-material purpose is documented above; they are not promoted into the global token system. They remain accepted local advisories for this scoped design extension, not silently fixed global drift.

The two finish-review findings were desktop task-overview density and missing keyboard camera operation. The same-size review recorded the last BOM row bottom at 737.125px and the robot rows bottom at 553.25px within the 900px desktop height. Keyboard coverage was confirmed from source and visible help; that review did not execute every key in a browser.

The fresh reviewer supplied the **ship** disposition only for the two fixes' scores. This documentation completion does not expand that review into a full-surface guarantee or runtime acceptance. The final 300-second wall-clock run, safe drain, physical placement checks, and repeated-order behavior remain owned by the runtime acceptance report.

## Assets, preservation, and limits

No shipping raster was added. Robot meshes are the existing originals; architecture, floor, fixtures, markings, and reflections are code-native resources. The PNGs listed above are review captures, not live-feed imagery or assets delivered to the page. No raster provenance ledger entry is needed for a new shipping image because there is none.

The documentation pass left `DESIGN.md` and `.impeccable/design.json` unchanged. Their SHA-256 values before and after this pass are:

```text
ce27f88e48399cb332cca9db1ccfc3a0a8dda89b4ed5633670ac89195235dfc6  DESIGN.md
e505829e981e1079b9d83e36f467e2e620bf5c642cd8ab94ba9e798083c58078  .impeccable/design.json
```

No diff was present for these two system files or the existing `dist/multi`, `dist/r1`, `dist/g1`, `dist/c5`, and `dist/Rebot_Arm_AGV` trees at the preservation check. That source check is not a new regression execution of those routes. Existing system drift and historical reports were not repaired or re-certified as a side effect.

The four inspected surface-source SHA-256 values were:

```text
95eb8d961435291a923f5c5b7ef60220ecb628448d3584b83b6c65ec3a772a41  dist/collab/index.html
48c90a698c435a04cc5a6c622d31d100e57229a72c865aba2aa3fb3557531c75  dist/collab/style.css
c307131de1d5a59b4f1258d1d50f8a6e667efff6de202e3dfb7f1b8189bd6de6  dist/collab/view.mjs
4e29cb4ee6983a495384ae25843b8f79fb9d2a541121ec7a8d5ffae651e00b00  dist/collab/app.mjs
```

There is no required global design-system change and no unresolved design-documentation blocker. Runtime acceptance remains explicitly open.
