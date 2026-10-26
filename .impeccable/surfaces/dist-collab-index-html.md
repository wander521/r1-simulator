---
version: 1
slug: "dist-collab-index-html"
primary_target: "dist/collab/index.html"
related_targets: ["dist/collab/style.css","dist/collab/view.mjs","dist/collab/app.mjs"]
---

# 工业齐套协作场景

## Loading performance · 2026-10-26

Impeccable optimize workflow preserves the rendered world and native loading,
retry and keyboard controls. Lossless STL transport compression reduces the
72-file source set from 51.82MB to 20.50MB, while two temporary workers decode
meshes and a lightweight visual config trims the main entry graph. First-ready
observation improved from 3.37s to 2.84s in the measured local cold contexts;
subsequent internal marks were 2.44s cold and 2.13s warm. Physics initialization
remains the critical path. Normal load, fallback, mobile appearance and short
start/orbit/stop smoke checks passed. Narrow polish review found no rendered UI
regression; screenshots and qualifications are recorded in
`work/COLLAB-LOAD-PERFORMANCE.md`. No global visual tokens or shipping raster
assets changed. Compressed files are byte-identical transport copies, not LODs.

## Rack visibility in the user's browser · 2026-10-26

The earlier geometry-only fix did not resolve the user's actual browser.
Reproduced visible shelves without backing in the IAB, while the independent
browser rendered both support meshes. The entry/render modules were versioned
but engine → simulation worker → model → fixture imports were not. Versioning
that entire chain restored the backing and floor support in the same IAB and
same front inspection angle. A metadata completeness guard now rejects a bin
scene missing either required support instead of silently rendering shelves.
Read-only rack mesh diagnostics and canvas data attributes expose actual mesh
creation; the user's canvas now reports both names and the current model version.

Actual user-browser proof: `output/playwright/rack-restored-user-browser.png`.
This supersedes the previous claim that shelf geometry alone fixed the missing
board. No further shape, robot, target or physics change in this correction.

## Rack support correction · 2026-10-26

The backing was present in live metadata and rendering, but the shelf rear
ended at x=.610 while the backing began at x=.652 (a 42mm structural gap).
Extend each shelf rear to x=.656, preserving its x=.400 pickup edge, height,
bin position and low 1.04m backing. Rendering and MuJoCo share those dimensions.
Impeccable refinement preserves the world while using existing aluminum and
worktop materials to make shelves and backing readable against the room.

Desktop/mobile captures `output/playwright/collab-rack-desktop.png` and
`output/playwright/collab-rack-mobile.png` inspected: visible support structure,
no horizontal overflow or initialization error. 58 tests passed, including the
new shelf-to-backing contact geometry contract, G1 entry collision checks and
two-second physical settling. No full five-minute grasp run is claimed for this
change. Detector returned only the same six incumbent advisories.

## Industrial training hall and downward task light · 2026-10-26

Route-local refinement using Impeccable's polish workflow: retain the steel-blue
industrial world, robot materials, control panel and physical workcell. Add a
display-only rear wall with modular metal seams, observation bays, cable tray,
perimeter service cabinets and slab joints. The rear banner reads exactly
“一脑多型具身训练场”. No new generated raster is shipped; the Chinese sign uses
a runtime canvas texture, tracked with the scene's geometry/material disposal.

Three fixture-mounted neutral spotlights point vertically down at G1, C5 and
the R1 transport aisle. Ambient/fill illumination is reduced for subject
separation; the existing single shadow map supplies overhead shadows, avoiding
three extra shadow passes. New architecture is outside the working area and
does not enter collision, motion planning or inventory. Explicit panorama
framing includes banner corners; operation progress still never moves the
user's camera. Reverse-side wall planes remain open for orbit inspection.

Verified desktop 1440×900 and mobile 390×844 with full banner and no horizontal
overflow. Captures: `output/playwright/collab-hall-final-desktop.png` and
`output/playwright/collab-hall-final-mobile.png`. All three robots loaded with
null initialization error. Keyboard orbit, explicit panorama, start and stop
were exercised; during a 12-second running smoke test the camera was unchanged.
All 57 existing bin tests, syntax checks and `git diff --check` passed. This is
not a new five-minute grasp or 20-round endurance certification. An existing
favicon 404 remains unrelated. Detector findings were only the six incumbent
advisories; global DESIGN.md/sidecar drift remains out of scope. No GitHub push.

## Category sorting refinement · 2026-10-26

The user's new task supersedes per-bin kitting on `/collab/`: C5 sorts red
cylinders, yellow cylinders, blue blocks and green blocks into four dedicated
regions, each with two distinct supported positions. Their original semantic
colors remain, with matching outlines and Chinese labels. Cabinet service-sign
glyph size is doubled (48→96 canvas pixels on the same planes). The rack backing
now ends at 1.04m, just above its top shelf, using one shared visual/physical
definition. Target simulation rate rises from 2× to 2.4×; the admission window
stays at 300 real seconds. Layout, palette and other routes are preserved.

Scoped rendered review: desktop 1440×900 and mobile 390×844, no horizontal
overflow; low rack and larger signage visible. Captures and verification are
recorded in `work/COLLAB-CATEGORY-SORT.md`. 57 tests and all 15 checks of one
complete browser run passed: 8 sorted parts, 4 returned bins, all robots home.
A separate concurrent offline diagnostic timed out at 480s without a station
fault and is not counted as a pass. This does not complete the old 20-run soak.

Global DESIGN.md/sidecar drift remains outside scope; no new raster assets or
changes to the global visual world.

## Line refinement documentation outcome · 2026-10-23

The user's request to make the line more coherent and less fragmented is a
route-local refinement of the approved industrial world. The built scene in
`dist/collab/view.mjs`, `cell-fixtures.mjs` and `bin-model.mjs` now uses steel-blue
cabinet bodies beneath the worktops, local grounding plinths, and one shared C5
plinth. A physical linking worktop joins C5's input and output surfaces. Cabinet
and plinth geometry is shared with the collision model; docking coordinates,
robot origins and the R1 transport aisle retain their established positions.

The scene keeps aluminum (`#879ba7`), dark steel (`#263d4b`), steel-blue worktops
(`#627b89`), slate floor (`#455a67`), amber registration (`#d5aa5e`) and mint finished
positions (`#6cbbae`). One suspended neutral task-light spine replaces scattered
fixtures. Open architectural cutaway removes the decorative wall that obscured
the work; the panorama fits from the opposite three-quarter direction. Cabinets
replace redundant table legs in this bin scene, service labels sit flush on the
cabinet faces, and a flush status lamp replaces its pole. Two short registration
strokes identify each bin position; the aisle keeps two continuous edge marks
without transverse stripes. These are scene-specific decisions, not global rules.

The UI retains the incumbent Chinese system font, 14px body, 12px supporting copy,
15px section titles, 18px header, 20px run title and 30px tabular timer. Native mint
actions, visible focus, quiet separators and the scene-first mobile stack remain.
The added material key matches actual parts: red and yellow cylinders, blue and
green blocks. The visible speed note states the requested 2× simulation target
and retains a real 300-second admission window, including the low-performance
device qualification.

Fresh finish evidence supplied to this documentation handoff: **ship**, scoped
to the camera-occlusion and cabinet refinements. Current captures are
`output/playwright/collab-refine-desktop.png` and
`output/playwright/collab-refine-mobile.png`. The separate physical run completed
four bins, eight parts, four empty-bin returns and all robots home; 54 tests
passed. A 114.9-second partial browser run reported no errors, long tasks or camera
jumps. This does not establish full five-minute browser acceptance. The reported
C5 fault was not reproduced, so no root-cause fix is claimed. This documentation
pass inspected source and existing design context, ran no browser or new audit,
and introduced no shipping raster.

`DESIGN.md` and `.impeccable/design.json` remain unchanged. Pre-existing drift
(DESIGN.md newer than the sidecar, unset `buildPath`) is reported, not repaired.
The detector's zero primary findings and six advisories are recorded as supplied:
the incumbent 4px progress radius and border, plus four explicitly requested part
colors. The incumbent advisories are not promoted into new design-system rules.

## Approved replacement workflow · 2026-10-23

The user replaced the serial-tray concept below. This section supersedes its
workflow and acceptance claims; the established industrial palette remains.

THESIS: G1 fetches full bins from a material wall; R1 transports the same bins
and returns empties; C5 packs their loose parts. Independent work, local handoffs.

OWN-WORLD: Preserve steel-blue structure, neutral overhead task light, amber
buffer/route markings, mint controls, Chinese system type and native buttons.

STORY: Start a five-minute admission window; watch different batches overlap,
pause and revalidate on resume, then drain and return home. Each pick/place
returns to ready, not home. Finite stock is visible and never replenished invisibly.

FIRST VIEWPORT: Shared physical cell at left; right panel keeps run controls,
three independent station rows, physical bin inventory and completion counts.
Mobile stacks scene above controls. Camera remains user-owned through operations.

FORM: Code-led extension of the approved Operate surface; no new visual-world
tournament. Material-wall shelves, carriers and fixture geometry match physics.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance

### Bin workflow documentation outcome · 2026-10-23

Ordinary extension of the established industrial Operate surface; documentation
is complete without a global visual-system change. Compared the finished
`dist/collab/index.html`, `style.css`, `app.mjs` and `view.mjs` with `PRODUCT.md`,
the incumbent `DESIGN.md` tokens/rules and the approved contract above. Inspected
the supplied 1440 × 900 desktop capture and 390px-wide full-page mobile capture:
`.impeccable/review/collab/bin-desktop-verdict.png` and
`.impeccable/review/collab/bin-mobile-verdict.png`.

The interface retains the incumbent night page/panel/field/ink palette, mint
action, Chinese system font, tabular counts, quiet separators, native buttons,
8px control corners and visible focus treatment. The steel-blue physical set,
neutral lighting and original robot finishes remain the established collab world.
The compact desktop control column and scene-first, normal-flow mobile panel
remain route-local composition; they do not change `/multi/` or reference routes.

New route-local component decisions:

- Three read-only station rows present G1 full-bin picking, R1 full-bin delivery
  and empty-bin return, and C5 two-part packing. Active/fault states combine written
  operation and phase with color; they are no longer participant checkboxes.
- The inventory table identifies four finite physical bins, current station and
  packed count out of two. Completion totals distinguish products, parts and
  returned bins. The view consumes bin/part poses and physical geometry metadata.
- Status copy distinguishes local handoff waiting, ready after each operation,
  transport hold while R1 carries a bin, paused path revalidation, draining after
  the admission deadline and home only after in-flight work finishes. These are
  documented UI semantics, not proof that every physical transition succeeded.
- The mint finished-position and amber handoff/route legend now shares the
  near-opaque HUD backing and restrained corners used by adjacent scene labels.
  This is the resolved P2 legibility finding, visible in both verdict captures.
- Camera help and explicit panorama action remain available independently of
  work state; the live scene note names concurrently active operations.

Fresh review disposition supplied to this handoff: **ship**, scoped to resolving
the P2 legend-backing finding. The captures show the loaded idle surface; this
documentation comparison adds no runtime, physical, five-minute or hardware
acceptance claim and ran no further audit. No new shipping raster was introduced;
robot meshes are reused and scene structures are code-native geometry.

`DESIGN.md` and `.impeccable/design.json` remain unchanged. Pre-existing context
drift was reported upstream: `DESIGN.md` is newer than its sidecar and `buildPath`
is unset. Neither was repaired by this route documentation pass.

Historical sections below describe the superseded implementation and prior
captures only, not acceptance of the new bin cell. No new raster assets needed.

Scope: new `/collab/` route only; existing routes, physical sources and their evidence stay intact.
Mode: Operate. The user approved an industrial kitting line with separated G1/C5 stations and
R1 low-speed mobile supply on a prescribed physical route. The same shared tray docks at the stations;
R1 supplies parts into the same product tray's unrestrained staging position. That tray carries the
same free part to C5 for slot placement, or to G1 for reachable packing relief. R1 is the necessary
supply chain; arbitrary single-robot completion of the whole order is not supported.
The workcell runs for a real 300 seconds.
The order is illustrative: one valve-body block, two pins and two sleeves. No learned-model or hardware claims.

## Direction contract

THESIS: One visible product and one tray move through separated workstations in one shared physical
cell. Refuse disconnected simulations and counters that advance without actual supported placements.

OWN-WORLD: Expand the existing laboratory UI with an industrial production variant: steel-blue shell,
neutral white task lights, aluminum framing, slate floor, amber work-zone markings, mint run action.
Chinese system sans, native controls, tabular measured data and quiet section separators; no neon HUD costume.

STORY: Inspect the loaded line, start a five-minute shift, follow R1's loaded/empty base travel,
stop-before-manipulation interlock, physical part handoffs into a marked tray staging position and
tray docking. Temporarily withdraw C5 and observe the same tray carry the same part to G1 for
reachable packing relief, then inspect actual completed trays.

FIRST VIEWPORT: The shared 3D cell owns roughly three quarters of desktop width; a compact right panel
contains start/stop/reset, real wall-clock elapsed time, current BOM and three availability rows.
Industrial tables, the connected R1 travel platform, prescribed route and tray docks make the
separation legible. An unobtrusive operation label identifies the live task without following the robot.
On phones the scene precedes one normal-flow control panel; neither width nor control text clips.

FORM: Code-led task-first operation screen. User-pinned industrial world and approved workcell concept;
no open direction tournament or generated mockup. Signature interaction is a real task-owner handoff,
reflected by the same physical part and its operation log, not decorative camera movement.

FINISH: bounded design review and route documentation complete on 2026-10-23. Fresh UI review verdict:
ship only for the two resolved findings: task overview at 1440 × 900 and source-defined keyboard camera
controls. This is not a blanket full-surface/runtime pass. The finish record is
`work/COLLAB-DESIGN-AUDIT.md`; captures are `.impeccable/review/collab/desktop.png` and `mobile.png`.
This ordinary extension inherits the incumbent night interface values. Industrial part materials,
responsive thresholds, state presentation and camera rules are route-local additions documented in
that audit. Existing `DESIGN.md` and `.impeccable/design.json` are preserved, not refreshed. No new
shipping raster exists; original robot meshes are reused. The real 300-second browser run and final
continuous-order acceptance remain separate and pending; the design verdict does not certify them.

## Execution truth

One MuJoCo model/data, common part bodies, actual contacts and support. No grasp attachment or object
teleport. G1 feet and C5 pedestal remain fixed. R1 mount movement is read from its real MuJoCo mount
body pose and driven by ideal planar actuators on the prescribed route, not visual interpolation,
wheel-traction simulation, arbitrary navigation or a learned policy. Conveyor tray movement is physical.
The mint staging ring is paint on each product tray, not an attachment or constraint. R1 hands parts
to that tray, not to a fixed C5 tabletop. Packing relief uses physical tray transport between reachable
stations, not copies or impossible cross-workcell reaches. No belt extension is part of this change.
R1 stops before arm manipulation; the camera never follows or resets during travel. Loading is excluded
from the 300-second wall-clock shift; planning waits count. At deadline stop admitting work and drain
the current safe operation. Paused/unavailable robots do not cause a global reset.

No new raster assets are required: the robot meshes are existing originals, and scene structures are
code-native Three.js geometry with explicit physical counterparts. Five-minute browser execution and
physical validity are separate required tests, not interchangeable with accelerated simulated time.
