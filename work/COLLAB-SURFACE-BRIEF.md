# 工业齐套协作场景

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
