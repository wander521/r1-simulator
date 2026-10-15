# Three-robot workcell extension

Mode: Operate. Ordinary extension of the existing reference workcell, not a new visual identity.

The user clarified that `dev_r1` is a branch, not a new repository. Preserve the
existing pure R1 `dev_r1` branch, and update the same repository's `main`: choose one or multiple
robots, start all selected robots together, and arrange R1, G1 and C5 in one row.
Each robot works on its own table and task objects; cross-robot collaboration or
shared-object manipulation is not requested. Existing individual routes remain.

## Direction contract

THESIS: One scene and one start action, with three independently simulated robot
stations and visibly separate per-robot progress.

OWN-WORLD: Preserve the existing dark-blue workcell, Chinese system typography,
cyan focus/selection, original robot finishes, flashing objects and colored
placement frames. Do not redesign the reference page.

STORY: Select one, two or all three robots; choose an object or stack task; start
the chosen set on the same fixed simulation tick; inspect a focused robot's
camera and measured contact/joint curves; stop or reset all safely.

FIRST VIEWPORT: Three full robots stand side by side with their grounded tables.
The familiar right panel begins with clear native robot checkboxes, followed by
one common task/start action and separate progress rows. An explicit focus
control changes the detail camera/telemetry without changing participation.
Narrow screens keep the complete shared scene above the same ordered controls.

FORM: Extend the incumbent scene-plus-panel composition. No concept tournament,
new identity, raster generation or global token change. New controls remain
keyboard accessible, with loading/error/empty/disabled states in Chinese.

FINISH: Native desktop and phone rendering, selection/drag/orbit/concurrent
grasp and recovery checks, exact-source evidence, independent finish review and
ordinary-extension documentation are required. Browser simulation is not real
robot calibration or hardware safety certification.

## Ordinary-extension documentation review

Source comparison on 2026-10-15 confirms the direction contract: one shared scene
holds the ordered R1/G1/C5 stations and separate grounded tables; native participant
checkboxes, one coordinated start, per-robot progress and independent observation
focus extend the existing controls. Chinese system typography, dark-blue fields,
cyan focus/selection, original robot material functions, pulsing task objects and
open-center colored placement frames retain the incumbent visual world.

The multi surface uses a 340px panel, 22px heading and 9px control corners versus
the global reference's 310px, 20px and 12px. These are surface-scoped density
adaptations for the added participation/progress controls, not new global tokens.
The existing 900px reflow and 58svh scene treatment are retained; 11px secondary
labels also occur in the incumbent G1 UI. No material source-level design repair
was identified. `PRODUCT.md` remains absent and no product context was invented.

`DESIGN.md` and `.impeccable/design.json` remain authoritative and unchanged. Their
SHA-256 values before and after this documentation-only review are respectively
`6e03acf9fb87241cb7705ccbe7cec964887e90153f4b7583245b6970b17d2356` and
`ea07b3d380c1821ed9ec11b22e433c17f15b41c61b1ef9f83fd430020279400e`.

Render evidence is bounded: `browser-final.json` records the root agent's actual
1280×720 desktop and 390×844 phone-width inspection and native interaction checks.
This documentation reviewer inspected source and reports, not rendered pixels.
The independent finish review remains `recapture` because the browser returned
in-memory PNGs without file export; no saved capture or independent pixel pass is
claimed. Real touch, full keyboard/screen-reader coverage, other browsers,
hardware certification and hosted deployment remain outside these observations.
