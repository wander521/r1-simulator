# Three-robot workcell extension

Mode: Operate. Ordinary extension of the existing reference workcell, not a new visual identity.

## Round-pipe classification extension

The additional unified task is 圆管分类. Each independent table has four red and
four yellow cylinders in randomized, nonoverlapping reachable positions on the
robot-near side. Two empty-center destination frames occupy the farther side;
each has four distinct resting slots. Selected robots begin together and each
finishes all eight items. R1/G1 automatically alternate right/left arms after
each safe return; the single-arm C5 uses its sole arm. No manual arm/object
choice is shown for this task. Preserve other tasks and single-robot routes.
The task picker, compact written color/count/near-far explanation, idle-only
randomize action and per-robot item counts extend the incumbent control panel.
Stop must cancel the remaining queue. Completion requires actual grasp/release
and final stable table support for every item, not an animated counter.

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

## 圆管分类：系统继承核对

2026-10-16 源码核对：圆管分类沿用现有任务下拉框、说明文字、随机/复位按钮和各机器人进度行；近侧红黄各四个圆柱、远侧同色框及自动用臂均有中文说明，任务内隐藏手动执行臂与单物体选项。未新增视觉体系，未发现本次新增的实质设计漂移。`dist/multi/styles.css` 与版本库基线一致；`DESIGN.md` 和 `.impeccable/design.json` 逐字节保留，SHA-256 仍与上节记录一致。

主代理本轮原生查看覆盖 1280×720 桌面和 390×844 手机宽度：桌面开始按钮位于 y=647–689，手机场景高 489.5px，两种尺寸均无横向溢出；三个模型、每桌八个圆柱和两个分类框可见，手动执行臂隐藏。此文档核对者仅核对源码与报告，未独立查看像素；浏览器未提供截图文件导出，不据此声称独立视觉通过。

`work/validation/multi-pipes/design-detector.json` 中的 11px 次要文字警告及调色板/圆角建议来自沿用的样式，作为既有偏差记录，本次未修复或提升为全局规则。上述源码核对时物理验收仍在进行；随后 `physical-final.json` 完成 FULL 8 个场景、72/72 件及 300 组布局检查，`sourceFrozen: true`；界面/调度契约 46/46、验收器正反例 35/35。原生浏览器另以种子 2577096563 完成三机 24/24 件，详见[最新验收结果](PIPE-ACCEPTANCE.md#本次冻结版本的结果)。独立像素复核仍为 `recapture`，原因是无截图文件导出，未记为视觉通过；`DESIGN.md`、sidecar 和样式仍保持原字节。
