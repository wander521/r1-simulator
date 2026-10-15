# Three-robot workcell extension

Mode: Operate. Ordinary extension of the existing reference workcell, not a new visual identity.

## Round-pipe classification extension

2026-10-16 follow-up: initial positions must read as an irregular two-dimensional
scatter, not one/two fixed rows with millimetre jitter. Preserve deterministic
seeds, near-side source placement and physically validated gripper corridors.
Enlarge both color frames visibly (roughly twice the footprint), retaining the
existing drop slots and near frame edges so the enlargement cannot intrude into
the source area. Keep the incumbent materials, controls and responsive layout.
New layout statistics must reject the old row fixture; actual physical grasp,
release, speed and neighbor-contact gates remain unchanged.

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

## 二维散点与放大分类框：本轮系统继承核对

2026-10-16 本轮仅扩展圆管初始位置与红黄分类框尺寸，继续采用本简报的 Operate 模式和方向合同。对 `main` 的源码及差异核对确认：`dist/multi/pipe-layout.mjs` 将旧的一/两排微抖动改为可复现、间距受限的二维散点；R1 先清近侧外缘圆管，以保留回程通道。框在 x 方向放大至原来的 1.5 倍，z 方向约 1.32–1.4 倍（G1 受两框间距约束），面积约 1.99–2.1 倍；近侧 x 边界和原有四个落点槽位保留。现有圆柱材质、红黄语义、空心四边框、中文说明、控件和响应式规则继续继承；`dist/multi/index.html` 的本轮差异仅为缓存版本号。

全球设计未刷新：`DESIGN.md`（3315 字节，SHA-256 `6e03acf9fb87241cb7705ccbe7cec964887e90153f4b7583245b6970b17d2356`）、`.impeccable/design.json`（3181 字节，SHA-256 `ea07b3d380c1821ed9ec11b22e433c17f15b41c61b1ef9f83fd430020279400e`）及 `dist/multi/styles.css`（8124 字节，SHA-256 `ea466c85d41a4c3a7ad459276a60199206c76e95f9a8c88127ef38ec0f9e94cd`）均与版本库基线逐字节一致。`PRODUCT.md` 仍缺失，本轮未虚构产品背景，也未修复既有样式偏差或把场景几何尺寸提升为全球设计规则。

本轮已核对的静态证据为 `work/validation/multi-pipes/scatter-final.json`：300/300 组布局通过，三台机器人各有 100 个不同、可复现的位置点云；4 个反例均被拒绝，覆盖旧一/两排微抖动以及仅改颜色、编号或数组顺序的固定散点。`ui-final.json` 的界面/调度契约为 46/46，通过报告绑定的源码哈希均与当前文件一致；布局文件 SHA-256 为 `b96f05460ba4801433d4745a1ea5b683b65421bf28b05fc6734678ec0b6bbf1d`。本轮 `design-detector.json` 为 `[]`。这些结果分别证明布局统计、几何净距和源码契约，不证明抓取、触摸操作或像素效果。

视觉范围按主代理本轮实际内联查看的记录：1280×720 桌面覆盖三站聚焦俯视，390×844 手机宽度覆盖全景及聚焦视角，并检查空白处拖动和无横向溢出。此文档核对者只查看源码与报告，未查看这些像素；CUA 未提供截图文件导出，当前无保存的 desktop/mobile PNG，独立像素复核仍待重新捕获，不记为通过。

初次文档核对时本轮物理验收仍在进行，未沿用上节旧布局 `78a9eb02…` 的通过记录。随后当前 `b96f0546…` 源码冻结版本已重新完成 FULL 8 个场景、72/72 件和 300 组基础布局，35/35 验收器正反例、46/46 界面契约均通过；本轮原生浏览器另以种子 2092268180 完成三机 24/24 件。当前报告已替换为本轮证据，数值和 SHA 见 [本次冻结版本结果](PIPE-ACCEPTANCE.md#本次冻结版本的结果)。独立像素复核仍为 `recapture`，不能因数值通过而提升为视觉通过；文档补充不改变上述系统继承核对及源码指纹。
