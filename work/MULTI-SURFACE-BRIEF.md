# Three-robot workcell extension

Mode: Operate. Ordinary extension of the existing reference workcell, not a new visual identity.

## Reference standby posture follow-up

Match the supplied C5 photographs with a near-upright upper arm, a forward-bent
elbow and a downward-facing Dex1. R1 must rest both upper arms and forearms
naturally down at its sides, rather than akimbo. Apply these as real joint
targets on initial load, reset and automatic return, not display-only rotations.
Keep original robot assets, physics limits, gripper contacts and task paths.
The authored continuity is standby → existing preparation → grasp → standby;
no decorative motion or new visual identity is introduced. Verify held poses,
safe preparation/return, source-bound grasping regressions and desktop/mobile
rendering; photographs constrain silhouette, not hardware calibration.

The follow-up also covers R1's source Realsense cameras: retain only the native
URDF camera meshes, bind the feed to their actual source frames, and preserve a
natural forearm branch throughout preparation, grasp and return. A horizontal
waist turn may precede approach when needed for a complete pick-and-place path;
never turn the waist while carrying an object or substitute a camera remount for
correct arm kinematics. Native optical axes inferred from the source mesh are
not calibrated hardware extrinsics. Cache-version the affected module graph so
an updated entry cannot combine new joint poses with obsolete synthetic cameras.

The default panorama should frame the real idle silhouettes more closely, as
in the supplied screenshot. Retain the larger measured motion envelope while
preparing/executing, participant/focus semantics, complete feet/tables, label
clearance, and all incumbent responsive controls. Do not shrink model geometry
or conceal clipping to obtain a larger apparent robot.

## Compact workstation follow-up

Bring the three stations closer to the middle and make each robot more legible
in the panorama. Reduce display-only center spacing from 2.2 m to 1.6 m, without
rescaling any model, table, cylinder or local physics coordinate. Keep the
three-station row and all incumbent controls/style. Fit the real oblique camera
frustum to all eight bounds corners, retaining HUD/head/foot margins and the
named preparation-motion envelopes, rather than the old loose world-size
estimate. Verify table separation, measured visual-arm clearance, all nonempty
participant subsets, focus views and desktop/mobile framing. This is a display
refinement, not a new grasp or cross-world collision model.

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

## 三站收拢与放大取景：本轮系统继承核对

2026-10-16 本轮继续采用 Operate 模式和既有方向合同。源码差异仅将三站显示父组的相邻中心间距由 2.2m 收为 1.6m，以实际斜视相机基底拟合边界的八个角点，并纳入具名准备姿态、桌台、标签锚点与腕部相机外壳的取景保护；标签按自身高度上移至头部上方。模型、桌台、圆柱及本地物理坐标均未缩放。现有中文控件、深蓝/青色视觉、三站横排、900px 重排及 58svh 场景规则继续继承，未新增全局设计规则或修复既有样式偏差。

`DESIGN.md`、`.impeccable/design.json`、`dist/multi/styles.css` 与 `dist/multi/pipe-layout.mjs` 均保留原字节及上节 SHA-256；`PRODUCT.md` 仍缺失，本轮未补造产品背景。`work/validation/multi-framing/final.json` 为 59/59，通过 7 个非空参与组合 × 7 种视口的 49 组投影检查，99 项源码哈希与当前文件一致。四种桌面视口中，三台静止机器人投影高度的平均增幅为 18.5%–24.8%；两处桌台净距约为 0.87m、0.90m。这些是源码网格、具名姿态保守边界及透视投影证据，不覆盖任意手动姿态、连续运动或跨站物理碰撞。此次设计检测记录为 `[]`，不能替代像素观察。

`work/validation/multi-framing/browser-final.json` 记录主代理实际最终内联查看的 1280×720 桌面和 390×844 手机宽度，场景分别为 940×720 和 390×489.5px：三台机器人及落地桌台可见、标签位于头部上方，页面无横向溢出；手机聚焦 R1 后三项参与选择保持不变。此前视口调整后的截图尺寸不一致已通过调整视口后重新载入解决，临时模拟设置已清除。此文档核对者只核对源码与报告，未查看这些像素；截图仅内联返回、无保存的桌面/手机 PNG，独立像素复核仍为 `recapture`，不记为通过。

本轮显示调整未在本地重新执行 FULL 物理验收。原圆管 72 件报告仍为既有证据，文件 SHA-256 为 `d091550a8b04b2962d81752c128ebe5090e65363fd9b8d72b634dbb54f5f15ee`，其 30 项物理源码绑定仍全部一致；原单件/堆叠报告的 26/27 项绑定也保持一致。本次追加不把旧物理结果写成新运行，不扩大到真实触摸、屏幕阅读器、其它浏览器或硬件安全结论。

## 参考待命、原生相机与自然抓取：本轮系统继承核对

2026-10-16 后续轮次继续采用 Operate 模式与既有方向合同。C5 的真实六轴 HOME 目标变为 `[0,.10,1.25,-.4207963268,π/2,0]rad`，用于初始、复位和返回；R1 的实际 REST 肩肘目标使上臂、前臂自然下垂，并在原关节范围内通过第4/第6轴分配纵向转角、第5轴补偿腕部俯仰，使原生相机朝前。显示模型不另加姿态旋转；源模型、材质、物体尺寸和任务接触判定继续继承。

R1 腕部画面现绑定原 URDF 左右 `realsense_link`，移除旧的额外合成相机、支架和镜头。光学中心/轴来自原 STL 的几何推断，FOV 是软件视野约定，均非实机标定。抓取使用左右镜像自然腕部支路、向下20°的工具方向及连续笛卡尔段；保留原 `.003m/.035rad` 求解门槛、URDF/MJCF 限制和双指接触。完整取放候选在源腰部坐标系比较，接近前转腰；叠放再次换向时先收臂清障，携物途中不转腰。IK seed 限定为执行臂六轴，避免覆盖正在规划的躯干或另一臂。

全景静止时按当前真实轮廓取景，准备/执行时加入较大运动保护范围；1.6m 三站间距、完整脚底/桌台、参与选择与聚焦语义、900px 重排和 58svh 场景沿用。R1/C5/multi 入口把相关模块及旧别名统一到 `20261016-native-camera-3`，防止旧缓存把本轮关节目标与已删除的合成相机混装。以上是运动、原生资产绑定与表现场景的扩展，无新增视觉身份或全局 token。

文档核对已查看当前实现、`DESIGN.md`、sidecar 及本简报。`DESIGN.md`、`.impeccable/design.json` 和 `dist/multi/styles.css` 保留原字节，SHA-256 仍分别为 `6e03acf9fb87241cb7705ccbe7cec964887e90153f4b7583245b6970b17d2356`、`ea07b3d380c1821ed9ec11b22e433c17f15b41c61b1ef9f83fd430020279400e`、`ea466c85d41a4c3a7ad459276a60199206c76e95f9a8c88127ef38ec0f9e94cd`。sidecar 陈旧提示仅作记录，本轮不修复；`PRODUCT.md` 仍缺失，不补造产品背景。

本轮验证状态以 [姿态与原生相机验收](POSE-AND-VISION-ACCEPTANCE.md) 为准。上文散点、旧取景、72件物理及浏览器24件记录均是各自注明轮次的历史运行，不自动覆盖本次 R1 路径变化。C5 已有 1280×720 桌面、390×844 手机宽度、8项控件及一次蓝块归位的实际浏览器观察；这些操作发生在未改动的 C5 核心，随后 v2 缓存开启重载另外确认 HOME 与双尺寸渲染，最终 v3 仅以缓存开启手机宽度重载确认模块 URL 和实际 HOME，不能改写为 v3 的完整交互或双尺寸视觉重跑。

R1 本轮自然待命14项、自然路径4项、转腰6项、原生相机16项及缓存13项已通过并绑定冻结源码；自然路径含10条规划的6090帧源 STL 几何和两臂实际蓝块抓放2036帧，原桌板无采样相交。当前归位10条及叠放/恢复6条均通过，证据核验27/27包含历史证据完整性，不代表旧36条本轮重跑。三机本轮新执行的归位8个场景/12次真实抓取、叠放2个场景/3座塔/9件均通过。随后本轮 FULL 8个场景/72件及300组基础布局亦完成，30项物理源码绑定全匹配。主代理最终 v3 原生浏览器另完成三机蓝块归位、R1左臂单独归位，以及种子 `609029927` 的圆管24/24件、各机稳定3.01秒；后者R1交替用臂。浏览器24件与 FULL 数值回归分别记录，不互相替代。

当前正式取景为113/113项、98组静止/运动投影，99项绑定均匹配。四种桌面视口下，相同当前待命几何使用新静止取景，相比先前总用运动边界的相机，三机平均投影高度增加17.7%–19.7%；这不是模型缩放，也没有用不同新旧姿态作大小对比。`work/validation/pose-vision/browser-final.json` 是本轮新记录，含12项生产源码绑定、1280×720桌面和390×844手机全页实际内联观察：三机/桌台/标签完整可见，手机无横向溢出，聚焦保持参与选择。旧取景浏览器报告已显式标为历史并指向本轮记录，未重绑旧证据。本轮静态设计检测为 `[]`。

原生观察包含转腰和早期接近帧、双路相机及非零接触曲线；一次准备阶段的短暂选择器/CDP超时后，实际任务继续完成。短暂闭合状态未捕获图像，因此不作闭合图像遮挡或取景批准。独立源码收尾8项通过、无P0/P1/P2，最终13项生产源码和18份报告绑定均匹配，状态为 `source-review-clear-final-evidence-confirmed`；不提升为独立像素结论。

文档核对者仅核对源码与报告，没有独立查看渲染像素。截图由原生浏览器内联返回，未导出文件；独立像素复核仍为 `recapture`。R1 动态接触模型仍只含指垫；原 STL 对桌台/身体的采样几何是有限姿态/时间点检查，不能证明全身自碰撞、连续时间或实机安全。实体触控、其它浏览器、完整辅助技术覆盖和硬件认证仍不在本次证据范围。
