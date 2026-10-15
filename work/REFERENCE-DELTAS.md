# arm 参考页实现契约与 R1 增量适配

审计对象为本仓库 `dist/Rebot_Arm_AGV/?variant=arm`，不包含 AGV、爆炸时间线和零件探索。以下是对当前打包源码和 XML 的静态观察；实现存在与运行验收通过分开记录。

## 可核查的源码位置

- `dist/Rebot_Arm_AGV/assets/index-BXF1-FJz.js`：`nh()` 背景；`th()` 地面贴图；`ih()` 环境反射；`sh()` Three.js 场景与渲染器；`Qu()` MJCF 材质到 Three.js 材质；`$h()` 自动任务；其前方 `Ph/Lh/L2/qh/jh/Yh` 常量；`N4()` 物理控制；`Kh()` 接触采样；`Zh()` 指标与曲线。
- `dist/Rebot_Arm_AGV/models/rs_grasp_scene.xml`：地面、台面、三物体、同色框、全局相机。
- `dist/Rebot_Arm_AGV/models/rs_arm.xml`：六关节电机、夹爪耦合、夹爪分块碰撞、腕部相机及其支架。
- `dist/r1/r1-app.mjs`、`grasp-controller.mjs`、`grasp-kinematics.mjs` 与 `models/r1-contact-scene.xml`：R1 现有实现。本文引用的函数名对应仓库包含的确定版本，不依赖线上页面更新。

## 本次场景与相机适配

新增 `dist/r1/r1-workcell.mjs`，所有位姿为 Three.js Y-up。参考 XML/任务常量使用 MuJoCo Z-up，二者位置映射为 `(x,y,z)_MJ → (x,z,-y)_Three`。

| 项目 | 参考页观察 | 新模块实现 / 必要适配 |
| --- | --- | --- |
| 背景 | `nh()` 1600×900 画布；渐变 `#010319 → #030629 → #06154b → #03153d → #01030f`；地平线位于高度 `.73`；五道蓝色竖光的横坐标比例为 `.035/.18/.5/.82/.965` | 直接复现同一画布算法、光束颜色/宽度、扫描横线、暗角；作为场景背景，随视口铺满，与参考一致 |
| 地面 | `th()` 1024² 画布，深蓝底、每纹理十格、青蓝外框；2m 平面上 repeat=5，格距4cm | 同算法；R1 地面8m，repeat=20，保留4cm格距 |
| 环境反射 | `ih()` 蓝白条形灯的1024×512等距柱状贴图，`sh()` 经 PMREM 用作环境 | 复现原画布并设置 EquirectangularReflectionMapping，由 Three.js 环境预滤处理 |
| 色调与输出 | `sh()` ACESFilmic，exposure `1.02`，sRGB；主视图软阴影2048² | `configureRenderer()` 设置相同色调与输出；主光设置2048²阴影 |
| 直接灯光 | Ambient `#6f9bd4/.26`，Hemisphere `#d9f6ff,#021026/.48`；Directional 白 `#f2fbff/1`、青 `#8bdcff/.38`、蓝 `#286fff/.62`；三盏 RectAreaLight 为 `#eaf8ff/2`、`#10cfff/2.8`、`#245cff/1.9` | 全部八盏真实灯光。面积灯使用参考包中同一组64×64 LTC查找表，Float32无损编码及同算法HalfFloat转换；按台面归一化映射面积灯的位置/目标，保留颜色和强度 |
| 台面 | XML 尺寸 `.50×.90×.03m`，高度 `.10m`；`Qu()` 将 rs_table 线性颜色 `[.34,.37,.36]` 乘 `.58` | 保留已适配 R1 的 `.72×.76×.065m`、现有高度与 x=.78 中心，不移动任务物；沿用参考颜色转换、金属度 `.12`、粗糙度 `.55`；四腿落地 |
| 全局俯视 | XML overhead_rgb：FOV70°，台面上方 `.68m`，原位置偏台面中心4cm | `configureOverheadCamera()` FOV70°，置于 R1 台面中心上方 `.68m`，Y-up与台面正交 |
| 腕部光学 | `rs_arm.xml` wrist_rgb FOV `62.82°`，`xyaxes="0 -1 0 0.5 0 .866025404"`；30°向下；随夹爪坐标变换更新 | 夹爪局部 +X 为前方，抓取时 +Z 朝下。镜头偏移 `[-.035,0,-.065]m`，局部视线 `[cos30°,0,sin30°]`、up `[-sin30°,0,-cos30°]`；镜头在夹爪上方65mm，观察 TCP/指尖/物体/台面 |
| 相机模型 | XML D405 盒体 `.042×.042×.023m`，两镜头半径5.5mm，含支架；模型可隐藏 | 同尺寸盒体/面板/双镜头，R1深灰支架；随当前执行臂 wrist frame 更新。模型在layer1，腕部相机只渲染layer0以防看见自身外壳；全局/主相机需要启用layer1 |

接口：

```js
const cell = createWorkcell(scene, {THREE, TABLE_HEIGHT, tableCenter: .78});
configureRenderer(mainRenderer, {THREE});
feedRenderers.forEach(renderer => configureRenderer(renderer, {THREE}));
mainCamera.layers.enable(1);
configureOverheadCamera(overheadCamera, {THREE, TABLE_HEIGHT, tableCenter: .78});
const wrist = createWristVision({scene, THREE, camera: wristCamera});
// 每次姿态更新后：
wrist.update(robot.getObjectByName(`${arm}_gripper_link`));
wrist.setVisible(showCameraModel);
```

`createWorkcell` 返回 `group/table/floor/grid/lights/dispose`；`grid` 指使用网格贴图的地面，不另叠一套有深度冲突的线段。`createWristVision` 返回 `model/update/setVisible/visible/dispose`，不修改机器人关节或碰撞。

面积灯补充：`r1-area-light-textures.mjs` 从已包含的参考包 `Hi.init()` 提取两张64×64 RGBA表，以Float32字节无损base64编码（约176KB模块）；`initAreaLightTextures()` 设置 `THREE.UniformsLib.LTC_FLOAT_1/2` 及 `LTC_HALF_1/2`，与参考 `Uu.init()` 对应，支持本仓库Three.js r180的float/half纹理分支。原面积灯 `position/target/size` 为白光 `[.28,-.58,1.28] / [.25,0,.16] / 1.6×1.15`、青光 `[-.58,.2,.68] / [.23,0,.18] / .75×1.05`、蓝光 `[.86,.32,.62] / [.25,0,.18] / .7×.9`。映射以参考台面中心 `[.38,0,.10]` 为基准，x与高度差乘 `.72/.50`、深度差乘 `.76/.90` 后移到R1台面中心 `[.78,TABLE_HEIGHT,0]`；发光面积的宽高按`.72/.50`等比调整，保证台面旁的软箱尺寸与距离比例一致。

本次静态数值检查：在真实抓取使用的水平夹爪旋转下，TCP `[.068,0,0]` 投影到腕部图像 NDC `[-0,-.06448,.87003]`，在画面中央附近；光轴世界方向约 `[.866,-.5,0]`，朝前下方；光学位置比腕部高65mm。模块语法检查通过。上述检查证明变换计算，不代替页面截图、遮挡和动态画面的最终验收。

## 自动归位与叠放的精确参考契约

### 任务对象与目的地

| 对象 | XML 完整尺寸 / 质量 | 初始中心（MuJoCo） | 同色框中心（MuJoCo） | 叠放中心（MuJoCo） |
| --- | --- | --- | --- | --- |
| 红色方块 | 45×45×45mm /45g | `[.24,-.30,.1225]` | `[.49,-.16,.1225]` | `[.3,.3,.1605]` |
| 蓝色长方块 | 52×32×38mm /50g | `[.34,.06,.119]` | `[.49,0,.119]` | `[.3,.3,.119]` |
| 黄色圆柱 | 半径20mm、高52mm /42g | `[.33,-.20,.126]` | `[.49,.16,.126]` | `[.3,.294,.209]` |

`$h.V0()` 开始叠放时不重置物体、不把物体恢复到初始位置。它设置队列 `[red,yellow]` 并先调用 `F(blue, L2.blue, stack)`。每次进入 `F` 都读取当前 `data.xpos` 作为源位置。准确顺序为 **蓝底 → 红方块 → 黄圆柱**。支撑体 `M` 依次必须为桌面、蓝块、红块。黄色相对底层横向偏6mm是源码常量，不应误读为位置误差。

### 状态阶段与判定

- 归位：preparing → aligning → opening → approach → descend → closing → lift → transfer → place → release → retreat → settling → complete。
- 叠放：opening → approach → descend → closing → lift → transfer → place → release → retreat → depart → settling；若还有物体则 centering 后继续；最终进入 celebrate-centering → celebrate → complete。
- approach 目标为源中心高100mm；descend 目标为源中心高3mm；闭合命令0，保持至少 `.8s` 后抬升；lift 为源中心高130mm。
- 实际物体必须比源中心高至少35mm，否则 `grasp-missed`。`carryOffset` 只用于计算放置 TCP 目标补偿，不是物体附着约束。
- `W0` 每次 `servoStep` 使用仿真时间差（夹到 `.001… .035s`），TCP 距目标小于6mm为到位；通常阶段超时 `4.5s`。
- place 必须满足物体XY距目标<6mm、相对目标Z在 `(-18,+30)mm`、与正确支撑体有实际接触、物体线速度<`.035m/s`，连续 `.1s` 才松爪。
- 松爪至少 `.35s` 且开度>.038m，或超时1.1s；retreat 高于目的物中心 `.11m`（归位）或 `.12m`（叠放）。
- 叠放 depart 再横移 `[-.05,-.08]m`（MuJoCo平面）避免机械臂停在塔顶。
- settling 为至少 `.35s` 且线速度<`.0175m/s`，或1.2s超时。完整目标要求释放后稳定至少3秒，验收应使用更严格的3秒，不能把参考的短超时当作证明。

### 完成与失败反应

反应中心 `Gh={joint1:0,joint2:.5515,joint3:.2688,joint4:.8,joint5:.0471,joint6:0}`。面向观察者的 joint1 由 `X0()` 计算 `clamp(-atan2(observer.y-workspaceY,observer.x-workspaceX))`；在 Three.js Y-up 中输入参考平面位置应为 `(camera.x,-camera.z)`。R1 对应关节及限位需要适配自身结构，不能直接套用 RS 角度。

进入反应先中心归位，全部臂关节误差<`.07rad`且夹爪误差<`.006m`，或超时1.8s后开始各步。每步相对中心偏移、限位截断；转换需达到保持时间且收敛，或达到该步timeout。

| 反应 | 相对中心目标 | hold / timeout（秒） |
| --- | --- | --- |
| happy1 | joint6=-.55 | .48 /1 |
| happy2 | joint6=+.55 | .48 /1 |
| happy3 | 回中心 | .35 /.85 |
| happy4 | joint5=+.36 | .65 /1.15 |
| happy5 | 回中心、夹爪闭合0 | .4 /1 |
| happy6 | 回中心、夹爪张开.046m | .48 /1.1 |
| shy1 | joint2=-.1、joint3=+.06、joint5=+.34 | .75 /1.25 |
| shy2 | joint2=-.14、joint3=+.1、joint5=+.48 | .75 /1.25 |
| shy3 | 回中心 | .5 /1 |

叠放全部成功后才执行happy，结束才置 `running=false / complete / stack-complete`。叠放任何 `b0(reason)` 先执行shy，保留失败原因，反应结束才进入failed。普通归位失败不执行shy，立即开爪、failed。`cancel()` 清空队列、反应、补偿和目标，将running=false、stage=idle、message=cancelled；停止不执行失败反应。`reset()` 清空任务状态；页面另调用物理reset恢复场景，不能以取消替代复位。

## 升级前审计记录与原始目标

以下是升级前记录，不代表当前实现状态。原始目标曾包含左右臂×三种物体各连续3次、完整三物体叠放连续3次且松爪稳定3秒、编辑/不可达/停止/复位/手动TCP/夹爪与关节重复操作、真实关节力矩与目标/实测误差、中英日与触控、同条件参考页/R1截图及操作录像。本轮收窄后的范围、实际完成证据与未验证边界见 [当前验收报告](ACCEPTANCE.md)，其中不包含三语言扩展或真实触屏设备验收。

升级前的 `r1-contact-scene.xml` 只模拟自由腕部和四个滑动指垫，没有六轴臂的动力学关节与电机；该模型的指尖力、腕部位置误差不能冒充六关节力矩或RMS角误差。参考 `N4` 使用全臂MJCF，`qfrc_bias + kp*(target-qpos)-kd*qvel` 驱动力矩，臂kp `[80,100,100,35,25,18]`、kd `[8,10,10,4,3,2.5]`、力矩限幅 `[36,36,36,14,14,14] N·m`。当前R1页面已切换到 `r1-dynamic-scene.xml` 与 `r1-dynamics.mjs` 的完整关节动力学，按R1自身URDF配置电机与限位，并读取实际关节状态和 `qfrc_actuator`；旧接触模型仅保留作历史参考。
