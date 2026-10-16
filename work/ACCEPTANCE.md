# R1 增量验收（2026-10-15 历史冻结报告）

版本提示（2026-10-16）：本文保存 2026-10-15 的实现、截图、录像及数值，不是当前姿态/相机版本的重新验收。下表“腕部相机”的夹爪坐标安装、合成盒体、65mm 偏移、30°光轴及对应投影结论已被原 URDF `realsense_link` 绑定取代；旧证据按原日期保留。本文出现的“当前源码”“最终”及哈希仅指当时冻结版本，不自动适用于后续 R1 自然腕部路径。最新实现与本轮验证状态见 [姿态与原生相机验收](POSE-AND-VISION-ACCEPTANCE.md)。

历史命令按原文保留；当前 `main` 的 `npm test` / `npm run test:repeat` 已用于多机器人流程，不能据此复现本文原覆盖。当前 R1 入口为 `npm run test:r1`，本轮专项与完整运行范围以新报告为准。

验收日期：2026-10-15。参考为本仓库 `dist/Rebot_Arm_AGV/?variant=arm`；目标为 `dist/r1/`。本报告只覆盖本轮六项增量：叠放与结果动作、关节标注、真实关节反馈、腕部相机、场景、加载。原完整目标中未列为本轮工作或未留下对应证据的内容，不因本报告而自动视为通过。

## 六项对应实现与证据

| 增量 | 当前实现 | 实际证据与结论 |
| --- | --- | --- |
| 叠放及结果动作 | `grasp-controller.mjs` 按蓝→红→黄读取当前物体位置；实际双指接触、抬升、搬运、释放后检查桌面→蓝→红的支撑链；整塔连续稳定3秒后面向观察者执行happy。叠放失败执行shy并保留原因；停止中断动作，不伪装失败或完成。结果动作先回适合R1的中心姿态，按真实关节收敛/超时推进。页面在反应开始前持续读取观察位置，避免执行中旋转视角后仍面向旧位置。 | [36组基线](validation/r1-dynamic-grasp-3x.jsonl)全部通过，其中左右臂各3次完整叠放；最终动作中心修正后，[带当前源码哈希的6组验证](validation/r1-dynamic-final-stack-recovery.jsonl)全部通过：左右臂各1次完整叠放，及不可达、失败后复位再归位、搬运中停止、happy中停止4项恢复。两次最终叠放均记录稳定3.01秒，完成后再推进3秒检查支撑。[全过程录像](validation/media/r1-stack-complete.mp4)与[阶段轨迹](validation/stack-screen-trace.json)到达complete/happy。另用实际W键将蓝块移到桌内远边x=1.106m，保存了自然不可达后的[失败动作录像](validation/media/r1-stack-failure.mp4)及[实际关节轨迹](validation/failure-screen-trace.json)：face_observer→shy→failed，无happy，失败原因保留，物体没有被搬运。 |
| 关节标注与手动操作 | `r1-joint-ui.mjs/css` 提供J1–J6、夹爪七个标注；真实/目标值、限位、滑块、悬浮或点击展开；连线指向实际机器人关节；运行时禁用手动输入，切换左右臂更新关节映射；夹爪用完整开度毫米显示。 | [浏览器记录](validation/browser-regressions.json)中jointKeyboard证明J6目标`.0102rad`、实际`.0102000247rad`；guides记录7条连线；narrowChip记录390px宽下滑块完整位于界内；outsideClose证明点击外部收起。TCP拖拽后有实际关节变化且orbit恢复，夹爪finalClosed/finalOpened、leftArm、stopped、finalReset均留下状态记录。 |
| 真实关节反馈 | 新的 `r1-dynamic-scene.xml` 与 `r1-dynamics.mjs` 使用URDF关节树、轴、质量、质心与完整惯量。渲染取同一MuJoCo `qpos`；关节目标来自实际控制命令；力矩取 `qfrc_actuator`，误差为目标减实测，RMS仅计算所选臂六个旋转关节。指垫力N、腕部位置误差mm与关节N·m/rad/mrad分开显示。 | 当前源码执行 `node work/test-r1-dynamics.mjs` 通过：质量误差0，编译后重建惯量最大误差`1.0474e-8`，FK位置误差`9.49e-16m`、角误差`4.21e-8rad`；平滑跟踪最大误差`.0001262rad`；完整展开/下蹲最大误差`.002486rad`。六组最终任务记录实际力矩有限且不超过各URDF电机限制；[UI记录](validation/browser-regressions.json)含真实非零力矩、目标/实际值。曲线读取同一物理状态，不从姿态角度估算力矩。 |
| 腕部相机 | `r1-workcell.mjs:createWristVision` 在当前执行臂夹爪坐标系安装相机，FOV62.82°、向下30°；正确使用局部负Z偏移使镜头位于水平夹爪上方65mm。相机外壳与支架放在独立图层，腕部图像不渲染自身。全局俯视FOV70°；并排/上下、画面及模型显隐保留。 | 光学数值检查中TCP投影NDC约`[0,-.0645,.8700]`，处于画面中央附近；[闭合阶段截图](validation/screenshots/r1-closing.jpg)中腕部画面可见黄色圆柱、夹爪和台面，证明抓取阶段无遮挡。UI记录cameraSwitch证明切上下、关画面、隐藏模型；leftArm记录随左臂移动的镜头位置和方向。待命姿态中画面可看到折回的手臂，这是镜头随真实腕部朝向的结果，没有另置固定镜头冒充。 |
| 场景与视觉 | `r1-workcell.mjs` 复用参考 `nh/th/ih/sh/Qu` 的背景蓝色竖光、扫描线、地面网格、环境反射和台面颜色转换。按用户后续“稍微调亮”要求，ACES曝光为1.1424、环境光.40、半球光.65、青色补光.55；主光、轮廓光与三盏面积灯不变。主视图及两路相机一致。LTC表取自已包含的参考包；保留三物体形状、颜色框和选中闪烁。 | 两页[桌面对照](#截图与录像)均为1280×800、中文、idle、蓝色任务物选中；另有[补光前](validation/screenshots/r1-lighting-before.jpg)/[补光后](validation/screenshots/r1-lighting-after.jpg)。模块校验32768个Float32及32768个HalfFloat值与参考逐项相同，八盏实际灯光含三面积灯。390×844同视口窄屏截图已补齐，R1无横向溢出。图像保留R1体型、台面尺寸与构图适配差异，未声明逐像素相同。 |
| 加载状态 | `r1-app.mjs:setLoading/wasmWithProgress/boot` 按模型零件完成、WASM字节及初始化、场景读取、物理创建更新进度；全部STL就绪后统一上材质；物理/手动控件在就绪前禁用；错误分支显示具体错误和刷新重试。 | [真实加载轨迹](validation/loading-progress.json)含100条进度事件、59种百分比，记录区间5%→100%单调增加；到physics初始化完成才记录100%并隐藏遮罩。未通过计时器模拟百分比。加载失败提示已做源码检查，本轮未注入网络/模型损坏故障来录制失败恢复。 |

## 数值回归的版本范围

[36组基线](validation/r1-dynamic-grasp-3x.jsonl)包括：左右臂×三物体×连续3次，共18次；移动位置、绕竖轴旋转蓝块、侧躺黄柱四个场景×3次，共12次；左右臂各3次完整叠放，共6次。全部pass=true。30次归位/编辑任务的最小实际抬升为`.129317m`，最大末端落点误差`.000435m`。

这36条记录是在最终结果动作中心姿态修正前生成，文件未记录源码哈希，不能声称36条均在最终reaction源码上执行。修正只改变结果动作中心，随后另跑[6条最终验证](validation/r1-dynamic-final-stack-recovery.jsonl)：左右臂各一次完整叠放和四项恢复，均通过。记录中的五个SHA256与本报告生成时的当前文件逐一匹配：

| 核心文件 | SHA256 |
| --- | --- |
| `dist/r1/grasp-controller.mjs` | `c042b05bb52c2a3e8e9940e90a0742fd5048681a078ca620902f65594ba77f0f` |
| `dist/r1/grasp-kinematics.mjs` | `8aabe6ae5d1664c04421aa72f70c2ce68fd95fbf7acce33bb207f87edf656893` |
| `dist/r1/r1-dynamics.mjs` | `2b6ce993b293b7d0b05f51355903b044dc51e53fdd4395d831b00a9bcd704366` |
| `dist/r1/models/r1-dynamic-scene.xml` | `eff0fece28b2cdf8ea37d89cc8c0bd33c74b504bc5f95af3d29d7d175031940d` |
| `dist/r1/assets/r1.urdf` | `5365fb487634ea519aa065f45311d9aae48ac33b5edfb75154ebc94bbf3464eb` |

重新验证入口：`npm test` 运行关节动力学校验及当前任务场景；`npm run test:repeat` 运行每场景3次。视频和浏览器交互证据覆盖可见运行行为，不代替上述物理断言。

## 截图与录像

桌面两页条件：1280×800、中文、初始就绪、蓝色物体选中。机器人自身和R1必要站位/台面适配保留，因此物体世界坐标及画面占比不完全相同。

| 参考页 | R1 |
| --- | --- |
| [参考桌面截图](validation/screenshots/reference-desktop.jpg) | [R1桌面截图](validation/screenshots/r1-desktop.jpg) |
| [参考390×844窄屏截图](validation/screenshots/reference-narrow.jpg) | [R1同视口窄屏截图](validation/screenshots/r1-narrow.jpg) |

两张窄屏对照均为390×844、中文、蓝色任务物选中、idle。另保留[R1窄屏全页](validation/screenshots/r1-narrow-fullpage.jpg)，记录面板的完整滚动范围。R1 DOM记录viewport宽390、scrollWidth390，面板位于场景下方，关节展开控件仍在画面内。手机行为使用粗指针/无悬停环境模拟和点击操作验证；未在实体手机上测试，也未把桌面指针点击称作真实硬件触控验证。

[R1叠放全过程录像](validation/media/r1-stack-complete.mp4)：1280×800，55.6875秒，MP4编码16fps、891编码帧；来自约4fps的连续实际屏幕采样，共193张采样画面，采样最大间隔588ms。编码会重复采样画面维持时间，不能称为原生16fps屏幕录制。未剪切拼接动作阶段。[对应状态采样](validation/stack-screen-trace.json)覆盖idle、展开、下蹲、接近、下降、闭合、抬升、搬运、放置、释放、撤离、稳定、面向观察者、happy、complete；最后支撑链有效，稳定时间3.01秒。

关键帧：[闭合抓取](validation/screenshots/r1-closing.jpg)、[整塔稳定判定](validation/screenshots/r1-stack_settle.jpg)、[成功动作](validation/screenshots/r1-happy.jpg)、[完成](validation/screenshots/r1-complete.jpg)。闭合帧腕部画面可以识别目标和台面；成功及完成帧对应实际三件自由物体的支撑结果。

[失败动作录像](validation/media/r1-stack-failure.mp4)为1280×800、5.1875秒、16fps编码，来自10张连续实际屏幕采样；记录规划拒绝后的面向观察者、害羞和失败，未宣称覆盖同步规划耗时。R1颈部没有独立俯仰关节，低头通过躯干轻微前倾、头部下降及收腕适配；[低头关键帧](validation/screenshots/r1-shy.jpg)对应实际躯干角`.25/-.45/-.30rad`，净前倾约`.10rad`，不是后仰。不可达的穷举IK搜索为同步操作，此桌边场景的独立计时约8.77秒，期间界面可能短暂停顿；本轮未扩展异步规划架构。完整成功录像录制于这次补光前，核心物理源码哈希未变化；调亮后的视觉以最新桌面、窄屏和失败动作画面为准。

## 明确保留的适配与边界

- R1真实外观、关节树、左右臂、夹爪与根据实机照片制作的分区材质保留；R1较高，主视图必须包含底盘和躯干，因此台面/物体在画面中较参考页小。台面为`.72×.76m`、顶高`.692m`、中心x=`.78m`，对应已有R1可达性布局；没有为装饰重新移动任务物。
- 参考RS机械臂与R1的自由度、关节限位、惯量、电机力矩限制不同；R1轨迹、下蹲及结果动作使用自身结构求解。R1当前采用URDF惯量与真实电机力矩的固定底座物理模型，所有活动关节有作动器；没有物体焊接、隐藏承托、膨胀碰撞球或运输时直接改物体位置。正常仿真中对象由接触、摩擦及重力运动；复位/用户编辑是显式位置写入入口。
- 本轮保留现有接触范围：机器人只有固定尺寸指垫与任务物发生接触，外壳/全臂自碰撞和移动底座动力学未加入。参考页包含RS全臂碰撞网格，二者不能据此宣称碰撞覆盖完全等价。此边界不妨碍上述抓取/叠放验收，但仍须在原完整目标中单独决定和验证。
- 本轮交付中文界面，未扩展中英日或AGV；实体设备多点触控、参考页同步操作录像及最终reaction哈希下左右臂各连续3次叠放的完整重跑，本报告没有对应证据。它们不属于本轮明确验收要求，不能由既有按钮、36组较早记录或单个成功视频推定已完成。

本轮六项已各有实现与可审查证据；报告同时保留上述覆盖边界，不作“全部参考功能100%还原”的结论。更详细的参考源码契约见[REFERENCE-DELTAS.md](REFERENCE-DELTAS.md)。

最终只读核验：`node work/verify-acceptance.mjs` 的27项检查全部通过，包括当前物理源码哈希、真实成功/失败轨迹、两组同视口截图、录像、加载进度及已有交互回归证据。此脚本核验已保存的证据，不将文档声明本身作为通过条件；布局、亮度、抓取取景与动作关键帧另经实际图像检查。
