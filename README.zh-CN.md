[English](README.md) · **简体中文**

# Orbit Manipulation Lab · 轨道机器人操作实验室

三台机器人，独立工作台，同一个物理时钟。

在浏览器中探索 **星海图 R1、宇树 G1 和遨博 C5**。摆放物体、选择机器人，并排完成取放、叠放与圆管分类。物体由 **MuJoCo 接触物理**驱动，不是绑在动画机械手上移动。

**[进入实验室](https://wander521.github.io/r1-simulator/)** · [快速开始](#本地运行) · [工程指南](GUIDE.zh-CN.md)

![深空实验室中的 R1、G1、C5 与独立工作台，右侧为机器人选择和圆管分类控制](work/validation/background-planning/screenshots/desktop-ready.jpg)

实际浏览器截图，应用界面当前为中文。**Orbit Manipulation Lab** 是项目展示名，仓库名及 Pages 网址仍为 `r1-simulator`。

## 核心看点

- **三机自由组合。** 一台、两台或三台同时参与，独立物理世界按统一 **1 ms 时钟**推进，各自处理本桌物体。
- **三类操作任务。** 同色归位、蓝→红→黄叠放、每桌八根圆管分类。R1/G1 交替使用双臂，C5 使用单臂。
- **看见真实执行状态。** 腕部相机、接触力、电机力矩与跟踪误差；完成必须满足接触、抬升、释放和稳定支撑。
- **后台路径规划。** 每机独立 Web Worker 执行路径搜索、逆运动学和碰撞检查，仍可操作镜头、停止、复位；过期计划自动拒绝。
- **两套实验场景。** 深空指挥舱与轨道机库随时切换，保留场景与任务状态。

<details>
<summary>查看后台规划与手机布局</summary>

![桌面浏览器中的后台路径规划](work/validation/background-planning/screenshots/desktop-planning.jpg)
<p><img src="work/validation/background-planning/screenshots/mobile-planning.jpg" width="300" alt="390 像素手机视口中的后台规划界面"></p>

截图来自已有浏览器观察，见[采集背景与审查](work/validation/background-planning/finish-review.md)。

</details>

## 本地运行

静态服务需要 **Python 3**；npm 命令及测试建议使用 **Node.js 22**。浏览器需要 WebGL2、WebAssembly 和模块 Worker。

```sh
git clone https://github.com/wander521/r1-simulator.git
cd r1-simulator
npm start
```

打开 **http://127.0.0.1:4173/multi/**。不需要 `npm install`、应用构建或外部推理服务；运行库和模型已包含在 `dist/`。

没有 npm 时运行：`python3 -m http.server 4173 --bind 127.0.0.1 --directory dist`。使用 HTTP 访问，不直接双击 HTML。

## 试一个任务

1. 在侧栏选择参与机器人及归位、叠放或圆管任务。
2. 左拖物体移动，右拖或 Shift 拖动旋转，WASD 移动物体；空白处拖动观察视角，滚轮缩放。
3. 开始执行：机器人共同准备，再操作各自物体。停止取消待处理工作；复位或重新随机后开启新的圆管批次。

观察对象与任务参与独立。规划时各物理世界一起等待；镜头可操作不等于机器人仍在运动。

## 实现链路

```text
任务与物体编辑 → 公共协调器
  → 每机独立 Worker：路径搜索 / 逆运动学 / 碰撞检查
  → 已验证命令 → 独立 MuJoCo 世界、统一 1 ms 时钟
  → 实际关节与接触状态 → Three.js、相机、实时曲线
```

| 进一步阅读 | 内容 |
| --- | --- |
| [工程指南](GUIDE.zh-CN.md) · [English](GUIDE.md) | 机器人模型、控制、架构、启动与部署 |
| [当前性能优化证据](work/validation/performance-v2/README.md) | 精确结果规划缓存、异步相机读回与发布回归范围 |
| [上一版后台规划](work/validation/background-planning/README.md) | 历史源码绑定报告、取消及编译模型一致性 |
| [实现历史](TECHNICAL_NOTES.zh-CN.md) | 保留的版本与分支记录 |

## 验证与参与

```sh
npm test
npm run test:pages
npm run test:acceptance
```

完整物理测试需要较长时间。上一版 **2026-10-16** 后台规划记录包括 **72/72 根圆管、12 次普通抓取及 3 次叠放**，保留为历史证据；当前优化版将新结果写入 [performance-v2](work/validation/performance-v2/README.md)，GitHub Actions 必须完整回归通过才会发布。

当前优化版的本地与 [GitHub CI](https://github.com/wander521/r1-simulator/actions/runs/39093502625) 完整回归均已通过，Pages 部署成功；线上九项关键资源与已验证源码逐字节一致。该目录保存了新源码绑定证据和规划结果精确对照。

`main` 默认进入 `/multi/`，保留 `/r1/`、`/g1/`、`/c5/`。GitHub Actions 验证后发布 `dist/`。问题反馈请附机器人、任务、布局/种子、浏览器及复现步骤。

## 范围与来源

固定底座操作，不包含行走、端到端学习控制或实机安全认证。物体不焊接、不在搬运中瞬移；可达范围和碰撞覆盖按机器人区分，R1 不代表完整全身自碰撞。

参考来自 [yang-ci/Rebot_Arm_AGV](https://yang-ci.github.io/Rebot_Arm_AGV/?variant=arm)。模型及依赖遵循各自许可：[G1 来源](dist/g1/assets/THIRD_PARTY.md)、[C5 来源](dist/c5/assets/THIRD_PARTY.md)。C5 上游许可正文缺口如实保留，不声称仓库所有资产获得统一授权。
