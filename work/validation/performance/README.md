# 三机响应性修复验收 · 2026-10-16

范围：仅 `/multi/` 的规划、调度、镜头所有权与观测开销。保留深空指挥舱 / 轨道机库视觉、原模型、关节限制、1ms步长、碰撞检查与真实接触门槛。独立机器人页面和 `dev_r1` 不改动。

## 原因与修复

- 原生浏览器三机开始出现约1419ms连续主线程任务；旧R1对x=2.5m不可达目标反复完整规划，在源URDF数值夹具中耗时约15秒。现在先做保守范围检查，再按完整route候选分段，允许候选之间停止、复位与操作镜头。
- 每毫秒重复同步完整机器人显示树，且协调器重复读snapshot，增加三世界成本。现在实际物理计算不变，只延迟显示树同步；需要FK/TCP时显式同步，不缓存或伪造遥测。
- 开始按钮原本调用自动取景，重置用户视角。现在开始、复位、任务切换与resize均不重设位置；首次加载、参与选择及用户显式全景/聚焦可重新取景。
- 双路相机原本在同帧读回GPU；现在只更新一个可见feed，轮换绘制，繁忙时有1秒尝试刷新兜底，隐藏/离屏不绘制。
- 规划时所有世界暂停物理时钟，恢复时不追赶规划耗时。运行帧有8ms/24tick上限和50ms墙钟积压上限，仍逐个执行全世界1ms步进；过载会减慢墙钟进度，不扩大物理步长。

## 新执行的证据

| 报告 | 实际结果 |
| --- | --- |
| [sort-physical-final.json](sort-physical-final.json) | 8案例，12次真实分拣 |
| [stack-physical-final.json](stack-physical-final.json) | 2案例，3个三层堆叠，9次真实抓取 |
| [pipes-physical-final.json](pipes-physical-final.json) | FULL 8案例，72件真实分类，300布局；停止、失败回滚、复位 |
| [runtime-cpu-20261016.json](runtime-cpu-20261016.json) | 96,000个对比步，qpos/qvel逐值一致；配对步进耗时降低17.07% |
| [responsiveness-contracts-final.json](responsiveness-contracts-final.json) | 20/20确定性调度、计划屏障/取消、镜头与feed契约 |
| [planning-contracts-final.json](planning-contracts-final.json) | 17/17原始/分段计划等价、远目标拒绝、预算、取消和恢复检查 |
| [framing-final.json](framing-final.json) | 113/113，98组投影 |
| [multi-acceptance-final.json](multi-acceptance-final.json) | 当前源绑定及普通/叠放综合验收通过；验证器正负例20/20 |
| [pipes-acceptance-final.json](pipes-acceptance-final.json) | 当前源绑定及FULL圆管验收通过；验证器正负例37/37 |

新物理报告均来自真实重新执行且`sourceFrozen:true`，没有重绑历史物理报告。网格简化数值夹具与原生浏览器观察是不同证据，CPU改善不能换算成任意设备FPS。

## 原生浏览器检查

实际通过本机4173页面的按钮、拖动、键盘与滚动测试，不用截图代替接触物理证据。

- 1280×720下，最终版本三机蓝色任务完成，三者均记录双指接触和约0.129/0.110/0.139m抬升。[完成截图](screenshots/desktop-complete.jpg)。
- 最终版本在已稳定镜头上点击开始，位置前后均为`[5.475611079641342,2.565117362545338,0.31986184841402554]`，进入“路径检查中”且停止、复位可用。[规划截图](screenshots/desktop-planning.jpg)。
- G1用W键把蓝块移到x≈0.746m，开始立即显示“目标超出站立抓取范围，请将物体移近桌面中央”，开始按钮恢复可用。[错误截图](screenshots/desktop-far-target.jpg)。
- R1规划中点击停止及复位均恢复可操作状态；下一次三机开始可完成。
- 顶部面板feed离屏时计数0；滚动相机区可见后计数增长至207，两路画面恢复绘制。
- 390×844调整尺寸前后镜头位置完全相同，文档宽度390无横向溢出。主动点击全景后能重新容纳三台。[手机截图](screenshots/mobile-ready.jpg)。这是浏览器尺寸测试，不是实体手机触控认证。
- 最终版本控制台warn/error为空。审查见[finish-review.md](finish-review.md)。

原生上述检查对应源码：

```text
dist/multi/app.mjs 7df402fd6342701ce4396570a6cf54c038aac5733017cd3b971128c67d4a5eaa
dist/multi/coordinator.mjs d207149524ec7f94550d2becfd1293de3624d9f50d0b6fffd2938fc3f7a4a4d0
dist/multi/planning.mjs 9cfa8681783ec616ea24aa27b2991e30a1574cea74f84252540301fd96c39810
dist/multi/robot-runtime.mjs 523c45b24aa34757de7123929521d7cce6d2c4d5f4611dc2fff5d47fad3a58ed
dist/multi/visuals/frame-scheduler.mjs bd37f98aa1e1806dbfacc664a34e6f6f9407eaee0a3fbde7ff6351d1b31a956f
dist/multi/index.html 8833b4b9b855461cb177e4c88ee5a756cbcc867ee95b1e130baaba0d6f7cf5d0
```

## 边界

采用协作式主线程规划，不是Worker。完整回归最长单段1073.05ms（包含完整候选碰撞检查、同步及OS调度），最长成功准备累计19632.47ms。不能承诺每段16ms、复杂位置即时规划或任意设备60fps；普通10秒、G1圆管30秒为累计计算片段预算，不是总墙钟倒计时。停止在候选之间生效。数学范围检查只拒绝明显不可能的目标，不声称范围内均可达。

没有放宽3mm/.035rad求解门槛，没有吸附、焊接或运输中赋值。既有模型/碰撞近似及非实机认证边界继续适用。本次未推送或部署，线上版本不等于本地修复版本。
