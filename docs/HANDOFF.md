# RDK X5 reBot 七轴控制台交接

交接基线：2026-11-25，RDK 在用部署。控制台为 `hand-teach-v5-workflow`，静态示教入口为 `teaching.js?v=9-return`；后端含 `teach-launch-lock-v1` 和 `communication-recovery-v1`。远端源目录为 `/home/sunrise/reBotArm_control_py-main`。本快照不携带正在运行的状态、现场标定或录制；克隆不会启动机械臂。

## 1. 项目目标与设备

目标是让操作者用手柄平滑控制六轴手臂和 J7 抓取，也能靠手推录下七轴动作，使用一个可复用的起点低速回位，再回放记录。要减少误触发和反复标定，不依靠绕过保护获取流畅感。机械臂受重力，失能时会下落，手推及 J7 松力时物体可能坠落。没有碰撞检测、力传感器或软件可替代的硬急停。

| 项目 | 在用配置 |
| --- | --- |
| 主控 | RDK X5，Linux ARM64，Python 3.10，默认网址 `http://192.168.127.10:8765/` |
| 总线 | 达妙 USB2FDCAN，原生 USB 通道 0，CAN 1 Mbps；电机 ID 0x01～0x07，反馈 ID 0x11～0x17 |
| 关节 | J1～J3 DM4340P；J4～J7 DM4310。J6 已由 4340P 换为 **4310** |
| 职能 | J1 基座；J2 大臂；J3 小臂；J4 腕部俯仰；J5 腕部滚转；J6 末端滚转；J7 夹爪 |
| 输入与显示 | 北通鲲鹏20 BFM 接收机，PID 0x5128；D435i 彩色预览；浏览器 URDF/STL |
| 旧方案 | STM32F407 + W5500/UDP 已放在独立私有仓库；不属于当前控制链路 |

J7 正角向收拢。历史实测口径约 49.3° 时 60mm、241.3° 时 8.5mm；第二个是机械收拢限位，**不是完全闭合**。在线的安全软件端点另行示教，不能用这两个物理测量替代；网页开口是插值显示。J7 的等价 360° 表示在反馈层归一化，MIT 原始位置仍用驱动器原始值，夹爪 Kp=0。不要照搬到 J6、修改电机零位、把 2 N·m 当作 2 N，或把 URDF 的角度范围当作碰撞检测。

## 2. 控制链路与主要文件

```
网页 (200ms状态轮询 + 会话心跳)  /  KP20 (本地输入线程)
  → panel/server.py: 校验、排队、运行/停止、记录与回传
  → gravity_mode.py + gamepad_control.py: Pinocchio重力、轨迹、限幅、MIT (100Hz目标)
  → MotorBridge / 达妙SDK / USB2FDCAN → J1…J7
  ← native/dm_observer.cpp + rx_freshness.py: CAN收到时刻和帧序号
```

- `server.py`：`Panel`、HTTP API、硬件工作线程、`Simulation`、连接/恢复与状态采样。请求只排队，硬件由工作线程及控制回路操作。`feedback_state.py` 定义新鲜度，状态年龄不是页面刷新间隔。
- `gamepad_control.py` 与 `teleop_motion.py`：常见操作为 LB 始能，摇杆/十字键/按键控制 J1～J6，LT 张开、RT 收拢 J7。保持、减速、边界、摇杆回中和人机互锁集中在这里；不要重控另开一个 USB 句柄。
- `gravity_mode.py`、`gravity_control.py`、`config/rebotarm_dm.yaml`：使用官方惯性/关节模型计算重力前馈；现用 J4 默认 75%，其余 100%。加装夹具或载荷变化须由现场验证，不照搬历史数据。手推使用柔顺控制，保持和回放使用有界参考。
- `gripper_teleop.py`、`gripper_torque.py`、`gripper_contact_motion.py`：J7 双向速度、检测接触、扭矩保持与协调暂停。2 N·m 为软件电机力矩上限。J7 200ms 驱动通信看门狗仅在空闲预检中写/读 RAM；未改 Flash。
- `teaching.py`：状态机 `STARTUP/DRAG/HOLD/REPLAY`，轨迹样本校验、保存、回位/回放生成。`teaching_panel.py`：API、签名、起点和记录装载/发布。`static/teaching.js`：流程、选择器、禁用原因、确认及历史提示。文件 I/O 不应放在控制线程或 CAN 发布状态锁内。
- `communication_recovery.py`、`usb_reconnect.py`：USB 断连后启动新进程，不恢复旧动作。驱动 13/0xD 是通信超时报警，即便角度更新也可以保持报警。仅在显式确认、在线静止、无其他故障/任务时清通信报警，并以新失能反馈复核；不会使能。
- `urdf_view.py`、`panel/viewer/viewer.js`、`static/urdf-viewer.js`：三维状态显示，输入为实测关节反馈，J7 映射到夹指开口；渲染不进入控制环。 `camera_preview.py` 用 FFmpeg/v4l2 提供 D435i 彩色视频，不提供深度避障。

常用只读入口是 `GET /api/state`，包括各轴 online/status/angle/feedback_age_ms，active/result，teaching.home/state/records，recovery，最近事件。写入口 UI 使用同源和会话确认；控制台应仅在可信隔离网络使用，不暴露公网。主要接口为 `POST /api/connect`、`/api/test`、`/api/stop`、`/api/recovery`、`/api/teaching`、`/api/heartbeat`。模拟注入 `/api/sim-input` 只在模拟版存在。

## 3. 示教当前行为

1. 安全支撑 → 进入手推 → 开始（可自由录制，或回位后“从起点开始录制”）。J1～J6 手推随动；J7 录角度，手推不主动夹紧。
2. “结束并保存”产出完成记录；中途暂停、停机或异常会留下草稿，非完整草稿不可回放，故障后一律不续播。新记录带中断原因。完整记录还须通过指纹、采样间隔、范围和跳变检查；“已存盘”不等于可回放。
3. 通用起点文件为 `panel/teaching_home.json`，保存七轴实测姿态而不是零位；原记录第一帧独立保存，不会被重标起点覆盖。运行中不反复重标。J1～J6 自动回位局部接入为每轴 15°；空夹爪可在独立标定行程内回位。J6 若在软件边界外不超过 10°、距目标仍在 15° 内，只向内退出，其他轴先保持；重入后继续回位。回位要求实测七轴均到 1° 内且三组新反馈静止，30 秒未到位暂停。回位仍要求操作者清空整段路径。
4. 回放独立于手推入口：选择可回放记录，显式确认后低速接入其第一帧，再按名义约 25% 时间倍率回放。手臂参考有限速度/加速度，关节或夹爪未跟上时轨迹时钟可等待。不循环、不自动恢复。
5. 手推 J7 超过 1.5 rad/s 要持续约 75ms 且有至少 3 个新反馈才暂停；达到 2.0 rad/s 立即暂停。自动模式未使用此宽限。过温、驱动故障、真实反馈超过 150ms、硬停等保留。新样本校验容忍有限的短时差分量，不修改旧记录或把采样差分报警说成实际电机超速。

## 4. 获取依赖、部署和复现

**只读来源**：[reBot 官方仓库固定提交](https://github.com/Seeed-Projects/reBotArm_control_py/tree/3524ebc5316985d6d74fe30269ae771fea715507)，已核对生产 `reBotArm_control_py/` 与 `urdf/` Git 文件哈希，除 `actuator/rebotarm.py` 外一致。运行层局部差异保存在 `rebotarm-runtime.patch`（新增 dm-device 配置/适配）。`bash bootstrap-upstream.sh` 只在干净目录获取这些文件并应用补丁；原配置不被覆盖。官方模型与源码不复制进本仓 Git 历史。

部署实际依赖：Python 3.10、MotorBridge 0.5.0、Pinocchio 3.9.0、NumPy 2.2.6、PyYAML 6.0.3；构建本地观察层还需 g++/libdl，摄像头需 FFmpeg，Web 视图重新打包需 Node/npm。 `requirements-rdk.txt` 是关键版本基线，不是整个生产虚拟环境的锁文件。浏览器仿真按根 README 启动，不需要 USB 权限，也不碰真机。

真机准备（由现场操作者安排，先用仿真完成验收）：

1. 从现有设备只备份非 Git 的 `panel/gripper_limits.json`、`directions.json`、`teaching_home.json`、`teaching_records/` 等；保存在现场，绝不复制仿真标定到正式路径。机构、已标定端点或零位变化后要复核指纹和实物，即使有旧记录也不能认定适用。
2. 现有服务和 `run_rebot.sh` 使用 `sunrise`、`/home/sunrise/reBotArm_control_py-main`，及 `/home/sunrise/rebot-usb-runtime/lib` 的 libusb 兼容层；移装时逐项确认路径和权限。 `deploy/rebot-panel.service` 为真实服务模板。 `70-rebot-damiao.rules`、`71-rebot-kp20.rules` 是现场 udev 规则。不要重载规则或替换库来“测试”，那会中断现有硬件会话。
3. 按 [native 文档](../native/README.md) 获取固定版本的 `dmcan.h`，构建 `librebot_dm_observer.so` 并运行离线测试。仅在 AArch64 的原始官方 SDK 二进制哈希与文档一致时，`prepare_dm_rx.py` 才能生成项目内 1ms 接收空闲等待的兼容副本。原 vendor SDK 不更改。**不要用不明来源的二进制替换，也别绕开 SHA256 校验。**
4. 任何服务重启前确认现场支撑 + 七轴实时失能 + 无活动或排队任务。启动 `./run_rebot.sh -u panel/server.py --standby`，页面可开但未连硬件。人工确认后“连接机械臂”只核验失能，不能自动恢复运动。服务运行日志用 `journalctl -u rebot-panel.service --no-pager`。保留每次发布替换前的文件备份。
5. `cd panel/viewer && npm ci && npm run build` 可从源码重建生产随附的三维资源。浏览器验收时正式页面仅 GET-only；机械臂动作必须由操作者独立确认。

## 5. 已知问题和排障证据

| 表现 | 已确认或待查 |
| --- | --- |
| 明明保存了还报“录制未完成” | 页面保留下拉框旧选项；13.9 秒新记录已校验通过，而 6.6 秒旧草稿不可播。让新保存的记录自动选中，并显示对应记录的具体原因。不要更改旧草稿的完成标记。 |
| 点击回位失败“七轴静止后才能回放” | 已在启动约 500ms 渐入后的现场日志见过；启动前静止不代表渐入后仍静止，根因可能是实际漂移或近边界配置，待测实际每轴速度。建议有界静等，且持续复核确认/停止。 |
| 回位完成后第一次点击“从起点开始录制”偶发不进 | 针对最新版有现场报文；当前分支要求控制 tick 恰好拿到全七轴新序号，失败原因混在一起。分清正在动、夹爪残留前馈和序号不足，等新帧并检查超时。 |
| 回位提示“超出七轴范围” | 2026-11-25 的实际情况为 J6 −189.05°，软件下限 −179.91°，已保存目标 −174.10°；同时 J7 与起点差约 49.8°。本版加入 J6 有界向内退回和 J7 独立行程；不改电机零位。报错姿态离线已通过新版预检，未代操执行。 |
| 已经 7/7 在线仍被拦 | 历史 J1 150ms 超时提示不等于现在失联。曾见七轴同时约 245ms 没新反馈且控制完成心跳停顿，内核无该时刻 USB 重置记录。恢复后 J7 可继续报告 13，显式恢复报警。还需采集 RX/TX、USB 事件、调度、线程阶段后定位共同空窗，未证实某单轴断线。 |
| 旧 18.5 秒记录不能播放 | 第 310 帧附近位置差分有明显跳变（不是电机实测超速）。保留原始文件，不把无效记录硬改为可回放。 |
| 一开始就报“反馈不新鲜” | 旧版本将约 200ms 文件校验放在 CAN 发布锁内，造成反馈缓存老化，已修复。若复现先查是不是跑了旧进程或当前另有总线/线程停顿。 |

最近发布证据：214 项单元、假电机和模拟网页通过；重点覆盖 J6 向内参考、J7 独立回位、保存起点/复载、录制和回放状态门槛。现场另有一段约 13.9 秒的完整记录在部署后成功保存并通过校验；**不能用它证明所有起点回位、带载稳定性或保护已完成真机验收**。本仓交接操作不重启或接管 RDK。

## 6. 下一次改动如何验收

优先参见 `GEMINI.md`。每次记录版本、复现方法、受影响状态转换、原始/模拟反馈范围、受影响关节、失败归因，以及恢复是否涉及失能。Linux 本地执行：

```bash
.venv/bin/python -m unittest discover -s panel -p 'test_*.py'
.venv/bin/python -m panel.check_teaching_adapter
.venv/bin/python -m panel.check_continuous_adapter
```

独立仿真起服务后跑 `panel/browser_home_check.py`，及适当的 teaching/workflow/recovery 检查。后者部分测试要使用合成记录夹具，测试脚本有预期字段/文件说明。涉及录制或启动改动，至少验证：正常保存/选中、草稿隔离、超时/停止不续播、J7 真到位而非只发出命令、不能沿用上个会话的确认。浏览器不要连真机做带写入的自动测试，也不要把现场录像、采样、私钥或账号写进日志样例。
