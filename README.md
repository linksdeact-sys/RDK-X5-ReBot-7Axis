# RDK X5 · reBot 七轴机械臂

这是当前在 RDK X5 上运行的 Python/Web 控制台：六轴手臂和 J7 夹爪，支持手柄遥操、持续重力补偿、七轴拖动示教、低速回位与回放、URDF 三维反馈和 D435i 彩色预览。RDK 通过达妙 USB2FDCAN 直接接七轴 CAN，不经过 STM32。

- [给 Gemini 的交接入口](GEMINI.md)
- [详细架构、部署和排障](docs/HANDOFF.md)
- [URDF 来源、下载和关节映射](docs/HANDOFF.md#urdf-模型交接)
- [面板原有维护手册和历史验证记录](panel/README.md)
- [本地 SDK 兼容层的构建与验证](native/README.md)
- [早期 STM32F407 控制器（独立私有仓库）](https://github.com/linksdeact-sys/STM32F407_7Axis_Arm)

本仓是从 2026-11-25 RDK 在用部署整理出的源码快照，包含面板、浏览器源码与生成资源、测试、配置、服务和 udev 规则。上游 reBot 运行库及 URDF/STL 未整包再分发；用固定提交及本仓补丁重建。个人标定、运动日志、私钥、令牌、虚拟环境和供应商二进制不在仓库内。

## 先跑仿真

在 Linux 或 WSL（优先用 RDK 隔离测试目录）：

```bash
bash bootstrap-upstream.sh
uv venv --python 3.10
uv pip install -r requirements-rdk.txt
.venv/bin/python -m unittest discover -s panel -p 'test_*.py'
cp -n panel/fixtures/gripper-test-limits.json panel/gripper_limits.sim.json
.venv/bin/python -u panel/server.py --simulate --host 127.0.0.1 --port 8766
```

打开 `http://127.0.0.1:8766/`。已有浏览器脚本默认测试 RDK 的独立仿真端口 `192.168.127.10:8766`，在本地跑时调整测试的 `URL`，不要指向真机端口。真实服务默认地址为 `192.168.127.10:8765`，以 `--standby` 启动；**不要用无参数或历史自动使能脚本测试真机**。
