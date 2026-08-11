# hil_test

HIL 分层测试框架（空壳骨架，待实现）。

## 目录结构

```
hil_test/
├── config/
│   ├── hil_test.yaml        # CAN 设备、波特率、测试参数
│   └── protocol.yaml        # 报文 ID / 信号位 / 定标（协议敲定后填充）
├── src/hil_test/
│   ├── bus_monitor.py       # 被动监听 CAN 帧 + 日志落盘 + 周期/ID 统计
│   ├── protocol_loader.py   # 解析 protocol.yaml，编解码统一入口
│   ├── ros_injector.py      # 向 ROS 话题注入控制指令 / 状态（驱动 FSD 输入）
│   ├── fault_injector.py    # 可选：CAN 故障注入（急停帧 / 停发），需独立测试通道
│   └── report.py            # 测试结果汇总，生成 Markdown 报告
├── test/
│   ├── test_protocol.py     # L1 协议一致性（pytest）
│   ├── test_safety.py       # L2 安全与状态联动（pytest）
│   └── test_motor_hil.py    # L3 电机闭环（pytest，标记 slow）
├── scripts/
│   └── run_hil.py           # 分层执行入口：--level L0..L3
└── README.md                # 本文件
```

## 用法（待实现后补充）

```bash
python scripts/run_hil.py --level L0
python scripts/run_hil.py --level L1
python scripts/run_hil.py --level L2
python scripts/run_hil.py --level L3
```

## 验收标准

- L0 链路自检：CAN 接口 up、心跳稳定、fail-safe 告警
- L1 协议一致性：配置驱动下编解码与期望一致
- L2 安全与状态联动：门控 / 急停 / 看门狗通过，仅零输出
- L3 电机闭环：闭环指标达标，输出报告

详见 [docs/HIL测试方案.md](../docs/HIL测试方案.md)。
