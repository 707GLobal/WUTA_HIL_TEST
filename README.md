# WUTA_HIL_TEST

FSD HIL（硬件在环）测试工程：工控机上的 FSD 与**真实 VCU** 通过 CAN 总线双向通信，验证协议、链路、安全与电机台架闭环。包含两部分：

- **WUTA-FSD**：FSD 算法仓库（git 子模块）
- **hil_test**：分层测试框架（L0.5 ~ L3），一键脚本完成「编译 FSD → 准备接口 → 启动节点 → 跑测试 → 输出结果」

## 目录结构

```
WUTA_HIL_TEST/
├── docs/
│   └── FSD HIL测试方案.md       # HIL 测试方案（协议、分层、里程碑）
├── hil_test/                    # 测试框架
│   ├── config/
│   │   ├── hil_test.yaml        # CAN 设备、心跳周期、看门狗阈值等
│   │   ├── protocol.yaml        # 0x210/0x501 报文 ID / 信号位 / 定标 / 周期
│   │   └── hil_fsd/
│   │       └── mission_manager.yaml  # HIL 专用参数覆盖（传感器自检全关）
│   ├── src/hil_test/
│   │   ├── can_socket.py        # SocketCAN 套接字封装（纯 stdlib）
│   │   ├── protocol_loader.py   # protocol.yaml 解析与编解码
│   │   ├── bus_monitor.py       # 被动监听 CAN + 周期统计 + 日志落盘
│   │   ├── ros_injector.py      # rclpy 注入/断言 + 台架代发（pose/velocity/waypoints/ready）
│   │   ├── vcu_sim.py           # L0.5：vcan 模拟 VCU（状态机+脚本切换，含 CLI）
│   │   ├── fault_injector.py    # CAN 故障注入（急停帧等）
│   │   └── report.py            # Markdown 报告
│   ├── test/
│   │   ├── test_protocol.py     # L0/L0.5/L1
│   │   ├── test_safety.py       # L2
│   │   └── test_motor_hil.py    # L3（slow，需 HIL_BENCH=1）
│   ├── scripts/
│   │   ├── hil_test.sh          # 一键脚本（编译+节点+测试+清理）
│   │   └── run_hil.py           # 分层 pytest 入口
│   └── conftest.py              # pytest fixtures / markers
└── WUTA-FSD/                    # FSD 算法仓库（git 子模块）
```

五层测试（前一层通过后再进下一层）：

| 层级 | 内容 | 环境 | 真实 VCU |
|---|---|---|---|
| L0.5 | 仿真预跑：vcu_sim 模拟 VCU | 开发机 vcan0 | 否 |
| L0 | 链路自检：接口 / 心跳 / fail-safe | vcan0 或 can0 | 否（vcan0） |
| L1 | 协议一致性：0x210/0x501 编解码 | vcan0 | 否 |
| L2 | 安全与状态联动：门控 / Go / 急停 / 车检 | can0 | **是** |
| L3 | 电机闭环：恒速 / 斜坡 / 急停刹停 | can0 + 台架 | **是** |

## 快速开始

```bash
cd hil_test
./scripts/hil_test.sh                     # 交互：编译 FSD → 选链路 → 跑 → 出结果
./scripts/hil_test.sh -l L1 -i vcan0      # 直接指定链路与接口
./scripts/hil_test.sh -l L3 -i can0 -n    # L3 台架
./scripts/hil_test.sh --no-build -l L0.5  # 跳过编译
./scripts/hil_test.sh -p "can_interface mission_manager controller" -l L2  # 只编指定包
./scripts/hil_test.sh -l L1 -k            # 测试后保留节点（调试）
```

各链路自动启动的节点：

| 链路 | 启动节点 |
|---|---|
| L0.5 | 无（pytest fixture 自起 vcu_sim） |
| L0 | can_interface + vcu_sim（vcan0 下） |
| L1 | can_interface |
| L2 | can_interface + mission_manager（HIL 覆盖）+ controller |
| L3 | can_interface + controller |

日志统一输出到 `hil_test/logs/`（节点日志、pytest 输出、CAN 抓帧 CSV）。

## 分步测试流程（L0 → L3）

### 步骤 0 · 一次性准备

```bash
sudo apt install python3-yaml python3-pytest   # 依赖（一般已装）
cd hil_test
```

脚本首次运行会自动 `colcon build` 编译 FSD（较慢），之后可加 `--no-build` 跳过。创建 vcan0 需要 sudo（脚本自动执行，会提示密码）。

> **CAN 设备预留**：真实 USB-CAN 适配器与端口（接口名）未定前，默认 `can0`。接入后只需改 `hil_test/config/hil_test.yaml` 的 `can.interface`（如 `can1`），脚本/测试自动适配；`can.sim_interfaces` 列表内的接口视为仿真（自动起 vcu_sim、允许 0x501 注入），其余视为真实接口。

### 步骤 1 · L0.5 仿真预跑（无硬件）

```bash
cd hil_test && ./scripts/hil_test.sh -l L0.5 -i vcan0
```

- 自动：编译 → 建 vcan0 → vcu_sim 模拟 VCU → 跑 `test_protocol.py -m sim`
- 验证：协议编解码、VCU 状态机（6→9→10）、0x501 心跳周期
- 预期：`test_sim_501_heartbeat / test_sim_online_advance / test_sim_state_override` 全绿

### 步骤 2 · L0 链路自检

```bash
cd hil_test && ./scripts/hil_test.sh -l L0 -i vcan0     # 仿真先跑通
cd hil_test && ./scripts/hil_test.sh -l L0 -i can0      # 再换真实接口（需 VCU 已在发 0x501）
```

- 自动：起 can_interface（发 0x210 心跳）+ vcu_sim（vcan0 下补 0x501 心跳）→ 跑 `-m link`
- 验证：接口 up、0x210/0x501 心跳 10Hz 稳定
- 预期：`test_link_*` 全绿

### 步骤 3 · L1 协议一致性

```bash
cd hil_test && ./scripts/hil_test.sh -l L1 -i vcan0
```

- 自动：起 can_interface → 跑 `-m protocol`
- 验证：
  - 纯单测：定标中心点 / 字节序 / 越界钳位 / 模式映射
  - 集成用例：注入 0x501 断言话题（start_command / emergency / mission_mode_cmd）、注入 `/control/command` 断言 0x210 定标
- 注意：真实接口下注入 0x501 的用例**自动跳过**（防污染真实 VCU）

### 步骤 4 · L2 安全与状态联动（真实 VCU）

```bash
cd hil_test && ./scripts/hil_test.sh -l L2 -i vcan0     # 先 vcan0 全自动预演
cd hil_test && ./scripts/hil_test.sh -l L2 -i can0      # 真实 VCU（部分用例需人工）
```

- 自动：起 can_interface + mission_manager（自动加载 HIL 覆盖关闭传感器自检）+ controller → 跑 `-m safety`
- 验证：启动门控 / Go 放行（推进到 EXPLORE）/ 急停优先级 / 保活防丢 / 车检流程
- 人工配合（真实接口模式）：操作 RES 给 Go、RES 急停、AMI 选车检，脚本会等待最多 30s
- 看门狗用例**跳过**（依赖 can_interface VCU 失联检测，待开发）

### 步骤 5 · L3 电机闭环（台架）

```bash
cd hil_test && ./scripts/hil_test.sh -l L3 -i can0 -n   # -n 开启台架模式
```

- 自动：起 can_interface + controller（定位/感知/规划不启动）→ `HIL_BENCH=1` → 跑 `-m motor`
- hil_test 代发位姿 / 车速 / 直路路径 / 就绪信号，controller 真实输出经 CAN（0x210）闭环
- 验证：恒速跟随出现驱动开度、加减速斜坡回零、急停刹停时间（仿真自动触发可量化）
- 前提：**L2 全部通过后再给台架通电**

### 直接跑 pytest（跳过脚本）

```bash
cd hil_test
python -m pytest test/test_protocol.py -m "protocol and not integration" -q  # 纯单测（无硬件/无 FSD）
python -m pytest test -m protocol -q   # L1（集成用例需 can_interface 运行）
```

## 依赖

- Python 3.10+、pytest、PyYAML（系统包即可）
- **CAN 层用纯 stdlib SocketCAN**（无需 python-can）
- ROS 集成用例需 ROS2（humble）+ FSD workspace（脚本自动 source）
- 调试可选：can-utils（candump / cansend）

## HIL 台架模式（车辆架起，传感器仅保在线）

台架下相机 / 激光雷达 / 华测**不参与决策**，由 hil_test 代发以下信号（`RosInjector` 方法）：

| 代发话题 | 方法 |
|---|---|
| `/system/lidar_ready` | `publish_lidar_ready()` |
| `/system/localization_ready` | `publish_localization_ready()` |
| `/localization/pose` | `publish_pose(x, y, yaw)` |
| `/chcnav/velocity` | `publish_velocity(vx)` |
| `/planning/final_waypoints` | `publish_waypoints_straight()` |
| `/system/devices_inspection`（上线） | `publish_devices_inspection(ok=True)` |

关键点：

- **mission_manager 必须关闭传感器自检**（脚本自动加载 `hil_test/config/hil_fsd/mission_manager.yaml`），否则华测未接 → 自检超时 → 锁死 EMERGENCY；
- **L3** 运行子集 can_interface + controller，mission_state 由 hil_test 注入 RACE 使能。

## 分层 marker 与跳过规则

| marker | 级别 | 运行依赖 | 跳过条件 |
|---|---|---|---|
| `sim` | L0.5 | vcan0 + vcu_sim（无 ROS） | 接口未 up |
| `link` | L0 | can_interface + CAN 接口 | 接口未 up / can_interface 未运行 |
| `protocol` | L1 | 单测无依赖；集成需 can_interface | 接口未 up / can_interface 未运行 / 真实接口注入用例 |
| `safety` | L2 | can_interface + mission_manager/controller | can_interface 未运行；看门狗用例恒跳过 |
| `motor` | L3 | 台架 + can_interface + controller | `HIL_BENCH` 未设 / can_interface 未运行 |

## 安全约定

- L0.5/L1 的 0x501 注入用例**仅限仿真接口**（`sim_interfaces`，如 vcan0）；真实接口下自动跳过（防污染真实 VCU）
- 台架通电前先通过 L2 门控 / 急停用例，人工急停（RES）随时可用
- 看门狗用例待 can_interface VCU 失联检测实现后启用（当前跳过）

## Git 子模块说明

`WUTA-FSD` 是通过 git submodule 引入的独立仓库（`https://github.com/GaoMingHa0/WUTA-FSD.git`），其提交记录与主仓库相互独立，主仓库只记录 WUTA-FSD 当前指向的 commit。

### 首次克隆（含子模块）

```bash
git clone <主仓库地址>
cd WUTA_HIL_TEST
git submodule update --init --recursive
```

### 子模块已存在，拉取最新代码

```bash
cd WUTA-FSD
git pull            # 拉取 WUTA-FSD 最新提交
cd ..
git add WUTA-FSD    # 更新主仓库中 WUTA-FSD 指向的 commit
git commit -m "update WUTA-FSD"
```

### 更新子模块到远程最新

```bash
git submodule update --remote WUTA-FSD
git add WUTA-FSD
git commit -m "update WUTA-FSD"
```

### 修改 WUTA-FSD 内部代码

```bash
cd WUTA-FSD
# 修改代码后提交到 WUTA-FSD 自己的分支
git add . && git commit -m "xxx"
git push
cd ..
git add WUTA-FSD
git commit -m "update WUTA-FSD"
```

## 注意事项

- 不要在子模块目录内直接修改主仓库的内容，WUTA-FSD 有自己的远程仓库。
- 子模块默认处于 detached HEAD 状态，若需在其上开发，请先切换到对应分支：
  `cd WUTA-FSD && git checkout <分支名>`
