# WUTA\_HIL\_TEST

FSD HIL（硬件在环）测试工程：工控机上的 FSD 与**真实 VCU** 通过 CAN 总线双向通信，验证协议、链路、安全与电机台架闭环。包含两部分：

- **WUTA-FSD**：FSD 算法仓库（git 子模块）
- **hil\_test**：分层测试框架（L1 \~ L4），一键脚本完成「编译 FSD → 准备接口 → 启动节点 → 跑测试 → 输出结果」

## 目录结构

```
WUTA_HIL_TEST/
├── docs/
│   └── FSD HIL测试方案.md       # HIL 测试方案（协议、分层、里程碑）
├── hil_test/                    # 测试框架
│   ├── config/
│   │   ├── hil_test.yaml        # CAN 设备、0x210 保活周期等
│   │   ├── protocol.yaml        # 0x210/0x501 报文 ID / 信号位 / 定标（0x210 周期）
│   │   └── hil_fsd/
│   │       └── mission_manager.yaml  # HIL 专用参数覆盖（传感器自检全关）
│   ├── src/hil_test/
│   │   ├── can_socket.py        # SocketCAN 套接字封装（纯 stdlib）
│   │   ├── protocol_loader.py   # protocol.yaml 解析与编解码
│   │   ├── bus_monitor.py       # 被动监听 CAN + 周期统计 + 日志落盘
│   │   ├── ros_injector.py      # rclpy 注入/断言 + 台架代发（pose/velocity/waypoints/ready/传感器心跳）
│   │   ├── vcu_sim.py           # L1 pre：vcan 模拟 VCU（状态机+脚本切换，含 CLI）
│   │   └── fault_injector.py    # CAN 故障注入（急停帧等）
│   ├── test/
│   │   ├── test_protocol.py     # L1（pre：协议单测/VCU 模型；post：链路/协议）
│   │   ├── test_selfcheck.py    # L2（传感器自检故障模拟）
│   │   ├── test_drive_hil.py    # L3（slow，真实台架需 HIL_BENCH=1）
│   │   └── test_inspection.py   # L4（slow，真实台架需 HIL_BENCH=1）
│   ├── scripts/
│   │   ├── hil_test.sh          # 一键脚本（编译+节点+测试+清理）
│   │   └── run_hil.py           # 分层 pytest 入口
│   └── conftest.py              # pytest fixtures / markers
├── zlgcan_bridge/               # ZLG USB-CAN 桥接（USBCAN-4E-U → SocketCAN can0；git 子模块）
│   ├── bridge.yaml              # 桥参数（devtype/devidx/chn/baud/iface），无需改源码
│   └── start_zlg_bridge.sh      # 一键：检查硬件 → 建 can0(vcan) → 编译 → 前台起桥
└── WUTA-FSD/                    # FSD 算法仓库（git 子模块）
```

四层测试（前一层通过后再进下一层）：

| 层级 | 内容                                               | 环境           | 真实 VCU      |
| -- | ------------------------------------------------ | ------------ | ----------- |
| L1 | 协议单测 + vcu\_sim 模拟（pre，无 FSD）；链路 + 协议通畅：0x210 保活 / 0x501→话题 / 0x210 定标透传（post，需 FSD） | vcan0 或 can0 | 否（vcan0 仿真） |
| L2 | 传感器自检故障模拟：数据缺失/断流 → 立即切 EMERGENCY（断电层，vcan0 全自动）       | vcan0 或 can0 | vcan0：否 / can0：是（仅 1 条联动） |
| L3 | 低速动态安全闭环：AMI 选直线加速（2）+ RES Go → EXPLORE → **低速驱动（轮子动）** → RES 急停（通电层） | can0 + VCU + 台架架起 | **是**       |
| L4 | 车检任务全链路：AMI 直选车检 → 正弦转向 + 27s 完成链（通电层）           | can0 + 台架    | **是**       |

## 环境变量

| 变量                                    | 默认值               | 作用                                        |
| ------------------------------------- | ----------------- | ----------------------------------------- |
| `HIL_BENCH`                           | 未设置               | `1` 启用 L3/L4 台架模式（等价于 `-n`）               |
| `HIL_INTERFACE`                       | 读 `hil_test.yaml` | CAN 接口名（脚本自动设置；直接跑 pytest 时手动设）           |
| `HIL_CONFIG`                          | 无                 | 配置目录（脚本自动设置；直接跑 pytest 时手动设）              |
| `HIL_MM_PARAMS` / `HIL_MM_HIL_PARAMS` | 无                 | L2/L4 用例自管 mission\_manager 的参数文件（脚本自动导出） |

各层级自动启动的节点：

| 层级 | 启动节点                                                  |
| -- | ----------------------------------------------------- |
| L1 | pre 不起节点（pytest fixture 自起 vcu\_sim）；post 起 can\_interface + vcu\_sim（vcan0 提供 0x501 状态源） |
| L2 | can\_interface（mission\_manager 由用例自起自停）              |
| L3 | can\_interface + mission\_manager（HIL 覆盖）+ controller |
| L4 | can\_interface + controller（mission\_manager 由用例自起自停） |

## LOG系统

**pytest 结果全部实时打印在终端**（含 `--tb=long` 完整回溯，可追溯到断言处原始报错），不生成报告文件。失败用例直接在终端看 FAIL/ERROR 详情与 traceback。

后台 FSD 节点的输出无法上终端（与 pytest 并发运行），写入节点日志。**每次运行生成一个批次目录（时间戳）**，多次执行互不覆盖，`logs/latest` 软链始终指向最新批次：

```text
logs/
└── 20260902_220701/               # 批次（时间戳）：每次运行一个
    ├── L1/                        # -l all 时每层一个子目录
    │   ├── can_interface_node.log
    │   └── vcu_sim.log            # L1 vcan0 下的 VCU 模拟
    ├── L2/
    │   └── can_interface_node.log  # L2 仅起 can_interface（mission_manager 由用例自管）
    ├── L3/
    │   ├── can_interface_node.log
    │   ├── mission_manager_node.log
    │   └── controller_node.log
    └── L4/                         # L4 仅起 can_interface + controller
        ├── can_interface_node.log
        └── controller_node.log     # mission_manager 由用例自管（日志在各用例 tmp_path）
```

- 脚本启动每个节点时打印其日志路径；节点起不来 / 行为异常时看对应日志（如 `tail -50 logs/latest/L2/can_interface_node.log`）；
- 单层或 `-l all` 跑完，脚本末尾打印 `[hil_test] <层级> 通过 / 未通过` 与本批次日志目录。

**排查定位**：用例失败直接看终端里 pytest 输出的 traceback（定位断言/报错位置）；节点行为问题看 `logs/latest/<层级>/` 下对应节点日志。

## 分步测试流程（L1 → L4）

> 每步都是一条 `./scripts/hil_test.sh -l <层级> -i <接口>` 命令：脚本自动完成「编译 FSD（可 `--no-build` 跳过）→ 准备接口 → 启动所需节点 → 跑该层全部用例（结果实时打印在终端）」。其中 **L1 分两阶段**（pre 无节点、post 起节点），脚本自动按序执行。

### 步骤 0 · 一次性准备

```bash
sudo apt install python3-yaml python3-pytest python3-colcon-common-extensions
sudo apt install can-utils        # 调试可选：candump / cansend
cd hil_test
```

CAN 内核模块（vcan0 仿真与桥接产生的本地 can0 都依赖 `vcan`/`can_raw`；一般随内核自动加载，精简内核需手动加载。周立功 USBCAN-4E-U 本身无内核驱动，经 `zlgcan_bridge` 用户态桥接，不依赖内核 CAN 驱动）：

```bash
sudo modprobe can can_raw vcan
```

脚本首次运行会自动 `colcon build` 编译 FSD（较慢），之后可加 `--no-build` 跳过。**`--no-build`** **仅当 FSD 已编译过（`WUTA-FSD/ros2_ws/install/setup.bash`** **存在）时可用**，首次运行或 FSD 代码更新后请不带该选项跑一次。编译时若出现内存溢出（进程被 OOM killer 杀掉 / 编译报错退出），加 `--lite-build` 将并行编译数限制为 1 重试。创建 vcan0 需要 sudo（脚本自动执行，会提示密码；非交互终端下请先 `sudo -v`）。

环境自检（任一步失败先解决对应依赖，再进入下一步）：

```bash
ros2 --version                                     # ROS2 已安装
python3 -c "import yaml, pytest"                   # 测试依赖
git submodule status                               # 子模块已拉取（WUTA-FSD / zlgcan_bridge，状态列无 "-" 前缀）
ip link show vcan0 2>/dev/null || echo "vcan0 未创建（脚本会自动创建）"
```

> **CAN 设备预留**：真实 CAN 硬件为**周立功 USBCAN-4E-U**（500k，`libusbcan-4e.so` 用户态驱动），经 `zlgcan_bridge` 桥接为本地 SocketCAN `can0`（vcan 类型）——接口名默认 `can0`（可在 `zlgcan_bridge/bridge.yaml` 改，改后测试的 `-i` 需同步）；`can.sim_interfaces` 列表内的接口（默认仅 `vcan0`）视为仿真（自动起 vcu\_sim、允许 0x501 注入），其余视为真实接口。完整接入步骤见下方「接入真实 VCU」。

### 接入真实 VCU（L1\~L4 前置）

> vcan0 纯仿真无需 CAN 硬件；**真实链路（L1 起）才需要接入真实 USB-CAN 适配器并上电 VCU**。

当前 CAN 盒硬件为 **周立功 USBCAN-4E-U**：USB 用户态驱动（`libusbcan-4e.so`），不是内核 SocketCAN 设备（`ip link` / `dmesg` 里看不到 CAN 网卡属正常），由 `zlgcan_bridge` 用户态桥接为本地 SocketCAN `can0`（vcan 类型）。此后 FSD `can_interface` 与 hil\_test 均无需任何修改：

```text
真实 VCU ⇄ CAN 总线(500k) ⇄ USBCAN-4E-U ⇄ libusbcan-4e.so ⇄ zlgcan_bridge ⇄ can0(vcan) ⇄ FSD can_interface / hil_test
```

**桥的完整说明（驱动安装、参数详解、日志落盘、故障排查）统一维护在 [zlgcan_bridge/README.md](zlgcan_bridge/README.md)**，本节只给最短路径：

1. **安装厂商 Linux 驱动**（一次性准备）：安装说明
   <https://fy63ozvxdv.feishu.cn/file/UC8IbcxTeo1a83x4wZbcAa26n5E>。
2. **插入并确认设备**：`lsusb -d 0471:126a` 应能列出周立功 USBCAN-4E-U（查不到时的排查见 [zlgcan_bridge/README.md](zlgcan_bridge/README.md) 的「运行」）。
3. **配置桥参数**：编辑 `zlgcan_bridge/bridge.yaml`（**无需改源码、无需重编译**，脚本每次启动时读取）。常改的只有三项——`iface`（桥接口名，默认 `can0`）、`baud`（须与 VCU 一致，本项目 500k）、`chn`（接 VCU 的 CAN 通道）；**改过 `iface` 后测试命令的 `-i` 必须同步**。也可不改文件、用命令行临时覆盖（优先级：命令行 > `bridge.yaml` > 内置默认，全部参数见 `--help`）：

```bash
sudo ./start_zlg_bridge.sh --chn=1 --log-level=debug   # 只改本次运行的通道与日志级别
```

**4. 启动桥并验证链路**（保持该终端运行）：

```bash
cd zlgcan_bridge
sudo ./start_zlg_bridge.sh              # 检查硬件 → 建 can0(vcan) → 编译 → 前台起桥
```

另开终端确认 VCU 状态帧（0x501 为事件驱动、无周期、可静默）：

```bash
candump can0        # 上电或切换状态/模式时才出现 0x501（非周期；静默属正常）
```

桥由本步骤手动启动并保持前台运行，**hil\_test 不负责启停桥**：后续 L1\~L4 直接 `-i can0` 即可（真实接口下 L1 的 vcu\_sim 模型用例与 0x501 注入用例自动跳过）。各层命令见下方「分步测试流程」。

跑不通时先看桥终端每 2s 的 `FSD->VCU / VCU->FSD / err` 收发统计（它把问题定位到「桥→VCU」还是「FSD→桥」），再对照 [zlgcan_bridge/README.md](zlgcan_bridge/README.md) 的「排查」表处理。

### 步骤 1 · L1 协议/链路自检（pre 无需 FSD，post 需 FSD）

> L1 已并入原 L0：**同一层分两阶段**——`pre` 跑协议编解码单测 + `vcu_sim` 模型（不依赖 FSD、不起任何节点），`post` 起 `can_interface` 跑链路自检 + 协议一致性。脚本按顺序自动执行两阶段（`run_hil.py --phase pre/post`）。

**目标**：① 不依赖 FSD 与硬件，先验证**协议定义**（`protocol.yaml` 编解码/定标/字节序）与 **VCU 状态机模拟**（vcu\_sim）；② 再验证 FSD 的 **can\_interface 节点**：链路通（0x210 保活 10Hz 稳定；0x501 为事件驱动状态帧）+ 协议实现与 `protocol.yaml` 一致（0x501 状态/模式 → ROS 话题、ROS 指令 → 0x210 定标透传）。

**需要设备**：vcan0 下只需开发机；真实接口需真实 VCU（上电/切模式时发 0x501 状态帧），硬件接入见上方「接入真实 VCU」。

**操作步骤**：

```bash
# 先在 vcan0 仿真跑通（pre + post 自动连跑）
cd hil_test && ./scripts/hil_test.sh -l L1 -i vcan0

# 再换真实 VCU（先在 zlgcan_bridge 下 sudo ./start_zlg_bridge.sh 起桥；需 VCU 已上电，0x501 事件驱动、可静默）
cd hil_test && ./scripts/hil_test.sh -l L1 -i can0 --no-build
```

脚本自动（vcan0）：**pre** 阶段创建/确认 vcan0 → 跑 `-m "sim or unit"`（10 个用例 = 协议单测 7 + 仿真 3，pytest 自起 vcu\_sim，**不起任何节点**）→ **post** 阶段起 can\_interface（+ vcu\_sim 提供 0x501 状态源）→ 跑 `-m "link or protocol"`（链路 2 + 集成 5）。合计 17 个用例。

**如何检验**：

- pre 预期 `10 passed`：协议单测（`test_scale_center / test_little_endian / test_clamp_out_of_range` 等）验证 0 控制→32767 中心点、小端字节序、越界钳位到 \[10, 65525]；仿真用例 `test_sim_501_event`（状态/模式变化后 0x501 到达且字节正确，事件驱动不判周期）、`test_sim_online_advance`（状态机 6→9→10）、`test_sim_state_override`（急停(12)>完成(11)>驾驶(10) 优先级）；
- post 预期 `7 passed`（vcan0）；真实接口下部分用例自动跳过，`N skipped` 属正常——链路：`test_link_can_interface_up` 接口 up、`test_link_210_heartbeat` 验证 0x210 保活周期 100ms±20%（0x501 事件驱动，不判周期）；协议集成：`test_501_go_trigger`（0x501 state=10 → `/system/start_command=true`）、`test_501_emergency_trigger`（state=12 → `/system/emergency=true`）、`test_501_mode_mapping`（Byte2=2/3/4/5/6 → 对应模式话题）、`test_210_scaling_from_command`（注入 `/control/command` → 0x210 帧定标一致）。

**失败排查**：

- vcan0 未创建/未 up：手动 `sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up`；
- pre 提示 `vcu_sim 无法启动`：vcu\_sim 由 pytest 进程内启动（无独立日志），检查 vcan0 是否 up、`protocol.yaml` 是否合法；
- pre 阶段 0x501 未到达：确认 vcan0 正常（vcu\_sim 仅在状态/模式变化时发帧）；
- post 提示 `can_interface 未运行`：确认 FSD 已编译且脚本已 source workspace；
- post 的 0x210 保活超差：确认脚本已起 can\_interface、接口未被其他进程占用；
- 真实接口下 vcu\_sim 模型用例与 0x501 注入用例被跳过不是失败，是防污染真实 VCU 的设计。

> **为什么 pre/post 必须分开**：pre 的 `vcu_sim` 模型用例需要一个"干净"总线；若与 can\_interface 同跑，其 10Hz 0x210（Signal3 默认 0）会反复把 vcu\_sim 的在线状态改回 0，使模型用例失真/失败。故脚本先跑 pre（无节点）再跑 post。

### 步骤 2 · L2 传感器自检故障模拟（断电层）

**目标**：验证 mission\_manager **传感器持续自检**与安全联动：三传感器（激光雷达 / IMU / 相机）数据缺失或断流 → `devices_inspection ok=false` → **立即切 EMERGENCY**，且故障锁存不可恢复；同时回归 HIL 覆盖（自检全关）不误报。

**真实链路只跑一次故障联动**：只有 `test_never_online_timeout` 在真实接口运行（并额外断言真实 VCU 进 EMERGENCY）；`test_mid_stream_dropout` / `test_fault_latched` 两条**仅 vcan0 运行**——它们会给真实 VCU 下发 Signal3=0 并把 VCU 锁死在 EMERGENCY 且无法回退，影响后续 L3/L4；该安全链在 vcan0（vcu\_sim）已完整覆盖。

**需要设备**：

- 真实链路：工控机 + USBCAN-4E-U（桥已起、VCU 已上电，见上方「接入真实 VCU」）；**此模式下只运行 `test_never_online_timeout` 一条故障用例**，且需保证三传感器不在线（断电层下传感器随动力电关闭）——若传感器仍在流数据，自检会通过、造不出故障；
- **车辆 / 电机台架一律不通电**——L2 是断电层，要求在给任何执行机构上电之前，先把「传感器故障 → 切 EMERGENCY」这条安全链验证通过（注意：这条会给真实 VCU 下发 Signal3=0，VCU 自身也进 EMERGENCY 并锁存，跑完须按下方「真车运行与复位」处理）；
- 或 vcan0 纯仿真预演（不需要任何硬件，全自动、无副作用，**推荐日常回归用它**）。

**准备与配置**：

- 无需改动任何配置：用例使用 mission\_manager 默认参数（`check_*` 全开），另用 `hil_test/config/hil_fsd/mission_manager.yaml`（自检全关）做回归；
- 这两个参数文件的路径由脚本导出为 `HIL_MM_PARAMS` / `HIL_MM_HIL_PARAMS`，**请勿绕过脚本直接跑 pytest**（缺变量时用例会 skip 并给出指引）；
- 脚本仅启动 can\_interface，`mission_manager` 由用例逐条起停（故障锁存与参数覆盖需要干净实例），其日志写在各用例的 tmp\_path 下。

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L2 -i vcan0            # vcan0 全自动预演（5 条全跑）
cd hil_test && ./scripts/hil_test.sh -l L2 -i can0 --no-build  # 真实链路：只跑一次故障联动，跑完需复位 VCU
```

**人工操作时序**：vcan0 无（全自动，整层约 40s）；真实链路需事先保证传感器不在线，跑完按下方「真车运行与复位」复位。

**测试流程与预期输出**（按文件内顺序执行；vcan0 预期 `5 passed`，真实链路预期 `3 passed, 2 skipped`——断流/锁存两条自动跳过）：

| 用例                                 | 用例做什么                            | 预期结果                                                              |
| ---------------------------------- | -------------------------------- | ----------------------------------------------------------------- |
| `test_selfcheck_all_pass`          | 以 10Hz 持续发布三传感器心跳 6s（覆盖 `selfcheck_grace_sec` 4s）  | `devices_inspection` ok=true 且 failures 为空；状态保持 IDLE(0) 不误报         |
| `test_never_online_timeout`        | 一帧传感器数据都不发（**真实链路唯一运行的故障用例**）     | 超宽限期后 ok=false + failures=lidar/imu/camera → **立即切 EMERGENCY(7)**；0x210 Signal3 置 0 通知 VCU；**真实链路**再断言 VCU 0x501 Byte1=12（仿真下该断言不执行，vcu\_sim 不建模此联动） |
| `test_mid_stream_dropout`          | **【仅 vcan0】** 先发布 2.5s 让三传感器上线，随后停发 | 超过 `sensor_timeout_sec`（2s）判故障 → **立即切 EMERGENCY(7)**                  |
| `test_fault_latched`               | **【仅 vcan0】** 进入 EMERGENCY 后恢复数据流并持续 6s | 状态与上报保持失败（`sensor_fault_` 锁存，需重启实例才能清）                             |
| `test_hil_override_selfcheck_disabled` | 加载 HIL 覆盖（`check_*` 全关）后静置 6s    | 不切 EMERGENCY、不上报 devices\_inspection（保证 L3/L4 台架配置可用）             |

**真车运行与复位（真实链路必读）**：真实链路只跑 `test_never_online_timeout` 一条故障用例，但它会给真实 VCU 下发 Signal3=0，**VCU 自身进入 EMERGENCY 并锁存**，且 FSD 侧 `sensor_fault_` 不可自愈。该用例是真实台架的终态用例，跑完必须复位才能继续 L3/L4：

1. 重启 `mission_manager`（清 `sensor_fault_` 内存锁存，新实例从 IDLE 开始）；
2. 确认三传感器恢复在线（否则新实例会再次自检失败、Signal3 又置 0）；
3. 按 VCU 侧流程复位 / 断电，解除 VCU 自身的 EMERGENCY；
4. `candump can0` 确认 0x501 Byte1 回到非 12、且 0x210 Byte5 回到 1；
5. 想验证自检逻辑又不碰真实 VCU 时，直接用 `-i vcan0`。

**失败排查**：

- 用例被 skip 并提示缺 `HIL_MM_PARAMS` / `HIL_MM_HIL_PARAMS`：必须用 `./scripts/hil_test.sh -l L2` 启动（脚本自动导出参数路径）；
- 提示 `mission_manager 启动失败` / `节点未就绪`：看用例终端打印的日志路径（tmp\_path 下 `mission_manager.log`），常见原因是 ROS 环境未 source，或上一实例的 DDS 残留未摘除（重跑一次即可）；
- `devices_inspection` 迟迟不上报：确认 `HIL_MM_PARAMS` 指向的 mission\_manager 参数文件里 `selfcheck_grace_sec` / `selfcheck_interval_sec` / `sensor_timeout_sec` 未被改动；
- 真实链路下 `test_mid_stream_dropout` / `test_fault_latched` 被 skip：这是设计（仅 vcan0，防污染真实 VCU），不是失败；
- 真实链路 `test_never_online_timeout` 报「真实 VCU 未进入 EMERGENCY」：确认 VCU 当前**未处于已锁存的 EMERGENCY**（先按「真车运行与复位」复位）、桥在转发 `VCU->FSD` 方向（桥终端计数）、且 VCU 固件确实把 Signal3=0 当急停处理；
- 真实链路跑完 VCU 卡在 EMERGENCY、L3/L4 起不来：按上方「真车运行与复位」复位后重试；
- 真实链路下连测试都起不来（桥未起 / VCU 未上电）：先解决 L1 的链路问题，再回到本层。

### 步骤 3 · L3 低速动态安全闭环（真实 VCU，通电层）

**目标**：确认 **AMI 选择直线加速（模式 2）+ RES Go 放行** 后 controller 真实参与控制、**台架轮子低速转动**，并验证 **RES 急停清零**：READY(1) → `mission_mode_cmd=acceleration` → 收到 Go 后进 EXPLORE(3) → controller 经 can\_interface 上 CAN（0x210）输出非零驱动开度 → 按 RES 急停后进 EMERGENCY(7) 且 **1s 内 0x210 纵向开度清零**。

**需要设备**：

- 工控机 + USBCAN-4E-U（桥已起、VCU 已上电，见上方「接入真实 VCU」）；
- **车辆架起、动力电接通**（四轮离地）；**AMI**（驾驶模式显示屏）可用；**RES Go** 与 **RES 急停** 可按；台架清场、专人守 RES 急停；
- 或 vcan0 先预演（vcu\_sim 自动驱动状态推进，不需要 `-n`）。

**准备与配置**：

- **安全限速**（本层唯一旋钮）：`hil_test/config/hil_test.yaml` 的 `bench.target_speed_mps`（默认 **0.2**）。台架把 `/chcnav/velocity` 代发为 0，该值经 waypoints 成为 controller 目标速度，并近似等比例决定纵向开度（`kp=1.0`），故须保持低位以免架起的轮子高速空转——**先低后调**；
- **HIL 覆盖自动加载**：`hil_test/config/hil_fsd/mission_manager.yaml`（自检全关）由脚本加载，无需手改；
- **前提**：L1/L2 已通过——否则 mission\_manager 会被传感器自检锁死在 EMERGENCY，不会响应 AMI 选择 / Go；
- **安全**：本层有驱动输出，台架必须架起、清场并通电，专人守 RES 急停。

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L3 -i vcan0              # vcan0 预演（无人工操作）
cd hil_test && ./scripts/hil_test.sh -l L3 -i can0 -n --no-build # 台架（-n=HIL_BENCH=1；需已起桥）
```

脚本自动：起 can\_interface + mission\_manager（HIL 覆盖关自检）+ **controller** → hil\_test 代发就绪信号 / 位姿 / 车速 / 直路路径 → 跑 `-m motor`。

**测试流程与人工操作时序**（需人工的用例会在终端打印提示，每 5s 打印剩余时间，超时 30s 判失败）：

| 用例                         | 需要你做什么                                       | 预期结果                                                                        |
| -------------------------- | -------------------------------------------- | --------------------------------------------------------------------------- |
| `test_ami_acceleration_go` | **先在 AMI 上选直线加速（模式 2），再按 RES Go**（VCU 进驾驶态 10） | READY(1) → `mission_mode_cmd=acceleration` → 收到 Go 后进 EXPLORE(3)              |
| `test_low_speed_follow`    | 无需操作（承接 EXPLORE 态）                            | 代发低速直路（`bench.target_speed_mps`）+ 车速反馈 0 → **0x210 出现非零驱动开度**（轮子动）        |
| `test_emergency_stop`      | **按 RES 急停**（承接 EXPLORE 态）                    | EXPLORE(3) → `/system/emergency=true` → EMERGENCY(7)，且 **1s 内 0x210 纵向清零**（终端打印实测响应时间） |

> 三用例按文件顺序执行：先 AMI 选直线加速 + Go 进入 EXPLORE，再验证低速驱动（轮子动），最后测 RES 急停清零。急停为终态，故置于末位。

真实台架预期 `3 passed`。

**失败排查**：

- 一直停在 READY 不进 EXPLORE：先确认 AMI 选的是**模式 2（直线加速）**、且确实按了 RES Go；再看 `logs/latest/L3/mission_manager_node.log`；
- 没进 READY / 一直 EMERGENCY：确认 mission\_manager 已加载 HIL 覆盖（被传感器自检锁死时参见 L2），并确认实例是本次新起的；
- `test_low_speed_follow` 报 `0x210 无驱动开度`：确认 controller 已随脚本启动（`logs/latest/L3/controller_node.log` 应有 `cmd(... vel=...)` 输出），且 `bench.target_speed_mps > 0`；
- 急停用例不进 EMERGENCY(7) / 清零超时：确认按的是 RES 急停、且 `mission_mode_cmd` 已放行（急停须在 EXPLORE 态下触发）；清零慢可看 `logs/latest/L3/can_interface_node.log`。

**前提与安全**：**L1/L2 全部通过后再接入真实 VCU**；台架必须架起通电、清场并专人守 RES 急停；AMI 模式须选对，RES Go / RES 急停为 VCU 侧真实信号。

### 步骤 4 · L4 车检任务全链路（台架，通电层）

**目标**：**AMI 直接选择车检**（模式 6，无需 Go 放行）→ INSPECTION 全链路：1.0 m/s 纵向驱动 + 正弦转向（15°@0.4Hz）→ `inspection_duration`（**27s**）后完成链（回零 + mission\_complete → FINISH + 0x210 finished=1）；另覆盖车检中 RES 急停立即停车。

**需要设备**：

- 工控机 + USBCAN-4E-U（桥已起、VCU 已上电，见上方「接入真实 VCU」）；
- **车辆架起、动力电接通**（四轮离地）；**RES 急停按钮**随手可按；**AMI** 可用（车检模式选择必需）；
- 或 vcan0 先预演（vcu\_sim 自动驱动状态推进，不需要 `-n`、也不需要 AMI）。

**准备与配置**：

- **车检参数**在 `WUTA-FSD/ros2_ws/src/control/controller/config/controller.yaml`：`inspection_duration`（默认 **27.0s**）、`inspection_speed`（1.0 m/s）、`inspection_steer_amp`（15°）、`inspection_steer_freq`（0.4Hz）；改后只需重启 controller，无需重新编译。**L4 车速即取此处的 `inspection_speed`，不走 L3 的 `bench.target_speed_mps`**；
- **HIL 覆盖自动加载**：`hil_test/config/hil_fsd/mission_manager.yaml`（自检全关），无需手改；
- **前提**：L1\~L3 已通过（L3 已验证 AMI 选模式 + RES Go → EXPLORE 并低速驱动）；
- **安全**：台架清场、车辆架起，专人守 RES 急停。

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L4 -i vcan0              # vcan0 预演（无人工操作）
cd hil_test && ./scripts/hil_test.sh -l L4 -i can0 -n --no-build # 台架（-n=HIL_BENCH=1；需已起桥）
```

脚本自动：起 can\_interface + controller → hil\_test 代发就绪信号 / 位姿 / 车速 → 跑 `-m inspection`。**mission\_manager 由用例逐条自起自停**：因为 FINISH(6) / EMERGENCY(7) 是终态且模式仅 IDLE/READY 可改，同一实例只能进车检一次，故每个用例都重启一个新实例（状态机从 IDLE 开始）。

**测试流程与人工操作时序**：**唯一人工操作是在 AMI 上选择车检模式（6）**，而且**每个用例都要重选一次**——终端会打印 `[L4] 等待操作: 用 AMI 选择车检模式（6）（超时 30s）`，每 5s 打印剩余时间。急停用例另需按 RES 急停。

| 用例                                | 需要你做什么                            | 预期结果                                                                    |
| --------------------------------- | --------------------------------- | ----------------------------------------------------------------------- |
| `test_ami_inspection_entry`       | **AMI 选车检模式（6）**（约 1s）            | `mission_mode_cmd=inspection` → **直接进 INSPECTION(2)**（无需 Go 放行）           |
| `test_inspection_motion`          | 再次 **AMI 选 6**；随后自动观测约 6s         | 纵向驱动开度非零（车速反馈代发 0.0 → 保持速度误差）+ 正弦转向幅值 \|lateral−32767\|>10000 raw 且出现过零 |
| `test_inspection_complete`        | 再次 **AMI 选 6**；随后等待约 27s          | 27s 后回零 + 进 FINISH(6) + 0x210 Byte6 finished=1                            |
| `test_emergency_during_inspection` | 再次 **AMI 选 6**，待车辆运动约 1s 后**按 RES 急停** | `/system/emergency=true`，**1s 内 0x210 清零**（终端打印实测响应时间）                    |

真实台架预期 `4 passed`。

**失败排查**：

- 用例超时并提示 `未收到 mission_mode_cmd=inspection`：说明 30s 内没在 AMI 上选到车检模式（或选成了其它模式）——按终端提示重新选 6；
- **用例之间必须重新选模式**：上一条用例跑完后实例已进 FINISH/EMERGENCY（终态），本用例是新实例、需重新选 6，这不是缺陷；
- 未进 INSPECTION 而报 `选择车检后未直接进入 INSPECTION`：看用例 tmp\_path 下 `mission_manager.log`，确认实例确实起来了（也确认没被传感器自检锁死，参见 L2）；
- 27s 未完成：核对 `controller.yaml` 的 `inspection_duration`（改过 yaml 后要重启 controller 才生效）；
- 正弦幅值 / 过零未观测到：确认观测窗口内 0x210 持续（`candump can0`、`logs/latest/L4/controller_node.log`）。

## 依赖

- Python 3.10+、pytest、PyYAML（系统包即可）
- **CAN 层用纯 stdlib SocketCAN**（无需 python-can）
- ROS 集成用例需 ROS2（humble）+ FSD workspace（脚本自动 source）
- 调试可选：can-utils（candump / cansend）；CAN 内核模块（can / can\_raw / vcan）



## 常见问题速查

脚本级 / 环境级报错按「症状 → 原因 → 处理」集中排查（用例级失败直接看终端里 pytest 的 traceback）：

| 症状                                         | 原因                                                         | 处理                                                                                                                            |
| ------------------------------------------ | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| 脚本报 `cd: WUTA-FSD/ros2_ws: 没有那个文件或目录`      | WUTA-FSD 子模块未初始化                                           | `git submodule update --init --recursive`                                                                                     |
| 创建 vcan0 报 `Operation not permitted`       | 无 CAP\_NET\_ADMIN（容器 / 权限受限）                               | 换真实开发机或提权运行                                                                                                                   |
| 创建 vcan0 提示 `sudo: a terminal is required` | 非交互终端下 sudo 无法读密码                                          | 先 `sudo -v` 预授权，或换交互终端                                                                                                        |
| 集成用例超时（话题收不到）                              | ROS\_DOMAIN\_ID 不一致 / 节点未启动 / workspace 未 source           | 两端设置相同 `ROS_DOMAIN_ID`；`ros2 node list` 确认节点在线                                                                                |
| 真实接口下桥启动失败 / CAN 收不到帧（`ZCAN_OpenDevice`、接口创建失败等） | 桥侧问题：VCU 未上电、`baud`/`chn` 不符、桥未运行或已 bus-off、驱动未装、设备被占用 | 见 [zlgcan_bridge/README.md](zlgcan_bridge/README.md) 的「排查」表 |
| 提示 `can_interface 未运行`                     | FSD 未编译 / 未 source workspace                               | 看 `logs/latest/L1/can_interface_node.log`，确认执行过编译                                                                             |
| 编译 FSD 时进程被杀（Killed / OOM）                 | 全量并行编译内存不足                                                 | 加 `--lite-build` 限制并行编译数为 1 重试                                                                                                |



## 开发时Git 子模块说明

`WUTA-FSD` 与 `zlgcan_bridge` 都是通过 git submodule 引入的独立仓库（`https://github.com/GaoMingHa0/WUTA-FSD.git` / `https://github.com/707GLobal/zlgcan_bridge.git`），提交记录与主仓库相互独立，主仓库只记录其当前指向的 commit。下方操作以 WUTA-FSD 为例，对 zlgcan\_bridge 同样适用（对应改路径/名称即可）。

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

