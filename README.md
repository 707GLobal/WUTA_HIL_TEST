# WUTA\_HIL\_TEST

FSD HIL（硬件在环）测试工程：工控机上的 FSD 与**真实 VCU** 通过 CAN 总线双向通信，验证协议、链路、安全与电机台架闭环。包含两部分：

- **WUTA-FSD**：FSD 算法仓库（git 子模块）
- **hil\_test**：分层测试框架（L0 \~ L3），一键脚本完成「编译 FSD → 准备接口 → 启动节点 → 跑测试 → 输出结果」

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
│   │   ├── vcu_sim.py           # L0：vcan 模拟 VCU（状态机+脚本切换，含 CLI）
│   │   └── fault_injector.py    # CAN 故障注入（急停帧等）
│   ├── test/
│   │   ├── test_protocol.py     # L0/L1
│   │   ├── test_safety.py       # L2
│   │   └── test_motor_hil.py    # L3（slow，需 HIL_BENCH=1）
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

| 层级 | 内容                                      | 环境           | 真实 VCU      |
| -- | --------------------------------------- | ------------ | ----------- |
| L0 | 纯仿真验证：协议编解码单测 + vcu\_sim 模拟 VCU         | 开发机 vcan0    | 否           |
| L1 | 链路自检 + 协议一致性：心跳 / 0x501→话题 / 0x210 定标透传 | vcan0 或 can0 | 否（vcan0 仿真） |
| L2 | 安全与状态联动：门控 / Go / 急停 / 车检               | can0         | **是**       |
| L3 | 电机闭环：恒速 / 斜坡 / 急停刹停                     | can0 + 台架    | **是**       |

## 快速开始

```bash
cd hil_test
./scripts/hil_test.sh                     # 交互：编译 FSD → 选链路 → 跑 → 出结果
./scripts/hil_test.sh -l L1 -i vcan0      # 直接指定链路与接口
./scripts/hil_test.sh -l all -i vcan0     # 一键流水线：L0→L1→L2 连跑（失败即停）
./scripts/hil_test.sh -l L3 -i can0 -n    # L3 台架（-n 开启台架模式；需先起桥，见「接入真实 VCU」）
./scripts/hil_test.sh -l L1 -i can0       # 真实 ZLG USB-CAN（需先起桥，见「接入真实 VCU」）
./scripts/hil_test.sh --no-build -l L0    # 跳过编译
./scripts/hil_test.sh -c -l L1            # 编译前清理 FSD 缓存（build/install/log），全量重编
./scripts/hil_test.sh -p "can_interface mission_manager controller" -l L2  # 只编指定包
./scripts/hil_test.sh --lite-build -l L1  # Lite 编译：并行编译数限制为 1（内存受限防 OOM）
./scripts/hil_test.sh -l L1 -k            # 测试后保留节点（调试）
```

> `-l all`：仿真接口（vcan0）下全自动连跑 L0→L1→L2，任一层失败即停；真实接口下只跑 L1，L2 需人工配合（RES/AMI），请单独运行 `-l L2`。
>
> `-k` 保留的 FSD 节点在脚本退出后仍驻留，排查完请手动停止：`ps aux | grep "ros2 run"` 找到进程后 kill（或直接关闭对应终端）。
>
> **真实接口（can0）需先手动起桥**：`cd zlgcan_bridge && sudo ./start_zlg_bridge.sh`（保持该终端运行），再用 `-i can0` 跑测试。hil_test 不再自动启停桥。

### 环境变量

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `HIL_BENCH` | 未设置 | `1` 启用 L3 台架模式（等价于 `-n`） |
| `HIL_INTERFACE` | 读 `hil_test.yaml` | CAN 接口名（脚本自动设置；直接跑 pytest 时手动设） |
| `HIL_CONFIG` | 无 | 配置目录（脚本自动设置；直接跑 pytest 时手动设） |

各链路自动启动的节点：

| 链路 | 启动节点                                                  |
| -- | ----------------------------------------------------- |
| L0 | 无（pytest fixture 自起 vcu\_sim）                         |
| L1 | can\_interface + vcu\_sim（vcan0 下补 0x501）             |
| L2 | can\_interface + mission\_manager（HIL 覆盖）+ controller |
| L3 | can\_interface + controller                           |

### 测试输出与日志

**pytest 结果全部实时打印在终端**（含 `--tb=long` 完整回溯，可追溯到断言处原始报错），不生成报告文件。失败用例直接在终端看 FAIL/ERROR 详情与 traceback。

后台 FSD 节点的输出无法上终端（与 pytest 并发运行），写入节点日志。**每次运行生成一个批次目录（时间戳）**，多次执行互不覆盖，`logs/latest` 软链始终指向最新批次：

```text
logs/
└── 20260902_220701/               # 批次（时间戳）：每次运行一个
    ├── L1/                        # -l all 时每层一个子目录
    │   ├── can_interface_node.log
    │   └── vcu_sim.log            # L1 vcan0 下的 VCU 模拟
    └── L2/
        ├── can_interface_node.log
        ├── mission_manager_node.log
        └── controller_node.log
```

- 脚本启动每个节点时打印其日志路径；节点起不来 / 行为异常时看对应日志（如 `tail -50 logs/latest/L2/can_interface_node.log`）；
- 单层或 `-l all` 跑完，脚本末尾打印 `[hil_test] <层级> 通过 / 未通过` 与本批次日志目录。

**排查定位**：用例失败直接看终端里 pytest 输出的 traceback（定位断言/报错位置）；节点行为问题看 `logs/latest/<层级>/` 下对应节点日志。

## 分步测试流程（L0 → L3）

> 每步都是一条 `./scripts/hil_test.sh -l <层级> -i <接口>` 命令：脚本自动完成「编译 FSD（可 `--no-build` 跳过）→ 准备接口 → 启动所需节点 → 跑该层全部用例（结果实时打印在终端）」

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

脚本首次运行会自动 `colcon build` 编译 FSD（较慢），之后可加 `--no-build` 跳过。**`--no-build` 仅当 FSD 已编译过（`WUTA-FSD/ros2_ws/install/setup.bash` 存在）时可用**，首次运行或 FSD 代码更新后请不带该选项跑一次。编译时若出现内存溢出（进程被 OOM killer 杀掉 / 编译报错退出），加 `--lite-build` 将并行编译数限制为 1 重试。创建 vcan0 需要 sudo（脚本自动执行，会提示密码；非交互终端下请先 `sudo -v`）。

环境自检（任一步失败先解决对应依赖，再进入下一步）：

```bash
ros2 --version                                     # ROS2 已安装
python3 -c "import yaml, pytest"                   # 测试依赖
git submodule status                               # 子模块已拉取（WUTA-FSD / zlgcan_bridge，状态列无 "-" 前缀）
ip link show vcan0 2>/dev/null || echo "vcan0 未创建（脚本会自动创建）"
```

> **CAN 设备预留**：真实 CAN 硬件为**周立功 USBCAN-4E-U**（500k，`libusbcan-4e.so` 用户态驱动），经 `zlgcan_bridge` 桥接为本地 SocketCAN `can0`（vcan 类型）——接口名默认 `can0`（可在 `zlgcan_bridge/bridge.yaml` 改，改后测试的 `-i` 需同步）；`can.sim_interfaces` 列表内的接口（默认仅 `vcan0`）视为仿真（自动起 vcu\_sim、允许 0x501 注入），其余视为真实接口。完整接入步骤见下方「接入真实 VCU」。

### 接入真实 VCU（L1/L2/L3 前置）

> L0 纯仿真验证无需 CAN 硬件；**L1 真实链路与 L2/L3 才需要接入真实 USB-CAN 适配器并上电 VCU**。
> 当前 CAN 盒硬件为 **周立功 USBCAN-4E-U**：USB 用户态驱动（`libusbcan-4e.so`），不是内核 SocketCAN 设备（`ip link` / `dmesg` 里看不到 CAN 网卡属正常），由 `zlgcan_bridge` 用户态桥接为本地 SocketCAN `can0`（vcan 类型）。此后 FSD `can_interface` 与 hil_test 均无需任何修改：

```text
真实 VCU ⇄ CAN 总线(500k) ⇄ USBCAN-4E-U ⇄ libusbcan-4e.so ⇄ zlgcan_bridge ⇄ can0(vcan) ⇄ FSD can_interface / hil_test
```

**1. 安装厂商 Linux 驱动**（一次性准备）：
已安装，后面再补充 https://fy63ozvxdv.feishu.cn/file/UC8IbcxTeo1a83x4wZbcAa26n5E

**2. 插入并确认设备**：`lsusb -d 0471:126a` 能看到周立功 USBCAN-4E-U 即可（无内核驱动，`dmesg`/`ip link` 看不到属正常）。

**3. 配置桥参数**：编辑 `zlgcan_bridge/bridge.yaml`（常用参数集中在此，**无需改源码**）。读取该文件依赖 PyYAML（步骤 0 已装 `python3-yaml`）；若 `python3 -c "import yaml"` 不可用，会**静默回落到内置默认值**（`devtype=31 devidx=0 chn=0 baud=500000 iface=can0`），此时改 yaml 不生效，请先补装 PyYAML：

```yaml
devtype: 31        # ZLG 设备类型: 31=USBCAN-4E-U
devidx: 0          # 同型号设备序号
chn: 0             # 使用的 CAN 通道（4E-U 有 0~3）
baud: 500000       # CAN 波特率，须与 VCU 一致
iface: can0        # 桥输出的 SocketCAN 接口名（不可用 vcan* 开头）
```

**4. 启动桥并验证链路**（保持该终端运行；VCU 上电后应能持续抓到 0x501）：

```bash
cd zlgcan_bridge
sudo ./start_zlg_bridge.sh              # 检查硬件 → 建 can0(vcan) → 编译 → 前台起桥
sudo ./start_zlg_bridge.sh --chn=1      # 临时覆盖 bridge.yaml 中的通道（优先级：命令行 > yaml > 默认）
sudo ./start_zlg_bridge.sh --help       # 查看全部参数
```

桥每 2 秒打印一次收发统计，**这是判断问题出在哪一段的关键**：

```text
[bridge] FSD->VCU:129  VCU->FSD:0  (err 0/0)
```

另开终端验证：

```bash
candump can0        # VCU 上电应持续看到 0x501，周期 100ms
```

**5. 经 hil_test 运行**（真实接口用 `-i can0`；桥已在上一步手动起好，脚本不再自动启停桥）：

```bash
cd hil_test
./scripts/hil_test.sh -l L1 -i can0 --no-build      # 链路自检 + 协议一致性（VCU 需在发 0x501）
./scripts/hil_test.sh -l L2 -i can0 --no-build      # 安全联动（需人工 RES/AMI 配合）
./scripts/hil_test.sh -l L3 -i can0 -n --no-build   # L3 电机闭环（台架，-n=HIL_BENCH）
```

> 真实接口语义与仿真不同：脚本拒绝 `-l L0`（防模拟帧污染真实 VCU），L1 的 0x501 注入用例自动跳过。

**排查**：

- 桥启动失败 `ZCAN_OpenDevice 失败`：驱动未装（`/usr/lib/libusbcan-4e.so` 缺失）/ 未用 sudo / 设备被其他进程占用——**同一 ZLG 设备仅一个进程可打开**（`pgrep -af zlg_can_bridge` 查是否有实例在跑）；
- VCU 上电后 `candump can0` 抓不到 0x501：先看桥终端的 `VCU->FSD` 计数与 `err`——
  - `VCU->FSD` 恒为 0 → ZLG 侧没收到：查 VCU 是否上电并在发 0x501、接线（CAN_H/CAN_L、共地）、120Ω 终端电阻、`bridge.yaml` 的 `chn` 与 `baud`；
  - `err` 持续增长 → ZLG 收发报错（接线/波特率）；若桥先于 VCU 上电启动，持续发送会因无应答累积错误直至 **bus-off**（当前桥无自动恢复），请 Ctrl-C 重启桥（**建议先上电 VCU，再起桥**）；
- 测试提示 `can_interface 未运行` 或心跳超时：看节点日志 `logs/latest/<层级>/can_interface_node.log`。

### 步骤 1 · L0 纯仿真验证（无硬件、无 FSD）

**目标**：不依赖 FSD 代码与任何 CAN 硬件，先把**协议定义**（`protocol.yaml` 的编解码/定标/字节序）和 **VCU 状态机模拟**（vcu\_sim）在软件层跑通，避免每次调试协议都接真实 VCU。

**需要设备**：仅需工控机，无需 CAN 硬件

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L0 -i vcan0
```

脚本自动：编译 FSD（可 `--no-build`）→ 创建/确认 vcan0 → pytest 自起 vcu\_sim → 跑 `-m "sim or unit"`（10 个用例 = 协议单测 7 + 仿真 3）。

**如何检验**：

- 预期终端输出 `10 passed`；
- 协议单测（`test_scale_center / test_little_endian / test_clamp_out_of_range` 等）：验证 0 控制→32767 中心点、小端字节序、越界钳位到 \[10, 65525]；
- 仿真用例：`test_sim_501_heartbeat` 验证 0x501 心跳周期 100ms±20%；`test_sim_online_advance` 验证状态机 6→9→10；`test_sim_state_override` 验证急停(12)>完成(11)>驾驶(10) 优先级。

**失败排查**：

- vcan0 未创建/未 up：手动 `sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up`；
- 提示 `vcu_sim 无法启动`：L0 的 vcu_sim 由 pytest 进程内启动（无独立日志），检查 vcan0 是否 up、`protocol.yaml` 是否合法；
- 心跳周期超差：确认 vcan0 无其他进程在发 0x501 干扰。

> 注意：L0 **仅限仿真接口**，对真实接口运行会直接报错退出（防模拟帧污染真实 VCU）。

### 步骤 2 · L1 链路自检 + 协议一致性（需 FSD）

**目标**：验证 FSD 的 **can\_interface 节点**：① 链路通（0x210/0x501 心跳 10Hz 稳定）；② 协议实现与 `protocol.yaml` 一致（0x501 状态/模式 → ROS 话题、ROS 指令 → 0x210 定标透传）。

**需要设备**：vcan0 下只需开发机；真实接口需真实 VCU（上电发 0x501），硬件接入见上方「接入真实 VCU」。

**操作步骤**：

```bash
# 先在 vcan0 仿真跑通（vcu_sim 自动补 0x501 心跳）
cd hil_test && ./scripts/hil_test.sh -l L1 -i vcan0

# 再换真实 VCU（先在 zlgcan_bridge 下 sudo ./start_zlg_bridge.sh 起桥；需 VCU 已上电并在发 0x501）
cd hil_test && ./scripts/hil_test.sh -l L1 -i can0 --no-build
```

脚本自动：起 can\_interface（发 0x210 心跳；vcan0 下另起 vcu\_sim 补 0x501）→ 跑 `-m "link or protocol"`（链路 3 + 集成 5）。

**如何检验**：

- 预期 `8 passed, N skipped`（真实接口下跳过 0x501 注入用例属正常）；
- 链路：`test_link_can_interface_up` 接口 up；`test_link_210_heartbeat / test_link_501_heartbeat` 心跳周期 100ms±20%；
- 协议集成：`test_501_go_trigger`（0x501 state=10 → `/system/start_command=true`）；`test_501_emergency_trigger`（state=12 → `/system/emergency=true`）；`test_501_mode_mapping`（Byte2=2/3/4/5/6 → 对应模式话题）；`test_210_scaling_from_command`（注入 `/control/command` → 0x210 帧定标一致）。

**失败排查**：

- 提示 `can_interface 未运行`：确认 FSD 已编译且脚本已 source workspace；
- 心跳超时：真实接口下先确认 VCU 已上电发 0x501（`candump can0` 应能抓到）；
- 真实接口下集成用例被跳过不是失败，是防污染真实 VCU 的设计。

### 步骤 3 · L2 安全与状态联动（真实 VCU）

**目标**：验证 FSD 与真实 VCU 的**状态联动与安全逻辑**：启动门控、Go 放行、急停优先级、车检流程。

**需要设备**：真实 VCU（上电，接入见上方「接入真实 VCU」）；vcan0 可先全自动预演。

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L2 -i vcan0      # 先 vcan0 全自动预演
cd hil_test && ./scripts/hil_test.sh -l L2 -i can0 --no-build  # 真实 VCU（需先起桥；部分用例需人工）
```

脚本自动：起 can\_interface + mission\_manager（自动加载 HIL 覆盖关闭传感器自检）+ controller → 跑 `-m safety`。

真实接口模式下，脚本会**等待你操作**（每 5s 打印剩余时间提示）：

1. `test_go_release`：等待你按 **RES 给 Go**（VCU 进入驾驶态 10）；
2. `test_emergency_priority`：等待你触发 **RES 急停**（VCU 状态 12）；
3. `test_inspection_flow`：等待你用 **AMI 选择车检**（模式 6）。

**如何检验**：

- vcan0 全自动预演应全绿（vcu\_sim 自动驱动状态）；真实接口下按提示操作后对应用例通过；
- `test_start_gate_zero_output`：未给 Go 时总线 0x210 恒为零控制（32767）；
- `test_go_release`：Go 后收到 `start_command=true` 且 mission\_state 推进到 EXPLORE(3)；
- `test_emergency_priority`：急停后 `/system/emergency=true` 且 0x210 回零；
- 看门狗用例**跳过**（依赖 can\_interface VCU 失联检测，待开发）。

**失败排查**：状态推进超时——确认 mission\_manager 已加载 HIL 覆盖（`config/hil_fsd/mission_manager.yaml`），否则传感器自检会锁死 EMERGENCY；人工操作请在等待提示内完成（超时 30s 判失败）。

### 步骤 4 · L3 电机闭环（台架）

**目标**：电机台架上验证 controller 真实闭环：恒速跟随、加减速斜坡、急停刹停时间。

**需要设备**：真实 VCU + **电机台架**（车辆架起，传感器仅保在线），硬件接入见上方「接入真实 VCU」。

**操作步骤**：

```bash
cd hil_test && ./scripts/hil_test.sh -l L3 -i can0 -n --no-build  # 需先起桥；-n 台架模式（HIL_BENCH=1）
```

脚本自动：起 can\_interface + controller（定位/感知/规划不启动）→ hil\_test 代发位姿 / 车速 / 直路路径 / 就绪信号 → 跑 `-m motor`。

**如何检验**：

- 恒速跟随：恒速段出现稳定驱动开度、速度误差在阈值内；
- 加减速斜坡：指令变化后输出平滑过渡并回零（不突变）；
- 急停刹停：急停触发后记录刹停时间，仿真自动触发可量化对比。

**前提与安全**：**L2 全部通过后再给台架通电**；人工急停（RES）随时可用。

### 直接跑 pytest（跳过脚本）

```bash
cd hil_test
python -m pytest test/test_protocol.py -m unit -q        # L0 纯协议单测（无硬件/无 FSD/无需 vcan）
python -m pytest test/test_protocol.py -m "sim or unit"  # L0 完整（需 vcan0）
python -m pytest test/test_protocol.py -m "link or protocol" -q  # L1（需 can_interface 运行）
python -m pytest test/test_safety.py -m safety -q        # L2（需节点 + 真实 VCU/人工）
```

> 直接跑 pytest 时需自行 source ROS 环境并先启动 FSD 节点（集成用例），依赖关系见下文 marker 表。

## 依赖

- Python 3.10+、pytest、PyYAML（系统包即可）
- **CAN 层用纯 stdlib SocketCAN**（无需 python-can）
- ROS 集成用例需 ROS2（humble）+ FSD workspace（脚本自动 source）
- 调试可选：can-utils（candump / cansend）；CAN 内核模块（can / can_raw / vcan）

## HIL 台架模式（车辆架起，传感器仅保在线）

台架下相机 / 激光雷达 / 华测**不参与决策**，由 hil\_test 代发以下信号（`RosInjector` 方法）：

| 代发话题                             | 方法                                    |
| -------------------------------- | ------------------------------------- |
| `/system/lidar_ready`            | `publish_lidar_ready()`               |
| `/system/localization_ready`     | `publish_localization_ready()`        |
| `/localization/pose`             | `publish_pose(x, y, yaw)`             |
| `/chcnav/velocity`               | `publish_velocity(vx)`                |
| `/planning/final_waypoints`      | `publish_waypoints_straight()`        |
| `/system/devices_inspection`（上线） | `publish_devices_inspection(ok=True)` |

关键点：

- **mission\_manager 必须关闭传感器自检**（脚本自动加载 `hil_test/config/hil_fsd/mission_manager.yaml`），否则华测未接 → 自检超时 → 锁死 EMERGENCY；
- **L3** 运行子集 can\_interface + controller，mission\_state 由 hil\_test 注入 RACE 使能。

## 分层 marker 与跳过规则

| marker     | 级别 | 运行依赖                                         | 跳过条件                                   |
| ---------- | -- | -------------------------------------------- | -------------------------------------- |
| `unit`     | L0 | 无（纯协议单测）                                     | 无                                      |
| `sim`      | L0 | vcan0 + vcu\_sim（无 ROS）                      | 接口未 up                                 |
| `link`     | L1 | can\_interface + CAN 接口                      | 接口未 up / can\_interface 未运行            |
| `protocol` | L1 | can\_interface + 仿真接口（注入用例）                  | 接口未 up / can\_interface 未运行 / 真实接口注入用例 |
| `safety`   | L2 | can\_interface + mission\_manager/controller | can\_interface 未运行；看门狗用例恒跳过            |
| `motor`    | L3 | 台架 + can\_interface + controller             | `HIL_BENCH` 未设 / can\_interface 未运行    |

## 常见问题速查

脚本级 / 环境级报错按「症状 → 原因 → 处理」集中排查（用例级失败直接看终端里 pytest 的 traceback）：

| 症状 | 原因 | 处理 |
| --- | --- | --- |
| 脚本报 `cd: WUTA-FSD/ros2_ws: 没有那个文件或目录` | WUTA-FSD 子模块未初始化 | `git submodule update --init --recursive` |
| 创建 vcan0 报 `Operation not permitted` | 无 CAP_NET_ADMIN（容器 / 权限受限） | 换真实开发机或提权运行 |
| 创建 vcan0 提示 `sudo: a terminal is required` | 非交互终端下 sudo 无法读密码 | 先 `sudo -v` 预授权，或换交互终端 |
| 集成用例超时（话题收不到） | ROS_DOMAIN_ID 不一致 / 节点未启动 / workspace 未 source | 两端设置相同 `ROS_DOMAIN_ID`；`ros2 node list` 确认节点在线 |
| 真实接口 0x501 心跳超时 | VCU 未上电 / 波特率不匹配 / 桥未运行或已 bus-off | 看桥终端 `[bridge] ... VCU->FSD:x  (err a/b)`；`candump can0` 抓帧；核对 `zlgcan_bridge/bridge.yaml` 的 `baud`（默认 500k，须与 VCU 一致）与 `chn` |
| 桥启动失败 `ZCAN_OpenDevice` | `libusbcan-4e.so` 未安装或路径不对；未用 sudo；设备被其他进程占用（同一设备仅一个进程可打开） | `ls -l /usr/lib/libusbcan-4e.so` 确认已装（缺则运行 ZLG 官方安装器）；`pgrep -af zlg_can_bridge` 查是否有实例在跑 |
| 提示 `can_interface 未运行` | FSD 未编译 / 未 source workspace | 看 `logs/latest/L1/can_interface_node.log`，确认执行过编译 |
| 编译 FSD 时进程被杀（Killed / OOM） | 全量并行编译内存不足 | 加 `--lite-build` 限制并行编译数为 1 重试 |

## 安全约定

- L0 仅限仿真接口（`sim_interfaces`，如 vcan0），脚本对真实接口直接报错；L1 的 0x501 注入用例**仅限仿真接口**，真实接口下自动跳过（防污染真实 VCU）
- 台架通电前先通过 L2 门控 / 急停用例，人工急停（RES）随时可用
- 看门狗用例待 can\_interface VCU 失联检测实现后启用（当前跳过）

## 开发时Git 子模块说明

`WUTA-FSD` 与 `zlgcan_bridge` 都是通过 git submodule 引入的独立仓库（`https://github.com/GaoMingHa0/WUTA-FSD.git` / `https://github.com/707GLobal/zlgcan_bridge.git`），提交记录与主仓库相互独立，主仓库只记录其当前指向的 commit。下方操作以 WUTA-FSD 为例，对 zlgcan_bridge 同样适用（对应改路径/名称即可）。

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

