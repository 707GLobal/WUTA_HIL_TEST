# FSD HIL 测试方案

## 1. 背景与目标

HIL 使用\*\*真实 VCU（整车控制器）\*\*直接连接工控机：

- 工控机侧 `can_interface` 经 CAN 总线与真实 VCU 双向通信，收发行为与实车一致；
- VCU 侧 AMI（任务选择）/ RES（Go / 急停）/ 安全回路等为真实电气信号，测试其真实行为；
- VCU 驱动真实电机台架（电机 + 驱动器 + 编码器），编码器/车速反馈经 VCU 回传工控机，形成完整闭环。

意义：

- 在真车跑车前，用台架闭环暴露链路、协议、安全、执行机构层面的问题；
- FSD 决策 / 控制链路跑在真实工控机与真实 CAN 总线上，行为与实车一致；
- 协议已定稿（0x210 / 0x501，见《26赛季 VCU - 工控机 CAN通信协议规划》），测试框架按协议配置驱动，协议变更只改 `protocol.yaml`、不改测试代码。

## 2. 系统架构

```
┌────────────────────────┐        CAN 500k        ┌──────────────────────────┐
│  工控机（FSD 全套）      │◀──────────────────────▶│  真实 VCU（整车控制器）  │
│  mission_manager        │      can0 (USB-CAN)    │    ├─ AMI 任务选择        │
│  controller             │                        │    ├─ RES Go / 急停       │
│  localization / …       │                        │    ├─ 安全回路 / 车速反馈   │
│  can_interface          │                        │    └─ 驱动电机 / 转向      │
│  hil_test（测试程序）     │                        │            │             │
└────────────────────────┘                        ┌────────────▼──────────┐
                                                  │  电机台架（真实硬件）      │
                                                  │  电机 + 驱动器 + 编码器   │
                                                  └────────────────────────┘
```

关键设计点：

- **VCU 为真实硬件**：HIL 测试验证的是工控机与真实电控的完整交互，不是模拟器；
- **工控机本体即测试上位机**：FSD 与测试程序同机运行，`hil_test` 在 `can0` 上另开 SocketCAN 套接字做**被动监听**（SocketCAN 支持同一接口多套接字，与 can\_interface 并存不冲突）；
- **测试程序以被动监听为主**：解码 CAN 帧、断言协议/安全行为、落盘日志；注入限 ROS 侧（控制指令 / 状态话题）与 CAN 故障注入（见 §4）；
- **AMI / RES / 安全回路是 VCU 侧真实信号**：Go、急停、任务选择的测试需实际操作 VCU 输入（按键 / 开关）或使用 VCU 调试口，CAN 无法代替。

### 2.1 通信协议（已定稿）

| 方向        | 报文 ID | 周期      | 内容                                                                                                                                                                    |
| --------- | ----- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 工控机 → VCU | 0x210 | 10Hz 保活 | Byte1-2 Signal1 纵向（10\~65525，32767=零，<32767 制动、>32767 驱动）；Byte3-4 Signal2 横向（10\~65525，32767 中心，10=左、65525=右）；Byte5 Signal3 工控机上线（1=正常）；Byte6 Signal4 任务已完成；Byte7-8 空 |
| VCU → 工控机 | 0x501 | 10Hz    | Byte1 VCU 状态（0 静默 / 1\~5 有人 / 6\~11 无人 / 12 EMERGENCY）；Byte2 测试模式（1 操控性=有人，忽略 / 2 直线加速 / 3 高速循迹 / 4 八字绕环 / 5 EBS / 6 车检）；Byte3-8 空                                    |

详细信号位与定标以《26赛季 VCU - 工控机 CAN通信协议规划.md》为准，测试配置镜像到 `hil_test/config/protocol.yaml`。

### 2.2 can\_interface ↔ ROS 话题映射

| CAN 信号              | 话题                                      | 类型                     | 方向 |
| ------------------- | --------------------------------------- | ---------------------- | -- |
| 0x210 Signal1/2     | 订阅 `/control/command`                   | autoware\_msgs/Command | 发送 |
| 0x210 Signal3（上线）   | 订阅 `/system/devices_inspection`         | DevicesInspection      | 发送 |
| 0x210 Signal4（完成）   | 订阅 `/system/mission_state`（FINISH）      | MissionState           | 发送 |
| 0x501 Byte1=10（驾驶态） | 发布 `/system/start_command=true`（1Hz 保活） | std\_msgs/Bool         | 接收 |
| 0x501 Byte1=12（急停）  | 发布 `/system/emergency=true`（1Hz 保活）     | std\_msgs/Bool         | 接收 |
| 0x501 Byte2（测试模式）   | 发布 `/system/mission_mode_cmd`（仅变化时）     | std\_msgs/String       | 接收 |

FSD 消费方：mission_manager 订阅 `/system/emergency`、`/system/start_command`、`/system/mission_mode_cmd`、`/system/mission_complete`；controller 订阅 `/system/emergency`、`/system/mission_state`。

### 2.3 HIL 台架模式（车辆架起，传感器仅保在线）

HIL 台架测试时车辆架起不动，相机 / 激光雷达 / 华测等传感器**不参与决策**，只要求 FSD 节点正常运行在线。位姿、车速、路径与就绪信号统一由 `hil_test` 代发：

| 代发话题 | 类型 | 作用 |
|---|---|---|
| `/system/lidar_ready` | Bool=true | mission_manager IDLE→READY 门控（当前无生产者，必须代发） |
| `/system/localization_ready` | Bool=true | mission_manager 门控 + path_generator 门槛 |
| `/localization/pose` | PoseStamped | controller 位姿 + mission_manager 计圈 |
| `/chcnav/velocity` | TwistStamped | controller 车速 PID 反馈 |
| `/planning/final_waypoints` | Lane（直路） | controller 路径输入 |
| `/system/devices_inspection` | ok=true | can_interface Signal3 上线（0x210 Byte5=1） |

FSD 侧配套（HIL 专用配置，默认配置不变）：

- mission_manager 传感器自检全部关闭（`config/hil_fsd/mission_manager.yaml` 参数覆盖，否则华测未接 → 自检超时锁死 EMERGENCY）；
- L3 台架运行子集：can_interface + controller（定位/感知/规划不启动），mission_state 由 hil_test 注入 RACE 使能 controller；L2 需同时跑 mission_manager。

## 3. 分层测试框架（由浅入深）

每层独立可跑，前一层通过后再进入下一层。L0 为纯仿真验证，不需要 FSD 代码与真实 VCU。

### L0 纯仿真验证（开发机 vcan）

目的：不依赖 FSD 代码，协议编解码与状态联动先用软件模拟跑通，避免每次调试都接真实 VCU。

| 检查项     | 方法                                               | 通过标准                                                 |
| ------- | ------------------------------------------------ | ---------------------------------------------------- |
| 协议编解码单测 | protocol_loader 单测（定标/字节序/钳位/模式映射）              | 与 protocol.yaml 配置一致                                 |
| vcan 链路 | `ip link add vcan0 type vcan` + up               | 接口 up，can_interface 可打开                             |
| VCU 模拟  | `vcu_sim.py` 周期发 0x501（10Hz），状态可脚本切换             | 0x501 稳定到达，周期 100ms ± 容忍                             |
| 状态联动    | 脚本切状态 10 / 12 / 模式 Byte2                         | start_command / emergency / mission_mode_cmd 话题正确 |

> L0 仅限仿真接口（`sim_interfaces`），对真实接口运行会报错退出（防模拟帧污染真实 VCU）。

### L1 链路自检 + 协议一致性（需 FSD）

链路自检：

| 检查项        | 方法                                            | 通过标准                                          |
| ---------- | --------------------------------------------- | --------------------------------------------- |
| CAN 接口     | `ip link set can0 up type can bitrate 500000` | 接口 up，无报错                                     |
| 节点上线       | 启动 can_interface                             | 日志显示 device opened、Tx 0x210 / Rx 0x501 active |
| 工控机→VCU 心跳 | 监听 0x210                                      | 每 100ms 稳定一帧，DLC=8                            |
| VCU→工控机心跳  | 监听 0x501                                      | 每 100ms 稳定一帧，DLC=8，Byte1/Byte2 有效             |
| fail-safe  | 断开 VCU / 接口 down 后重启节点                        | 日志告警且 0x210 无任何非零控制量（Signal1/2=32767）         |

协议一致性（配置驱动）：

- 协议不写死在测试代码里，用 `protocol.yaml` 描述报文 ID / 字节序 / 信号位 / 定标 / 周期；
- **工控机 → VCU 方向（0x210）**：通过 ROS 注入已知输入（`speed`、`angle`、mission state、devices\_inspection），抓 CAN 帧解码，断言与配置期望一致；
- **VCU → 工控机方向（0x501）**：注入固定状态/模式，断言 can\_interface 发布的 ROS 话题正确。

覆盖项：

| 用例      | 注入                                 | 断言                                                                      |
| ------- | ---------------------------------- | ----------------------------------------------------------------------- |
| 定标与中心点  | `throttle_brake=0, angle=0`        | Signal1/2=32767（零控制）                                                    |
| 字节序     | 已知正/负控制量                           | Byte1=低字节、Byte2=高字节（little-endian）                                      |
| 越界钳位    | 输入超出 \[-1,1]                       | Signal1/2 钳位到 \[10, 65525]                                              |
| 无指令时    | 停发 `/control/command`              | 0x210 保活帧 Signal1/2=32767（零控制）                                          |
| 上线信号    | 发布 `/system/devices_inspection` 失败 | 0x210 Byte5=0（上线失效）                                                     |
| 完成信号    | mission state=FINISH               | 0x210 Byte6=1（任务完成）                                                     |
| Go 触发   | 0x501 Byte1=10                     | `/system/start_command=true`；保持 10 期间 1Hz 重复发布                          |
| 急停触发    | 0x501 Byte1=12                     | `/system/emergency=true`；保持 12 期间 1Hz 重复发布                              |
| 模式映射    | 0x501 Byte2=2/3/4/5/6              | mission\_mode\_cmd=acceleration/trackdrive/skidpad/ebs\_test/inspection |
| 操控性模式   | 0x501 Byte2=1（有人驾驶）                | 忽略，不改动任务模式，仅日志                                                          |
| 未知状态/模式 | Byte1/Byte2 越界值                    | WARN 日志，无副作用                                                            |

### L2 安全与状态联动（真实 VCU 信号）

| 场景       | 注入 / 操作                                            | 断言                                                                                                        |
| -------- | -------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| 启动门控     | VCU 状态非 10（如 9 待命）时注入控制指令                          | 0x210 Signal1/2 恒为 32767（零控制）                                                                             |
| Go 放行    | 代发 ready 信号 → READY；实际操作 RES 给 Go（VCU 状态=10） | `/system/start_command=true`；mission_manager READY→EXPLORE；总线开始透传控制量 |
| 急停优先级    | 实际触发 RES 急停（VCU 状态=12）                             | `/system/emergency=true`；controller 立即发全零；mission\_manager 锁死 EMERGENCY                                   |
| 保活防丢     | VCU 保持状态 10/12 时乱序重启 mission\_manager / controller | 1s 内重新收到 start\_command / emergency=true                                                                  |
| 看门狗（待开发） | 断开 VCU（停发 0x501）超时                                 | 视为设备自检失败：0x210 Signal3=0 + 错误日志，无残留使能                                                                     |
| 车检流程     | VCU 测试模式=6（车检）                                     | mission\_mode\_cmd="inspection" → INSPECTION → controller 演示 → mission\_complete → FINISH → 0x210 Byte6=1 |

注：看门狗依赖 can\_interface 的 VCU 失联检测（待开发项，见 §7），实现前该用例不可执行。

### L3 电机闭环（真实 VCU + 台架）

- 台架通电，VCU 使能驱动；车辆架起，传感器仅保在线：**位姿 / 车速 / 路径 / 就绪信号均由 hil\_test 代发**（`/localization/pose`、`/chcnav/velocity`、`/planning/final_waypoints` 直路、ready 信号、`/system/devices_inspection` ok=true 上线），FSD 运行子集为 can\_interface + controller，mission_state 由 hil\_test 注入 RACE 使能；
- 依次下发：恒速跟随 → 加减速斜坡 → 急停刹停（RES）；
- 指标：稳态驱动开度、加减速跟随、急停刹停响应时间（仿真自动触发可量化，真实 RES 由操作者计时）；
- 注意事项：先断电跑 L2 验证门控与急停无误，再通电跑 L3。

## 4. 测试程序组成（`hil_test` 模块）

```
hil_test/
├── config/
│   ├── hil_test.yaml        # CAN 设备、波特率、心跳周期等测试参数
│   ├── protocol.yaml        # 0x210/0x501 报文 ID / 信号位 / 定标 / 周期
│   └── hil_fsd/
│       └── mission_manager.yaml  # HIL 专用参数覆盖（传感器自检全关）
├── src/hil_test/
│   ├── can_socket.py        # SocketCAN 套接字封装（纯 stdlib）
│   ├── bus_monitor.py       # 被动监听 CAN 帧 + 日志落盘 + 周期/ID 统计
│   ├── protocol_loader.py   # 解析 protocol.yaml，编解码统一入口
│   ├── ros_injector.py      # 注入控制/状态/车速 + 台架代发（pose/ready/waypoints）
│   ├── vcu_sim.py           # L0：vcan 模拟 VCU（周期发 0x501，状态可脚本切换；解析 0x210）
│   ├── fault_injector.py    # 可选：CAN 故障注入（急停帧 / 停发），需独立测试通道
│   └── report.py            # 测试结果汇总，生成 Markdown 报告
├── test/
│   ├── test_protocol.py     # L0/L1 用例（pytest）
│   ├── test_safety.py       # L2 安全与状态联动（pytest）
│   └── test_motor_hil.py    # L3 电机闭环（pytest，标记 slow）
├── scripts/
│   ├── hil_test.sh          # 一键脚本（编译+节点+测试+清理）
│   └── run_hil.py           # 分层执行入口：--level L0..L3
```
（用法与验收见工程根目录 README.md）

依赖：pytest、PyYAML（CAN 层用纯 stdlib SocketCAN，无需 python-can）；ROS 集成用例需 ROS2 + FSD workspace；can-utils（candump/cansend）仅调试用。

设计要点：

- `bus_monitor` 在 `can0` 上以只读套接字被动监听，不占用 can\_interface 的收发；
- `ros_injector` 驱动 FSD 输入（等价于实车场景的决策层输入）；**HIL 台架下统一代发位姿 / 车速 / 路径 / 就绪信号**（§2.3），FSD 传感器节点不参与；
- `vcu_sim` 仅用于 L0 仿真预跑（vcan0），真实 VCU 接入后不再启用；
- VCU 侧真实信号（Go / 急停 / 任务）不走软件注入，由**人工操作**配合用例流程执行；
- `fault_injector` 仅用于故障注入场景，默认关闭，且建议用独立测试通道避免污染正常测试。

## 5. 开发实施步骤（里程碑）

| 里程碑 | 内容                                                                                                        | 环境                       | 验收标准                                   |
| --- | --------------------------------------------------------------------------------------------------------- | ------------------------ | -------------------------------------- |
| M1  | 搭建 `hil_test` 框架：填充 protocol.yaml + bus\_monitor + protocol\_loader + ros\_injector + vcu\_sim + run\_hil | 开发机 vcan0                | vcan0 抓帧正常，编解码配置可解析，vcu\_sim 能模拟 0x501 |
| M2  | L0 + L1 用例跑通                                                                                        | 开发机 vcan0 / 工控机 + 真实 VCU | 仿真预跑与链路用例绿；协议用例在配置驱动下绿                 |
| M3  | L2 安全用例（含人工操作 Go / 急停）                                                                                    | 工控机 + 真实 VCU             | 门控 / 急停 / 保活防丢通过，且仅零输出；看门狗待开发项完成后并入    |
| M4  | L3 电机闭环                                                                                                   | 工控机 + VCU + 电机台架         | 台架通电后闭环指标达标，输出报告                       |

关键前置项：

- `protocol.yaml` 按 0x210/0x501 定稿协议填充（§4），测试代码无需改动；
- can\_interface 收发编码**已实现**（0x210 发送、0x501 解析、GO/EMERGENCY 1Hz 保活），HIL 测试可直接依赖；
- HIL 台架模式：mission_manager 加载 `hil_fsd/mission_manager.yaml` 关闭传感器自检（§2.3）；L3 运行子集 can\_interface + controller，hil\_test 代发位姿/车速/路径/就绪信号；
- 待开发：can\_interface **VCU 失联检测**（0x501 超时 → 设备自检失败路径 + 错误日志，支撑 L2 看门狗用例）；
- 工控机安装 pytest、PyYAML，确认 USB-CAN 适配器驱动与 `can0` 名称；
- VCU 上电联调：确认 AMI / RES / 安全回路接线与信号极性符合协议。

## 6. 台架安全注意事项

- 台架通电前必须先通过 L2 全部用例（门控 / 急停 / 看门狗），确保故障方向收敛为"零输出"；
- 首次通电使用低速 / 低力矩档位，人工急停开关（RES）常备；
- 电机台架加机械限位与过流保护，编码器断线 / VCU 断线按急停处理；
- 看门狗超时按 SCS 要求（规则第四章 2.1）强制置急停，禁止残留使能；VCU 失联超时阈值建议 1s，最终以 VCU 侧为准；
- 测试期间任何时刻人工急停应优先于所有软件逻辑。

## 7. 待确认事项

- **VCU 驱动协议**：电机转速 / 电流指令格式、编码器 / 车速反馈报文（L3 依赖，影响 hil\_test 发布 `/chcnav/velocity` 的数据来源）；
- **AMI / RES 在 VCU 侧的电气定义与操作方式**（按键 / 开关 / 调试口）；
- **VCU 失联超时阈值与行为**：can\_interface 看门狗（0x501 超时）的阈值与设备自检失败路径的实施方式。

已定稿无需再确认：0x210 / 0x501 报文 ID、信号位、定标、周期（10Hz）。

## 8. 文档同步

- 新增话题（如 VCU 失联状态）后同步 `docs/ROS_INTERFACE.md`（若建立）；
- `hil_test/config/protocol.yaml` 与《26赛季 VCU - 工控机 CAN通信协议规划.md》保持一致；
- 车检完成信号以 `/system/mission_complete` → 0x210 Byte6 为准（controller 上报，非独立 CAN 报文）。

