#!/usr/bin/env bash
# hil_test 一键脚本：编译 FSD → 选链路 → 跑测试 → 输出结果
#
# 用法:
#   ./scripts/hil_test.sh                     # 交互: 编译 → 选链路 → 跑 → 出结果
#   ./scripts/hil_test.sh -l L1 -i vcan0      # 直接指定链路与接口
#   ./scripts/hil_test.sh -l L3 -i can0 -n    # L3 台架模式
#   ./scripts/hil_test.sh --no-build -l L0.5  # 跳过编译
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FSD_WS="$HIL_ROOT/WUTA-FSD/ros2_ws"
HIL_CFG="$HIL_ROOT/config"
LOG_DIR="$HIL_ROOT/logs"

LEVEL=""
INTERFACE="vcan0"
MODE=""                  # sim=仿真(vcan) | real=真实(USB-CAN)，缺省按接口名推断
DO_BUILD=1
BUILD_PKGS=""
KEEP_NODES=0
BENCH=0

usage() {
  cat <<EOF
用法: $(basename "$0") [选项]

  -l, --level <L0.5|L0|L1|L2|L3>   测试链路（缺省交互选择）
  -i, --interface <接口名>          CAN 接口（缺省 vcan0；真实 USB-CAN 按实际名称，如 can0/can1）
  -m, --mode <sim|real>            sim=仿真(vcan) / real=真实接口（缺省按接口名自动推断）
  -b, --build                       编译 FSD（默认开启）
      --no-build                    跳过编译
  -p, --build-packages <pkgs>       只编译指定包（空格分隔；缺省全量）
  -k, --keep-nodes                  测试后保留 FSD 节点（便于调试）
  -n, --bench                       L3 台架模式（HIL_BENCH=1）
  -h, --help                        帮助
EOF
}

# ---- 参数解析 ----
while [ $# -gt 0 ]; do
  case "$1" in
    -l|--level) LEVEL="$2"; shift 2 ;;
    -i|--interface) INTERFACE="$2"; shift 2 ;;
    -m|--mode) MODE="$2"; shift 2 ;;
    -b|--build) DO_BUILD=1; shift ;;
    --no-build) DO_BUILD=0; shift ;;
    -p|--build-packages) BUILD_PKGS="$2"; shift 2 ;;
    -k|--keep-nodes) KEEP_NODES=1; shift ;;
    -n|--bench) BENCH=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知选项: $1" >&2; usage; exit 1 ;;
  esac
done

# ---- 接口模式推断（sim=仿真 / real=真实） ----
if [ -z "$MODE" ]; then
  case "$INTERFACE" in
    vcan*) MODE="sim" ;;
    *) MODE="real" ;;
  esac
fi
case "$MODE" in sim|real) ;; *) echo "!! 非法 mode: $MODE" >&2; exit 1 ;; esac

# ---- 链路合法性校验 / 交互选择 ----
valid_level() {
  case "$1" in L0.5|L0|L1|L2|L3) return 0 ;; *) return 1 ;; esac
}

if [ -z "$LEVEL" ]; then
  echo "选择测试链路:"
  echo "  L0.5) 仿真预跑 (vcan0, 无需 FSD 节点)"
  echo "  L0)   链路自检"
  echo "  L1)   协议一致性"
  echo "  L2)   安全与状态联动"
  echo "  L3)   电机闭环 (需台架)"
  read -rp "输入 [L0.5/L0/L1/L2/L3]: " LEVEL
fi
if ! valid_level "$LEVEL"; then
  echo "!! 非法链路: $LEVEL" >&2; exit 1
fi

# ---- 环境 ----
if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$FSD_WS/install/setup.bash" ]; then
  source "$FSD_WS/install/setup.bash"  # --no-build 时也保证 FSD 消息/节点可用
fi
mkdir -p "$LOG_DIR"

# ---- 编译 FSD ----
if [ "$DO_BUILD" -eq 1 ]; then
  echo "==> 编译 FSD ($FSD_WS)"
  cd "$FSD_WS"
  if [ -n "$BUILD_PKGS" ]; then
    colcon build --packages-select $BUILD_PKGS
  else
    colcon build
  fi
  source "$FSD_WS/install/setup.bash"
fi

# ---- 接口准备 ----
prepare_interface() {
  if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
    if [ "$MODE" = "sim" ]; then
      echo "==> 创建 $INTERFACE（需 sudo）"
      sudo ip link add "$INTERFACE" type vcan
      sudo ip link set "$INTERFACE" up
    else
      echo "!! $INTERFACE 不存在，请先接入 USB-CAN 并配置接口（modprobe + ip link set $INTERFACE up）" >&2
      exit 1
    fi
  fi
  if ! ip link show "$INTERFACE" | grep -q " UP "; then
    echo "!! $INTERFACE 未 up（ip link set $INTERFACE up）" >&2
    exit 1
  fi
  echo "==> 接口 $INTERFACE 就绪（mode=$MODE）"
}
prepare_interface

# ---- FSD 节点管理 ----
NODE_PIDS=()
start_node() {
  local pkg="$1" exe="$2"; shift 2
  ros2 run "$pkg" "$exe" "$@" >"$LOG_DIR/$exe.log" 2>&1 &
  local pid=$!
  NODE_PIDS+=("$pid")
  echo "==> 启动 $exe (pid $pid, log $LOG_DIR/$exe.log)"
}

wait_node_ready() {
  local name="$1" tries=40
  for _ in $(seq 1 "$tries"); do
    if ros2 node list 2>/dev/null | grep -q "/$name"; then
      return 0
    fi
    sleep 0.5
  done
  echo "!! 节点 $name 未就绪，见 $LOG_DIR/$name.log" >&2
  return 1
}

cleanup() {
  if [ "$KEEP_NODES" -eq 1 ]; then
    echo "==> 保留 FSD 节点（-k）"
    return
  fi
  for pid in "${NODE_PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  echo "==> FSD 节点已清理"
}
trap cleanup EXIT

CAN_PARAMS="$FSD_WS/src/system/can_interface/config/can_interface.yaml"
CTRL_PARAMS="$FSD_WS/src/control/controller/config/controller.yaml"
MM_PARAMS="$FSD_WS/src/system/mission_manager/config/mission_manager.yaml"
MM_HIL_PARAMS="$HIL_CFG/hil_fsd/mission_manager.yaml"

case "$LEVEL" in
  L0.5)
    # 无 FSD 节点；vcu_sim 由 pytest fixture 启动
    ;;
  L0)
    start_node can_interface can_interface_node \
      --ros-args --params-file "$CAN_PARAMS" -p can_device:="$INTERFACE"
    if [ "$MODE" = "sim" ]; then
      # 仿真接口补 VCU 模拟，提供 0x501 心跳
      PYTHONPATH="$HIL_ROOT/src" python3 -m hil_test.vcu_sim \
        --interface "$INTERFACE" --protocol "$HIL_CFG/protocol.yaml" \
        >"$LOG_DIR/vcu_sim.log" 2>&1 &
      NODE_PIDS+=($!)
      echo "==> 启动 vcu_sim (pid ${NODE_PIDS[-1]})"
    fi
    wait_node_ready can_interface_node
    ;;
  L1)
    start_node can_interface can_interface_node \
      --ros-args --params-file "$CAN_PARAMS" -p can_device:="$INTERFACE"
    wait_node_ready can_interface_node
    ;;
  L2)
    start_node can_interface can_interface_node \
      --ros-args --params-file "$CAN_PARAMS" -p can_device:="$INTERFACE"
    start_node mission_manager mission_manager_node \
      --ros-args --params-file "$MM_PARAMS" --params-file "$MM_HIL_PARAMS"
    start_node controller controller_node \
      --ros-args --params-file "$CTRL_PARAMS"
    wait_node_ready can_interface_node
    ;;
  L3)
    start_node can_interface can_interface_node \
      --ros-args --params-file "$CAN_PARAMS" -p can_device:="$INTERFACE"
    start_node controller controller_node \
      --ros-args --params-file "$CTRL_PARAMS"
    wait_node_ready can_interface_node
    ;;
esac

# ---- 跑测试 ----
echo "==> 运行 $LEVEL (interface=$INTERFACE)"
export HIL_INTERFACE="$INTERFACE"
export HIL_CONFIG="$HIL_CFG"
if [ "$BENCH" -eq 1 ]; then
  export HIL_BENCH=1
fi

# shellcheck disable=SC2016
python3 "$HIL_ROOT/scripts/run_hil.py" --level "$LEVEL" --interface "$INTERFACE" \
  2>&1 | tee "$LOG_DIR/pytest_$LEVEL.log"
TEST_RC=${PIPESTATUS[0]}

# ---- 输出结果 ----
RESULT="$(grep -E "[0-9]+ (passed|failed|error)" "$LOG_DIR/pytest_$LEVEL.log" | tail -1 || true)"
echo ""
echo "=============================================="
echo "[hil_test] $LEVEL 结果: ${RESULT:-无输出}"
echo "[hil_test] 日志: $LOG_DIR/pytest_$LEVEL.log"
echo "=============================================="
exit "$TEST_RC"
