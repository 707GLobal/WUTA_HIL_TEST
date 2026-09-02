#!/usr/bin/env bash
# hil_test 一键脚本：编译 FSD → 选链路 → 跑测试 → 输出结果
#
# 用法:
#   ./scripts/hil_test.sh                     # 交互: 编译 → 选链路 → 跑 → 出结果
#   ./scripts/hil_test.sh -l L1 -i vcan0      # 直接指定链路与接口
#   ./scripts/hil_test.sh -l all -i vcan0     # 一键流水线: L0→L1→L2 连跑(失败即停)
#   ./scripts/hil_test.sh -l L3 -i can0 -n    # L3 台架模式
#   ./scripts/hil_test.sh --no-build -l L0    # 跳过编译
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
FSD_WS="$REPO_ROOT/WUTA-FSD/ros2_ws"
HIL_CFG="$HIL_ROOT/config"
LOG_DIR="$HIL_ROOT/logs"

LEVEL=""
INTERFACE="vcan0"
MODE=""                  # sim=仿真(vcan) | real=真实(USB-CAN)，缺省按接口名推断
DO_BUILD=1
BUILD_PKGS=""
CLEAN_BUILD=0
KEEP_NODES=0
BENCH=0

usage() {
  cat <<EOF
用法: $(basename "$0") [选项]

  -l, --level <L0|L1|L2|L3|all>     测试链路（缺省交互选择；all=流水线连跑）
  -i, --interface <接口名>          CAN 接口（缺省 vcan0；真实 USB-CAN 按实际名称，如 can0/can1）
  -m, --mode <sim|real>            sim=仿真(vcan) / real=真实接口（缺省按接口名自动推断）
  -b, --build                       编译 FSD（默认开启）
      --no-build                    跳过编译
  -c, --clean                       编译前清理 FSD 缓存（build/install/log，全量重编）
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
    -c|--clean) CLEAN_BUILD=1; shift ;;
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
  case "$1" in L0|L1|L2|L3|all) return 0 ;; *) return 1 ;; esac
}

if [ -z "$LEVEL" ]; then
  echo "选择测试链路:"
  echo "  all)   流水线连跑 (L0→L1→L2, 失败即停)"
  echo "  L0)    纯仿真验证 (vcan0, 无 FSD, 协议单测+VCU 模拟)"
  echo "  L1)    链路自检 + 协议一致性"
  echo "  L2)    安全与状态联动"
  echo "  L3)    电机闭环 (需台架)"
  read -rp "输入 [all/L0/L1/L2/L3]: " LEVEL
fi
if ! valid_level "$LEVEL"; then
  echo "!! 非法链路: $LEVEL" >&2; exit 1
fi

# ---- 环境 ----
# colcon 生成的 setup.bash 引用未定义的 COLCON_TRACE，需在 set -u 下临时放开
set +u
if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$FSD_WS/install/setup.bash" ]; then
  source "$FSD_WS/install/setup.bash"  # --no-build 时也保证 FSD 消息/节点可用
fi
set -u
# ---- 日志目录：按批次时间戳归档，logs/latest 指向最新一批 ----
mkdir -p "$HIL_ROOT/logs"
LOG_DIR="$HIL_ROOT/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$HIL_ROOT/logs/latest"

PROJECT_NAME="${PROJECT_NAME:-WUTA_HIL}"   # 报告文件名用：日期-时间-项目
export HIL_PROJECT="$PROJECT_NAME"

# ---- 编译 FSD ----
if [ "$CLEAN_BUILD" -eq 1 ] && [ "$DO_BUILD" -eq 0 ]; then
  echo "!! -c/--clean 需配合编译，不能与 --no-build 同时使用" >&2
  exit 1
fi
if [ "$DO_BUILD" -eq 1 ]; then
  echo "==> 编译 FSD ($FSD_WS)"
  cd "$FSD_WS"
  if [ "$CLEAN_BUILD" -eq 1 ]; then
    echo "==> 清理 FSD 编译缓存（build/install/log）"
    rm -rf "$FSD_WS/build" "$FSD_WS/install" "$FSD_WS/log"
  fi
  if [ -n "$BUILD_PKGS" ]; then
    colcon build --packages-select $BUILD_PKGS
  else
    colcon build
  fi
  set +u
  source "$FSD_WS/install/setup.bash"
  set -u
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
  # 按 flags 中的独立词 UP 判断（vcan 状态列显示 UNKNOWN 而非 UP，不能用 " UP " 匹配）
  if ! ip link show "$INTERFACE" | grep -qw "UP"; then
    if [ "$MODE" = "sim" ]; then
      echo "==> 启动 $INTERFACE（需 sudo）"
      sudo ip link set "$INTERFACE" up
    else
      echo "!! $INTERFACE 未 up（ip link set $INTERFACE up）" >&2
      exit 1
    fi
  fi
  echo "==> 接口 $INTERFACE 就绪（mode=$MODE）"
}
prepare_interface

# ---- FSD 节点管理 ----
NODE_PIDS=()
start_node() {
  local pkg="$1" exe="$2"; shift 2
  local log_dir="${HIL_LEVEL_DIR:-$LOG_DIR}"
  ros2 run "$pkg" "$exe" "$@" >"$log_dir/$exe.log" 2>&1 &
  local pid=$!
  NODE_PIDS+=("$pid")
  echo "==> 启动 $exe (pid $pid, log $log_dir/$exe.log)"
}

wait_node_ready() {
  local name="$1" tries=40
  for _ in $(seq 1 "$tries"); do
    if ros2 node list 2>/dev/null | grep -q "/$name"; then
      return 0
    fi
    sleep 0.5
  done
  echo "!! 节点 $name 未就绪，见 ${HIL_LEVEL_DIR:-$LOG_DIR}/$name.log" >&2
  return 1
}

# ---- 停止节点（层间切换 / 退出清理） ----
stop_nodes() {
  [ "${#NODE_PIDS[@]}" -eq 0 ] && return 0
  local n=${#NODE_PIDS[@]}
  kill "${NODE_PIDS[@]}" 2>/dev/null || true
  NODE_PIDS=()
  echo "==> 已停止节点 ($n)"
}

cleanup() {
  if [ "$KEEP_NODES" -eq 1 ] && [ "$LEVEL" != "all" ]; then
    echo "==> 保留 FSD 节点（-k）"
    return
  fi
  stop_nodes
  echo "==> FSD 节点已清理"
}
trap cleanup EXIT

CAN_PARAMS="$FSD_WS/src/system/can_interface/config/can_interface.yaml"
CTRL_PARAMS="$FSD_WS/src/control/controller/config/controller.yaml"
MM_PARAMS="$FSD_WS/src/system/mission_manager/config/mission_manager.yaml"
MM_HIL_PARAMS="$HIL_CFG/hil_fsd/mission_manager.yaml"

# ---- 按链路启动 FSD 节点 ----
start_level_nodes() {
  local level="$1"
  case "$level" in
    L0)
      # 纯仿真验证：无 FSD 节点；vcu_sim 由 pytest fixture 启动
      ;;
    L1)
      start_node can_interface can_interface_node \
        --ros-args --params-file "$CAN_PARAMS" -p can_device:="$INTERFACE"
      if [ "$MODE" = "sim" ]; then
        # 仿真接口补 VCU 模拟，提供 0x501 心跳
        PYTHONPATH="$HIL_ROOT/src" python3 -m hil_test.vcu_sim \
          --interface "$INTERFACE" --protocol "$HIL_CFG/protocol.yaml" \
          >"${HIL_LEVEL_DIR:-$LOG_DIR}/vcu_sim.log" 2>&1 &
        NODE_PIDS+=($!)
        echo "==> 启动 vcu_sim (pid ${NODE_PIDS[-1]})"
      fi
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
}

# ---- 跑单层：起节点 → pytest → 记录结果 → 停节点 ----
SUMMARY_LOG="$LOG_DIR/levels.txt"
: > "$SUMMARY_LOG"

run_level() {
  local level="$1"
  LEVEL_DIR="$LOG_DIR/$level"
  mkdir -p "$LEVEL_DIR"
  export HIL_LEVEL_DIR="$LEVEL_DIR"
  echo ""
  echo "==> 运行 $level (interface=$INTERFACE)"
  start_level_nodes "$level"
  export HIL_INTERFACE="$INTERFACE"
  export HIL_CONFIG="$HIL_CFG"
  if [ "$BENCH" -eq 1 ]; then
    export HIL_BENCH=1
  fi
  python3 "$HIL_ROOT/scripts/run_hil.py" --level "$level" --interface "$INTERFACE" \
    --report-dir "$LEVEL_DIR" \
    2>&1 | tee "$LEVEL_DIR/pytest_$level.log"
  local rc=${PIPESTATUS[0]}
  local line report
  line="$(grep -E "[0-9]+ (passed|failed|error)" "$LEVEL_DIR/pytest_$level.log" | tail -1 || echo '无输出')"
  report="$(basename "$(ls "$LEVEL_DIR"/*.md 2>/dev/null | head -1)" 2>/dev/null || true)"
  echo "$level|$line|$report" >> "$SUMMARY_LOG"
  if [ "$LEVEL" = "all" ] || [ "$KEEP_NODES" -eq 0 ]; then
    stop_nodes
  fi
  return $rc
}

# ---- 汇总 Markdown ----
write_summary() {
  local md="$LOG_DIR/summary.md"
  {
    echo "# HIL 测试汇总"
    echo ""
    echo "- 时间: $(date '+%F %T')"
    echo "- 接口: $INTERFACE (mode=$MODE)"
    echo "- 层级: $LEVEL"
    echo ""
    echo "| 层级 | 结果 | 详细报告 |"
    echo "|---|---|---|"
    while IFS='|' read -r lv res rep; do
      if [ -n "$rep" ]; then
        echo "| $lv | ${res:-未执行} | [$rep]($lv/$rep) |"
      else
        echo "| $lv | ${res:-未执行} | - |"
      fi
    done < "$SUMMARY_LOG"
    echo ""
    echo "- 日志: $LOG_DIR"
  } > "$md"
  echo "$md"
}

# ---- 执行 ----
if [ "$LEVEL" = "L0" ] && [ "$MODE" = "real" ]; then
  echo "!! L0 为纯仿真验证，仅限仿真接口（如 vcan0）；真实接口请用 -l L1" >&2
  exit 2
fi

TEST_RC=0
if [ "$LEVEL" = "all" ]; then
  if [ "$MODE" = "sim" ]; then
    LEVELS_RUN="L0 L1 L2"
  else
    LEVELS_RUN="L1"
    echo "!! all(real): L2 需人工配合真实 VCU（RES/AMI），请单独运行: ./scripts/hil_test.sh -l L2 -i $INTERFACE"
  fi
  for lv in $LEVELS_RUN; do
    if ! run_level "$lv"; then
      TEST_RC=1
      echo "!! $lv 未通过，流水线终止（后续层级未执行）"
      break
    fi
  done
else
  run_level "$LEVEL" || TEST_RC=$?
fi

SUMMARY_MD="$(write_summary)"

# ---- 输出结果 ----
LAST_LINE="$(tail -1 "$SUMMARY_LOG")"
RESULT="${LAST_LINE#*|}"
RESULT="${RESULT%%|*}"
echo ""
echo "=============================================="
echo "[hil_test] $LEVEL 结果: ${RESULT:-无输出}"
echo "[hil_test] 日志: $LOG_DIR"
echo "[hil_test] 汇总: $SUMMARY_MD"
echo "=============================================="
exit "$TEST_RC"
