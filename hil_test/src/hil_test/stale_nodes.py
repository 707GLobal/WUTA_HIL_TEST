"""清理上一次运行残留的 FSD 节点进程.

为什么需要：**mission_manager 由测试用例自起**（L2 故障锁存 / L4 状态机终态需要干净
实例），它不在 `hil_test.sh` 的 NODE_PIDS 里，脚本的 trap 管不到。运行被 Ctrl-C、
超时或硬中断时它就可能残留，后果（2026-10-06 20:21~20:24 台架实测，连续 4 次 L4 失败）：

  ① ROS 图里一直有 `mission_manager` → 下一次运行 `mm_factory` 的「等节点出现在图上」
     立刻通过（早于新实例建好订阅）→ 用例一次性注入的就绪信号被静默丢弃 → 新实例
     永远停在 IDLE；
  ② 残留实例还在 10Hz 广播它的旧状态（实测停在 EMERGENCY=7）→
     `/system/mission_state` 上 0/7 交替，测试永远等不到 READY=1。

判定方式：只认 `/proc/<pid>/cmdline` 里的**可执行体**——
  - FSD 安装目录下的节点二进制：`.../install/<pkg>/lib/<pkg>/<node>`
  - 对应的 `ros2 run <pkg> <node>` 包装进程（argv[0] 是 ros2，或 python3 + .../ros2）
不匹配 bash / grep / pgrep 之类「命令行里恰好含关键字」的进程：实测 `pgrep -f
mission_manager_node` 会把自己的 `bash -c` 命令行也匹配上，直接按 pgrep 结果 kill
有误杀风险。
"""

import os
import signal
import time

# 默认清理对象：脚本会起 can_interface/controller，用例会起 mission_manager
NODES = (
    ('mission_manager', 'mission_manager_node'),
    ('can_interface', 'can_interface_node'),
    ('controller', 'controller_node'),
)


def _argv(pid):
    """读 /proc/<pid>/cmdline；进程已退出/无权限返回 []."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            raw = f.read()
    except OSError:
        return []
    return [p.decode('utf-8', 'replace') for p in raw.split(b'\0') if p]


def _matches(argv, pkg, node):
    """argv 是否就是 `pkg/node` 的节点二进制或其 ros2 run 包装进程."""
    if not argv:
        return False
    if f'install/{pkg}/lib/{pkg}/{node}' in ' '.join(argv):
        return True
    head = [os.path.basename(a) for a in argv[:2]]
    is_ros2 = 'ros2' in head or (
        len(head) > 1 and head[0].startswith('python') and head[1] == 'ros2')
    return is_ros2 and node in argv


def find_stale(pairs=NODES):
    """返回 [(pid, pkg, node), ...]：正在运行的残留节点进程."""
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == os.getpid():
            continue
        argv = _argv(pid)
        for pkg, node in pairs:
            if _matches(argv, pkg, node):
                out.append((pid, pkg, node))
                break
    return out


def kill_stale(pairs=NODES, wait_sec=5.0):
    """SIGTERM 掉残留节点（整合进程组），等其退出；返回被杀的 [(pid, pkg, node), ...]."""
    stale = find_stale(pairs)
    if not stale:
        return []
    own_pgid = os.getpgid(0)
    for pid, _pkg, _node in stale:
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            continue
        try:
            # ros2 run 包装进程与节点同组（setsid / start_new_session），整组收掉；
            # 万一它与本进程同组（不该发生），只杀该 PID，避免误伤自己
            if pgid != own_pgid:
                os.killpg(pgid, signal.SIGTERM)
            else:
                os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    deadline = time.time() + wait_sec
    pids = [pid for pid, _p, _n in stale]
    while time.time() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
        if not alive:
            break
        time.sleep(0.1)
    return stale
