"""pytest 公共配置：路径、fixtures、markers.

分层 marker（L1 分两阶段：pre 不依赖 FSD / post 需 can_interface）：
  sim        - L1 pre：vcan 模拟 VCU + RES（vcu_sim 模型，无需 FSD）
  unit       - L1 pre：协议编解码单测（无需硬件）
  link       - L1 post：链路自检（需 can_interface）
  protocol   - L1 post：协议一致性集成（需 can_interface）
  selfcheck  - L2 传感器自检故障模拟（断电层，mission_manager 默认参数）
  motor      - L3 低速动态安全闭环：选模式 + RES 放行 + 低速驱动（slow，通电层）
  inspection - L4 车检任务全链路：选模式 + RES 放行 + 车检（slow，通电层）
  integration - 依赖 can_interface/FSD 运行，需工控机环境
"""

import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, 'src'))

import pytest  # noqa: E402


def _config_path(name):
    return os.environ.get('HIL_CONFIG', os.path.join(ROOT, 'config')) + os.sep + name


def _load_config():
    """读取 hil_test.yaml 配置."""
    import yaml
    with open(_config_path('hil_test.yaml'), 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _interface():
    """默认接口：环境变量 > hil_test.yaml."""
    env_if = os.environ.get('HIL_INTERFACE')
    if env_if:
        return env_if
    return _load_config()['can']['interface']


def _is_sim(interface):
    """是否为仿真接口（在 config sim_interfaces 列表内）."""
    return interface in _load_config()['can'].get('sim_interfaces', ['vcan0'])


def _if_up(name):
    """检查接口是否 up."""
    try:
        import fcntl
        import socket
        import struct
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SIOCGIFFLAGS = 0x8913（0x8912 是 SIOCGIFCONF，取不到 flags）
        res = fcntl.ioctl(sock.fileno(), 0x8913, struct.pack('256s', name.encode()[:15]))
        flags = struct.unpack('H', res[16:18])[0]
        return bool(flags & 0x1)  # IFF_UP
    except OSError:
        return False


def pytest_configure(config):
    """注册分层 markers."""
    for marker in ('sim', 'unit', 'link', 'protocol', 'selfcheck', 'motor',
                   'inspection', 'integration', 'slow'):
        config.addinivalue_line('markers', marker)


# ---------------------------------------------------------------------------
# 结果统计：全部用例被跳过 / 一条都没收集到 → 判为未通过
#
# pytest 对「全 skip」返回退出码 0，脚本只看退出码会把「什么都没跑」报成「通过」。
# 而 skip 恰恰覆盖了最容易误判的路径：L3/L4 忘带 -n（HIL_BENCH 门控）、can 接口
# 不在（can_ready）、can_interface/mission_manager 没起来（fsd_ready / mm_factory）、
# vcan0 未创建。故在会话收尾把这种「零通过」改成失败。
# ---------------------------------------------------------------------------
_OUTCOMES = {'passed': 0, 'failed': 0, 'error': 0, 'skipped': 0}


def pytest_runtest_logreport(report):
    if report.skipped:
        _OUTCOMES['skipped'] += 1
    elif report.failed:
        if report.when == 'call':
            _OUTCOMES['failed'] += 1
        else:
            _OUTCOMES['error'] += 1
    elif report.when == 'call' and report.passed:
        _OUTCOMES['passed'] += 1


def pytest_sessionfinish(session, exitstatus):
    """没有一条用例真正通过时强制判未通过（failures/errors 保持原样）."""
    if _OUTCOMES['failed'] or _OUTCOMES['error']:
        return
    collected = getattr(session, 'testscollected', 0)
    if collected == 0:
        reason = '本层没有收集到任何用例（检查测试文件/marker 选择）'
    elif _OUTCOMES['passed'] == 0:
        reason = (f"本层 {_OUTCOMES['skipped']}/{collected} 条用例全部被跳过，"
                  '没有一条真正执行（真实台架记得加 -n/--bench，并确认 can 接口'
                  '与 FSD 节点已就绪）')
    else:
        return
    session.exitstatus = 1
    reporter = session.config.pluginmanager.get_plugin('terminalreporter')
    if reporter is not None:
        reporter.write_line(f'!! {reason} → 判为未通过', red=True)
    else:  # pragma: no cover - 无终端插件时的兜底
        print(f'!! {reason} → 判为未通过')


@pytest.fixture
def protocol():
    """协议编解码实例（纯逻辑，无需硬件）."""
    from hil_test.protocol_loader import Protocol
    return Protocol(_config_path('protocol.yaml'))


@pytest.fixture
def interface():
    """CAN 接口名."""
    return _interface()


@pytest.fixture
def is_sim(interface):
    """是否为仿真接口（配置 sim_interfaces 内）."""
    return _is_sim(interface)


@pytest.fixture
def can_ready(interface):
    """接口可用性：不可用则跳过集成用例."""
    if not _if_up(interface):
        pytest.skip(f'CAN 接口 {interface} 未 up，集成用例跳过')
    return interface


@pytest.fixture
def bus_monitor(interface, tmp_path):
    """被动总线监听器（集成用例用）."""
    from hil_test.bus_monitor import BusMonitor
    mon = BusMonitor(interface, log_dir=str(tmp_path))
    if not mon.start():
        pytest.skip(f'无法在 {interface} 上监听')
    yield mon
    mon.stop()


@pytest.fixture
def vcu_sim(can_ready, is_sim):
    """仿真 VCU（L1 前置模型用例用）.

    仅仿真接口可用：真实接口下会向真实总线注入 0x501/0x210，必须跳过以
    防污染真实 VCU。
    """
    if not is_sim:
        pytest.skip(f'{can_ready} 为真实接口：vcu_sim 模型用例仅仿真接口运行'
                    f'（防模拟帧污染真实 VCU）')
    from hil_test.vcu_sim import VcuSim
    sim = VcuSim(can_ready, _config_path('protocol.yaml'))
    if not sim.start():
        pytest.skip('vcu_sim 无法启动')
    yield sim
    sim.stop()


@pytest.fixture
def ros():
    """ROS2 注入/断言器；环境缺失则跳过."""
    try:
        from hil_test.ros_injector import RosInjector
        inj = RosInjector()
    except RuntimeError as e:
        pytest.skip(str(e))
    yield inj
    inj.destroy()


@pytest.fixture
def fsd_ready(ros):
    """can_interface 节点在线；否则跳过集成用例（轮询等待 DDS 发现，避免偶发漏判）."""
    deadline = time.time() + 5.0
    while time.time() < deadline:
        ros.spin_once()
        if 'can_interface' in ros.graph_nodes():
            return ros
        time.sleep(0.2)
    pytest.skip('can_interface 未运行（先启动 FSD），集成用例跳过')


def _kill_stale_mission_manager(wait_sec=5.0):
    """清掉残留的 mission_manager 实例，返回被杀掉的 PID 列表.

    残留来源：上一次运行被 Ctrl-C / 硬中断时，**用例自起**的实例可能没被 fixture teardown
    收走（它也不在脚本的 NODE_PIDS 里，脚本 trap 管不到）。残留实例的危害：
      ① ROS 图里一直有 'mission_manager' → 本 fixture 的「等节点出现」立刻通过（早于
         新实例建好订阅），用例一次性注入的就绪信号被丢弃、新实例永远停在 IDLE；
      ② 残留实例还在 10Hz 广播它的旧状态（实测残留实例停在 EMERGENCY=7），
         /system/mission_state 上 0/7 交替，永远等不到 READY=1。
    台架实测（2026-10-06 20:21~20:24 连续 4 次失败）就是 20:18 那次运行留下的实例所致。
    判定规则（只认可执行体，不用 pgrep -f 关键字）见 hil_test/stale_nodes.py。
    """
    from hil_test.stale_nodes import kill_stale
    return [pid for pid, _pkg, _node in
            kill_stale((('mission_manager', 'mission_manager_node'),), wait_sec)]


@pytest.fixture
def mm_factory(fsd_ready, tmp_path):
    """按用例启动/停止独立 mission_manager 实例，返回 callable(*extra_params).

    以 HIL_MM_PARAMS 为基础，extra_params 逐个追加 `--params-file`（后者覆盖前者）。
    自管实例的用例（L2 故障锁存 / L4 状态机终态）需用完即停以获得干净状态机。
    """
    params = os.environ.get('HIL_MM_PARAMS')
    if not params:
        pytest.skip('缺少 HIL_MM_PARAMS（请用 hil_test.sh 启动）')
    procs = []

    def _start(*extra_params):
        # 先清残留实例：否则下面的「等节点出现」会被上一次的残留实例骗过，
        # 且它会继续广播旧状态（详见 _kill_stale_mission_manager 说明）
        stale = _kill_stale_mission_manager()
        if stale:
            print(f'[hil_test] 警告: 发现残留 mission_manager {stale}，已清理'
                  f'（上一次运行未收尾；下一次请让它跑完或先手动 pkill）', flush=True)
            deadline = time.time() + 3.0
            while time.time() < deadline:
                fsd_ready.spin_once()
                if 'mission_manager' not in fsd_ready.graph_nodes():
                    break
                time.sleep(0.1)
        cmd = ['ros2', 'run', 'mission_manager', 'mission_manager_node',
               '--ros-args', '--params-file', params]
        for p in extra_params:
            cmd += ['--params-file', p]
        log_path = os.path.join(str(tmp_path), 'mission_manager.log')
        with open(log_path, 'wb') as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        procs.append(proc)
        t_start = time.time()   # 只认本次启动之后收到的状态广播
        # ① 等节点出现在 ROS 图中（DDS 发现可能滞后）
        deadline = time.time() + 15.0
        while time.time() < deadline:
            fsd_ready.spin_once()
            if 'mission_manager' in fsd_ready.graph_nodes():
                break
            if proc.poll() is not None:
                pytest.skip(f'mission_manager 启动失败，日志: {log_path}')
            time.sleep(0.2)
        else:
            pytest.skip('mission_manager 节点未就绪')
        # ② 「出现在 ROS 图上」≠「初始化完成」：节点一创建就会被 DDS 发现，而它的订阅
        #    （/system/lidar_ready、/system/localization_ready 等 volatile 话题）要等构造
        #    函数跑完才匹配上。用例若在这段窗口里一次性注入就绪信号，消息会被静默丢弃，
        #    状态机永远停在 IDLE——台架实测（2026-10-06 20:21~20:24）：注入早于
        #    "Mission Manager initialized" 0.16s 时 4/4 全失败，晚 0.03~0.10s 时全通过。
        #    故再等它广播本实例的 10Hz /system/mission_state（构造完成、定时器已起）。
        deadline = time.time() + 15.0
        while time.time() < deadline:
            fsd_ready.spin_once()
            if fsd_ready.latest_since('/system/mission_state', t_start) is not None:
                return proc
            if proc.poll() is not None:
                pytest.skip(f'mission_manager 启动失败，日志: {log_path}')
            time.sleep(0.1)
        pytest.skip('mission_manager 未开始广播 /system/mission_state（初始化未完成？）')

    yield _start

    for proc in procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        proc.wait(timeout=5.0)
    time.sleep(0.5)  # 等 DDS 摘除旧实例，避免同名节点残留


@pytest.fixture
def vcu(can_ready, ros):
    """L3/L4 模式源：仿真接口自动驱动 vcu_sim 选模式；真实接口观察等待人工操作."""
    if not _is_sim(can_ready):
        yield None
        return
    from hil_test.vcu_sim import VcuSim
    sim = VcuSim(can_ready, _config_path('protocol.yaml'))
    if not sim.start():
        pytest.skip('vcu_sim 无法启动')
    yield sim
    sim.stop()
