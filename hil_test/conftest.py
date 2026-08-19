"""pytest 公共配置：路径、fixtures、markers.

分层 marker：
  sim      - L0.5 仿真预跑（vcan 模拟 VCU）
  link     - L0  链路自检
  protocol - L1  协议一致性
  safety   - L2  安全与状态联动
  motor    - L3  电机闭环（slow）
  integration - 依赖 can_interface/FSD 运行，需工控机环境
"""

import os
import sys

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
        res = fcntl.ioctl(sock.fileno(), 0x8912, struct.pack('256s', name.encode()[:15]))
        flags = struct.unpack('H', res[16:18])[0]
        return bool(flags & 0x1)  # IFF_UP
    except OSError:
        return False


def pytest_configure(config):
    """注册分层 markers."""
    for marker in ('sim', 'link', 'protocol', 'safety', 'motor', 'integration', 'slow'):
        config.addinivalue_line('markers', marker)


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
def vcu_sim(can_ready):
    """L0.5 仿真 VCU（集成用例用）."""
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
    except RuntimeError as e:
        pytest.skip(str(e))
    inj = RosInjector()
    yield inj
    inj.destroy()


@pytest.fixture
def fsd_ready(ros):
    """can_interface 节点在线；否则跳过集成用例."""
    if 'can_interface' not in ros.graph_nodes():
        pytest.skip('can_interface 未运行（先启动 FSD），集成用例跳过')
    return ros


@pytest.fixture
def vcu(can_ready, ros):
    """L2 状态源：仿真接口自动驱动 vcu_sim；真实接口观察等待人工操作."""
    if not _is_sim(can_ready):
        yield None
        return
    from hil_test.can_socket import CanSocket
    from hil_test.vcu_sim import VcuSim
    sim = VcuSim(can_ready, _config_path('protocol.yaml'))
    if not sim.start():
        pytest.skip('vcu_sim 无法启动')
    # 模拟工控机上线（0x210 Signal3=1），使状态机推进到待命(9)
    tx = CanSocket(can_ready)
    if tx.open():
        tx.send(sim._proto.tx['id'], sim._proto.encode_210(0.0, 0.0, True, False))
        tx.close()
    yield sim
    sim.stop()
