"""L1 协议/链路测试（pytest）：协议单测 + vcu_sim 模型（pre）+ 链路/协议集成（post）.

marker（L1 分两阶段执行，见 run_hil.py / hil_test.sh）：
  sim      - L1 pre：vcan 模拟 VCU（纯 CAN，无需 ROS/FSD）
  unit     - L1 pre：协议编解码单测（无需硬件）
  link     - L1 post：接口 up / 心跳 / fail-safe（需 can_interface 运行）
  protocol - L1 post：ROS 集成，0x501 模式→话题映射 / 0x210 定标透传（需 can_interface 运行）

新协议：0x501 Byte1 = 测试模式，原 Byte1「VCU 状态」定义已取消（不再有 Go/急停）。
"""

import time

import pytest

ZERO_LE = bytes([0xFF, 0x7F])  # 32767 小端


def _require_sim_interface(interface, is_sim):
    """0x501 注入仅限仿真接口，避免污染真实 VCU."""
    if not is_sim:
        pytest.skip(f'{interface} 非仿真接口，注入用例跳过（防污染真实 VCU）')


def _inject_501(interface, protocol, mode, count=20):
    """向总线注入 0x501 模式帧（FaultInjector）."""
    from hil_test.fault_injector import FaultInjector
    fi = FaultInjector(interface, protocol.path)
    assert fi.open()
    try:
        return fi.inject_mode(mode, count=count)
    finally:
        fi.close()


# ================= L1 pre · 协议编解码单测（无需硬件） =================

@pytest.mark.unit
def test_scale_center(protocol):
    """定标中心点：0 控制 → 32767."""
    data = protocol.encode_210(0.0, 0.0, False, False)
    assert data[:2] == ZERO_LE and data[2:4] == ZERO_LE


@pytest.mark.unit
def test_little_endian(protocol):
    """字节序：小端，Byte1=低字节."""
    # 纵向满驱动 scale(1)=65525=0xFFF5 → [F5 FF]；横向左满 scale(-1)=10 → [0A 00]
    data = protocol.encode_210(1.0, 25.0, False, False)
    assert data[:2] == bytes([0xF5, 0xFF])
    assert data[2:4] == bytes([0x0A, 0x00])


@pytest.mark.unit
def test_clamp_out_of_range(protocol):
    """越界钳位：[10, 65525]."""
    data = protocol.encode_210(5.0, 0.0, False, False)
    assert data[:2] == bytes([0xF5, 0xFF])
    data = protocol.encode_210(-5.0, 0.0, False, False)
    assert data[:2] == bytes([0x0A, 0x00])


@pytest.mark.unit
def test_mid_scale(protocol):
    """中值定标：与 C++ scaleControl 一致."""
    from hil_test.protocol_loader import scale_control
    data = protocol.encode_210(0.5, 0.0, False, False)
    assert protocol.decode_210(data)['longitudinal'] == scale_control(0.5)
    data = protocol.encode_210(0.0, -12.5, False, False)  # 右半圈
    assert protocol.decode_210(data)['lateral'] == scale_control(0.5)


@pytest.mark.unit
def test_online_finished_bytes(protocol):
    """上线/完成信号位."""
    data = protocol.encode_210(0.0, 0.0, True, True)
    assert data[4] == 1 and data[5] == 1
    data = protocol.encode_210(0.0, 0.0, False, False)
    assert data[4] == 0 and data[5] == 0


@pytest.mark.unit
def test_decode_501(protocol):
    """0x501 解析：Byte1 测试模式（原状态字节已取消）."""
    assert protocol.decode_501(bytes([3, 0, 0, 0, 0, 0, 0, 0])) == 3


@pytest.mark.unit
def test_mode_topic_map(protocol):
    """测试模式 → mission_mode_cmd 映射."""
    assert protocol.mode_topic(2) == 'acceleration'
    assert protocol.mode_topic(3) == 'trackdrive'
    assert protocol.mode_topic(4) == 'skidpad'
    assert protocol.mode_topic(5) == 'ebs_test'
    assert protocol.mode_topic(6) == 'inspection'
    assert protocol.mode_topic(1) is None  # 操控性（有人驾驶）忽略
    assert protocol.mode_topic(99) is None  # 未知模式


# ================= L1 pre · VCU 模拟（vcan + vcu_sim） =================

@pytest.mark.sim
def test_sim_501_event(vcu_sim, bus_monitor, protocol):
    """VCU 模拟：模式变化时发帧（事件驱动，无周期心跳），Byte1=模式.

    vcu_sim 启动时已发过首帧（mode=1），该帧可能早于 bus_monitor 打开而错过；
    事件驱动语义下只有模式变化才会再发，故先切一次模式触发新帧再断言。
    """
    def _mode_is(mode):
        frame = bus_monitor.latest(protocol.rx['id'])
        return frame is not None and frame[1][protocol.rx['mode_byte']] == mode

    vcu_sim.set_mode(3)
    assert _wait_until(lambda: _mode_is(3), 3.0), '模式变化后未收到 0x501'
    vcu_sim.set_mode(6)
    assert _wait_until(lambda: _mode_is(6), 3.0), '模式再变化后未收到 0x501'
    assert vcu_sim.mode == 6


def _wait_until(predicate, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


# ================= L1 post · 链路自检（需 can_interface 运行） =================

@pytest.mark.link
def test_link_can_interface_up(can_ready):
    """CAN 接口 up."""
    assert can_ready


@pytest.mark.link
@pytest.mark.integration
def test_link_210_heartbeat(fsd_ready, bus_monitor, protocol):
    """工控机→VCU 心跳：0x210 10Hz 保活."""
    assert bus_monitor.wait_for(protocol.tx['id'], timeout=3.0)
    time.sleep(1.1)
    stats = bus_monitor.period_stats(protocol.tx['id'])
    assert stats is not None and 0.06 <= stats['avg'] <= 0.14


# ================= L1 post · ROS 集成（需 can_interface 运行 + vcan0 注入） =================

@pytest.mark.protocol
@pytest.mark.integration
def test_501_mode_mapping(fsd_ready, protocol, interface, is_sim):
    """模式映射：Byte1=2/3/4/5/6 → mission_mode_cmd."""
    _require_sim_interface(interface, is_sim)
    for mode, expect in ((2, 'acceleration'), (3, 'trackdrive'),
                         (4, 'skidpad'), (5, 'ebs_test'), (6, 'inspection')):
        assert _inject_501(interface, protocol, mode=mode) > 0
        assert fsd_ready.wait_for('/system/mission_mode_cmd', expect, timeout=5.0), \
            f'mode {mode} 未映射为 {expect}'


@pytest.mark.protocol
@pytest.mark.integration
def test_501_mode1_ignored(fsd_ready, protocol, interface, is_sim):
    """操控性模式：Byte1=1 不改动 mission_mode_cmd."""
    _require_sim_interface(interface, is_sim)
    # 先设一个已知模式，再注入 1，确认值不变
    assert _inject_501(interface, protocol, mode=3) > 0
    assert fsd_ready.wait_for('/system/mission_mode_cmd', 'trackdrive', timeout=5.0)
    before = fsd_ready.latest('/system/mission_mode_cmd')
    assert _inject_501(interface, protocol, mode=1) > 0
    time.sleep(0.5)
    fsd_ready.spin_once()
    assert fsd_ready.latest('/system/mission_mode_cmd') == before


@pytest.mark.protocol
@pytest.mark.integration
def test_210_scaling_from_command(fsd_ready, bus_monitor, protocol):
    """0x210 透传：注入 /control/command → 总线信号与定标一致.

    0x210 为 10Hz 保活，命令生效前的旧帧可能先到总线；
    故轮询等待携带新定标值的帧，而非「出现任意一帧」即断言。
    """
    from hil_test.protocol_loader import scale_control
    fsd_ready.publish_command(speed=0.0, angle=12.5, throttle_brake=0.5)
    expect_long = scale_control(0.5)

    def _got_scaled_frame():
        latest = bus_monitor.latest(protocol.tx['id'])
        return (latest is not None
                and protocol.decode_210(latest[1])['longitudinal'] == expect_long)

    assert _wait_until(_got_scaled_frame, 3.0), \
        f'0x210 纵向未按 throttle_brake 定标: {bus_monitor.latest(protocol.tx["id"])}'
    dec = protocol.decode_210(bus_monitor.latest(protocol.tx['id'])[1])
    assert dec['lateral'] == scale_control(-12.5 / protocol.max_steer_deg)
