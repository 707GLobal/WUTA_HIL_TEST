"""L0 链路自检 + L0.5 仿真预跑 + L1 协议一致性（pytest）.

marker:
  link     - L0：接口 up / 心跳 / fail-safe
  sim      - L0.5：vcan 模拟 VCU（纯 CAN，无需 ROS）
  protocol - L1：编解码单测（无需硬件）+ ROS 集成用例（需 can_interface 运行）
"""

import time

import pytest

ZERO_LE = bytes([0xFF, 0x7F])  # 32767 小端


def _require_sim_interface(interface, is_sim):
    """0x501 注入仅限仿真接口，避免污染真实 VCU."""
    if not is_sim:
        pytest.skip(f'{interface} 非仿真接口，注入用例跳过（防污染真实 VCU）')


def _inject_501(interface, protocol, state, mode=None, count=20):
    """向总线注入 0x501 帧（FaultInjector）."""
    from hil_test.fault_injector import FaultInjector
    fi = FaultInjector(interface, protocol.path)
    assert fi.open()
    try:
        return fi.inject_state(state, mode=mode, count=count)
    finally:
        fi.close()


# ================= L1 纯协议单测（无需硬件） =================

@pytest.mark.protocol
def test_scale_center(protocol):
    """定标中心点：0 控制 → 32767."""
    data = protocol.encode_210(0.0, 0.0, False, False)
    assert data[:2] == ZERO_LE and data[2:4] == ZERO_LE


@pytest.mark.protocol
def test_little_endian(protocol):
    """字节序：小端，Byte1=低字节."""
    # 纵向满驱动 scale(1)=65525=0xFFF5 → [F5 FF]；横向左满 scale(-1)=10 → [0A 00]
    data = protocol.encode_210(1.0, 25.0, False, False)
    assert data[:2] == bytes([0xF5, 0xFF])
    assert data[2:4] == bytes([0x0A, 0x00])


@pytest.mark.protocol
def test_clamp_out_of_range(protocol):
    """越界钳位：[10, 65525]."""
    data = protocol.encode_210(5.0, 0.0, False, False)
    assert data[:2] == bytes([0xF5, 0xFF])
    data = protocol.encode_210(-5.0, 0.0, False, False)
    assert data[:2] == bytes([0x0A, 0x00])


@pytest.mark.protocol
def test_mid_scale(protocol):
    """中值定标：与 C++ scaleControl 一致."""
    from hil_test.protocol_loader import scale_control
    data = protocol.encode_210(0.5, 0.0, False, False)
    assert protocol.decode_210(data)['longitudinal'] == scale_control(0.5)
    data = protocol.encode_210(0.0, -12.5, False, False)  # 右半圈
    assert protocol.decode_210(data)['lateral'] == scale_control(0.5)


@pytest.mark.protocol
def test_online_finished_bytes(protocol):
    """上线/完成信号位."""
    data = protocol.encode_210(0.0, 0.0, True, True)
    assert data[4] == 1 and data[5] == 1
    data = protocol.encode_210(0.0, 0.0, False, False)
    assert data[4] == 0 and data[5] == 0


@pytest.mark.protocol
def test_decode_501(protocol):
    """0x501 解析：Byte1 状态、Byte2 模式."""
    state, mode = protocol.decode_501(bytes([12, 3, 0, 0, 0, 0, 0, 0]))
    assert (state, mode) == (12, 3)


@pytest.mark.protocol
def test_mode_topic_map(protocol):
    """测试模式 → mission_mode_cmd 映射."""
    assert protocol.mode_topic(2) == 'acceleration'
    assert protocol.mode_topic(3) == 'trackdrive'
    assert protocol.mode_topic(4) == 'skidpad'
    assert protocol.mode_topic(5) == 'ebs_test'
    assert protocol.mode_topic(6) == 'inspection'
    assert protocol.mode_topic(1) is None  # 操控性（有人驾驶）忽略
    assert protocol.mode_topic(99) is None  # 未知模式


# ================= L0.5 仿真预跑（vcan + vcu_sim） =================

@pytest.mark.sim
def test_sim_501_heartbeat(vcu_sim, bus_monitor, protocol):
    """VCU 模拟：0x501 10Hz 稳定周期 ≈100ms."""
    assert bus_monitor.wait_for(protocol.rx['id'], timeout=3.0)
    time.sleep(1.2)
    stats = bus_monitor.period_stats(protocol.rx['id'])
    assert stats is not None and 0.06 <= stats['avg'] <= 0.14


@pytest.mark.sim
def test_sim_online_advance(vcu_sim, interface, protocol):
    """VCU 模拟：上线(0x210 Signal3=1) 后 6→9；请求出发 → 10."""
    from hil_test.can_socket import CanSocket
    tx = CanSocket(interface)
    assert tx.open()
    try:
        assert vcu_sim.state == 6
        tx.send(protocol.tx['id'], protocol.encode_210(0.0, 0.0, True, False))
        assert _wait_until(lambda: vcu_sim.state == 9, 2.0)
        vcu_sim.request_go()
        assert _wait_until(lambda: vcu_sim.state == 10, 2.0)
    finally:
        tx.close()


@pytest.mark.sim
def test_sim_state_override(vcu_sim):
    """VCU 模拟：状态优先级 EMERGENCY(12) > FINISHED(11) > DRIVING(10)."""
    vcu_sim.set_finished(True)
    assert vcu_sim.state == 11
    vcu_sim.set_emergency(True)
    assert vcu_sim.state == 12
    vcu_sim.set_emergency(False)
    assert vcu_sim.state == 11
    vcu_sim.set_finished(False)
    vcu_sim.set_mode(6)
    assert vcu_sim.mode == 6


def _wait_until(predicate, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


# ================= L0 链路自检（需 can_interface 运行） =================

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


@pytest.mark.link
@pytest.mark.integration
def test_link_501_heartbeat(fsd_ready, bus_monitor, protocol):
    """VCU→工控机心跳：0x501 10Hz."""
    assert bus_monitor.wait_for(protocol.rx['id'], timeout=3.0)
    time.sleep(1.1)
    stats = bus_monitor.period_stats(protocol.rx['id'])
    assert stats is not None and 0.06 <= stats['avg'] <= 0.14


# ================= L1 ROS 集成（需 can_interface 运行 + vcan0 注入） =================

@pytest.mark.protocol
@pytest.mark.integration
def test_501_go_trigger(fsd_ready, protocol, interface, is_sim):
    """Go 触发：0x501 state=10 → /system/start_command=true."""
    _require_sim_interface(interface, is_sim)
    assert _inject_501(interface, protocol, state=10) > 0
    assert fsd_ready.wait_for('/system/start_command', True, timeout=5.0)


@pytest.mark.protocol
@pytest.mark.integration
def test_501_emergency_trigger(fsd_ready, protocol, interface, is_sim):
    """急停触发：0x501 state=12 → /system/emergency=true."""
    _require_sim_interface(interface, is_sim)
    assert _inject_501(interface, protocol, state=12) > 0
    assert fsd_ready.wait_for('/system/emergency', True, timeout=5.0)


@pytest.mark.protocol
@pytest.mark.integration
def test_501_mode_mapping(fsd_ready, protocol, interface, is_sim):
    """模式映射：Byte2=2/3/4/5/6 → mission_mode_cmd."""
    _require_sim_interface(interface, is_sim)
    for mode, expect in ((2, 'acceleration'), (3, 'trackdrive'),
                         (4, 'skidpad'), (5, 'ebs_test'), (6, 'inspection')):
        assert _inject_501(interface, protocol, state=9, mode=mode) > 0
        assert fsd_ready.wait_for('/system/mission_mode_cmd', expect, timeout=5.0), \
            f'mode {mode} 未映射为 {expect}'


@pytest.mark.protocol
@pytest.mark.integration
def test_501_mode1_ignored(fsd_ready, protocol, interface, is_sim):
    """操控性模式：Byte2=1 不改动 mission_mode_cmd."""
    _require_sim_interface(interface, is_sim)
    # 先设一个已知模式，再注入 1，确认值不变
    assert _inject_501(interface, protocol, state=9, mode=3) > 0
    assert fsd_ready.wait_for('/system/mission_mode_cmd', 'trackdrive', timeout=5.0)
    before = fsd_ready.latest('/system/mission_mode_cmd')
    assert _inject_501(interface, protocol, state=9, mode=1) > 0
    time.sleep(0.5)
    fsd_ready.spin_once()
    assert fsd_ready.latest('/system/mission_mode_cmd') == before


@pytest.mark.protocol
@pytest.mark.integration
def test_210_scaling_from_command(fsd_ready, bus_monitor, protocol):
    """0x210 透传：注入 /control/command → 总线信号与定标一致."""
    fsd_ready.publish_command(speed=0.0, angle=12.5, throttle_brake=0.5)
    assert bus_monitor.wait_for(protocol.tx['id'], timeout=3.0)
    latest = bus_monitor.latest(protocol.tx['id'])
    dec = protocol.decode_210(latest[1])
    from hil_test.protocol_loader import scale_control
    assert dec['longitudinal'] == scale_control(0.5)
    assert dec['lateral'] == scale_control(-12.5 / protocol.max_steer_deg)
