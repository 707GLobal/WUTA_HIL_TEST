"""L2 安全与状态联动（pytest）.

运行方式（由 hil_test.yaml can.sim_interfaces 判定）：
  - 真实接口（如 can0，真实 VCU）：观察总线与 ROS，测试等待人工操作（RES Go/急停、AMI 选择）；
  - 仿真接口（如 vcan0）：vcu_sim 自动驱动状态推进，全自动。
"""

import time

import pytest


@pytest.mark.safety
@pytest.mark.integration
def test_start_gate_zero_output(fsd_ready, bus_monitor, protocol, vcu):
    """启动门控：未给 Go（状态≠10）时注入控制指令 → 0x210 恒为零控制."""
    if vcu is not None:  # 仿真：确保非驾驶态
        vcu.request_go(False)
        vcu.set_emergency(False)
        vcu.set_finished(False)
    fsd_ready.publish_command(speed=2.0, angle=10.0, throttle_brake=0.5)
    time.sleep(0.5)
    assert bus_monitor.wait_for(protocol.tx['id'], timeout=3.0)
    dec = protocol.decode_210(bus_monitor.latest(protocol.tx['id'])[1])
    assert dec['longitudinal'] == 32767 and dec['lateral'] == 32767, \
        '未放行时总线出现非零控制量'


@pytest.mark.safety
@pytest.mark.integration
def test_go_release(fsd_ready, vcu):
    """Go 放行：ready 代发 → READY；VCU 驾驶态(10) → start_command → EXPLORE."""
    # 台架代发就绪信号（IDLE→READY 门控）
    fsd_ready.publish_lidar_ready()
    fsd_ready.publish_localization_ready()
    fsd_ready.publish_pose()
    assert fsd_ready.wait_for('/system/mission_state', 1, timeout=10.0), \
        '未进入 READY（需 mission_manager 运行）'
    if vcu is not None:
        vcu.request_go(True)
    # 真实 VCU：人工操作 RES 给 Go；仿真：vcu_sim 自动进入 10
    assert fsd_ready.wait_for('/system/start_command', True, timeout=30.0), \
        '未收到 start_command=true（真实 VCU 请操作 RES 给 Go）'
    assert fsd_ready.wait_for('/system/mission_state', 3, timeout=10.0), \
        '收到 Go 后未进入 EXPLORE'


@pytest.mark.safety
@pytest.mark.integration
def test_emergency_priority(fsd_ready, bus_monitor, protocol, vcu):
    """急停优先级：VCU 状态 12 → /system/emergency=true + 总线零控制."""
    if vcu is not None:
        vcu.set_emergency(True)
    assert fsd_ready.wait_for('/system/emergency', True, timeout=30.0), \
        '未收到 emergency=true（真实 VCU 请触发 RES 急停）'
    # controller 急停后发布全零（若 controller 运行），至多等 5s
    deadline = time.time() + 5.0
    zero_seen = False
    while time.time() < deadline:
        if bus_monitor.count(protocol.tx['id']) > 0:
            dec = protocol.decode_210(bus_monitor.latest(protocol.tx['id'])[1])
            if dec['longitudinal'] == 32767 and dec['lateral'] == 32767:
                zero_seen = True
                break
        time.sleep(0.05)
    assert zero_seen, '急停后 0x210 仍非零'


@pytest.mark.safety
@pytest.mark.integration
def test_watchdog_vcu_loss(vcu):
    """看门狗：VCU 失联超时 → 设备自检失败路径（Signal3=0）+ 错误日志."""
    pytest.skip('can_interface VCU 失联检测待开发（方案 §7）')


@pytest.mark.safety
@pytest.mark.integration
def test_inspection_flow(fsd_ready, vcu):
    """车检流程：测试模式=6 → /system/mission_mode_cmd="inspection"."""
    if vcu is not None:
        vcu.set_mode(6)
    assert fsd_ready.wait_for('/system/mission_mode_cmd', 'inspection', timeout=30.0), \
        '未收到 mission_mode_cmd=inspection（真实 VCU 请用 AMI 选择车检）'
