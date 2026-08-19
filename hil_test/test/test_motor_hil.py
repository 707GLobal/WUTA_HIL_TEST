"""L3 电机闭环（pytest，标记 slow；需真实台架与 HIL_BENCH=1）.

台架模式：车辆架起，传感器仅保在线。位姿/车速/路径/就绪信号由 hil_test 代发
（/localization/pose、/chcnav/velocity、/planning/final_waypoints 直路、
/system/lidar_ready、/system/localization_ready、/system/devices_inspection），
controller 真实输出，经 can_interface 上 CAN（0x210）闭环验证。
指标：稳态驱动、加减速跟随、急停刹停时间。
"""

import os
import time

import pytest

pytestmark = [
    pytest.mark.motor,
    pytest.mark.slow,
    pytest.mark.skipif(
        os.environ.get('HIL_BENCH') != '1',
        reason='L3 需要真实电机台架：设置 HIL_BENCH=1 后执行'),
]


def _drive_bench(inj, vx, duration):
    """以 20Hz 代发台架位姿 + 车速反馈."""
    deadline = time.time() + duration
    while time.time() < deadline:
        inj.publish_pose()
        inj.publish_velocity(vx)
        time.sleep(0.05)


def _enable_bench(inj):
    """台架就绪：上线 + 就绪信号 + 直路 + RACE 使能 controller."""
    inj.publish_devices_inspection(ok=True)  # Signal3 上线
    inj.publish_lidar_ready()
    inj.publish_localization_ready()
    inj.publish_waypoints_straight()
    inj.publish_mission_state(5)  # RACE
    time.sleep(0.5)


def _bus_longitudinal(mon, proto):
    """最近 0x210 纵向开度；无帧返回 32767（零）."""
    latest = mon.latest(proto.tx['id'])
    if latest is None:
        return 32767
    return proto.decode_210(latest[1])['longitudinal']


def _wait_bus_value(mon, proto, value, timeout):
    """等待 0x210 纵向开度出现指定值（或非零/零）."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _bus_longitudinal(mon, proto) == value:
            return True
        time.sleep(0.05)
    return False


@pytest.mark.integration
def test_constant_speed_follow(fsd_ready, bus_monitor, protocol):
    """恒速跟随：车速 3m/s 反馈 → 0x210 出现驱动开度；车速归零 → 回零."""
    _enable_bench(fsd_ready)
    _drive_bench(fsd_ready, 3.0, 2.0)
    driven = False
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if _bus_longitudinal(bus_monitor, protocol) != 32767:
            driven = True
            break
        time.sleep(0.05)
    assert driven, '恒速跟随期间无驱动开度'
    _drive_bench(fsd_ready, 0.0, 1.0)
    assert _wait_bus_value(bus_monitor, protocol, 32767, 3.0), '停车后未回零'


@pytest.mark.integration
def test_ramp_accel_decel(fsd_ready, bus_monitor, protocol):
    """加减速斜坡：0→3→0 m/s，跟随输出并收敛回零."""
    _enable_bench(fsd_ready)
    for v in (1.0, 2.0, 3.0, 2.0, 1.0, 0.0):
        _drive_bench(fsd_ready, v, 1.0)
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '斜坡后纵向未回零'


@pytest.mark.integration
def test_emergency_stop_time(fsd_ready, bus_monitor, protocol, vcu):
    """急停刹停：行驶中触发急停 → 纵向开度清零，记录刹停时间."""
    _enable_bench(fsd_ready)
    _drive_bench(fsd_ready, 3.0, 1.0)  # 先行驶
    t0 = time.time()
    if vcu is not None:
        vcu.set_emergency(True)  # 仿真自动触发
    assert fsd_ready.wait_for('/system/emergency', True, timeout=30.0), \
        '未收到 emergency=true（真实 VCU 请触发 RES 急停）'
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '急停后纵向未清零'
    if vcu is not None:
        stop_time = time.time() - t0
        assert stop_time < 1.0, f'刹停响应过慢: {stop_time:.2f}s'
