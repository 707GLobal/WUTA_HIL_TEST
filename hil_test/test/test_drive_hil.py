"""L3 低速动态安全闭环（pytest，标记 motor + slow）.

职责：AMI 直线加速模式选择 + RES Go 放行 + 低速驱动（轮子动）+ RES 急停立即停车——
READY(1) → mission_mode_cmd=acceleration → RES Go 放行 → EXPLORE(3)
→ controller 经 can_interface 上 CAN（0x210）输出驱动开度 → RES 急停
→ /system/emergency=true + mission_state EMERGENCY(7) + 0x210 纵向清零。

目标车速由 hil_test.yaml bench.target_speed_mps 限制（默认 0.2 m/s 安全低速，
先低后调），经 /planning/final_waypoints 的直路下发。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，位姿/车速/路径/就绪信号
由 hil_test 代发（/localization/pose、/chcnav/velocity、/planning/final_waypoints
直路、/system/lidar_ready、/system/localization_ready、/system/devices_inspection）；
AMI 选模式、RES Go、RES 急停为 VCU 侧真实信号，由人工操作
（仿真接口下 vcu_sim 自动驱动）。
vcan0 预演（仿真接口）无需 HIL_BENCH 门控，vcu_sim 自动驱动状态推进。
"""

import os
import time

import pytest


def _sim_interface():
    """当前接口是否为仿真接口（vcan0 预演免 HIL_BENCH 门控）."""
    try:
        import yaml
    except ImportError:
        return False
    cfg_dir = os.environ.get(
        'HIL_CONFIG',
        os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'config'))
    try:
        with open(os.path.join(cfg_dir, 'hil_test.yaml'), 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
    except OSError:
        return False
    ifname = os.environ.get('HIL_INTERFACE') or cfg.get('can', {}).get('interface', 'can0')
    return ifname in cfg.get('can', {}).get('sim_interfaces', ['vcan0'])


_ON_SIM = _sim_interface()

pytestmark = [
    pytest.mark.motor,
    pytest.mark.slow,
    pytest.mark.skipif(
        not _ON_SIM and os.environ.get('HIL_BENCH') != '1',
        reason='L3 真实台架需车辆通电：设置 HIL_BENCH=1 后执行（vcan0 预演无需）'),
]


def _human_wait(fsd_ready, topic, expected, timeout, hint):
    """等待话题值；等待期间周期性打印人工操作提示与剩余时间."""
    deadline = time.time() + timeout
    print(f'\n[L3] 等待操作: {hint}（超时 {timeout:.0f}s）')
    last_report = time.time()
    while time.time() < deadline:
        fsd_ready.spin_once()
        if fsd_ready.latest(topic) == expected:
            return True
        if time.time() - last_report >= 5.0:
            print(f'[L3]   等待中… {hint}（剩余 {deadline - time.time():.0f}s）')
            last_report = time.time()
        time.sleep(0.2)
    return False


def _enable_bench(inj):
    """台架就绪：上线 + 门控就绪信号 + 直路（目标车速取 yaml bench 限速）."""
    inj.publish_devices_inspection(ok=True)  # Signal3 上线
    inj.publish_lidar_ready()
    inj.publish_localization_ready()
    inj.publish_pose()
    inj.publish_waypoints_straight()
    time.sleep(0.5)


def _drive_bench(inj, vx, duration):
    """以 20Hz 代发台架位姿 + 车速反馈（controller 需要连续位姿/车速流才能闭环输出）."""
    deadline = time.time() + duration
    while time.time() < deadline:
        inj.publish_pose()
        inj.publish_velocity(vx)
        time.sleep(0.05)


def _bus_longitudinal(mon, proto):
    """最近 0x210 纵向开度；无帧返回 32767（零）."""
    latest = mon.latest(proto.tx['id'])
    if latest is None:
        return 32767
    return proto.decode_210(latest[1])['longitudinal']


def _wait_bus_value(mon, proto, value, timeout):
    """等待 0x210 纵向开度出现指定值（非零/零）."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _bus_longitudinal(mon, proto) == value:
            return True
        time.sleep(0.05)
    return False


def _enter_explore(fsd_ready, vcu):
    """确保系统进入 EXPLORE(3)：已在则直接返回；否则按 AMI 直线加速 + RES Go 走一遍.

    mission_manager 跨用例长驻（单实例），状态可能已推进到 EXPLORE(3) 且无回退
    路径；RosInjector 则每用例新建、缓存为空，故先 spin 探测当前状态再决定短路。
    """
    # 已在 EXPLORE(3) 直接返回（状态机单向：EXPLORE 无回到 READY 的路径）
    if fsd_ready.wait_for('/system/mission_state', 3, timeout=1.0):
        return
    _enable_bench(fsd_ready)
    assert fsd_ready.wait_for('/system/mission_state', 1, timeout=10.0), \
        '未进入 READY（需 mission_manager 运行）'
    if vcu is not None:
        vcu.set_mode(2)  # 仿真自动 AMI；真实接口人工在 AMI 上选直线加速
    assert _human_wait(fsd_ready, '/system/mission_mode_cmd', 'acceleration', 30.0,
                       '用 AMI 选择直线加速模式（2）'), \
        '未收到 mission_mode_cmd=acceleration（真实 VCU 请用 AMI 选择直线加速）'
    if vcu is not None:
        vcu.request_go(True)  # 仿真自动 RES Go；真实接口人工按 RES Go
    assert _human_wait(fsd_ready, '/system/start_command', True, 30.0,
                       '按 RES Go 放行（VCU 进入驾驶态 10）'), \
        '未收到 start_command=true（真实 VCU 请按 RES Go）'
    assert fsd_ready.wait_for('/system/mission_state', 3, timeout=10.0), \
        '收到 Go 后未进入 EXPLORE'


@pytest.mark.integration
def test_ami_acceleration_go(fsd_ready, vcu):
    """AMI 直线加速 + RES Go 放行：READY → mission_mode_cmd=acceleration → EXPLORE."""
    _enter_explore(fsd_ready, vcu)


@pytest.mark.integration
def test_low_speed_follow(fsd_ready, vcu, bus_monitor, protocol):
    """低速驱动：EXPLORE 下代发直路（yaml 限速）、车速反馈静止 → 0x210 出现驱动开度.

    车辆架起、目标车速 = bench.target_speed_mps（安全限速），故轮子会低速转动。
    """
    _enter_explore(fsd_ready, vcu)
    fsd_ready.publish_waypoints_straight()  # 目标速度 = bench.target_speed_mps
    _drive_bench(fsd_ready, 0.0, 2.0)  # 车辆静止反馈 → 速度误差驱动输出
    driven = False
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if _bus_longitudinal(bus_monitor, protocol) != 32767:
            driven = True
            break
        time.sleep(0.05)
    assert driven, '低速跟随期间 0x210 无驱动开度（controller 未参与或未闭环输出）'


@pytest.mark.integration
def test_emergency_stop(fsd_ready, vcu, bus_monitor, protocol):
    """RES 急停：EXPLORE 运行态触发 → emergency=true、mission_state→EMERGENCY(7)，
    且 1s 内 0x210 纵向开度清零（停发控制）。

    EMERGENCY 为终态锁存，本用例置于文件末位、最后执行。
    """
    _enter_explore(fsd_ready, vcu)
    _drive_bench(fsd_ready, 0.0, 1.0)  # 保持驱动（目标限速 > 反馈 0），以便验证清零
    if vcu is not None:
        vcu.set_emergency(True)  # 仿真自动触发；真实接口人工按 RES 急停
    assert _human_wait(fsd_ready, '/system/emergency', True, 30.0,
                       '按 RES 急停'), \
        '未收到 emergency=true（真实 VCU 请按 RES 急停）'
    assert fsd_ready.wait_for('/system/mission_state', 7, timeout=10.0), \
        '急停后未进入 EMERGENCY(7)'
    t0 = time.monotonic()  # 计时起点 = emergency 观测时刻（非等待前）
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '急停后 0x210 纵向未清零'
    stop_time = bus_monitor.latest(protocol.tx['id'])[0] - t0
    assert stop_time < 1.0, f'急停清零响应过慢: {stop_time:.2f}s'
    print(f'\n[L3] 急停清零响应: {stop_time:.3f}s')
