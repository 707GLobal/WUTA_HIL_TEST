"""L3 低速动态安全闭环（pytest，标记 motor + slow）.

职责：AMI 直线加速模式选择 + RES Go 放行 + RES 急停立即停车；
目标车速由 hil_test.yaml bench.target_speed_mps 限制（默认 1.0 m/s
安全低速，先低后调）；另覆盖启动门控零输出与加减速跟随。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，位姿/车速/路径/
就绪信号由 hil_test 代发（/localization/pose、/chcnav/velocity、
/planning/final_waypoints 直路、/system/lidar_ready、
/system/localization_ready、/system/devices_inspection），controller
真实输出经 can_interface 上 CAN（0x210）闭环验证。
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


def _drive_bench(inj, vx, duration):
    """以 20Hz 代发台架位姿 + 车速反馈."""
    deadline = time.time() + duration
    while time.time() < deadline:
        inj.publish_pose()
        inj.publish_velocity(vx)
        time.sleep(0.05)


def _enable_bench(inj):
    """台架就绪：上线 + 门控就绪信号 + 直路（目标车速取 yaml bench 限速）."""
    inj.publish_devices_inspection(ok=True)  # Signal3 上线
    inj.publish_lidar_ready()
    inj.publish_localization_ready()
    inj.publish_pose()
    inj.publish_waypoints_straight()
    time.sleep(0.5)


def _require_explore(inj):
    """确保 controller 使能（EXPLORE 态）；否则跳过（需先完成 AMI+Go 用例）."""
    if inj.latest('/system/mission_state') != 3:
        pytest.skip('当前非 EXPLORE 态，需先完成 test_ami_acceleration_go')


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


@pytest.mark.integration
def test_ami_acceleration_go(fsd_ready, vcu):
    """AMI 直线加速 + RES Go 放行：READY → mission_mode_cmd=acceleration → EXPLORE."""
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
def test_low_speed_follow(fsd_ready, bus_monitor, protocol):
    """低速驱动：目标 yaml 限速、反馈静止 → 0x210 出现稳定驱动开度."""
    _require_explore(fsd_ready)
    fsd_ready.publish_waypoints_straight()  # 目标速度 = bench.target_speed_mps
    _drive_bench(fsd_ready, 0.0, 2.0)  # 车辆静止反馈 → 速度误差驱动输出
    driven = False
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if _bus_longitudinal(bus_monitor, protocol) != 32767:
            driven = True
            break
        time.sleep(0.05)
    assert driven, '低速跟随期间无驱动开度'


@pytest.mark.integration
def test_ramp_accel_decel(fsd_ready, bus_monitor, protocol, bench_speed):
    """加减速斜坡：目标速度 0→限速→0（waypoints 重发），开度随目标增大并最终回零.

    车速反馈代发 0.0（车辆静止），使 PID 速度误差 = 当前目标速度；若反馈
    跟随目标则误差恒 0、开度恒为零，断言将失效。
    """
    _require_explore(fsd_ready)
    peaks = []
    for ratio in (0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0):
        fsd_ready.publish_waypoints_straight(speed=bench_speed * ratio)
        _drive_bench(fsd_ready, 0.0, 1.0)
        peaks.append(_bus_longitudinal(bus_monitor, protocol))
    assert peaks[3] > peaks[0], f'纵向开度未随目标车速增大: {peaks}'
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '斜坡后纵向未回零'


@pytest.mark.integration
def test_emergency_stop_time(fsd_ready, bus_monitor, protocol, vcu):
    """RES 急停：行驶中触发 → /system/emergency=true，1s 内 0x210 清零."""
    _require_explore(fsd_ready)
    _drive_bench(fsd_ready, 0.0, 1.0)  # 保持驱动误差（目标限速 > 反馈 0）
    if vcu is not None:
        vcu.set_emergency(True)  # 仿真自动触发
    assert fsd_ready.wait_for('/system/emergency', True, timeout=30.0), \
        '未收到 emergency=true（真实 VCU 请按 RES 急停）'
    t0 = time.monotonic()  # 计时起点 = emergency 观测时刻（非等待前）
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '急停后纵向未清零'
    stop_time = bus_monitor.latest(protocol.tx['id'])[0] - t0
    assert stop_time < 1.0, f'急停清零响应过慢: {stop_time:.2f}s'
    print(f'\n[L3] 急停清零响应: {stop_time:.3f}s')


@pytest.mark.integration
def test_watchdog_vcu_loss(vcu):
    """看门狗：VCU 失联超时 → 设备自检失败路径（Signal3=0）+ 错误日志."""
    pytest.skip('can_interface VCU 失联检测待开发（方案 §7）')
