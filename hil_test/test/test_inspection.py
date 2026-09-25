"""L4 车检任务全链路（pytest，标记 inspection + slow）.

职责：AMI 直接选择车检模式（mode 6，无需 Go 放行）→ INSPECTION 全链路：
正弦转向（15°@0.4Hz）+ 1.0 m/s 纵向驱动 + inspection_duration（27s）后
完成链（回零 + mission_complete → FINISH + 0x210 Byte6 finished=1），
以及车检中 RES 急停立即停车。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，就绪信号由 hil_test
代发，controller 真实输出经 can_interface 上 CAN（0x210）闭环验证。
vcan0 预演（仿真接口）无需 HIL_BENCH 门控，vcu_sim 自动驱动状态推进。

mission_manager 由本文件按用例起停（HIL 覆盖关自检）：FINISH(6)/EMERGENCY(7)
为终态且模式仅 IDLE/READY 可改，同一实例只能进入车检一次；每用例重启实例
以获得干净状态机，故 4 个用例彼此独立、可单独也可全量运行。
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
    pytest.mark.inspection,
    pytest.mark.slow,
    pytest.mark.skipif(
        not _ON_SIM and os.environ.get('HIL_BENCH') != '1',
        reason='L4 真实台架需车辆通电：设置 HIL_BENCH=1 后执行（vcan0 预演无需）'),
]


def _human_wait(fsd_ready, topic, expected, timeout, hint):
    """等待话题值；等待期间周期性打印人工操作提示与剩余时间."""
    deadline = time.time() + timeout
    print(f'\n[L4] 等待操作: {hint}（超时 {timeout:.0f}s）')
    last_report = time.time()
    while time.time() < deadline:
        fsd_ready.spin_once()
        if fsd_ready.latest(topic) == expected:
            return True
        if time.time() - last_report >= 5.0:
            print(f'[L4]   等待中… {hint}（剩余 {deadline - time.time():.0f}s）')
            last_report = time.time()
        time.sleep(0.2)
    return False


def _enable_bench(inj):
    """台架就绪：上线 + 门控就绪信号."""
    inj.publish_devices_inspection(ok=True)  # Signal3 上线
    inj.publish_lidar_ready()
    inj.publish_localization_ready()
    inj.publish_pose()
    time.sleep(0.5)


@pytest.fixture
def mm(mm_factory):
    """L4：每用例起停独立 mission_manager（HIL 覆盖关自检），状态机从 IDLE 开始."""
    hil_params = os.environ.get('HIL_MM_HIL_PARAMS')
    if not hil_params:
        pytest.skip('缺少 HIL_MM_HIL_PARAMS（请用 hil_test.sh -l L4 启动）')
    mm_factory(hil_params)


def _enter_inspection(inj, vcu):
    """从 IDLE 进入 INSPECTION(2)：台架就绪 → AMI 选 mode 6（无需 Go 放行）."""
    _enable_bench(inj)
    assert inj.wait_for('/system/mission_state', 1, timeout=10.0), \
        '未进入 READY（需 mission_manager 运行）'
    if vcu is not None:
        vcu.set_mode(6)  # 仿真自动 AMI；真实接口人工在 AMI 上选车检
    assert _human_wait(inj, '/system/mission_mode_cmd', 'inspection', 30.0,
                       '用 AMI 选择车检模式（6）'), \
        '未收到 mission_mode_cmd=inspection（真实 VCU 请用 AMI 选择车检）'
    assert inj.wait_for('/system/mission_state', 2, timeout=10.0), \
        '选择车检后未直接进入 INSPECTION（无需 Go 放行）'


def _wait_bus_value(mon, proto, value, timeout):
    """等待 0x210 纵向开度出现指定值."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        latest = mon.latest(proto.tx['id'])
        if latest is not None and proto.decode_210(latest[1])['longitudinal'] == value:
            return True
        time.sleep(0.05)
    return False


@pytest.mark.integration
def test_ami_inspection_entry(fsd_ready, mm, vcu):
    """AMI 车检直入：mode 6 → mission_mode_cmd=inspection → INSPECTION(2)，无需 Go."""
    _enter_inspection(fsd_ready, vcu)


@pytest.mark.integration
def test_inspection_motion(fsd_ready, mm, bus_monitor, protocol):
    """车检运动：纵向驱动开度 + 正弦 15°@0.4Hz 转向幅值与过零.

    车速反馈代发 0.0（车辆静止/架起），与目标 1.0 m/s 保持速度误差，
    确保纵向开度持续非零（反馈等于目标时 PID 误差为 0，开度会回零）。
    """
    _enter_inspection(fsd_ready, None)
    amp = zero = driven = False
    deadline = time.time() + 6.0  # 覆盖两个以上正弦周期（2.5s/周期）
    while time.time() < deadline:
        fsd_ready.publish_pose()
        fsd_ready.publish_velocity(0.0)
        latest = bus_monitor.latest(protocol.tx['id'])
        if latest is not None:
            dec = protocol.decode_210(latest[1])
            if abs(dec['lateral'] - 32767) > 10000:
                amp = True  # ±15° 对应约 ±19660 raw，阈值 ±10000 判幅值
            if dec['lateral'] == 32767:
                zero = True
            if dec['longitudinal'] != 32767:
                driven = True
        time.sleep(0.02)
    assert driven, '车检期间无纵向驱动开度'
    assert amp, '未观测到正弦转向幅值（|lateral-32767|>10000 raw）'
    assert zero, '未观测到转向过零'


@pytest.mark.integration
def test_inspection_complete(fsd_ready, mm, bus_monitor, protocol, vcu):
    """车检完成链：inspection_duration(27s) 后回零 + FINISH(6) + finished=1."""
    _enter_inspection(fsd_ready, vcu)
    assert fsd_ready.wait_for('/system/mission_state', 6, timeout=60.0), \
        '车检未在 inspection_duration 内完成（未进入 FINISH）'
    fin = False
    deadline = time.time() + 5.0
    while time.time() < deadline:
        latest = bus_monitor.latest(protocol.tx['id'])
        if latest is not None:
            dec = protocol.decode_210(latest[1])
            if dec['finished'] == 1 and dec['longitudinal'] == 32767:
                fin = True
                break
        time.sleep(0.05)
    assert fin, '完成后 0x210 未置 finished=1 或纵向未回零'


@pytest.mark.integration
def test_emergency_during_inspection(fsd_ready, mm, bus_monitor, protocol, vcu):
    """车检中 RES 急停：立即清零（急停锁存为终态；每用例独立实例，互不影响）."""
    _enter_inspection(fsd_ready, vcu)
    time.sleep(1.0)  # 确认车检运动输出中
    if vcu is not None:
        vcu.set_emergency(True)  # 仿真自动触发
    assert fsd_ready.wait_for('/system/emergency', True, timeout=30.0), \
        '未收到 emergency=true（真实 VCU 请按 RES 急停）'
    t0 = time.monotonic()  # 计时起点 = emergency 观测时刻
    assert _wait_bus_value(bus_monitor, protocol, 32767, 5.0), '急停后纵向未清零'
    stop_time = bus_monitor.latest(protocol.tx['id'])[0] - t0
    assert stop_time < 1.0, f'急停清零响应过慢: {stop_time:.2f}s'
    print(f'\n[L4] 车检急停清零响应: {stop_time:.3f}s')
