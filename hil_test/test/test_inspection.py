"""L4 车检任务全链路（pytest，标记 inspection + slow）.

职责：AMI 直接选择车检模式（mode 6，选模式即启动）→ INSPECTION 全链路：
正弦转向（15°@0.4Hz）+ 1.0 m/s 纵向驱动 + inspection_duration（27s）后
完成链（回零 + mission_complete → FINISH + 0x210 Byte6 finished=1）。
进入 INSPECTION 后留一个人工按 RES Go 的等待窗（VCU 侧放行；FSD 本身不需要 GO），
由 hil_test.yaml 的 bench.res_go_wait_sec 配置。

AMI 模式判据由测试自己用 BusMonitor 直接解析 0x501（Byte1=6），不再依赖
can_interface 转发的 /system/mission_mode_cmd（那是一次性边沿 + 去重，而本文件
每个用例都重启 mission_manager，需要重新投递模式，走话题会永久漏收）。

新协议：RES 急停不再经 CAN 0x501 下发（原 Byte1 状态定义取消），车检中急停
通路由 L2 自检失败覆盖。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，就绪信号由 hil_test
代发，controller 真实输出经 can_interface 上 CAN（0x210）闭环验证。
vcan0 预演（仿真接口）无需 HIL_BENCH 门控，vcu_sim 自动选模式。

mission_manager 由本文件按用例起停（HIL 覆盖关自检）：FINISH(6) 为终态且模式
仅 IDLE/READY 可改，同一实例只能进入车检一次；每用例重启实例以获得干净状态机，
故 3 个用例彼此独立、可单独也可全量运行。
"""

import os
import time

import pytest

from hil_test.bench_ui import pause_for_res_go


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


def _human_wait(mon, proto, expect_mode, timeout, hint, inj=None,
                state=None, state_mode=None):
    """等待人工操作结果：**本机直接解析 0x501**，Byte1 达标即返回.

    判据优先级：
      1) 本机总线监听解析到的 0x501 Byte1 == expect_mode（主判据）；
      2) 电平证据 /system/mission_state == (state, state_mode)（兜底，需传 inj）。

    为什么不用 /system/mission_mode_cmd：那是 can_interface 仅在 0x501 模式字节
    「变化」时发布一次的话题（去重 + volatile 不 latch），而本文件每个用例都重启
    mission_manager，需要重新投递一次模式；走话题会永久漏收并空等超时。
    详见 test_drive_hil._human_wait 的说明。
    """
    deadline = time.time() + timeout
    t_wait = time.monotonic()
    print(f'\n[L4] 等待操作: {hint}（超时 {timeout:.0f}s）', flush=True)
    last_report = time.time()
    while time.time() < deadline:
        if inj is not None:
            inj.spin_once()
        frame = mon.latest(proto.rx['id'])
        if frame is not None and proto.decode_501(frame[1]) == expect_mode:
            age = time.monotonic() - frame[0]
            how = ('等待期间新到帧' if frame[0] >= t_wait
                   else f'等待前已到达，帧龄 {age:.1f}s')
            print(f'[L4]   本机 0x501 解析通过: Byte1={expect_mode}（{how}）',
                  flush=True)
            return True
        if state is not None and inj is not None:
            cur_state, cur_mode = inj.latest_state_mode()
            if cur_state == state and (state_mode is None or cur_mode == state_mode):
                print(f'[L4]   状态证据通过: state={state} mode={cur_mode}'
                      f'（耗时 {time.monotonic() - t_wait:.1f}s）', flush=True)
                return True
        if time.time() - last_report >= 5.0:
            seen = ('未收到' if frame is None
                    else f'Byte1={proto.decode_501(frame[1])}')
            print(f'[L4]   等待中… {hint}（剩余 {deadline - time.time():.0f}s）'
                  f'｜本机 0x501: {seen}', flush=True)
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


def _enter_inspection(inj, vcu, mon, proto):
    """从 IDLE 进入 INSPECTION(2)：台架就绪 → AMI 选 mode 6（选模式即启动）."""
    _enable_bench(inj)
    assert inj.wait_for('/system/mission_state', 1, timeout=10.0), \
        '未进入 READY（需 mission_manager 运行）'
    if vcu is not None:
        vcu.set_mode(6)  # 仿真自动 AMI；真实接口人工在 AMI 上选车检
    assert _human_wait(mon, proto, 6, 30.0,
                       '用 AMI 选择车检模式（6）'
                       '（若 AMI 已停在 6 档，请先切到其它档再切回 6）',
                       inj=inj, state=2, state_mode=3), \
        '本机未在 0x501 上解析到 Byte1=6（车检），且状态未达 INSPECTION+INSPECTION'
    assert inj.wait_for('/system/mission_state', 2, timeout=10.0), \
        '选择车检后未直接进入 INSPECTION（若本机 0x501 已解析到 Byte1=6 却卡在这里：' \
        'can_interface 对同值模式帧去重、未转发 mission_mode_cmd，新起的 ' \
        'mission_manager 收不到模式；见 can_interface.cpp:151）'
    # 台架：VCU 还需人工按 RES Go 放行才会执行 0x210；给操作员留出按键时间窗
    pause_for_res_go('[L4]')


@pytest.mark.integration
def test_ami_inspection_entry(fsd_ready, mm, vcu, bus_monitor, protocol):
    """AMI 车检直入：0x501 Byte1=6 → INSPECTION(2)（选模式即启动）."""
    _enter_inspection(fsd_ready, vcu, bus_monitor, protocol)


@pytest.mark.integration
def test_inspection_motion(fsd_ready, mm, vcu, bus_monitor, protocol):
    """车检运动：纵向驱动开度 + 正弦 15°@0.4Hz 转向幅值与过零.

    车速反馈代发 0.0（车辆静止/架起），与目标 1.0 m/s 保持速度误差，
    确保纵向开度持续非零（反馈等于目标时 PID 误差为 0，开度会回零）。
    """
    _enter_inspection(fsd_ready, vcu, bus_monitor, protocol)
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
    _enter_inspection(fsd_ready, vcu, bus_monitor, protocol)
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
