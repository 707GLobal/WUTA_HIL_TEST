"""L2 传感器自检故障模拟（pytest，断电层）.

验证 mission_manager 传感器持续自检机制（默认参数，check_* 全开）：
  - 三传感器（lidar/imu/camera）数据心跳正常 → devices_inspection ok=true，不误报；
  - 从未上线：超过宽限期（4s）→ ok=false + failures + 立即切 EMERGENCY；
  - 中途断流：超过 sensor_timeout（2s）→ ok=false + 立即切 EMERGENCY；
  - 故障锁存：EMERGENCY 后恢复数据流不得解除（sensor_fault_ 不可恢复）；
  - HIL 覆盖回归：check_* 全关（hil_fsd/mission_manager.yaml）时自检停用，
    不得误切 EMERGENCY（保证 L3/L4 台架配置可用）。

节点：can_interface 由 hil_test.sh 启动；mission_manager 由 conftest 的
mm_factory fixture 按用例独立起停（故障锁存与参数覆盖需要干净实例）。
通电顺序：L2 通过后才允许给 FSD 通电进入 L3/L4。
"""

import os
import time

import pytest

pytestmark = [pytest.mark.selfcheck, pytest.mark.integration]

# HIL 覆盖参数文件（由 hil_test.sh 导出；缺省时用例跳过并给出指引）
MM_HIL_PARAMS = os.environ.get('HIL_MM_HIL_PARAMS')

STATE_EMERGENCY = 7


def _feed_sensors(inj, duration, hz=10.0):
    """以 hz 频率发布三传感器数据心跳（内容无关，仅刷新在线时间戳）."""
    period = 1.0 / hz
    deadline = time.time() + duration
    while time.time() < deadline:
        inj.publish_lidar_data()
        inj.publish_imu_data()
        inj.publish_camera_data()
        inj.spin_once()
        time.sleep(period)


def _hold(inj, duration):
    """静置观察（持续 spin 接收话题，不发布任何传感器数据）."""
    deadline = time.time() + duration
    while time.time() < deadline:
        inj.spin_once()
        time.sleep(0.1)


def _wait_until(predicate, timeout):
    """轮询等待条件成立（0x210 为 10Hz 保活，须等新值帧而非「出现任意帧」）."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _require_sim_fault(is_sim, interface):
    """断流/锁存故障用例仅仿真接口运行.

    真实链路下这两条会给真实 VCU 下发 Signal3=0，把 VCU 打入 EMERGENCY 且
    VCU 侧锁存、无法回退，影响后续测试；该安全链在 vcan0 已完整覆盖，
    真实链路只保留 test_never_online_timeout 一条端到端联动。
    """
    if not is_sim:
        pytest.skip(
            f'{interface} 为真实接口：断流/锁存故障注入仅 vcan0 运行'
            f'（避免真实 VCU 被锁死在 EMERGENCY）')


@pytest.mark.integration
def test_selfcheck_all_pass(mm_factory, fsd_ready):
    """自检通过：三传感器持续在线 → ok=true，状态保持 IDLE 不误报."""
    mm_factory()
    _feed_sensors(fsd_ready, 6.0)  # 覆盖宽限期 4s
    insp = fsd_ready.latest('/system/devices_inspection')
    assert insp is not None, '全部在线期间未上报 devices_inspection'
    assert insp == (True, ()), f'自检结果异常: {insp}'
    assert fsd_ready.latest('/system/mission_state') == 0, '全部在线却被切离 IDLE'


@pytest.mark.integration
def test_never_online_timeout(mm_factory, fsd_ready, bus_monitor, protocol, is_sim, interface):
    """从未上线：超过宽限期 → ok=false + failures，立即切 EMERGENCY，Signal3=0.

    本用例是 L2 在真实链路上唯一运行的故障用例（断流/锁存两条仅 vcan0）：
    FSD 侧（devices_inspection/mission_state）与 0x210 Signal3=0 之后，
    真实链路再断言真实 VCU 的联动响应（0x501 Byte1=12）。
    注意：真实 VCU 因此进入 EMERGENCY 并锁存，跑完须复位（见 README「真车运行与复位」）。
    """
    mm_factory()
    # 不发布任何传感器数据：宽限 4s + 检查周期 1s + 余量
    expected = (False, ('lidar', 'imu', 'camera'))
    assert fsd_ready.wait_for('/system/devices_inspection', expected, timeout=10.0), \
        '超时未上报自检失败（failures 应为 lidar/imu/camera）'
    assert fsd_ready.wait_for('/system/mission_state', STATE_EMERGENCY, timeout=3.0), \
        '自检失败后未立即切 EMERGENCY'
    # VCU 通知链路：can_interface 将 Signal3（Byte5）置 0
    # 注意：mission_manager 先切 EMERGENCY 再发自检结果，故 EMERGENCY 后仍可能
    # 先出现携带 Signal3=1 的帧，需轮询等待置 0 的那一帧，而非取当前最新帧。
    assert bus_monitor.wait_for(protocol.tx['id'], timeout=3.0)

    def _signal3_off():
        latest = bus_monitor.latest(protocol.tx['id'])
        return (latest is not None
                and protocol.decode_210(latest[1])['online'] == 0)

    assert _wait_until(_signal3_off, 1.0), \
        '自检失败后 0x210 Signal3 未置 0（VCU 不会切 EMERGENCY）'

    # 真实链路：验证真实 VCU 收到 Signal3=0 后自身进 EMERGENCY（0x501 Byte1=12）。
    # vcu_sim 不建模「Signal3=0 → EMERGENCY」该联动，故仿真接口下不做此断言。
    if not is_sim:
        assert bus_monitor.wait_for(protocol.rx['id'], timeout=10.0), \
            '未收到 VCU 0x501 状态帧（真实链路需 VCU 上电，见 README「接入真实 VCU」）'

        def _vcu_emergency():
            latest = bus_monitor.latest(protocol.rx['id'])
            return (latest is not None
                    and latest[1][protocol.rx['state_byte']] == 12)

        assert _wait_until(_vcu_emergency, 10.0), \
            '真实 VCU 未进入 EMERGENCY（0x501 Byte1 未变为 12）'


@pytest.mark.integration
def test_mid_stream_dropout(mm_factory, fsd_ready, is_sim, interface):
    """中途断流：先上线再停发 → 超过 sensor_timeout 2s 判故障并切 EMERGENCY.

    仅 vcan0：真实链路制造断流需停真实传感器且会给 VCU 下发 Signal3=0（见 _require_sim_fault）。
    """
    _require_sim_fault(is_sim, interface)
    mm_factory()
    _feed_sensors(fsd_ready, 2.5)  # 全部上线（宽限期内）
    expected = (False, ('lidar', 'imu', 'camera'))
    assert fsd_ready.wait_for('/system/devices_inspection', expected, timeout=8.0), \
        '断流后未上报自检失败'
    assert fsd_ready.wait_for('/system/mission_state', STATE_EMERGENCY, timeout=3.0), \
        '断流后未立即切 EMERGENCY'


@pytest.mark.integration
def test_fault_latched(mm_factory, fsd_ready, is_sim, interface):
    """故障锁存：EMERGENCY 后恢复数据流，状态与上报保持失败（不可恢复）.

    仅 vcan0：真实链路会把真实 VCU 锁死在 EMERGENCY 且无法回退（见 _require_sim_fault）。
    """
    _require_sim_fault(is_sim, interface)
    mm_factory()
    expected = (False, ('lidar', 'imu', 'camera'))
    assert fsd_ready.wait_for('/system/devices_inspection', expected, timeout=10.0), \
        '未进入自检失败（前置条件不满足）'
    assert fsd_ready.wait_for('/system/mission_state', STATE_EMERGENCY, timeout=3.0)
    # 恢复传感器数据流：锁存的故障不得解除
    _feed_sensors(fsd_ready, 6.0)
    assert fsd_ready.latest('/system/mission_state') == STATE_EMERGENCY, \
        '故障锁存被解除（sensor_fault_ 应不可恢复）'
    insp = fsd_ready.latest('/system/devices_inspection')
    assert insp == expected, f'锁存期间自检上报被改写: {insp}'


@pytest.mark.integration
def test_hil_override_selfcheck_disabled(mm_factory, fsd_ready):
    """HIL 覆盖回归：check_* 全关时自检停用，无传感器数据也不得误切 EMERGENCY."""
    if not MM_HIL_PARAMS:
        pytest.skip('缺少 HIL_MM_HIL_PARAMS（请用 hil_test.sh -l L2 启动）')
    mm_factory(MM_HIL_PARAMS)
    _hold(fsd_ready, 6.0)  # 静置超过宽限期，不发任何传感器数据
    state = fsd_ready.latest('/system/mission_state')
    assert state in (0, None), f'自检已关闭仍切离 IDLE: state={state}'
    assert fsd_ready.latest('/system/devices_inspection') is None, \
        '自检关闭时不应上报 devices_inspection（台架由测试代发）'
