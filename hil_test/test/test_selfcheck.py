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
def test_never_online_timeout(mm_factory, fsd_ready, bus_monitor, protocol):
    """从未上线：超过宽限期 → ok=false + failures，立即切 EMERGENCY，Signal3=0."""
    mm_factory()
    # 不发布任何传感器数据：宽限 4s + 检查周期 1s + 余量
    expected = (False, ('lidar', 'imu', 'camera'))
    assert fsd_ready.wait_for('/system/devices_inspection', expected, timeout=10.0), \
        '超时未上报自检失败（failures 应为 lidar/imu/camera）'
    assert fsd_ready.wait_for('/system/mission_state', STATE_EMERGENCY, timeout=3.0), \
        '自检失败后未立即切 EMERGENCY'
    # VCU 通知链路：can_interface 将 Signal3（Byte5）置 0
    assert bus_monitor.wait_for(protocol.tx['id'], timeout=3.0)
    dec = protocol.decode_210(bus_monitor.latest(protocol.tx['id'])[1])
    assert dec['online'] == 0, '自检失败后 0x210 Signal3 未置 0（VCU 不会切 EMERGENCY）'


@pytest.mark.integration
def test_mid_stream_dropout(mm_factory, fsd_ready):
    """中途断流：先上线再停发 → 超过 sensor_timeout 2s 判故障并切 EMERGENCY."""
    mm_factory()
    _feed_sensors(fsd_ready, 2.5)  # 全部上线（宽限期内）
    expected = (False, ('lidar', 'imu', 'camera'))
    assert fsd_ready.wait_for('/system/devices_inspection', expected, timeout=8.0), \
        '断流后未上报自检失败'
    assert fsd_ready.wait_for('/system/mission_state', STATE_EMERGENCY, timeout=3.0), \
        '断流后未立即切 EMERGENCY'


@pytest.mark.integration
def test_fault_latched(mm_factory, fsd_ready):
    """故障锁存：EMERGENCY 后恢复数据流，状态与上报保持失败（不可恢复）."""
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
