"""L3 低速动态安全闭环（pytest，标记 motor + slow）.

职责：AMI 选模式 + RES 发车放行 + 低速驱动（轮子动）——
READY(1) → 选 mode 2 → 【本机解析 0x501 Byte1=2】→ **未按 GO 不得启动**
→ 【本机解析 0x1E4 Byte1=0x13 发车按钮】→ EXPLORE(3)
→ controller 经 can_interface 上 CAN（0x210）输出驱动开度。

RES 放行：发车按钮经 CAN 0x1E4（Byte1=0x13）下发，can_interface 解析后发布
/system/start_command，mission_manager 收到才从 READY 进 EXPLORE。

AMI 模式判据由测试自己用 BusMonitor 直接解析 0x501，不再依赖 can_interface 转发
的 /system/mission_mode_cmd（那是一次性边沿 + 去重，测试侧每用例新建节点会漏收）。

目标车速由 hil_test.yaml bench.target_speed_mps 限制（默认 0.2 m/s 安全低速，
先低后调），经 /planning/final_waypoints 的直路下发。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，位姿/车速/路径/就绪信号
由 hil_test 代发（/localization/pose、/chcnav/velocity、/planning/final_waypoints
直路、/system/lidar_ready、/system/localization_ready、/system/devices_inspection）；
AMI 选模式与 RES 发车为真实信号，由人工操作（仿真接口下 vcu_sim 自动驱动）。
vcan0 预演（仿真接口）无需 HIL_BENCH 门控，vcu_sim 自动驱动。
"""

import os
import time

import pytest

from hil_test.bench_ui import wait_for_res_go
from hil_test.protocol_loader import RES_START


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


def _bench_target_speed(default=0.2):
    """读取 hil_test.yaml 的 bench.target_speed_mps（与 RosInjector 代发直路同源）."""
    try:
        import yaml
    except ImportError:
        return default
    cfg_dir = os.environ.get(
        'HIL_CONFIG',
        os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'config'))
    try:
        with open(os.path.join(cfg_dir, 'hil_test.yaml'), 'r', encoding='utf-8') as f:
            return float(yaml.safe_load(f).get(
                'bench', {}).get('target_speed_mps', default))
    except (OSError, ValueError, TypeError):
        return default


def _go_seen_since(mon, proto, t0):
    """自 t0（monotonic）起本机总线上是否出现过 0x1E4 Byte1=0x13（RES 发车）.

    供门控断言用：若选档之后总线上确实来过 GO，那么进 EXPLORE 就是 GO 放行的合法
    行为（操作员可能没等门控窗走完就按了按钮），不能判成「未按 GO 就启动」。
    """
    return any(
        cid == proto.res['id'] and t >= t0 and proto.decode_1e4(data) == RES_START
        for t, cid, data in mon.frames())


pytestmark = [
    pytest.mark.motor,
    pytest.mark.slow,
    pytest.mark.skipif(
        not _ON_SIM and os.environ.get('HIL_BENCH') != '1',
        reason='L3 真实台架需车辆通电：设置 HIL_BENCH=1 后执行（vcan0 预演无需）'),
]


def _human_wait(mon, proto, expect_mode, timeout, hint, inj=None,
                state=None, state_mode=None):
    """等待人工操作结果：**本机直接解析 0x501**，Byte1 达标即返回.

    判据优先级：
      1) 本机总线监听解析到的 0x501 Byte1 == expect_mode（主判据，见下）；
      2) 电平证据 /system/mission_state == (state, state_mode)（兜底，需传 inj）。

    为什么改用本机解析：原先等 can_interface 转发的话题 /system/mission_mode_cmd，
    那是「边沿」信号——can_interface 仅在 0x501 模式字节「变化」时发布一次
    （can_interface.cpp:151 去重 + :169 一次性发布，DDS volatile 不 latch）。
    测试侧每个用例都新建节点，订阅建立晚于发布即永久漏收，且 AMI 停在原档位时
    VCU 重发同值帧会被去重、压根不发，于是只能空等到超时（车辆侧其实早已接受
    该模式）。改用 BusMonitor 直接读总线：VCU 只要在发帧，本机就能看到，
    不经过 can_interface 的去重与一次性发布两跳。
    """
    deadline = time.time() + timeout
    t_wait = time.monotonic()
    print(f'\n[L3] 请操作: {hint}（{timeout:.0f}s 超时）', flush=True)
    last_report = time.time()
    while time.time() < deadline:
        if inj is not None:
            inj.spin_once()
        frame = mon.latest(proto.rx['id'])
        if frame is not None and proto.decode_501(frame[1]) == expect_mode:
            age = time.monotonic() - frame[0]
            how = '新帧' if frame[0] >= t_wait else f'帧龄 {age:.1f}s'
            print(f'[L3]   OK: 0x501 Byte1={expect_mode}（{how}）', flush=True)
            return True
        if state is not None and inj is not None:
            cur_state, cur_mode = inj.latest_state_mode()
            if cur_state == state and (state_mode is None or cur_mode == state_mode):
                print(f'[L3]   OK: state={state} mode={cur_mode}'
                      f'（{time.monotonic() - t_wait:.1f}s）', flush=True)
                return True
        if time.time() - last_report >= 5.0:
            seen = ('未收到' if frame is None
                    else f'Byte1={proto.decode_501(frame[1])}')
            print(f'[L3]   …剩 {deadline - time.time():.0f}s（0x501: {seen}）',
                  flush=True)
            last_report = time.time()
        time.sleep(0.2)
    return False


def _enable_bench(inj, timeout=15.0):
    """台架就绪：**反复**发上线 + 门控就绪信号 + 直路，直到 READY(1)；返回是否到达.

    不能只发一次：`/system/lidar_ready`、`/system/localization_ready` 都是 volatile
    （非 latched），若注入赶在 mission_manager 订阅匹配之前会被**静默丢弃**，状态机
    永远停在 IDLE（L4 台架实测出现过早 0.16s 的窗口，连续 4 次失败）。重复发同一电平值
    对「订阅早已就绪」的正常情况无副作用。
    """
    deadline = time.time() + timeout
    while True:
        inj.publish_devices_inspection(ok=True)  # Signal3 上线
        inj.publish_lidar_ready()
        inj.publish_localization_ready()
        inj.publish_pose()
        inj.publish_waypoints_straight()
        inj.spin_once()
        if inj.latest('/system/mission_state') == 1:
            return True
        if time.time() >= deadline:
            return False
        time.sleep(0.2)


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


def _enter_explore(fsd_ready, vcu, mon, proto):
    """确保系统进入 EXPLORE(3)：已在则直接返回；否则按 AMI 直线加速 + RES GO 走一遍.

    流程：READY(1) → AMI 选 2（0x501 Byte1=2）→ **门控：未 GO 不得启动**
    → RES 发车（0x1E4 Byte1=0x13）→ EXPLORE(3)。

    mission_manager 跨用例长驻（单实例），状态可能已推进到 EXPLORE(3) 且无回退
    路径；RosInjector 则每用例新建、缓存为空，故先 spin 探测当前状态再决定短路。
    """
    # 已在 EXPLORE(3) 直接返回（状态机单向：EXPLORE 无回到 READY 的路径）
    if fsd_ready.wait_for('/system/mission_state', 3, timeout=1.0):
        # 跳过 _enable_bench 时单独补一次开闸：can_interface 的 0x210 发送门控
        # 要收到首份自检结论才开（仅 can_interface 单独重启过的场景才会缺闸）
        fsd_ready.publish_devices_inspection(ok=True)
        return
    assert _enable_bench(fsd_ready), \
        '未进入 READY（需 mission_manager 运行；就绪信号已按 0.2s 重发 15s）'
    t_mode = time.monotonic()  # 选档等待起点：门控判定只看这之后的 GO 帧
    if vcu is not None:
        vcu.set_mode(2)  # 仿真自动 AMI；真实接口人工在 AMI 上选直线加速
    assert _human_wait(mon, proto, 2, 30.0,
                       'AMI 选直线加速（2）；若已停在 2 档，先切走再切回',
                       inj=fsd_ready, state=1, state_mode=2), \
        '本机未在 0x501 上解析到 Byte1=2（直线加速），且状态未达 READY+ACCELERATION'
    # 门控：选模式本身不得启动（RES GO 是必经放行）。窗口内若已进 EXPLORE，用总线
    # 上的 GO 帧定性：有 GO → 操作员提前按（合法，直接继续，不再等一次新的 GO）；
    # 无 GO → 说明门控失效，判失败。避免把「手快先按了 GO」误报成门控 bug。
    started = fsd_ready.wait_for('/system/mission_state', 3, timeout=2.0)
    if started:
        assert _go_seen_since(mon, proto, t_mode), \
            '未按 RES GO 就进了 EXPLORE：检查 mission_manager 门控（选模式不得启动）'
    else:
        # RES 发车：仿真自动按；真实台架提示人工按，判据为本机总线上的 0x1E4 发车帧
        if vcu is not None:
            vcu.press_start()
        assert wait_for_res_go(mon, proto, '[L3]'), \
            '未在本机总线上看到 RES 发车信号（0x1E4 Byte1=0x13）'
    assert fsd_ready.wait_for('/system/mission_state', 3, timeout=10.0), \
        '收到 RES GO 后未进 EXPLORE（检查 can_interface 的 /system/start_command ' \
        '与 mission_manager 的 start_requested_ 门控）'


@pytest.mark.integration
def test_ami_acceleration_go(fsd_ready, vcu, bus_monitor, protocol):
    """AMI 直线加速 + RES GO：READY → 0x501 Byte1=2 →（门控）→ 0x1E4 0x13 → EXPLORE."""
    _enter_explore(fsd_ready, vcu, bus_monitor, protocol)


@pytest.mark.integration
def test_low_speed_follow(fsd_ready, vcu, bus_monitor, protocol):
    """低速驱动：EXPLORE 下代发直路（yaml 限速）、车速反馈静止 → 0x210 给出驱动开度.

    判据（全部取自本机 0x210 原始帧，不需要轮速反馈）：
      方向  纵向必须 > 32767（32767 = 中位零输出、<32767 = 制动）——只判
            「!= 32767」时，制动甚至全制动都会被当成“有驱动开度”；
      量级  稳态开度 ≈ PID 期望：err = target - 0 = target，kp=1 → 定标
            ≈ 32767 + target*32768（target = bench.target_speed_mps），容差 0.5~2×。

    车辆架起、目标车速 = bench.target_speed_mps（安全限速），故轮子会低速转动。
    """
    _enter_explore(fsd_ready, vcu, bus_monitor, protocol)
    fsd_ready.publish_waypoints_straight()  # 目标速度 = bench.target_speed_mps
    target = _bench_target_speed()
    _drive_bench(fsd_ready, 0.0, 2.0)  # 车辆静止反馈 → 速度误差驱动输出
    exp = 32767 + int(target * 32768)
    lo = 32767 + int(0.5 * target * 32768)
    hi = 32767 + int(2.0 * target * 32768)
    # 只取驱动回路跑开之后的稳态帧（_drive_bench 期间的起控瞬态不在窗口内）
    peak = 32767
    lowest = 32767
    deadline = time.time() + 3.0
    while time.time() < deadline:
        value = _bus_longitudinal(bus_monitor, protocol)
        peak = max(peak, value)
        lowest = min(lowest, value)
        if peak > 32767:
            break
        time.sleep(0.05)
    assert peak > 32767, (
        f'低速跟随期间 0x210 纵向未出现驱动开度（窗口内范围 {lowest}~{peak}，'
        '均为中位/制动）：controller 未给出驱动（注意：制动方向同样不算驱动）')
    assert lo <= peak <= hi, (
        f'0x210 纵向开度 {peak} 偏离驱动期望区间 {lo}~{hi}'
        f'（bench.target_speed_mps={target}，kp=1 → 定标 ≈ {exp}）')
