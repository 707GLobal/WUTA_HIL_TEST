"""L4 车检任务全链路（pytest，标记 inspection + slow）.

职责：AMI 选择车检模式（mode 6）+ RES 发车放行 → INSPECTION 全链路：
正弦转向（15°@0.4Hz）+ 受限纵向驱动开度（车举升无速度反馈，PID 顶到
inspection_throttle_max 限幅，转速可控）+ inspection_duration（27s）后
完成链（回零 + mission_complete → FINISH + 0x210 Byte6 finished=1）。

单一用例 test_inspection_full 一次连续跑完：入口门控 → 运动波形 → 完成链三组
判据都在同一段演示里检查，中途不插人工操作、也不分段重启实例。

流程：READY(1) → AMI 选 mode 6 → 【本机解析 0x501 Byte1=6】→ **未按 GO 不得启动**
→ 【本机解析 0x1E4 Byte1=0x13 发车按钮】→ INSPECTION(2) → …27s… → FINISH(6)。
车检与普通任务同样需要 RES GO：发车按钮经 CAN 0x1E4 下发，can_interface 解析后
发布 /system/start_command，mission_manager 收到才从 READY 进 INSPECTION。

AMI 模式判据由测试自己用 BusMonitor 直接解析 0x501（Byte1=6），不再依赖
can_interface 转发的 /system/mission_mode_cmd（那是一次性边沿 + 去重，而本文件
用例自起 mission_manager 实例，需要重新投递模式，走话题会永久漏收）。

新协议：RES 急停不再经 CAN 0x501 下发（原 Byte1 状态定义取消），急停改由
0x1E4 Byte1=0x10 经 can_interface 发布到 /system/emergency（锁存），
车检中急停判据见 L1 post 的 test_1e4_estop_mapping。

台架模式（真实接口 + HIL_BENCH=1）：车辆架起通电，就绪信号由 hil_test
代发，controller 真实输出经 can_interface 上 CAN（0x210）闭环验证。
vcan0 预演（仿真接口）无需 HIL_BENCH 门控，vcu_sim 自动选模式。

mission_manager 由本文件自起自停（HIL 覆盖关自检）：FINISH(6) 为终态且模式仅
IDLE/READY 可改，同一实例只能进入车检一次。故全文件只保留一个用例、一个实例，
从进入一直跑到 FINISH——现场只需一轮人工操作（AMI 选 6 + 按 RES 发车），与真车
「一次选档、一次发车、跑完进 FINISH」的流程一致。
"""

import os
import time

import pytest

from hil_test.bench_ui import wait_for_res_go


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
    「变化」时发布一次的话题（去重 + volatile 不 latch），而本文件用例自起
    mission_manager 实例，需要重新投递一次模式；走话题会永久漏收并空等超时。
    详见 test_drive_hil._human_wait 的说明。
    """
    deadline = time.time() + timeout
    t_wait = time.monotonic()
    print(f'\n[L4] 请操作: {hint}（{timeout:.0f}s 超时）', flush=True)
    last_report = time.time()
    while time.time() < deadline:
        if inj is not None:
            inj.spin_once()
        frame = mon.latest(proto.rx['id'])
        if frame is not None and proto.decode_501(frame[1]) == expect_mode:
            age = time.monotonic() - frame[0]
            how = '新帧' if frame[0] >= t_wait else f'帧龄 {age:.1f}s'
            print(f'[L4]   OK: 0x501 Byte1={expect_mode}（{how}）', flush=True)
            return True
        if state is not None and inj is not None:
            cur_state, cur_mode = inj.latest_state_mode()
            if cur_state == state and (state_mode is None or cur_mode == state_mode):
                print(f'[L4]   OK: state={state} mode={cur_mode}'
                      f'（{time.monotonic() - t_wait:.1f}s）', flush=True)
                return True
        if time.time() - last_report >= 5.0:
            seen = ('未收到' if frame is None
                    else f'Byte1={proto.decode_501(frame[1])}')
            print(f'[L4]   …剩 {deadline - time.time():.0f}s（0x501: {seen}）',
                  flush=True)
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
    """L4：本文件唯一用例起停独立 mission_manager（HIL 覆盖关自检），从 IDLE 开始."""
    hil_params = os.environ.get('HIL_MM_HIL_PARAMS')
    if not hil_params:
        pytest.skip('缺少 HIL_MM_HIL_PARAMS（请用 hil_test.sh -l L4 启动）')
    mm_factory(hil_params)


def _enter_inspection(inj, vcu, mon, proto):
    """从 IDLE 进入 INSPECTION(2)：台架就绪 → AMI 选 mode 6 → 门控 → RES GO."""
    _enable_bench(inj)
    assert inj.wait_for('/system/mission_state', 1, timeout=10.0), \
        '未进入 READY（需 mission_manager 运行）'
    if vcu is not None:
        vcu.set_mode(6)  # 仿真自动 AMI；真实接口人工在 AMI 上选车检
    assert _human_wait(mon, proto, 6, 30.0,
                       'AMI 选车检模式（6）；若已停在 6 档，先切走再切回',
                       inj=inj, state=1, state_mode=3), \
        '本机未在 0x501 上解析到 Byte1=6（车检），且状态未达 READY+INSPECTION'
    # 门控：选模式本身不得启动（车检同样必须等 RES GO）
    assert not inj.wait_for('/system/mission_state', 2, timeout=2.0), \
        '未按 RES GO 就进了 INSPECTION：检查 mission_manager 门控（选模式不得启动）'
    # RES 发车：仿真自动按；真实台架提示人工按，判据为本机总线上的 0x1E4 发车帧
    if vcu is not None:
        vcu.press_start()
    assert wait_for_res_go(mon, proto, '[L4]'), \
        '未在本机总线上看到 RES 发车信号（0x1E4 Byte1=0x13）'
    assert inj.wait_for('/system/mission_state', 2, timeout=10.0), \
        '收到 RES GO 后未进 INSPECTION（检查 can_interface 的 /system/start_command ' \
        '与 mission_manager 对 mission_mode=INSPECTION 的分支）'


# 车检观测窗口（秒，相对进入 INSPECTION 的时刻）
_SAMPLE_SKIP = 0.5      # 丢弃进入瞬间的残留帧与速度爬升
_SAMPLE_SPAN = 10.0     # 波形采样窗：覆盖 ≥4 个正弦周期
_FINISH_TIMEOUT = 30.0  # 采样结束后等 FINISH(6) 的上限：演示 27s + 余量
_FINISH_MIN = 25.0      # 赛规 2.8.3：车检动作持续 25~30s
_FINISH_MAX = 31.0      # 上限留 1s 余量（进入时刻按本机 10Hz 状态广播判定）


@pytest.mark.integration
def test_inspection_full(fsd_ready, mm, vcu, bus_monitor, protocol):
    """车检全流程：入口门控 → 正弦转向 + 受限开度 → 27s 完成链，一次连续跑完.

    现场只需一轮「AMI 选车检模式（6）+ 按 RES 发车按钮」，之后本用例自动走完
    整个演示并检查三组判据（赛规 2.8.2 / 2.8.3 / 2.8.4），判据全部取自本机
    0x210 原始帧（帧级快照，不是抽样）：

      入口  选模式 ≠ 启动：未按 GO 时 2s 内不得进 INSPECTION(2)；收到 0x1E4
            Byte1=0x13（RES 发车）后才进 INSPECTION(2)
      动作  幅值 ≈ 19654 raw（15°/max_steer 25°×32757，±10%）
            周期 = 2.5s（1/0.4Hz，±10%，按上升沿过零间隔取中位）
            回中帧（横向恰为 32767）占比 < 2%：真过零时相邻帧也贴近中位，不会
            孤立出现；历史上「未使能」零指令插队会让 13% 的帧被拉回中位
            纵向开度恒为正且不顶满：车举升无速度反馈，PID 误差恒 +1、输出顶到
            controller.yaml 的 inspection_throttle_max 限幅，限幅值即下发值
      完成  25~30s 内进 FINISH(6)，且 0x210 Byte6 finished=1、纵向回零

    三组判据在跑完后一次性汇总上报（bad 列表），不逐条 assert：整轮要占约 30s
    台架时间 + 一轮人工操作，不该在第一条断言上中断、把后面的证据丢掉。

    车速反馈代发 0.0（车辆静止/架起），与 PID 目标保持误差，开度不会回零。
    """
    # ① 入口：台架就绪 → AMI 选 6 →（门控）→ RES GO → INSPECTION(2)
    _enter_inspection(fsd_ready, vcu, bus_monitor, protocol)

    # ② 前段采样：代发台架位姿/车速（controller 闭环需要连续流），供波形判据用
    t0 = time.monotonic()
    while time.monotonic() - t0 < _SAMPLE_SPAN:
        fsd_ready.publish_pose()
        fsd_ready.publish_velocity(0.0)
        time.sleep(0.05)

    # ③ 后段等待：演示到点后 controller 回零 + mission_complete → FINISH(6)。
    #    finished_at 即车检动作实际时长（相对进入 INSPECTION），用于比对赛规窗口。
    ok = fsd_ready.wait_for('/system/mission_state', 6, timeout=_FINISH_TIMEOUT)
    finished_at = (time.monotonic() - t0) if ok else None

    # ④ 完成链证据：FINISH 后 0x210 应置 finished=1 且纵向回零
    fin_frame = False
    if finished_at is not None:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            latest = bus_monitor.latest(protocol.tx['id'])
            if latest is not None:
                dec = protocol.decode_210(latest[1])
                if dec['finished'] == 1 and dec['longitudinal'] == 32767:
                    fin_frame = True
                    break
            time.sleep(0.05)

    # ⑤ 波形：只取采样窗内的帧（③④ 的收尾帧含回零，会污染统计）
    frames = [(t - t0, protocol.decode_210(data))
              for t, cid, data in bus_monitor.frames()
              if cid == protocol.tx['id']
              and t0 + _SAMPLE_SKIP <= t <= t0 + _SAMPLE_SPAN]
    ts = [t for t, _ in frames]
    lat = [d['lateral'] for _, d in frames]
    lon = [d['longitudinal'] for _, d in frames]

    bad = []
    if len(frames) <= 100:
        bad.append(f'采样窗内 0x210 帧过少（{len(frames)}）')
    else:
        nonzero = [v for v in lat if v != 32767]
        amp = max(abs(v - 32767) for v in nonzero) if nonzero else 0
        if not 0.9 * 19654 <= amp <= 1.1 * 19654:
            bad.append(f'转向幅值 {amp} raw 偏离 15°（期望 ≈19654±10%）：'
                       '检查 inspection_steer_amp')

        clean = [(t, v) for t, v in zip(ts, lat) if v != 32767]
        cross = [clean[i][0] for i in range(1, len(clean))
                 if clean[i - 1][1] < 32767 <= clean[i][1]]
        if len(cross) < 3:
            bad.append(f'过零次数不足（{len(cross)}），无法判定 0.4Hz 正弦')
        else:
            periods = sorted(b - a for a, b in zip(cross, cross[1:]))
            period = periods[len(periods) // 2]
            if not 2.25 <= period <= 2.75:
                bad.append(f'转向周期 {period:.2f}s 偏离 0.4Hz（期望 2.5s±10%）：'
                           '检查 inspection_steer_freq')

        ratio = lat.count(32767) / len(lat)
        if ratio >= 0.02:
            bad.append(f'{ratio * 100:.0f}% 的帧横向=32767（期望 <2%）：'
                       '『未使能』零指令与 runInspection() 抢 /control/command，'
                       '会把正弦拉回中位')

        if min(lon) < 32767:
            bad.append(f'车检出现制动方向开度（最小 {min(lon)}）')
        if max(lon) > 32767 + int(0.20 * 32758):
            bad.append(f'纵向开度 {max(lon)} 超过 0.20（顶满/未限幅）：'
                       '检查 inspection_throttle_max')

    if finished_at is None:
        bad.append(f'进入后 {_SAMPLE_SPAN:.0f}+{_FINISH_TIMEOUT:.0f}s 内未进 '
                   'FINISH(6)：核对 inspection_duration 与 mission_complete 通路')
    else:
        if not _FINISH_MIN <= finished_at <= _FINISH_MAX:
            bad.append(f'车检动作时长 {finished_at:.1f}s 不在赛规 25~30s 窗口内：'
                       '核对 inspection_duration')
        if not fin_frame:
            bad.append('完成后 0x210 未置 finished=1 或纵向未回零')

    assert not bad, (
        '车检全流程不符（共 %d 项）：\n  - %s' % (len(bad), '\n  - '.join(bad)))
