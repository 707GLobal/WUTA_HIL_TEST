"""L4 车检任务全链路（pytest，标记 inspection + slow）.

职责：AMI 选择车检模式（mode 6）+ RES 发车放行 → INSPECTION 全链路：
正弦转向（方向盘 ±30°＝前轮 5.77°@9.0s 周期）+ 恒定纵向驱动开度（inspection_throttle，**不走 PID**：
车举升无速度反馈、速度环不可观测）+ inspection_duration（27.0s）后
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


def _ctrl_params():
    """读取 FSD controller.yaml 的 controller_node.ros__parameters.

    路径由脚本导出为 HIL_CTRL_PARAMS。车检的期望值（转向幅值/周期/纵向开度上限）
    直接取 controller 实际加载的参数，而不是在测试里再抄一份常量：现场调
    inspection_* 时不需要改测试（缺环境变量/文件时调用方各自退回默认值）。
    """
    path = os.environ.get('HIL_CTRL_PARAMS')
    if not path:
        return {}
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
    except (OSError, ValueError, TypeError):
        return {}
    return (cfg.get('controller_node', {}) or {}).get('ros__parameters', {}) or {}


def _go_seen_since(mon, proto, t0):
    """自 t0（monotonic）起本机总线上是否出现过 0x1E4 Byte1=0x13（RES 发车）.

    供门控断言用：若选档之后总线上确实来过 GO，那么进 INSPECTION 就是 GO 放行的
    合法行为（操作员可能没等门控窗走完就按了按钮），不能判成「未按 GO 就启动」。
    """
    return any(
        cid == proto.res['id'] and t >= t0 and proto.decode_1e4(data) == RES_START
        for t, cid, data in mon.frames())


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
    t_mode = time.monotonic()  # 选档等待起点：门控判定只看这之后的 GO 帧
    if vcu is not None:
        vcu.set_mode(6)  # 仿真自动 AMI；真实接口人工在 AMI 上选车检
    assert _human_wait(mon, proto, 6, 30.0,
                       'AMI 选车检模式（6）；若已停在 6 档，先切走再切回',
                       inj=inj, state=1, state_mode=3), \
        '本机未在 0x501 上解析到 Byte1=6（车检），且状态未达 READY+INSPECTION'
    # 门控：选模式本身不得启动（车检同样必须等 RES GO）。窗口内若已进 INSPECTION，
    # 用总线上的 GO 帧定性：有 GO → 操作员提前按（合法，直接继续，不再等新的 GO）；
    # 无 GO → 门控失效，判失败。避免把「手快先按了 GO」误报成门控 bug。
    started = inj.wait_for('/system/mission_state', 2, timeout=2.0)
    if started:
        assert _go_seen_since(mon, proto, t_mode), \
            '未按 RES GO 就进了 INSPECTION：检查 mission_manager 门控（选模式不得启动）'
    else:
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
_SAMPLE_SPAN = 23.5     # 波形采样窗：周期 9.0s 下需覆盖 ≥3 个上升过零（4.5/13.5/22.5s）
_FINISH_TIMEOUT = 30.0  # 采样结束后等 FINISH(6) 的上限：演示 27s + 余量
_FINISH_MIN = 25.0      # 赛规 2.8.3：车检动作持续 25~30s
_FINISH_MAX = 31.0      # 上限留 1s 余量（进入时刻按本机 10Hz 状态广播判定）
_MAX_LAT_STEP_FRAC = 0.25  # 收尾单帧跳变上限 = 正弦幅值的 25%（幅值越小，绝对上限也要越小）


@pytest.mark.integration
def test_inspection_full(fsd_ready, mm, vcu, bus_monitor, protocol):
    """车检全流程：入口门控 → 正弦转向 + 恒定开度 → 27.0s 完成链，一次连续跑完.

    现场只需一轮「AMI 选车检模式（6）+ 按 RES 发车按钮」，之后本用例自动走完
    整个演示并检查三组判据（赛规 2.8.2 / 2.8.3 / 2.8.4），判据全部取自本机
    0x210 原始帧（帧级快照，不是抽样）：

      入口  选模式 ≠ 启动：未按 GO 时 2s 内不得进 INSPECTION(2)；收到 0x1E4
            Byte1=0x13（RES 发车）后才进 INSPECTION(2)。窗口内若已出现 GO 帧，
            说明是操作员提前按下（合法），不判成门控失效
      动作  幅值 ≈ inspection_steer_amp/max_steer_deg×跨度（左半 32762 / 右半 32773，
            5.7692°/25° → 7563，±10%；即方向盘 ±30°）
            周期 = inspection_steer_period（9.0s，±10%，按上升沿过零间隔取中位）
            回中帧（横向恰为 32762，Signal2 新规格中心）占比 < 2%：真过零时相邻帧
            也贴近中位，不会孤立出现；历史上「未使能」零指令插队会让 13% 的帧被拉回中位
            纵向必须**持续在驱动**（>32767）且恒为 inspection_throttle 的定标值
            （0.16 → 38008，±2 raw）：车检不走 PID，直接发常量。只判「不制动/
            不超上限」时，全程 32767（一点驱动都没有）会从两条边界中间漏过去
      完成  25~30s 内进 FINISH(6)，且 0x210 Byte6 finished=1、纵向/横向回零；
            回中不得有跳变：横向单帧变化 ≤ 幅值的 25%（当前 5.77° → 1.44°）

    期望值（转向幅值/周期/纵向恒定开度）从 controller.yaml 推导（HIL_CTRL_PARAMS），
    现场调 inspection_* 不需要同步改测试。

    三组判据在跑完后一次性汇总上报（bad 列表），不逐条 assert：整轮要占约 30s
    台架时间 + 一轮人工操作，不该在第一条断言上中断、把后面的证据丢掉。

    车速反馈代发 0.0（车辆静止/架起）；车检开度不依赖它（常量直接下发）。
    """
    # ① 入口：台架就绪 → AMI 选 6 →（门控）→ RES GO → INSPECTION(2)
    _enter_inspection(fsd_ready, vcu, bus_monitor, protocol)

    # 车检期望值：与 controller 实际加载的参数同源（缺 HIL_CTRL_PARAMS 时退回默认）
    ctrl = _ctrl_params()
    steer_amp = float(ctrl.get('inspection_steer_amp', 5.7692))
    throttle_const = float(ctrl.get('inspection_throttle', 0.16))
    # 横向定标（Signal2 新规格：0~65535、32762 为中心 = 回正；见 C++ scaleLateral）
    lat_cfg = protocol.tx['signals']['lateral']
    lat_c = int(lat_cfg['center'])              # 32762
    lat_span_l = lat_c - int(lat_cfg['min'])    # 32762（左半段）
    lat_span_r = int(lat_cfg['max']) - lat_c    # 32773（右半段）
    amp_exp = steer_amp / protocol.max_steer_deg * max(lat_span_l, lat_span_r)
    # 转向周期：优先 inspection_steer_period；旧配置只有 freq 时用其倒数
    period_exp = float(ctrl.get('inspection_steer_period', 0.0) or 0.0)
    if period_exp <= 0.0:
        steer_freq = float(ctrl.get('inspection_steer_freq', 0.0) or 0.0)
        period_exp = (1.0 / steer_freq) if steer_freq > 0 else 6.0
    # 纵向定标：恒定开度直接下发 → 总线值应精确等于 throttle_const 的定标值
    lon_exp = 32767 + int(throttle_const * 32758)
    lon_lo = lon_exp - 2   # 仅留 C++ 截断 vs Python int 的取整余量
    lon_hi = lon_exp + 2

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

    # ④ 完成链证据：FINISH 后 0x210 应置 finished=1 且纵向/横向都回零
    fin_frame = False
    if finished_at is not None:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            latest = bus_monitor.latest(protocol.tx['id'])
            if latest is not None:
                dec = protocol.decode_210(latest[1])
                if (dec['finished'] == 1 and dec['longitudinal'] == 32767
                        and dec['lateral'] == lat_c):
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
        nonzero = [v for v in lat if v != lat_c]
        amp = max(abs(v - lat_c) for v in nonzero) if nonzero else 0
        if not 0.9 * amp_exp <= amp <= 1.1 * amp_exp:
            bad.append(f'转向幅值 {amp} raw 偏离 {steer_amp:.1f}°'
                       f'（期望 ≈{amp_exp:.0f}±10%）：检查 inspection_steer_amp')

        clean = [(t, v) for t, v in zip(ts, lat) if v != lat_c]
        cross = [clean[i][0] for i in range(1, len(clean))
                 if clean[i - 1][1] < lat_c <= clean[i][1]]
        if len(cross) < 3:
            bad.append(f'过零次数不足（{len(cross)}），无法判定 {period_exp:.1f}s 周期的正弦'
                       f'（或需加大 _SAMPLE_SPAN）')
        else:
            periods = sorted(b - a for a, b in zip(cross, cross[1:]))
            period = periods[len(periods) // 2]
            if not 0.9 * period_exp <= period <= 1.1 * period_exp:
                bad.append(f'转向周期 {period:.2f}s 偏离（期望 {period_exp:.2f}s±10%）：'
                           f'检查 inspection_steer_period')

        ratio = lat.count(lat_c) / len(lat)
        if ratio >= 0.02:
            bad.append(f'{ratio * 100:.0f}% 的帧横向={lat_c}（期望 <2%）：'
                       '『未使能』零指令与 runInspection() 抢 /control/command，'
                       '会把正弦拉回中位')

        # 纵向必须是“持续驱动”。原来只有「不制动(min>=32767)」和「不超上限」两条
        # 边界判据：全程 32767（零输出，一条驱动指令都没发）两条都不触发，故补正向。
        driving_ratio = sum(1 for v in lon if v > 32767) / len(lon)
        if driving_ratio < 0.9:
            bad.append(f'纵向驱动帧占比 {driving_ratio * 100:.0f}%（期望 ≥90%）：'
                       f'0x210 Signal1 未持续给出驱动开度（期望 ≈{lon_exp}）：'
                       '检查 inspection_throttle 常量下发通路（车检不走 PID）')
        med_lon = sorted(lon)[len(lon) // 2]
        if not lon_lo <= med_lon <= lon_hi:
            bad.append(f'纵向开度中位 {med_lon} 偏离 inspection_throttle='
                       f'{throttle_const:.2f} 的定标值 {lon_exp}'
                       f'（允许 {lon_lo}~{lon_hi}）：检查常量下发通路')
        if min(lon) < 32767:
            bad.append(f'车检出现制动方向开度（最小 {min(lon)}）')
        if max(lon) > lon_hi:
            bad.append(f'纵向开度 {max(lon)} 超过 {throttle_const:.2f} 的定标值 '
                       f'{lon_exp}（恒定开度不应有更高帧）：检查 inspection_throttle')

    if finished_at is None:
        bad.append(f'进入后 {_SAMPLE_SPAN:.0f}+{_FINISH_TIMEOUT:.0f}s 内未进 '
                   'FINISH(6)：核对 inspection_duration 与 mission_complete 通路')
    else:
        if not _FINISH_MIN <= finished_at <= _FINISH_MAX:
            bad.append(f'车检动作时长 {finished_at:.1f}s 不在赛规 25~30s 窗口内：'
                       '核对 inspection_duration')
        if not fin_frame:
            bad.append('完成后 0x210 未置 finished=1 或纵向/横向未回零')

        # ⑥ 收尾回中不得有跳变：duration 与转向周期成 0.5 的整数倍时末帧本就近中位；若演示
        #    停在幅值附近，finishInspection() 直接发 angle=0 会形成单帧大跳变（实测
        #    27.0s/0.4Hz → -14.3°@20ms ≈ 715°/s，是 180°/s 限幅的 4 倍）。这里对全段
        #    0x210 相邻帧的横向变化量设上限；正弦本身仅 0.12°/帧，真跳变 ≥14° 必触发。
        lat_all = [protocol.decode_210(data)['lateral']
                   for _, cid, data in bus_monitor.frames()
                   if cid == protocol.tx['id']]
        # raw → deg：用较小的跨度（左半 32762）作保守换算，角度估计只会偏大
        step_deg_per_raw = protocol.max_steer_deg / float(min(lat_span_l, lat_span_r))
        max_step = max((abs(b - a) * step_deg_per_raw
                        for a, b in zip(lat_all, lat_all[1:])), default=0.0)
        max_step_lim = max(1.0, _MAX_LAT_STEP_FRAC * steer_amp)
        if max_step > max_step_lim:
            bad.append(f'横向单帧最大跳变 {max_step:.2f}°（期望 ≤{max_step_lim:.2f}°'
                       f' = 幅值 {steer_amp:.2f}° 的 {_MAX_LAT_STEP_FRAC * 100:.0f}%）：'
                       f'收尾回中过冲——让 inspection_duration 与转向周期 '
                       f'{period_exp:.1f}s 成 0.5 的整数倍关系')

    assert not bad, (
        '车检全流程不符（共 %d 项）：\n  - %s' % (len(bad), '\n  - '.join(bad)))
