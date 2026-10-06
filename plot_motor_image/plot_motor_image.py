#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_motor_image.py —— L4CAN「0x210 指令 → 电机层执行核查」总图

输入 : zlgcan_bridge 按 ID 落盘的日志目录（内含 fsd2vcu/ 与 vcu2fsd/）
输出 : 0x210_motor_level_execution_<目录戳>.png —— 5 面板、**统一时间轴**、t=0 = 0x210 首帧

用法 :
    python3 plot_motor_image.py <log目录> [选项]

常用选项 :
    -o, --out PATH          输出 png（默认：zlgcan_bridge 的上一级）
    --report-only           只打印体检报告，不出图
    --xlim A B              只画 [A, B] 秒（相对 0x210 首帧），用于把某个车检窗口放大
    --eps-cmd-byte N        0x469 速度指令所在字节（**0 基**，默认 6 = 第 7 字节 = 0xC8 = 200）
    --eps-torque-byte N     0x401 扭矩反馈所在字节（**0 基**，默认 1 = 第 2 字节）
    --eps-torque-encoding E 0x401 该字节的坐标系：offset（默认，0x80 = 0 Nm，见下）
                            / signed（二进制补码，DBC 文字描述；实测会产生 ±12.8Nm 假跳变）
    --210-byte-order ORD    0x210 Signal1/Signal2 字节序：big（默认，2026-10-06 起的新固件）
                            / little（更早的旧日志）
    --max-steer-deg D       横向满量程前轮转角（默认 25.0，与 can_interface 一致）
    --steer-ratio R         方向盘/前轮 转向比（默认 5.2，仅用于刻度文字）
    --drv-torque-regid 0x90 0x201 中扭矩给定的 REGID
    --drv-speed-regid 0x30  0x181 中实际转速的 REGID
    --dpi N                 出图 dpi（默认 140）
    --title-prefix TEXT     标题前缀（默认 "L4CAN 日志"）

坐标系约定（**出图前先看这一节**）:
  0x210 Signal1 纵向 : 32767 = 中位/0%；>32767 驱动（半量程 32768），<32767 制动（半量程 32767）
  0x210 Signal2 横向 : 32767 = 回正；**0 = 满左、65535 = 满右**（protocol.yaml / scaleLateral）
                       所以 raw 比 32767 **小** = 左打，比 32767 **大** = 右打
                       实测幅值 ±7562 raw ⇔ 前轮 ±5.7692° ⇔ 方向盘 ±30°（转向比 5.2）
  0x201 TORQUE_CMD   : 16 位**小端有符号**，32767 = 100% 额定扭矩
  0x181 SPEED_IST    : 16 位**小端有符号**，32767 = 100% 额定转速
  0x401 EPS_Torque   : 该字节的**零点在 0x80（=128）= 0 Nm**，物理值 = (raw-128)*0.1 Nm
                       —— 协议文档写「signed 8-bit, -128~127 → -12.8~12.7 Nm」，但实测字节
                       在 0x80 两侧**单调平滑**（相邻帧最大跳变 6 LSB），若按补码读会在 0x7F/0x80
                       处凭空出现 ±12.8 Nm 的假跳变。默认按 offset 出图，可用
                       `--eps-torque-encoding signed` 复现旧画法对照。

依赖 : python3 + matplotlib；中文标签需要系统内有 CJK 字体（Noto Sans CJK / 文泉驿 / Droid Sans Fallback）。
"""
import argparse
import bisect
import collections
import os
import re
import statistics as st

import matplotlib
matplotlib.use('Agg')                      # 无显示环境下也必须先设后端
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC',
                                   'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 一行一帧: "2026-10-06 17:31:07.788  (+  10.07ms)  [8]  90 00 00 00 00 00 00 00"
LINE = re.compile(r'^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\s+\(([^)]*)\)\s+\[(\d+)\]\s+(.*)$')

# 报文 → 所在方向目录（桥的落盘规则: fsd2vcu = 工控机发出, vcu2fsd = 总线上收到）
DIRS = {'0x210': 'fsd2vcu', '0x301': 'fsd2vcu', '0x469': 'vcu2fsd', '0x402': 'vcu2fsd',
        '0x401': 'vcu2fsd', '0x201': 'vcu2fsd', '0x181': 'vcu2fsd', '0x1E4': 'vcu2fsd',
        '0x501': 'vcu2fsd'}

LON_CENTER = 32767.0          # 0x210 Signal1 中位（协议常量，不随日志变）
LAT_CENTER = 32767.0          # 0x210 Signal2 回正中位
EPS_ZERO_RAW = 128            # 0x401 EPS_Torque 的零点（0x80 = 0 Nm）


# ----------------------------------------------------------------------------- 基础工具
def to_sec(s):
    _, t = s.split(' ')
    h, m, sec = t.split(':')
    return int(h) * 3600 + int(m) * 60 + float(sec)


def hhmmss(sec):
    sec = sec % 86400
    return '%02d:%02d:%06.3f' % (sec // 3600, (sec % 3600) // 60, sec % 60)


def load(root, cid, direction=None):
    """读单个 ID 的日志 -> [(t_sec, dlc, [byte...])]；文件不存在返回 None"""
    direction = direction or DIRS.get(cid, 'vcu2fsd')
    path = os.path.join(root, direction, cid + '.log')
    if not os.path.exists(path):
        return None
    frames = []
    with open(path) as fh:
        for ln in fh.read().splitlines()[1:]:          # 首行是 "# id=... dir=... start=..."
            m = LINE.match(ln)
            if not m:
                continue
            t, _dt, dlc, data = m.groups()
            frames.append((to_sec(t), int(dlc), [int(x, 16) for x in data.split()]))
    return frames or None


def u16le(b, i):
    """小端 16 位（BAMOCAR 0x201/0x181 的 Data2~3 是小端）"""
    return b[i] | b[i + 1] << 8


def u16be(b, i):
    """大端 16 位（0x210 Signal1/Signal2：2026-10-06 起 FSD 按 Motorola 大端发送）"""
    return b[i] << 8 | b[i + 1]


def s16le(b, i):
    """小端 16 位有符号 —— 驱动器反馈量必须按有符号解，否则 65496 会被读成 199.9%"""
    v = u16le(b, i)
    return v - 65536 if v > 32767 else v


def s8(v):
    """8 位有符号（0x469 速度指令、0x401 扭矩字节按补码读时的老画法）"""
    return v - 256 if v > 127 else v


def lon_pct(v):
    """0x210 Signal1 raw -> 开度 %（+ 驱动 / - 制动；两半量程不同）"""
    span = 32768.0 if v >= LON_CENTER else 32767.0
    return (v - LON_CENTER) / span * 100.0


def lat_deg(v, max_steer_deg):
    """0x210 Signal2 raw -> 前轮转角（正 = 右；raw 越大越靠右）"""
    span = 32768.0 if v >= LAT_CENTER else 32767.0
    return (v - LAT_CENTER) / span * max_steer_deg


def side(v):
    """0x210 Signal2 raw -> 左/右/中位 文字（协议: 0=满左, 65535=满右）"""
    if v > LAT_CENTER:
        return '右'
    if v < LAT_CENTER:
        return '左'
    return '中位'


def step(frames, get, x0, gap=0.5):
    """帧序列 -> 阶梯折线（前向保持），配合 ax.step(..., where='post')

    gap: 相邻帧间隔超过该值就插一个 NaN 把线断开。
    **关键**：NaN 必须挂在**上一帧的 x**（prev+ε），不能挂在下一帧的 x ——
    where='post' 会先补一个 (x_next, y_prev) 中间点，若 NaN 落在 x_next 上，
    水平保持段 (x_prev,y_prev)→(x_next,y_prev) 两端都是有限值，照样会被画出来，
    假平台依旧存在（跨 100s 的“76% 扭矩”就是这么来的）。挂在 prev 上才能真断线。
    """
    xs, ys = [], []
    prev = None
    for t, _dlc, b in frames:
        v = get(b)
        if xs:
            if prev is not None and (t - prev) > gap:
                xs.append(prev - x0 + 1e-6)      # 断在上一帧之后，不留水平假平台
                ys.append(float('nan'))
            else:
                xs.append(t - x0)
                ys.append(ys[-1])
        xs.append(t - x0)
        ys.append(v)
        prev = t
    return xs, ys


def runs(items, pred, gap=1.0):
    """[(t,v)] 中满足 pred 的连续段 -> [(t0,t1)]；段内相邻帧间隔 > gap 视为断开"""
    segs, s, last = [], None, None
    for t, v in items:
        if pred(v):
            if s is None:
                s = t
            elif t - last > gap:
                segs.append((s, last))
                s = t
            last = t
        elif s is not None:
            segs.append((s, last))
            s = None
    if s is not None:
        segs.append((s, last))
    return segs


def is_const(vals):
    return len(set(vals)) == 1


def corr(xs, ys):
    if len(xs) < 3 or st.pstdev(xs) == 0 or st.pstdev(ys) == 0:
        return float('nan')
    mx, my = st.mean(xs), st.mean(ys)
    return sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / len(xs) / (st.pstdev(xs) * st.pstdev(ys))


class Series:
    """按时间取最近值的小工具（算相关性用）"""

    def __init__(self, pairs):
        self.t = [t for t, _v in pairs]
        self.v = [v for _t, v in pairs]

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return self.v[0]
        if i >= len(self.t):
            return self.v[-1]
        return self.v[i] if abs(self.t[i] - t) < abs(self.t[i - 1] - t) else self.v[i - 1]


# ----------------------------------------------------------------------------- 解析 + 体检
def analyze(root, args):
    d = {}
    for cid in ('0x210', '0x469', '0x402', '0x401', '0x201', '0x181', '0x1E4', '0x301', '0x501'):
        d[cid] = load(root, cid)
    if not d['0x210']:
        raise SystemExit(f'[错误] {root} 下找不到 fsd2vcu/0x210.log —— 这不是一次 L4 车检日志？')

    x0 = d['0x210'][0][0]                                   # t=0 约定: 0x210 首帧
    A = {'x0': x0, 'files': {k: v for k, v in d.items() if v},
         'regid_torque': args.drv_torque_regid, 'regid_speed': args.drv_speed_regid}
    # 0x210 Signal1/Signal2 字节序：2026-10-06 起 FSD 改为**大端**（Motorola）；
    # 更早的日志是旧的小端固件，用 --210-byte-order little 才能正确解码。
    dec210 = u16le if args.byte_210_order == 'little' else u16be
    A['dec210'] = dec210
    A['byte_210_order'] = args.byte_210_order
    A['lon'] = [(t - x0, dec210(b, 0)) for t, _d, b in d['0x210']]
    A['lat'] = [(t - x0, dec210(b, 2)) for t, _d, b in d['0x210']]

    # 0x210 Signal3/Signal4：上线 / 任务完成
    A['s3'] = sorted(set(b[4] for _t, _d, b in d['0x210']))
    A['fin_t'] = next((t - x0 for t, _d, b in d['0x210'] if b[5]), None)

    # 0x210 纵向驱动窗口（每个窗口 = 一次车检的下发段）
    A['drive_wins'] = runs(A['lon'], lambda v: v != LON_CENTER, gap=1.0)
    A['drive_vals'] = sorted(set(v for _t, v in A['lon'] if v != LON_CENTER))
    A['idle_vals'] = sorted(set(v for _t, v in A['lon'] if v == LON_CENTER))
    A['drive_dur'] = st.mean([b - a for a, b in A['drive_wins']]) if A['drive_wins'] else float('nan')
    A['drive_t0'] = A['drive_wins'][0][0] if A['drive_wins'] else float('nan')
    A['drive_t1'] = A['drive_wins'][-1][1] if A['drive_wins'] else float('nan')
    A['win_info'] = []
    for a, b in A['drive_wins']:
        cnt = collections.Counter(v for t, v in A['lon'] if a <= t <= b)
        n_win_frames = sum(cnt.values())
        # 只保留在窗口内至少出现 3 帧或占比 >5% 的取值，避免边界上的重帧/个别杂帧污染标题
        vals = sorted(v for v, c in cnt.items()
                      if v != LON_CENTER or c >= max(3, 0.05 * n_win_frames))
        if not vals:
            vals = sorted(cnt)
        A['win_info'].append({'a': a, 'b': b, 'dur': b - a, 'vals': vals,
                              'complete': A['fin_t'] is not None and a <= A['fin_t'] <= b + 1.0})

    # 横向正弦（周期按窗口内零穿越计算，避免跨窗口求出假周期）
    sine = [v for _t, v in A['lat'] if v != LAT_CENTER]
    A['amp'] = (max(sine) - min(sine)) / 2 if sine else float('nan')
    A['lat_min'] = min(sine) if sine else float('nan')
    A['lat_max'] = max(sine) if sine else float('nan')
    cross = [t for i, (t, v) in enumerate(A['lat'])
             if i and (v > LAT_CENTER) != (A['lat'][i - 1][1] > LAT_CENTER)]
    A['cross'] = cross
    half = []
    for a, b in A['drive_wins']:
        c = [t for t in cross if a <= t <= b]
        half += [c[i + 2] - c[i] for i in range(len(c) - 2)]
    A['period'] = st.median(half) if half else float('nan')
    A['lat_series'] = Series(A['lat'])

    # 0x469 / 0x402：VCU→EPS 速度指令流
    if d['0x469']:
        A['eps_cmd_byte'] = args.eps_cmd_byte
        A['eps_cmd'] = [(t - x0, b[args.eps_cmd_byte]) for t, _d, b in d['0x469']]
        A['eps_wins'] = [(d['0x469'][0][0] - x0, d['0x469'][-1][0] - x0)]
        A['eps_const'] = is_const([v for _t, v in A['eps_cmd']])
        # 与横摆指令的相关性（判断是不是“时变转向指令”）
        A['eps_corr'] = corr([v for _t, v in A['eps_cmd']],
                             [A['lat_series'].at(t) for t, _v in A['eps_cmd']])
        # 其它"常量候选"字节（排除已知的计数/校验字节），供 0/1 基歧义时对照
        cand = []
        for i in range(8):
            vals = [b[i] for _t, _d, b in d['0x469']]
            if is_const(vals) and 20 <= vals[0] <= 250:
                cand.append((i, vals[0]))
        A['eps_const_cands'] = cand
        # 0x469 字节4/字节7 与时间近似线性 → 计数/校验对
        A['eps_b4'] = [(t - x0, b[4]) for t, _d, b in d['0x469']]

    # 0x401：EPS→VCU 扭矩反馈（坐标系：默认 offset，0x80 = 0 Nm）
    if d['0x401']:
        i = args.eps_torque_byte
        A['eps_torque_byte'] = i
        A['eps_torque_encoding'] = args.eps_torque_encoding
        A['tq_raw'] = [(t - x0, b[i]) for t, _d, b in d['0x401']]
        if args.eps_torque_encoding == 'offset':
            A['tq'] = [(t, (r - EPS_ZERO_RAW) * 0.1) for t, r in A['tq_raw']]
        else:
            A['tq'] = [(t, s8(r) * 0.1) for t, r in A['tq_raw']]
        A['tq_subtypes'] = collections.Counter(b[0] for _t, _d, b in d['0x401'])
        A['tq_jump'] = max([abs(A['tq_raw'][k + 1][1] - A['tq_raw'][k][1])
                            for k in range(len(A['tq_raw']) - 1)
                            if A['tq_raw'][k + 1][0] - A['tq_raw'][k][0] < 0.2] or [0])
        A['tq_raw_min'] = min(r for _t, r in A['tq_raw'])
        A['tq_raw_max'] = max(r for _t, r in A['tq_raw'])
        # 车检窗口内 / 静止段内 分开统计（车检段才是真的被驱动的时段）
        in_win = lambda t: any(a <= t <= b for a, b in A['drive_wins'])
        A['tq_act'] = [(t, v) for t, v in A['tq'] if in_win(t)]
        A['tq_idle'] = [(t, v) for t, v in A['tq'] if not in_win(t)]
        A['tq_act_corr'] = corr([v for _t, v in A['tq_act']],
                                [A['lat_series'].at(t) for t, _v in A['tq_act']]) \
            if len(A['tq_act']) > 3 else float('nan')

    # 0x201：VCU→驱动器 —— 同一 ID 里按 REGID 复用，**必须先按 Data1 过滤再解码**
    if d['0x201']:
        A['regid_201_all'] = collections.Counter(b[0] for _t, _d, b in d['0x201'])
        A['regid_201_other'] = {}
        for rid in A['regid_201_all']:
            if rid == args.drv_torque_regid:
                continue
            vals = sorted(set(u16le(b, 1) for _t, _d, b in d['0x201'] if b[0] == rid))
            A['regid_201_other'][rid] = vals
        only = [(t, b) for t, _d, b in d['0x201'] if b[0] == args.drv_torque_regid]
        if only:
            A['tq_cmd'] = [(t - x0, s16le(b, 1) / 32767 * 100) for t, b in only]
            A['tq_cmd_raw'] = [(t - x0, s16le(b, 1)) for t, b in only]
            A['drv_wins'] = runs(A['tq_cmd'], lambda v: True, gap=0.5)
            A['pulses'] = runs(A['tq_cmd'], lambda v: abs(v) > 1e-9, gap=0.5)
            A['pulse_max'] = max((max(v for t, v in A['tq_cmd'] if a <= t <= b)
                                  for a, b in A['pulses']), default=0.0)
            A['pulse_min'] = min((min(v for t, v in A['tq_cmd'] if a <= t <= b)
                                  for a, b in A['pulses']), default=0.0)
            A['pulse_frames'] = sum(1 for t, v in A['tq_cmd']
                                    if abs(v) > 1e-9 and any(a <= t <= b for a, b in A['pulses']))
            A['drv_frames'] = only
            A['cmd_wins_covered'] = sum(1 for a, b in A['drive_wins']
                                        if any(a - 0.5 <= t - x0 <= b + 0.5 for t, _b in only))

    # 0x181：驱动器→VCU 实际转速（同样按 REGID 过滤 + 有符号 16 位）
    if d['0x181']:
        A['regid_181_all'] = collections.Counter(b[0] for _t, _d, b in d['0x181'])
        only = [(t, b) for t, _d, b in d['0x181'] if b[0] == args.drv_speed_regid]
        if only:
            A['spd'] = [(t - x0, s16le(b, 1) / 32767 * 100) for t, b in only]
            A['spd_raw'] = [(t - x0, s16le(b, 1)) for t, b in only]
            A['spd_max'] = max(abs(v) for _t, v in A['spd'])
            A['spd_raw_peak'] = max(v for _t, v in A['spd_raw'])
            A['tails'] = runs(A['spd'], lambda v: abs(v) > 1e-9, gap=0.5)
            A['spd_frames_nz'] = sum(1 for _t, v in A['spd'] if abs(v) > 1e-9)
            A['spd_frames'] = only
            A['spd_cov'] = sum(1 for a, b in A['drive_wins']
                               if any(a - 0.5 <= t <= b + 0.5 and v != 0
                                      for t, v in A['spd']))
            A['spd_cov_all'] = sum(1 for a, b in A['drive_wins']
                                   if any(a - 0.5 <= t - x0 <= b + 0.5 for t, _b in only))

    # 0x1E4 事件（RES 遥控器）
    if d['0x1E4']:
        A['res_states'] = collections.Counter(b[0] for _t, _d, b in d['0x1E4'])
        A['go_runs'] = runs([(t - x0, b[0]) for t, _d, b in d['0x1E4']],
                            lambda v: v == 0x13, gap=0.5)
        A['go'] = A['go_runs'][0][0] if A['go_runs'] else None

    # 0x301 心跳 / 0x501 模式（背景信息）
    if d['0x301']:
        A['hb_vals'] = collections.Counter(b[0] for _t, _d, b in d['0x301'])
    if d['0x501']:
        A['mode_vals'] = [(t - x0, b[0]) for t, _d, b in d['0x501']]
    return A


def report(root, A):
    x0 = A['x0']
    print(f'日志目录 : {root}')
    print(f't = 0    : {hhmmss(x0)}  (0x210 首帧)')
    print('-' * 100)
    for cid in ('0x210', '0x469', '0x402', '0x401', '0x201', '0x181', '0x301', '0x1E4'):
        f = A['files'].get(cid)
        if not f:
            print(f'{cid:6s}  —  无文件')
            continue
        dt = [f[i + 1][0] - f[i][0] for i in range(len(f) - 1)]
        print(f'{cid:6s}  {DIRS.get(cid, "?"):8s} n={len(f):6d}  DLC={f[0][1]}  '
              f'中位周期={st.median(dt)*1000:7.2f}ms  最大间隔={max(dt)*1000:8.1f}ms  '
              f'窗口 {f[0][0]-x0:8.3f} ~ {f[-1][0]-x0:8.3f} s')
    print('-' * 100)
    print(f'0x210 纵向   : {len(A["drive_wins"])} 段驱动, 中位 raw={A["idle_vals"]}, '
          f'驱动 raw={A["drive_vals"]}  [字节序={A["byte_210_order"]}]')
    for k, w in enumerate(A['win_info'], 1):
        pcts = ', '.join('%+.2f%%' % lon_pct(v) for v in w['vals'])
        print(f'   窗口{k}: {w["a"]:8.3f}~{w["b"]:8.3f}s  时长 {w["dur"]:6.2f}s  '
              f'{pcts}  {"完整(fin=1)" if w["complete"] else "未跑满/被中断"}')
    print(f'0x210 横向   : 幅值 ±{A["amp"]:.0f} raw = 前轮 '
          f'{lat_deg(A["lat_max"], 25.0):.4f}°（方向盘 ±30°），实测周期 {A["period"]:.3f}s, '
          f'{len(A["cross"])} 次零穿越')
    print(f'0x210 Signal3: {A["s3"]} (1=上线)   Signal4 完成时刻: '
          f'{("%.3fs" % A["fin_t"]) if A["fin_t"] is not None else "无"}')
    if 'eps_cmd' in A:
        v = [x for _t, x in A['eps_cmd']]
        rtxt = ('—（指令恒定，相关性无定义）' if A['eps_const'] or A['eps_corr'] != A['eps_corr']
                else '%.3f' % A['eps_corr'])
        print(f'0x469 速度指令: {"恒定" if A["eps_const"] else "变化"} {min(v)}~{max(v)} '
              f'(取第 {A["eps_cmd_byte"]+1} 字节/0基 {A["eps_cmd_byte"]}), 与横摆指令 r={rtxt}')
        print(f'      其它落在 20~250 的常量候选字节: '
              f'{[(f"Data{i+1}", val) for i, val in A["eps_const_cands"]]}')
    if 'tq_raw' in A:
        print(f'0x401 EPS扭矩: 字节{ [f"0x{k:02X}:{v}帧" for k,v in A["tq_subtypes"].items()] } '
              f'raw {A["tq_raw_min"]}~{A["tq_raw_max"]}  相邻帧最大跳变={A["tq_jump"]} LSB'
              f'  → 坐标系={A["eps_torque_encoding"]}(0x80=0Nm)')
        act = A['tq_act']
        if act:
            print(f'      车检窗口内 n={len(act)}  (raw-128)*0.1Nm = '
                  f'{min(v for _t,v in act):+.2f}~{max(v for _t,v in act):+.2f} Nm, '
                  f'与横向指令 r={A["tq_act_corr"]:.3f}')
        idle = A['tq_idle']
        if idle:
            print(f'      静止段   n={len(idle)}  '
                  f'{min(v for _t,v in idle):+.2f}~{max(v for _t,v in idle):+.2f} Nm')
    if 'regid_201_all' in A:
        print(f'0x201 REGID 分布 : {{{", ".join(f"0x{k:02X}:{v}帧" for k, v in A["regid_201_all"].items())}}} '
              f'(扭矩只看 0x{A["regid_torque"]:02X})')
        for rid, vals in A.get('regid_201_other', {}).items():
            print(f'      0x{rid:02X}: Data2~3 取值 {vals if len(vals) <= 6 else f"{vals[:6]}…"}（非扭矩，勿混画）')
    if 'tq_cmd' in A:
        print(f'0x201 扭矩给定: {len(A["tq_cmd"])} 帧, 覆盖 {A["cmd_wins_covered"]}/{len(A["drive_wins"])} 个驱动窗口, '
              f'窗口 {A["drv_wins"][0][0]:.2f}~{A["drv_wins"][-1][1]:.2f}s, '
              f'非零 {len(A["pulses"])} 段 / {A["pulse_frames"]} 帧, '
              f'{A["pulse_min"]:.2f}~{A["pulse_max"]:.2f}%')
    if 'regid_181_all' in A:
        print(f'0x181 REGID 分布 : {{{", ".join(f"0x{k:02X}:{v}帧" for k, v in A["regid_181_all"].items())}}} '
              f'(转速只看 0x{A["regid_speed"]:02X})')
    if 'spd' in A:
        print(f'0x181 实际转速: {len(A["spd"])} 帧, 覆盖 {A["spd_cov"]}/{len(A["drive_wins"])} 个驱动窗口, '
              f'非零 {A["spd_frames_nz"]} 帧 / {len(A["tails"])} 段, 峰值 {A["spd_max"]:.2f}% '
              f'(raw {A["spd_raw_peak"]})')
    if A.get('go_runs'):
        print(f'RES GO(0x13): {len(A["go_runs"])} 次 → ' +
              ', '.join('%.3fs(%s)' % (a, hhmmss(x0 + a)) for a, _b in A['go_runs']))
    if 'res_states' in A:
        print(f'0x1E4 状态分布: {{{", ".join(f"0x{k:02X}:{v}帧" for k, v in sorted(A["res_states"].items()))}}}')
    if 'hb_vals' in A:
        print(f'0x301 心跳字节: {{{", ".join(f"0x{k:02X}:{v}帧" for k, v in sorted(A["hb_vals"].items()))}}} (0x01=上线)')
    if 'mode_vals' in A:
        modes = ' -> '.join('%.1fs:0x%02X' % (t, v) for t, v in A['mode_vals'][:6])
        print(f'0x501 模式字节: {modes}')


# ----------------------------------------------------------------------------- 出图
def plot(root, A, out, args):
    stamp = os.path.basename(os.path.abspath(root).rstrip('/'))
    x0 = A['x0']
    n_win = len(A['drive_wins'])
    span = [max(t for t, _ in A['lon'])] + \
           [max(t for t, _ in A[k]) for k in ('eps_cmd', 'tq', 'tq_cmd', 'spd') if k in A]
    xmax = max(span) + 2
    xmin = -1.0
    if args.xlim:
        xmin, xmax = args.xlim

    fig, axs = plt.subplots(5, 1, figsize=(16.5, 15.4), dpi=args.dpi, sharex=True)
    for ax in axs:
        ax.grid(True, ls='-', lw=0.6, color='#dddddd')
        ax.set_axisbelow(True)
        ax.set_xlim(xmin, xmax)                 # sharex 只同步缩放，必须逐个显式设范围
        for k, w in enumerate(A['win_info'], 1):
            for x in (w['a'], w['b']):
                if x == x:
                    ax.axvline(x, color='#999999', ls=':', lw=0.8)
        w = A['win_info']
        if w and w[0]['a'] < xmax and w[-1]['b'] > xmin:
            for k, ww in enumerate(w, 1):
                if ww['b'] < xmin or ww['a'] > xmax:
                    continue
                ax.axvspan(ww['a'], ww['b'], color='#1f77b4', alpha=0.045, lw=0)

    dec = A['dec210']
    # ---------------- ① 纵向（0x210 Signal1） ----------------
    ax = axs[0]
    xs, ys = step(A['files']['0x210'], lambda b: dec(b, 0), x0)
    ax.step(xs, ys, where='post', color='#1f77b4', lw=1.6)
    vals = [v for _t, v in A['lon']]
    lo, hi = min(vals), max(vals)
    pad = max(60.0, 0.08 * max(1.0, hi - lo))
    ax.set_ylim(lo - pad, hi + pad)
    ticks = sorted(set([int(A['idle_vals'][0])] + [int(v) for v in A['drive_vals']]))[:6]
    ax.set_yticks(ticks)
    ax.set_yticklabels([f'{v}\n' + ('(中位/0%)' if v == int(A['idle_vals'][0]) else
                                   f'({lon_pct(v):+.2f}% ' + ('驱动)' if v > A['idle_vals'][0] else '制动)'))
                      for v in ticks], fontsize=9)
    ax.set_ylabel('0x210 Signal1\n纵向 raw', fontsize=10)
    order_txt = '大端' if A['byte_210_order'] == 'big' else '小端'
    seg_txt = ' + '.join(f'{w["dur"]:.1f}s' for w in A['win_info']) or '无'
    drive_txt = '、'.join(f'{lon_pct(v):+.2f}%' for v in A['drive_vals'])
    ax.set_title(f'FSD→VCU 纵向: {n_win} 段驱动（{seg_txt}），恒定 {drive_txt} 开度'
                 f'（32767=中位）　[0x210 Signal1 {order_txt}解码]', fontsize=11, loc='left')
    for k, w in enumerate(A['win_info'], 1):
        if w['b'] < xmin or w['a'] > xmax:
            continue
        ax.annotate(f'{k}·{w["dur"]:.1f}s ' + ('完整' if w['complete'] else '中断'),
                    xy=((max(w['a'], xmin) + min(w['b'], xmax)) / 2, hi),
                    xytext=(0, 5), textcoords='offset points',
                    ha='center', va='bottom', fontsize=8.5, color='#1f4e79')

    # ---------------- ② 横向（0x210 Signal2） ----------------
    ax = axs[1]
    xs, ys = step(A['files']['0x210'], lambda b: dec(b, 2), x0)
    ax.step(xs, ys, where='post', color='#2ca02c', lw=1.4)
    amp = A['amp']
    ax.set_ylim(LAT_CENTER - amp * 1.25, LAT_CENTER + amp * 1.25)
    tks = [LAT_CENTER - amp, LAT_CENTER, LAT_CENTER + amp]
    ax.set_yticks(tks)
    labs = []
    for v in tks:
        if v == LAT_CENTER:
            labs.append(f'{v:.0f}\n中位 0°')
        else:
            d = abs(lat_deg(v, args.max_steer_deg))
            labs.append(f'{v:.0f}\n{side(v)} {d:.2f}°(前轮)\n{side(v)} {d*args.steer_ratio:.1f}°(盘)')
    ax.set_yticklabels(labs, fontsize=8.5)
    ax.set_ylabel('0x210 Signal2\n横向 raw', fontsize=10)
    ax.set_title(f'FSD→VCU 横向: 正弦 ±{amp:.0f} raw（前轮 ±{abs(lat_deg(amp + LAT_CENTER, args.max_steer_deg)):.4f}°'
                 f'/方向盘 ±{abs(lat_deg(amp + LAT_CENTER, args.max_steer_deg))*args.steer_ratio:.1f}°）、周期 '
                 f'{A["period"]:.3f}s，仅在 {n_win} 段驱动窗口内输出　'
                 f'[协议: 0=满左 ↔ 65535=满右，故 raw<32767=左]', fontsize=11, loc='left')

    # ---------------- ③ 0x201 扭矩给定 ----------------
    ax = axs[2]
    if 'tq_cmd' in A:
        xs, ys = step([(t, 0, b) for t, b in A['drv_frames']],
                      lambda b: s16le(b, 1) / 32767 * 100, x0)
        ax.step(xs, ys, where='post', color='#d62728', lw=1.8,
                label='0x201 REGID 0x%02X TORQUE-CMD' % args.drv_torque_regid)
        ax.plot([t for t, _v in A['tq_cmd']], [v for _t, v in A['tq_cmd']],
                '.', color='#d62728', ms=5)   # 窄脉冲（几十 ms）只能靠散点看见
        raw_lo = min(r for _t, r in A['tq_cmd_raw'])
        raw_hi = max(r for _t, r in A['tq_cmd_raw'])
        if not A['pulses']:
            title3 = (f'VCU→驱动器: TORQUE-CMD 全程 0.00%（{len(A["tq_cmd"])} 帧全零，'
                      f'{A["cmd_wins_covered"]}/{n_win} 个驱动窗口有帧）：本轮驱动电机没有被给扭矩')
            ax.text(0.30, 0.60, f'本轮 {len(A["tq_cmd"])} 帧全为 0.00%（max {A["pulse_max"]:.2f}%）',
                    transform=ax.transAxes, fontsize=10.5, color='#d62728', va='center')
        else:
            title3 = (f'VCU→驱动器: TORQUE-CMD 非零 {len(A["pulses"])} 段 / {A["pulse_frames"]} 帧，'
                      f'{A["pulse_min"]:.2f}~{A["pulse_max"]:.2f}%（raw {raw_lo}~{raw_hi}，32767=100% 额定），'
                      f'覆盖 {A["cmd_wins_covered"]}/{n_win} 个驱动窗口')
        top = max(5.0, A['pulse_max'] * 1.35)
        ax.set_ylim(min(-0.5, A['pulse_min'] * 1.2), top)
    else:
        title3 = 'VCU→驱动器: 无 0x201 帧'
        ax.set_ylim(-2, 5)
    ax.set_yticks([0, round(max(1.0, A.get('pulse_max', 0)), 2) if A.get('pulse_max') else 1])
    ax.set_ylabel('BAMOCAR 扭矩给定\n(% 额定, + = 驱动方向)', fontsize=10)
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title(title3, fontsize=11, loc='left')

    # ---------------- ④ 0x181 实际转速 ----------------
    ax = axs[3]
    if 'spd' in A:
        xs, ys = step([(t, 0, b) for t, b in A['spd_frames']],
                      lambda b: s16le(b, 1) / 32767 * 100, x0)
        ax.step(xs, ys, where='post', color='#9467bd', lw=1.8,
                label='0x181 REGID 0x%02X SPEED_IST' % args.drv_speed_regid)
        ax.plot([t for t, _v in A['spd']], [v for _t, v in A['spd']],
                '.', color='#9467bd', ms=5)
        if not A['tails']:
            title4 = (f'驱动器→VCU: {n_win} 段驱动窗口内实际转速全程 = 0.00%'
                      f'（{len(A["spd"])} 帧全零，无衰减拖尾）')
        else:
            tail = max(b - a for a, b in A['tails'])
            title4 = (f'驱动器→VCU: SPEED_IST 非零 {A["spd_frames_nz"]} 帧 / {len(A["tails"])} 段'
                      f'（最长 {tail:.2f}s），峰值 {A["spd_max"]:.2f}%（raw {A["spd_raw_peak"]}，'
                      f'32767=100% 额定），覆盖 {A["spd_cov"]}/{n_win} 个驱动窗口')
        ax.set_ylim(min(-0.3, min(v for _t, v in A['spd']) * 1.2),
                    max(1.0, A['spd_max'] * 1.35))
        tks = [0, round(A['spd_max'], 2)]
        ax.set_yticks(tks)
        ax.set_yticklabels(['0\n(不转)', f'{tks[1]}\n(峰值)'], fontsize=9)
    else:
        title4 = '驱动器→VCU: 无 0x181 帧'
        ax.set_ylim(-1, 5)
    ax.set_ylabel('BAMOCAR 实际转速\n(% 额定)', fontsize=10)
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title(title4, fontsize=11, loc='left')

    # ---------------- ⑤ EPS 指令/反馈（双坐标轴：左=0x469 raw，右=0x401 扭矩 Nm） ----------------
    ax = axs[4]
    ax2 = ax.twinx()
    if 'eps_cmd' in A:
        xs, ys = step(A['files']['0x469'], lambda b: b[args.eps_cmd_byte], x0)
        lbl = ('0x469 Byte%d(0基)=0x%02X=%d 速度指令（恒定）'
               % (args.eps_cmd_byte, ys[0], ys[0])) if A['eps_const'] else \
              ('0x469 Byte%d(0基)（%d~%d，与横向 r=%.2f）'
               % (args.eps_cmd_byte, min(ys), max(ys), A['eps_corr']))
        ax.step(xs, ys, where='post', color='#ff7f0e', lw=2.2, label=lbl)
    if 'tq' in A:
        ax2.plot([t for t, _v in A['tq']], [v for _t, v in A['tq']], '.', color='#17becf', ms=4,
                 label='0x401 Byte%d(0基) raw-128 → EPS_Torque (0.1Nm/LSB)' % args.eps_torque_byte)
        ax2.axhline(0, color='#17becf', ls='--', lw=0.8, alpha=0.6)
        if A['tq_act']:
            av = [v for _t, v in A['tq_act']]
            t = ('VCU↔EPS: 转向速度指令恒为 %d（≈1200rpm，不随正弦变化）；EPS 扭矩反馈零点 0x80=0Nm，'
                 '车检段 %+.2f~%+.2f Nm（r=%.2f 与横向指令同相），静止段 ≈%+.2f Nm'
                 % (A['eps_cmd'][0][1], min(av), max(av), A['tq_act_corr'],
                    st.median([v for _t, v in A['tq_idle']]) if A['tq_idle'] else float('nan'))) \
                if A.get('eps_const') else \
                ('VCU↔EPS: 速度指令 %d~%d；EPS 扭矩反馈（零点 0x80）%+.2f~%+.2f Nm'
                 % (min(v for _t, v in A['eps_cmd']), max(v for _t, v in A['eps_cmd']),
                    min(v for _t, v in A['tq']), max(v for _t, v in A['tq'])))
        else:
            t = ('VCU↔EPS: 0x401 raw %d~%d（零点 0x80=0Nm）→ %+.2f~%+.2f Nm'
                 % (A['tq_raw_min'], A['tq_raw_max'],
                    min(v for _t, v in A['tq']), max(v for _t, v in A['tq'])))
    else:
        t = 'VCU↔EPS: 无 0x401 反馈帧'
    ax.set_ylim(0, 262)
    ax.set_yticks([20, 200, 250])
    ax.set_yticklabels(['20\n(≈120rpm)', '200\n(≈1200rpm)', '250\n(≈1500rpm)'], fontsize=9)
    ax.set_ylabel('0x469 速度指令\n(raw 20~250)', fontsize=10)
    if 'tq' in A:
        span_nm = max(0.4, max(abs(v) for _t, v in A['tq']) * 1.25)
        ax2.set_ylim(-span_nm, span_nm)
        tks = [-round(span_nm, 2), 0, round(span_nm, 2)]
        ax2.set_yticks(tks)
        ax2.set_yticklabels([f'{v:+.1f} Nm' for v in tks], fontsize=9, color='#17becf')
        ax2.set_ylabel('0x401 EPS 扭矩\n(Nm, 0x80=0)', fontsize=10, color='#17becf')
        ax2.grid(False)
    ax.set_xlabel(f'时间 (s, 相对 0x210 首帧 {hhmmss(x0)})', fontsize=11)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=9)
    ax.set_title(t, fontsize=11, loc='left')

    fig.suptitle(f'{args.title_prefix} {stamp}: 0x210 指令 → 电机层执行核查'
                 f'（5 面板统一时间轴，t=0 = 0x210 首帧 {hhmmss(x0)}'
                 + (f'；本图只画 {xmin:.1f}~{xmax:.1f}s' if args.xlim else '') + '）',
                 fontsize=13, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.982])
    fig.savefig(out)
    plt.close(fig)
    print(f'\n图已保存: {out}   请务必回读图片确认（无文字压线、无 0 字节文件）')


def main():
    ap = argparse.ArgumentParser(description='L4CAN 0x210→电机层执行核查总图')
    ap.add_argument('logdir', help='zlgcan_bridge 的 log/<时间戳> 目录')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--report-only', action='store_true', help='只打印体检报告，不出图')
    ap.add_argument('--xlim', type=float, nargs=2, default=None, metavar=('A', 'B'),
                    help='只画 A~B 秒（相对 0x210 首帧），用于放大某个车检窗口')
    ap.add_argument('--eps-cmd-byte', type=int, default=6, help='0x469 速度指令字节（0 基，默认 6）')
    ap.add_argument('--eps-torque-byte', type=int, default=1, help='0x401 扭矩字节（0 基，默认 1）')
    ap.add_argument('--eps-torque-encoding', choices=['offset', 'signed'], default='offset',
                    help='0x401 扭矩字节坐标系：offset（默认，0x80=0Nm）/ signed（补码，旧画法）')
    ap.add_argument('--drv-torque-regid', type=lambda x: int(x, 0), default=0x90,
                    help='0x201 中扭矩给定的 REGID（默认 0x90）')
    ap.add_argument('--drv-speed-regid', type=lambda x: int(x, 0), default=0x30,
                    help='0x181 中实际转速的 REGID（默认 0x30）')
    ap.add_argument('--max-steer-deg', type=float, default=25.0,
                    help='横向满量程前轮转角（默认 25.0，与 can_interface 一致）')
    ap.add_argument('--steer-ratio', type=float, default=5.2, help='方向盘/前轮转向比（默认 5.2）')
    ap.add_argument('--dpi', type=int, default=140)
    ap.add_argument('--210-byte-order', dest='byte_210_order',
                    choices=['big', 'little'], default='big',
                    help='0x210 Signal1/Signal2 字节序（默认 big＝2026-10-06 起的新固件；'
                         '更早的旧日志用 little）')
    ap.add_argument('--title-prefix', default='L4CAN 日志')
    args = ap.parse_args()

    root = os.path.abspath(args.logdir)
    A = analyze(root, args)
    report(root, A)
    if args.report_only:
        return
    stamp = os.path.basename(root.rstrip('/'))
    out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(root))),
                                   f'0x210_motor_level_execution_{stamp}.png')
    plot(root, A, out, args)


if __name__ == '__main__':
    main()
