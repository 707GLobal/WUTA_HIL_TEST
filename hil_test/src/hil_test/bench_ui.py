"""台架人工操作辅助（L3/L4 共用）：等 RES 发车放行.

台架（真实 VCU/RES）侧：AMI 选完模式后还需**人工按 RES 发车按钮**放行，
否则 VCU 不执行 0x210 指令——车不动、方向盘也不动（FSD 侧照样在发指令，
只是 VCU 不理会）。

RES 通过 CAN 0x1E4 广播（实测 33.3Hz 电平广播，Byte1=0x13 为发车按钮脉冲），
故本模块直接监听总线上的 0x1E4，不用固定时间窗盲等：

  wait_for_res_go()  —— 等本机总线上**新到**一帧 0x13（人按没按 GO 的唯一客观证据）

超时取自 hil_test.yaml 的 bench 段（res_go_timeout_sec）。

L4 车检到 FINISH 后不立即下 FSD：hold_after_finish() 保持整条链路在线
（默认 30s，可取 bench.post_finish_hold_sec / HIL_POST_FINISH_HOLD_SEC 覆盖），
给现场留出观察/确认窗口，之后用例 teardown 停 mission_manager、脚本停其余节点。
"""

import os
import time

from hil_test.protocol_loader import RES_START

_CONFIG_DIR = os.environ.get(
    'HIL_CONFIG',
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'config'))

_DEFAULT_TIMEOUT_SEC = 60.0
_DEFAULT_POST_FINISH_HOLD_SEC = 30.0


def _bench_cfg(key, default):
    try:
        import yaml
        with open(os.path.join(_CONFIG_DIR, 'hil_test.yaml'), 'r', encoding='utf-8') as f:
            return float(yaml.safe_load(f).get('bench', {}).get(key, default))
    except (OSError, ValueError, TypeError):
        return float(default)


def res_go_timeout_sec():
    """人工按 RES 发车按钮的等待超时（s）."""
    return _bench_cfg('res_go_timeout_sec', _DEFAULT_TIMEOUT_SEC)


def post_finish_hold_sec():
    """L4 车检到 FINISH 后、下 FSD 之前的保持时长（s）.

    优先级：环境变量 HIL_POST_FINISH_HOLD_SEC（现场临时调整，不改配置）>
    hil_test.yaml 的 bench.post_finish_hold_sec > 默认 30s；<=0 表示不等待。
    """
    env = os.environ.get('HIL_POST_FINISH_HOLD_SEC')
    if env not in (None, ''):
        try:
            return float(env)
        except ValueError:
            print(f'[L4]   !! HIL_POST_FINISH_HOLD_SEC={env!r} 不是数字，改用配置值',
                  flush=True)
    return _bench_cfg('post_finish_hold_sec', _DEFAULT_POST_FINISH_HOLD_SEC)


def hold_after_finish(prefix, seconds=None):
    """车检完成后保持 FSD 在线：等 seconds 秒，再让调用方下 FSD.

    目的：FINISH 后不立刻收尾——用例返回即 teardown 停 mission_manager、脚本随即
    停 can_interface/controller——给现场留出观察/确认窗口。等待中每 5s 打印一行
    剩余时间（pytest 的 tee-sys 让提示实时可见）。
    """
    if seconds is None:
        seconds = post_finish_hold_sec()
    if seconds <= 0:
        print(f'{prefix}   跳过车检后保持（{seconds:.0f}s）：直接下 FSD', flush=True)
        return
    print(f'\n{prefix} 车检已完成：保持 FSD 在线 {seconds:.0f}s 后再下 FSD', flush=True)
    t0 = time.monotonic()
    deadline = t0 + seconds
    while True:
        remain = deadline - time.monotonic()
        if remain <= 0:
            break
        print(f'{prefix}   …保持中，剩 {remain:.0f}s', flush=True)
        time.sleep(min(5.0, remain))
    print(f'{prefix}   保持结束（{time.monotonic() - t0:.0f}s），可以下 FSD', flush=True)


def wait_for_res_go(mon, proto, prefix, timeout=None, hint=None):
    """等本机总线上新到一帧 RES 发车信号（0x1E4 Byte1=0x13）；等到返回 True.

    判据是**纯总线**的（不经 FSD）：这是"人按没按 GO"的唯一客观证据，
    也避免 can_interface/mission_manager 有问题时把失败伪装成"人没按"。
    FSD 是否响应（进入 EXPLORE/INSPECTION）由调用方另行断言 mission_state。

    注意必须用 latest_newer_than：0x1E4 是 33.3Hz 电平广播，latest() 永远有帧，
    无法区分"刚按下"与"一直停在该状态"。
    """
    if timeout is None:
        timeout = res_go_timeout_sec()
    if hint is None:
        hint = '按 RES 发车按钮'
    t0 = time.monotonic()
    deadline = t0 + timeout
    print(f'\n{prefix} 请操作: {hint}（{timeout:.0f}s 超时）', flush=True)
    last_report = t0
    while time.monotonic() < deadline:
        frame = mon.latest_newer_than(proto.res['id'], t0)
        if frame is not None:
            state = proto.decode_1e4(frame[1])
            if state == RES_START:
                print(f'{prefix}   OK: 0x1E4 Byte1=0x{state:02X}'
                      f'（{time.monotonic() - t0:.1f}s）', flush=True)
                return True
        if time.monotonic() - last_report >= 5.0:
            frame = mon.latest(proto.res['id'])
            seen = '未收到' if frame is None else f'0x{proto.decode_1e4(frame[1]):02X}'
            print(f'{prefix}   …剩 {deadline - time.monotonic():.0f}s'
                  f'（0x1E4: {seen}）', flush=True)
            last_report = time.monotonic()
        time.sleep(0.05)
    return False


