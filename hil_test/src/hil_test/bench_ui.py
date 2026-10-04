"""台架人工操作辅助（L3/L4 共用）：等 RES 发车放行.

台架（真实 VCU/RES）侧：AMI 选完模式后还需**人工按 RES 发车按钮**放行，
否则 VCU 不执行 0x210 指令——车不动、方向盘也不动（FSD 侧照样在发指令，
只是 VCU 不理会）。

RES 通过 CAN 0x1E4 广播（实测 33.3Hz 电平广播，Byte1=0x13 为发车按钮脉冲），
故本模块直接监听总线上的 0x1E4，不用固定时间窗盲等：

  wait_for_res_go()  —— 等本机总线上**新到**一帧 0x13（人按没按 GO 的唯一客观证据）

超时取自 hil_test.yaml 的 bench 段（res_go_timeout_sec）。
"""

import os
import time

from hil_test.protocol_loader import RES_START

_CONFIG_DIR = os.environ.get(
    'HIL_CONFIG',
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'config'))

_DEFAULT_TIMEOUT_SEC = 30.0


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


