"""台架人工操作辅助（L3/L4 共用）.

台架（真实 VCU）侧：选完 AMI 档位后还需**人工按 RES Go 放行**，否则 VCU 不执行
0x210 指令——车不动、方向盘也不动（FSD 侧照样在发指令，只是 VCU 不理会）。

新协议下 0x501 只剩模式字节，VCU 的 Go/驾驶态没有任何可观测信号，故无法做事件
驱动等待，只能用固定时间窗给操作员留出按 RES 的时间——本模块提供该窗口。

注意：窗口只解决"人来不来得及按"，不参与任何断言。L3/L4 的断言看的是 FSD 侧
输出（0x210 / mission_state），与 VCU 是否已放行无关，所以按没按 GO 都不会让
断言变红或变绿。
"""

import os
import time

_CONFIG_DIR = os.environ.get(
    'HIL_CONFIG',
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'config'))

_DEFAULT_WAIT_SEC = 10.0


def res_go_wait_sec():
    """读取 hil_test.yaml bench.res_go_wait_sec（缺省 10s）；<=0 关闭等待窗."""
    try:
        import yaml
        with open(os.path.join(_CONFIG_DIR, 'hil_test.yaml'), 'r', encoding='utf-8') as f:
            return float(yaml.safe_load(f).get('bench', {}).get(
                'res_go_wait_sec', _DEFAULT_WAIT_SEC))
    except (OSError, ValueError, TypeError):
        return _DEFAULT_WAIT_SEC


def pause_for_res_go(prefix, seconds=None, hint=None):
    """打印建议人工按 RES Go 的倒计时窗口；seconds<=0 直接返回.

    仅等待与提示，不做断言：窗口结束后调用方照常往下走。
    """
    if seconds is None:
        seconds = res_go_wait_sec()
    if seconds <= 0:
        return
    if hint is None:
        hint = ('请按 RES Go 放行——不按 GO，VCU 不执行 0x210 指令'
                '（车不动、方向盘不动）')
    print(f'\n{prefix} 等待操作: {hint}（{seconds:.0f}s）', flush=True)
    deadline = time.time() + seconds
    while True:
        left = deadline - time.time()
        if left <= 0.5:
            break
        print(f'{prefix}   剩余 {left:.0f}s，未按 GO 则本用例只会看到 FSD 侧指令',
              flush=True)
        time.sleep(min(2.0, left))
    print(f'{prefix}   等待窗结束，继续执行', flush=True)
