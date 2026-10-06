#!/usr/bin/env python3
"""清理残留 FSD 节点进程（供 hil_test.sh 启动前 / 退出时调用）.

被中断的运行可能留下**用例自起**的 mission_manager（不在脚本 NODE_PIDS 里），
残留实例会让下一次 L4 卡在 READY —— 判定与原因见 hil_test/src/hil_test/stale_nodes.py。

用法:
    python3 scripts/kill_stale_nodes.py                 # 清 mission_manager/can_interface/controller
    python3 scripts/kill_stale_nodes.py -n              # 只列出，不杀（排查用）
    python3 scripts/kill_stale_nodes.py controller:controller_node   # 只清指定节点
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from hil_test.stale_nodes import NODES, find_stale, kill_stale  # noqa: E402


def main(argv):
    dry_run = any(a in ('-n', '--dry-run') for a in argv)
    names = [a for a in argv if not a.startswith('-')]
    pairs = tuple(tuple(a.split(':', 1)) for a in names) if names else NODES
    if dry_run:
        for pid, pkg, node in find_stale(pairs):
            print(f'{pkg}/{node} (pid {pid})')
        return 0
    for pid, pkg, node in kill_stale(pairs):
        print(f'==> 清理残留节点 {pkg}/{node} (pid {pid})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
