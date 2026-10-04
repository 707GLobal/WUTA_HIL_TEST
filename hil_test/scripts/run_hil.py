"""分层执行入口：--level L1 / L2 / L3 / L4.

四层测试框架：
  L1 链路 + 协议（原 L0 已并入，分两阶段）：
      pre  —— 协议编解码单测 + vcu_sim 模型（不依赖 FSD，脚本不起任何节点）
      post —— 链路自检 + 0x501/0x1E4/0x210 协议集成（需 can_interface）
  L2 传感器自检故障模拟（selfcheck，故障即切 EMERGENCY，断电层）
  L3 AMI 模式选择 + RES 放行 + 低速驱动
     （motor：AMI 选直线加速 → 门控 → RES GO(0x1E4 0x13) → EXPLORE，通电层）
  L4 车检任务全链路
     （inspection：AMI 直选车检模式 → 门控 → RES GO → INSPECTION，通电层）

L1 分两阶段的原因：vcu_sim 模型用例需要一个"干净"的总线，若与 can_interface
同跑，can_interface 的 10Hz 0x210（Signal3=0）会反复改写 vcu_sim 的在线状态，
使模型用例失真/失败；故 pre 阶段不起节点，post 阶段才起。

用法：
  source /opt/ros/humble/setup.bash
  source <FSD workspace>/install/setup.bash     # post 阶段需要 FSD 消息与节点
  python scripts/run_hil.py --level L1 --phase pre  --interface vcan0
  python scripts/run_hil.py --level L1 --phase post --interface vcan0
  python scripts/run_hil.py --level L2 --interface can0
  python scripts/run_hil.py --level L3 --interface can0   # 真实台架需 HIL_BENCH=1
  python scripts/run_hil.py --level L4 --interface can0   # 真实台架需 HIL_BENCH=1

pytest 完整输出（--tb=long 含回溯）直接打到终端，不生成报告/日志文件。
"""

import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_default_interface():
    """从 hil_test.yaml 读取默认 CAN 接口."""
    import yaml
    path = os.path.join(ROOT, 'config', 'hil_test.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)['can']['interface']


# 级别 → 阶段 → (测试文件, pytest marker)
#   L1 分 pre（不依赖 FSD）/ post（需 can_interface）两阶段；其余层单阶段 only。
LEVELS = {
    'L1': {
        'pre':  ('test_protocol.py', 'sim or unit'),      # 协议单测 + vcu_sim 模型（无需 FSD）
        'post': ('test_protocol.py', 'link or protocol'),  # 链路自检 + 协议集成（需 FSD）
    },
    'L2': {'only': ('test_selfcheck.py', 'selfcheck')},    # 传感器自检故障模拟（断电层）
    'L3': {'only': ('test_drive_hil.py', 'motor')},        # AMI 模式选择 + 低速驱动（通电层）
    'L4': {'only': ('test_inspection.py', 'inspection')},  # 车检任务全链路（通电层）
}


def main():
    ap = argparse.ArgumentParser(description='FSD HIL 分层测试入口')
    ap.add_argument('--level', required=True, choices=sorted(LEVELS))
    ap.add_argument('--phase', choices=['pre', 'post', 'only'], default=None,
                    help='L1: pre/post 两阶段；其余层固定 only（缺省按层推断）')
    ap.add_argument('--interface', default=None, help='CAN 接口（默认读取 hil_test.yaml）')
    args = ap.parse_args()

    interface = args.interface or _load_default_interface()
    phases = LEVELS[args.level]
    phase = args.phase or ('only' if 'only' in phases else 'pre')
    if phase not in phases:
        print(f'[run_hil] 错误: 层级 {args.level} 无阶段 {phase}', file=sys.stderr)
        sys.exit(2)
    test_file, marker = phases[phase]

    env = os.environ.copy()
    env['HIL_INTERFACE'] = interface
    env['HIL_CONFIG'] = os.path.join(ROOT, 'config')

    # L3/L4 需人工操作，等待提示（等待中…剩余 Ns）必须实时可见：pytest 默认捕获
    # print，用例结束才吐（仅失败时显示），人工会看不到提示而空等超时。
    #   --capture=tee-sys  tee 到真实 stdout，同时保留捕获（失败报告仍有 Captured stdout）
    #   -u                 必须配合：TeeCaptureIO 只转发 write、不转发 flush，非 tty
    #                      （重定向到文件/管道）时真实 stdout 仍块缓冲，提示会迟到几十秒
    cmd = [sys.executable, '-u', '-m', 'pytest',
           os.path.join(ROOT, 'test', test_file),
           '-m', marker, '-q', '--tb=long', '-p', 'no:cacheprovider', '-ra',
           '--capture=tee-sys']
    rc = subprocess.call(cmd, env=env)  # 输出直通终端
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
