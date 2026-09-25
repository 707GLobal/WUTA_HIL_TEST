"""分层执行入口：--level L0 / L1 / L2 / L3 / L4.

五层测试框架：
  L0 纯仿真（sim/unit，vcan 模拟 VCU，无 FSD）
  L1 链路 + 协议通畅（link/protocol，需 can_interface）
  L2 传感器自检故障模拟（selfcheck，故障即切 EMERGENCY，断电层）
  L3 低速动态安全闭环（motor：AMI 直线加速 + RES Go 放行 + RES 急停，通电层）
  L4 车检任务全链路（inspection：AMI 直选车检模式，通电层）

用法：
  source /opt/ros/humble/setup.bash
  source <FSD workspace>/install/setup.bash     # 集成用例需要 FSD 消息与节点
  python scripts/run_hil.py --level L0 --interface vcan0
  python scripts/run_hil.py --level L1 --interface vcan0
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


def _is_sim_interface(interface):
    """是否为仿真接口（config sim_interfaces 列表内）."""
    import yaml
    path = os.path.join(ROOT, 'config', 'hil_test.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return interface in yaml.safe_load(f)['can'].get('sim_interfaces', ['vcan0'])


# 级别 → (测试文件, pytest marker)
LEVELS = {
    'L0': ('test_protocol.py', 'sim or unit'),   # 纯仿真验证（无 FSD）
    'L1': ('test_protocol.py', 'link or protocol'),  # 链路自检 + 协议一致性（需 FSD）
    'L2': ('test_selfcheck.py', 'selfcheck'),    # 传感器自检故障模拟（断电层）
    'L3': ('test_drive_hil.py', 'motor'),        # 低速动态安全闭环（通电层）
    'L4': ('test_inspection.py', 'inspection'),  # 车检任务全链路（通电层）
}


def main():
    ap = argparse.ArgumentParser(description='FSD HIL 分层测试入口')
    ap.add_argument('--level', required=True, choices=sorted(LEVELS))
    ap.add_argument('--interface', default=None, help='CAN 接口（默认读取 hil_test.yaml）')
    args = ap.parse_args()

    interface = args.interface or _load_default_interface()
    test_file, marker = LEVELS[args.level]

    print(f'[run_hil] level={args.level} file={test_file} marker={marker} '
          f'interface={interface}')
    if args.level == 'L0' and not _is_sim_interface(interface):
        print(f'[run_hil] 错误: L0 为纯仿真验证，仅限仿真接口（如 vcan0）；'
              f'真实接口 {interface} 上会向 VCU 注入模拟帧，禁止运行', file=sys.stderr)
        sys.exit(2)

    env = os.environ.copy()
    env['HIL_INTERFACE'] = interface
    env['HIL_CONFIG'] = os.path.join(ROOT, 'config')

    cmd = [sys.executable, '-m', 'pytest',
           os.path.join(ROOT, 'test', test_file),
           '-m', marker, '-q', '--tb=long', '-p', 'no:cacheprovider', '-ra']
    rc = subprocess.call(cmd, env=env)  # 输出直通终端
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
