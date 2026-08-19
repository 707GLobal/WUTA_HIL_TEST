"""分层执行入口：--level L0.5 / L0 / L1 / L2 / L3.

用法：
  source /opt/ros/humble/setup.bash
  source <FSD workspace>/install/setup.bash     # 集成用例需要 FSD 消息与节点
  python scripts/run_hil.py --level L0.5 --interface vcan0
  python scripts/run_hil.py --level L1 --interface vcan0
  python scripts/run_hil.py --level L2 --interface can0
  python scripts/run_hil.py --level L3 --interface can0
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
    'L0.5': ('test_protocol.py', 'sim'),
    'L0': ('test_protocol.py', 'link'),
    'L1': ('test_protocol.py', 'protocol'),
    'L2': ('test_safety.py', 'safety'),
    'L3': ('test_motor_hil.py', 'motor'),
}


def main():
    ap = argparse.ArgumentParser(description='FSD HIL 分层测试入口')
    ap.add_argument('--level', required=True, choices=sorted(LEVELS))
    ap.add_argument('--interface', default=None, help='CAN 接口（默认读取 hil_test.yaml）')
    ap.add_argument('--log-dir', default=None, help='报告/日志输出目录')
    args = ap.parse_args()

    interface = args.interface or _load_default_interface()
    test_file, marker = LEVELS[args.level]

    print(f'[run_hil] level={args.level} file={test_file} marker={marker} '
          f'interface={interface}')
    if not _is_sim_interface(interface) and args.level in ('L0.5', 'L1'):
        print('[run_hil] 警告: L0.5/L1 注入用例建议用仿真接口（如 vcan0），'
              '真实接口下注入 0x501 会污染真实 VCU')

    env = os.environ.copy()
    env['HIL_INTERFACE'] = interface
    env['HIL_CONFIG'] = os.path.join(ROOT, 'config')
    if args.log_dir:
        env['HIL_LOG_DIR'] = args.log_dir

    cmd = [sys.executable, '-m', 'pytest',
           os.path.join(ROOT, 'test', test_file),
           '-m', marker, '-q', '--tb=short', '-p', 'no:cacheprovider', '-ra']
    raise SystemExit(subprocess.call(cmd, env=env))


if __name__ == '__main__':
    main()
