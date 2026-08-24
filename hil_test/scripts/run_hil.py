"""分层执行入口：--level L0 / L1 / L2 / L3.

用法：
  source /opt/ros/humble/setup.bash
  source <FSD workspace>/install/setup.bash     # 集成用例需要 FSD 消息与节点
  python scripts/run_hil.py --level L0 --interface vcan0
  python scripts/run_hil.py --level L1 --interface vcan0 --report-dir logs/L1
  python scripts/run_hil.py --level L2 --interface can0
  python scripts/run_hil.py --level L3 --interface can0
"""

import argparse
import datetime
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))  # 供生成报告 import hil_test.report


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
    'L2': ('test_safety.py', 'safety'),
    'L3': ('test_motor_hil.py', 'motor'),
}


def main():
    ap = argparse.ArgumentParser(description='FSD HIL 分层测试入口')
    ap.add_argument('--level', required=True, choices=sorted(LEVELS))
    ap.add_argument('--interface', default=None, help='CAN 接口（默认读取 hil_test.yaml）')
    ap.add_argument('--log-dir', default=None, help='报告/日志输出目录')
    ap.add_argument('--report-dir', default=None,
                    help='该层报告输出目录（生成 junit.xml + 详细 Markdown 报告）')
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
    if args.log_dir:
        env['HIL_LOG_DIR'] = args.log_dir

    cmd = [sys.executable, '-m', 'pytest',
           os.path.join(ROOT, 'test', test_file),
           '-m', marker, '-q', '--tb=long', '-p', 'no:cacheprovider', '-ra']
    junit = None
    if args.report_dir:
        os.makedirs(args.report_dir, exist_ok=True)
        junit = os.path.join(args.report_dir, 'junit.xml')
        cmd += ['--junitxml', junit]
    rc = subprocess.call(cmd, env=env)

    # 生成该层详细报告：日期-时间-项目.md（含每个用例结果与失败 traceback）
    if junit and os.path.exists(junit):
        try:
            from hil_test.report import generate_detailed_report
            project = os.environ.get('HIL_PROJECT', 'WUTA_HIL')
            stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
            out = os.path.join(args.report_dir, f'{stamp}-{project}.md')
            generate_detailed_report(junit, out, title=f'{args.level} HIL 测试报告')
            print(f'[run_hil] 详细报告: {out}')
        except Exception as exc:  # 报告生成失败不影响测试退出码
            print(f'[run_hil] 生成报告失败: {exc}', file=sys.stderr)
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
