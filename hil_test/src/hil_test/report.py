"""测试结果汇总，生成 Markdown 报告."""

import os
import xml.etree.ElementTree as ET


class Report:
    """收集用例结果并输出 Markdown 报告."""

    def __init__(self, title):
        self.title = title
        self.cases = []  # [(name, passed, detail)]

    def add(self, name, passed, detail=''):
        """记录一条用例结果."""
        self.cases.append((name, bool(passed), detail))

    @property
    def passed_count(self):
        return sum(1 for _, passed, _ in self.cases if passed)

    def to_markdown(self):
        """渲染报告文本."""
        lines = [f'# {self.title}', '',
                 f'- 通过: {self.passed_count}/{len(self.cases)}', '']
        for name, passed, detail in self.cases:
            mark = 'PASS' if passed else 'FAIL'
            lines.append(f'- [{mark}] {name}' + (f' — {detail}' if detail else ''))
        return '\n'.join(lines)

    def save(self, path):
        """落盘；返回路径."""
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(self.to_markdown())
        return path


def generate_detailed_report(junit_xml, output_md, title='HIL 测试报告'):
    """解析 pytest 的 JUnitXML，生成含每个用例结果与失败详情的 Markdown 报告.

    返回输出路径；junit_xml 不存在时抛出 FileNotFoundError。
    """
    tree = ET.parse(junit_xml)
    root = tree.getroot()
    # pytest 的 junit.xml 根为 <testsuites>，统计属性在子 <testsuite> 上
    suites = root.findall('testsuite') or [root]
    tests = sum(int(s.get('tests', 0)) for s in suites)
    failures = sum(int(s.get('failures', 0)) for s in suites)
    errors = sum(int(s.get('errors', 0)) for s in suites)
    skipped = sum(int(s.get('skipped', 0)) for s in suites)
    passed = max(0, tests - failures - errors - skipped)
    timestamp = suites[0].get('timestamp', 'unknown') if suites else 'unknown'
    total_time = str(sum(float(s.get('time', 0) or 0) for s in suites))

    lines = [
        f'# {title}', '',
        f'- 时间: {timestamp}',
        f'- 用例: {tests}，通过: {passed}，失败: {failures}，错误: {errors}，跳过: {skipped}',
        f'- 总耗时: {total_time}s',
        '',
        '| 用例 | 结果 | 耗时(s) | 信息 |',
        '|---|---|---|---|',
    ]
    for tc in root.iter('testcase'):
        name = f'{tc.get("classname", "")}.{tc.get("name", "")}'
        time_s = tc.get('time', '')
        failure = tc.find('failure')
        error = tc.find('error')
        skipped_node = tc.find('skipped')
        if failure is not None:
            lines.append(f'| {name} | FAIL | {time_s} | {failure.get("message", "")} |')
        elif error is not None:
            lines.append(f'| {name} | ERROR | {time_s} | {error.get("message", "")} |')
        elif skipped_node is not None:
            lines.append(f'| {name} | SKIP | {time_s} | {skipped_node.get("message", "")} |')
        else:
            lines.append(f'| {name} | PASS | {time_s} | |')

    # 失败/错误详情：完整 traceback，可追溯到具体报错
    lines += ['', '## 失败 / 错误详情', '']
    has_fail = False
    for tc in root.iter('testcase'):
        for tag in ('failure', 'error'):
            node = tc.find(tag)
            if node is None:
                continue
            has_fail = True
            lines += [f'### {tc.get("classname", "")}.{tc.get("name", "")}', '']
            msg = node.get('message', '')
            if msg:
                lines += [f'**信息**: {msg}', '']
            text = (node.text or '').strip()
            if text:
                lines += ['```', text, '```', '']
    if not has_fail:
        lines += ['- 无失败 / 错误用例。', '']

    lines.append(f'- 原始日志: {os.path.basename(junit_xml)}')
    directory = os.path.dirname(os.path.abspath(output_md))
    os.makedirs(directory, exist_ok=True)
    with open(output_md, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return output_md
