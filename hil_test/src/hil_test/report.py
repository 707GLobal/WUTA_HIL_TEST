"""测试结果汇总，生成 Markdown 报告."""

import os


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
