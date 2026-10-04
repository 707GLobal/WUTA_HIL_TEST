"""L1 前置：vcan 模拟 VCU（事件驱动发 0x501 模式帧）.

新协议：0x501 Byte1 = 测试模式（1~6），原 Byte1「VCU 状态」定义已取消，
故 vcu_sim 不再建模状态机/Go/急停，只发模式字节。
"""

import sys
import threading
import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import Protocol


class VcuSim:
    """模拟 VCU 行为：

    - 事件驱动发 0x501（Byte1=测试模式）：仅在模式变化时发帧；真实 VCU 无周期心跳，
      故默认不做周期广播（period_ms=0）；
    - 脚本可 set_mode。

    模式：1操控性 / 2直线加速 / 3高速循迹 / 4八字绕环 / 5EBS / 6车检。
    """

    # 默认模式 1（操控性/未选任务）：can_interface 视为忽略、不发布 mission_mode_cmd
    DEFAULT_MODE = 1

    def __init__(self, interface, protocol_path, period_ms=0, poll_ms=10):
        self._sock = CanSocket(interface, recv_timeout=0.01)
        self._proto = Protocol(protocol_path)
        # period_ms=0：仅事件驱动（默认）；>0 时额外按该周期广播（兼容旧用例）
        self.period = period_ms / 1000.0
        self._poll = poll_ms / 1000.0
        self._last_sent = None
        self._lock = threading.Lock()
        self._mode = self.DEFAULT_MODE
        self._running = False
        self._thread = None

    # ---- 模式 ----
    @property
    def mode(self):
        with self._lock:
            return self._mode

    # ---- 脚本控制 ----
    def set_mode(self, mode):
        """设置测试模式（Byte1：2加速/3循迹/4绕环/5EBS/6车检/1操控性）."""
        with self._lock:
            self._mode = mode

    # ---- 发送线程 ----
    def start(self):
        """启动模拟；接口不可用返回 False."""
        if not self._sock.open():
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def _loop(self):
        last_periodic = 0.0
        while self._running:
            mode = self.mode
            now = time.monotonic()
            if mode != self._last_sent:
                # 事件驱动：模式变化即发一帧
                self._send_mode(mode)
                self._last_sent = mode
                last_periodic = now
            elif self.period > 0 and now - last_periodic >= self.period:
                # 可选的周期广播（默认关闭）
                self._send_mode(mode)
                last_periodic = now
            time.sleep(self._poll)

    def _send_mode(self, mode):
        """发送一帧 0x501（Byte1=模式）."""
        data = bytearray(8)
        data[self._proto.rx['mode_byte']] = mode
        self._sock.send(self._proto.rx['id'], bytes(data))

    def stop(self):
        """停止模拟."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._sock.close()


def main(argv=None):
    """CLI：后台常驻模拟 VCU（供 hil_test.sh 等调用）."""
    import argparse
    parser = argparse.ArgumentParser(description='模拟 VCU（事件驱动发 0x501 模式帧）')
    parser.add_argument('--interface', required=True)
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--period-ms', type=int, default=0,
                        help='额外周期广播周期(ms)，0=仅事件驱动（默认）')
    args = parser.parse_args(argv)

    sim = VcuSim(args.interface, args.protocol, period_ms=args.period_ms)
    if not sim.start():
        print('vcu_sim: cannot open interface %s' % args.interface, file=sys.stderr)
        return 1
    print('vcu_sim running on %s' % args.interface, flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
