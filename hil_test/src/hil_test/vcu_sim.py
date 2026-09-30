"""L1 前置：vcan 模拟 VCU（事件驱动发 0x501，状态可脚本切换；解析 0x210）."""

import sys
import threading
import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import Protocol


class VcuSim:
    """模拟 VCU 行为：

    - 事件驱动发 0x501（Byte1=状态、Byte2=测试模式）：仅在状态/模式变化时发帧；
      真实 VCU 无周期心跳，故默认不做周期广播（period_ms=0）；
    - 解析 0x210，收到 Signal3 上线(1) 后状态由 6（等待高压）自动推进到 9（无人待命）；
    - 脚本可 request_go / set_emergency / set_finished / set_mode。

    状态优先级：EMERGENCY(12) > FINISHED(11) > DRIVING(10) > STANDBY(9) > WAIT_HV(6)。
    """

    STATE_WAIT_HV = 6
    STATE_STANDBY = 9
    STATE_DRIVING = 10
    STATE_FINISHED = 11
    STATE_EMERGENCY = 12
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
        self._online = False
        self._go = False
        self._emergency = False
        self._finished = False
        self._mode = self.DEFAULT_MODE
        self._running = False
        self._thread = None

    # ---- 状态 ----
    @property
    def state(self):
        """按优先级合成当前状态."""
        with self._lock:
            if self._emergency:
                return self.STATE_EMERGENCY
            if self._finished:
                return self.STATE_FINISHED
            if self._go and self._online:
                return self.STATE_DRIVING
            return self.STATE_STANDBY if self._online else self.STATE_WAIT_HV

    @property
    def mode(self):
        with self._lock:
            return self._mode

    # ---- 脚本控制 ----
    def set_mode(self, mode):
        """设置测试模式（Byte2：2加速/3循迹/4绕环/5EBS/6车检）."""
        with self._lock:
            self._mode = mode

    def request_go(self, enable=True):
        """请求无人驾驶（需已上线才进入状态 10）."""
        with self._lock:
            self._go = enable

    def set_emergency(self, enable=True):
        """急停（状态 12）."""
        with self._lock:
            self._emergency = enable

    def set_finished(self, enable=True):
        """任务完成（状态 11）."""
        with self._lock:
            self._finished = enable

    # ---- 收发线程 ----
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
            # 读取 0x210（非阻塞），解析上线信号以推进自身状态机
            frame = self._sock.recv()
            if frame is not None:
                can_id, payload = frame
                if can_id == self._proto.tx['id']:
                    with self._lock:
                        self._online = self._proto.decode_210(payload)['online']

            snapshot = (self.state, self.mode)
            now = time.monotonic()
            if snapshot != self._last_sent:
                # 事件驱动：状态/模式变化即发一帧
                self._send_state(*snapshot)
                self._last_sent = snapshot
                last_periodic = now
            elif self.period > 0 and now - last_periodic >= self.period:
                # 可选的周期广播（默认关闭）
                self._send_state(*snapshot)
                last_periodic = now
            time.sleep(self._poll)

    def _send_state(self, state, mode):
        """发送一帧 0x501（Byte1=状态、Byte2=模式）."""
        data = bytearray(8)
        data[self._proto.rx['state_byte']] = state
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
    parser = argparse.ArgumentParser(description='模拟 VCU（事件驱动发 0x501）')
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
