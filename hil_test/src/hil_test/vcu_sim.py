"""L1 前置：vcan 模拟 VCU（事件驱动发 0x501 模式帧）+ RES 遥控器（周期发 0x1E4）.

新协议：0x501 Byte1 = 测试模式（1~6），原 Byte1「VCU 状态」定义已取消。

RES（遥控器）独立于 VCU：实测以 33.3Hz 周期广播 0x1E4，Byte1 为遥控器状态
（0x11 上线 / 0x13 发车按钮 / 0x10 急停）。本模拟按实测行为建模：
周期电平广播 + 发车按钮为 0.5s 瞬时脉冲 + 急停为持续电平。
"""

import sys
import threading
import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import RES_ESTOP, RES_ONLINE, RES_START, Protocol

# 发车按钮脉冲时长（s）：实测 0.15~0.51s，取上界
RES_PULSE_SEC = 0.5


class VcuSim:
    """模拟 VCU + RES 行为：

    - 事件驱动发 0x501（Byte1=测试模式）：仅在模式变化时发帧；真实 VCU 无周期心跳，
      故默认不做周期广播（period_ms=0）；
    - 周期广播 0x1E4（Byte1=RES 状态）：模拟 RES 遥控器，默认 0x11 上线；
    - 脚本可 set_mode / press_start / press_estop。

    模式：1操控性 / 2直线加速 / 3高速循迹 / 4八字绕环 / 5EBS / 6车检。
    """

    # 默认模式 1（操控性/未选任务）：can_interface 视为忽略、不发布 mission_mode_cmd
    DEFAULT_MODE = 1

    def __init__(self, interface, protocol_path, period_ms=0, poll_ms=10,
                 res_period_ms=None):
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
        # RES：周期广播周期取 protocol.yaml rx_1e4.period_ms（实测 30ms）
        if res_period_ms is None:
            res_period_ms = self._proto.res.get('period_ms', 30)
        self._res_period = float(res_period_ms) / 1000.0
        self._res_state = RES_ONLINE   # 脉冲结束后回落到该状态
        self._go_until = 0.0           # 发车脉冲截止时刻（monotonic）
        self._last_res_sent = 0.0

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

    # ---- RES（遥控器）模拟 ----
    @property
    def res_state(self):
        """当前 RES 状态字节：发车脉冲期间为 0x13，否则为常驻状态."""
        with self._lock:
            if time.monotonic() < self._go_until:
                return RES_START
            return self._res_state

    def press_start(self, pulse_sec=RES_PULSE_SEC):
        """按发车按钮：发 0x13 脉冲（实测 0.15~0.51s），随后回落到 0x11 上线态.

        can_interface 只在 0x13 变化沿发一次 /system/start_command（不重放不锁存），
        故调用时 mission_manager 必须已在运行。
        """
        with self._lock:
            self._res_state = RES_ONLINE
            self._go_until = time.monotonic() + max(0.0, pulse_sec)

    def press_estop(self):
        """按急停：切到 0x10 持续电平（实测按下后保持 9.6~475s）."""
        with self._lock:
            self._res_state = RES_ESTOP
            self._go_until = 0.0

    def release_estop(self):
        """松开急停：回到 0x11 上线态."""
        with self._lock:
            self._res_state = RES_ONLINE
            self._go_until = 0.0

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
            # RES：周期电平广播（实测 33.3Hz），状态变化在下一拍（<=30ms）发出
            if self._res_period > 0 and now - self._last_res_sent >= self._res_period:
                self._send_res(self.res_state)
                self._last_res_sent = now
            time.sleep(self._poll)

    def _send_mode(self, mode):
        """发送一帧 0x501（Byte1=模式）."""
        data = bytearray(8)
        data[self._proto.rx['mode_byte']] = mode
        self._sock.send(self._proto.rx['id'], bytes(data))

    def _send_res(self, state):
        """发送一帧 0x1E4（Byte1=RES 状态）."""
        self._sock.send(self._proto.res['id'], self._proto.encode_1e4(state))

    def stop(self):
        """停止模拟."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._sock.close()


def main(argv=None):
    """CLI：后台常驻模拟 VCU + RES（供 hil_test.sh 等调用）."""
    import argparse
    parser = argparse.ArgumentParser(
        description='模拟 VCU（事件驱动发 0x501 模式帧）+ RES（周期发 0x1E4 状态帧）')
    parser.add_argument('--interface', required=True)
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--period-ms', type=int, default=0,
                        help='0x501 额外周期广播周期(ms)，0=仅事件驱动（默认）')
    parser.add_argument('--res-period-ms', type=int, default=None,
                        help='0x1E4 RES 广播周期(ms)，缺省取 protocol.yaml（30ms）')
    args = parser.parse_args(argv)

    sim = VcuSim(args.interface, args.protocol, period_ms=args.period_ms,
                 res_period_ms=args.res_period_ms)
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
