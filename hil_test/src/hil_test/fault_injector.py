"""可选：CAN 故障注入（急停帧 / 任意状态帧），默认独立测试通道使用."""

import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import Protocol


class FaultInjector:
    """向总线注入故障帧：急停（0x501 state=12）、任意状态/模式帧."""

    def __init__(self, interface, protocol_path):
        self._sock = CanSocket(interface)
        self._proto = Protocol(protocol_path)

    def open(self):
        """打开通道；失败返回 False."""
        return self._sock.open()

    def close(self):
        self._sock.close()

    def inject_emergency(self, count=1):
        """注入急停帧（0x501 Byte1=12）."""
        return self.inject_state(12, count=count)

    def inject_state(self, state, mode=None, count=1):
        """注入任意状态帧；mode 为 None 时沿用当前模式字节（0）."""
        data = bytearray(8)
        data[self._proto.rx['state_byte']] = state
        if mode is not None:
            data[self._proto.rx['mode_byte']] = mode
        sent = 0
        for _ in range(count):
            if self._sock.send(self._proto.rx['id'], bytes(data)):
                sent += 1
            time.sleep(0.01)
        return sent
