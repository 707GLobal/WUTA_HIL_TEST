"""可选：CAN 故障注入（任意模式帧），默认独立测试通道使用.

新协议：0x501 Byte1 = 测试模式，原「VCU 状态」定义已取消，故只注入模式帧。
"""

import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import Protocol


class FaultInjector:
    """向总线注入 0x501 模式帧（Byte1=mode）."""

    def __init__(self, interface, protocol_path):
        self._sock = CanSocket(interface)
        self._proto = Protocol(protocol_path)

    def open(self):
        """打开通道；失败返回 False."""
        return self._sock.open()

    def close(self):
        self._sock.close()

    def inject_mode(self, mode, count=1):
        """注入模式帧（0x501 Byte1=mode），返回成功发送帧数."""
        data = bytearray(8)
        data[self._proto.rx['mode_byte']] = mode
        sent = 0
        for _ in range(count):
            if self._sock.send(self._proto.rx['id'], bytes(data)):
                sent += 1
            time.sleep(0.01)
        return sent
