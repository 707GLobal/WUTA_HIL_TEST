"""可选：CAN 故障注入（任意模式帧），默认独立测试通道使用.

新协议：0x501 Byte1 = 测试模式，原「VCU 状态」定义已取消，故只注入模式帧；
发车放行与急停由 RES 的 0x1E4 承载（Byte1=0x13/0x10），见 inject_res。
"""

import time

from hil_test.can_socket import CanSocket
from hil_test.protocol_loader import Protocol


class FaultInjector:
    """向总线注入 0x501 模式帧（Byte1=mode）与 0x1E4 RES 状态帧（Byte1=state）."""

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

    def inject_res(self, state, count=1):
        """注入 RES 状态帧（0x1E4 Byte1=state，如 RES_START/RES_ESTOP），返回发送帧数.

        帧长按 protocol.yaml rx_1e4.dlc 组装（can_socket 以传入长度作为 DLC）。
        """
        data = self._proto.encode_1e4(state)
        sent = 0
        for _ in range(count):
            if self._sock.send(self._proto.res['id'], data):
                sent += 1
            time.sleep(0.01)
        return sent
