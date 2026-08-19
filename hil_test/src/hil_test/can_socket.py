"""SocketCAN 原始套接字封装（纯 stdlib，不依赖 python-can）.

与 can_interface（C++ SocketCAN）同一接口行为，便于后续整体替换为 python-can。
"""

import socket
import struct


class CanSocket:
    """单个 SocketCAN 套接字：open / send / recv / close."""

    _FRAME_FMT = '=IB3x8s'  # can_frame: uint32 id + uint8 dlc + 3 pad + 8 data

    def __init__(self, interface, recv_timeout=0.05):
        self.interface = interface
        self._sock = None
        self._recv_timeout = recv_timeout

    def open(self):
        """打开并绑定接口；失败返回 False（静默降级，与 can_interface 一致）."""
        try:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
            sock.settimeout(self._recv_timeout)
        except OSError:
            self._sock = None
            return False
        self._sock = sock
        return True

    def send(self, can_id, data):
        """发送一帧；data 最长 8 字节。成功返回 True."""
        if self._sock is None:
            return False
        payload = bytes(data)[:8].ljust(8, b'\x00')
        frame = struct.pack(self._FRAME_FMT, can_id, len(bytes(data)[:8]), payload)
        try:
            self._sock.send(frame)
            return True
        except OSError:
            return False

    def recv(self):
        """非阻塞读一帧；无数据/错误返回 None。返回 (can_id, bytes)."""
        if self._sock is None:
            return None
        try:
            frame = self._sock.recv(16)
        except (socket.timeout, OSError, InterruptedError):
            return None
        if len(frame) < 16:
            return None
        can_id, dlc = struct.unpack('=IB', frame[:5])
        return can_id & 0x1FFFFFFF, frame[8:8 + dlc]

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
