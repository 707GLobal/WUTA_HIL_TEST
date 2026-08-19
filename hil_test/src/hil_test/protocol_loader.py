"""解析 protocol.yaml，编解码统一入口（0x210 / 0x501）.

定标规则与 can_interface.cpp scaleControl() 保持一致：
  控制量 x∈[-1,1] → 10~65525，32767 为中心（0 控制）
  驱动/右：32767 + x*32758；制动/左：32767 + x*32757；钳位 [10, 65525]
"""

import os

import yaml


def scale_control(x):
    """归一化控制量 → 16bit 定标值（镜像 C++ 实现，含钳位）."""
    value = 32767.0 + x * 32758.0 if x >= 0.0 else 32767.0 + x * 32757.0
    return int(min(65525.0, max(10.0, value)))


class Protocol:
    """协议配置加载与编解码."""

    def __init__(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.path = os.path.abspath(path)
        self.tx = cfg['tx_210']
        self.rx = cfg['rx_501']
        self.max_steer_deg = float(self.tx['max_steer_deg'])

    # ---- 编码 0x210（工控机→VCU）----
    def encode_210(self, throttle_brake, angle_deg, online, finished):
        """组装 0x210 帧数据（Signal1 纵向 / Signal2 横向 / Signal3 上线 / Signal4 完成）."""
        s1 = scale_control(float(throttle_brake))
        s2 = scale_control(-float(angle_deg) / self.max_steer_deg)  # 与 C++ 一致：angle 正=左→小值
        data = bytearray(8)
        self._put_u16(data, 0, s1)
        self._put_u16(data, 2, s2)
        data[4] = 1 if online else 0
        data[5] = 1 if finished else 0
        return bytes(data)

    def decode_210(self, data):
        """解析 0x210 帧，供总线断言."""
        return {
            'longitudinal': self._get_u16(data, 0),
            'lateral': self._get_u16(data, 2),
            'online': bool(data[4]),
            'finished': bool(data[5]),
        }

    # ---- 解析 0x501（VCU→工控机）----
    def decode_501(self, data):
        """解析 0x501：返回 (状态 Byte1, 测试模式 Byte2)."""
        return data[self.rx['state_byte']], data[self.rx['mode_byte']]

    def mode_topic(self, mode):
        """测试模式 → mission_mode_cmd 字符串；None 表示忽略（操控性=有人驾驶）."""
        return self.rx['mode_topic_map'].get(mode)

    @staticmethod
    def _put_u16(data, offset, value):
        """小端写入 16bit."""
        data[offset] = value & 0xFF
        data[offset + 1] = (value >> 8) & 0xFF

    @staticmethod
    def _get_u16(data, offset):
        """小端读出 16bit."""
        return data[offset] | (data[offset + 1] << 8)
