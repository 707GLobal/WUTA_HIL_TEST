"""解析 protocol.yaml，编解码统一入口（0x210 / 0x301 / 0x501 / 0x1E4）.

定标规则与 can_interface.cpp scaleControl() 保持一致：
  控制量 x∈[-1,1] → 10~65525，32767 为中心（0 控制）
  驱动/右：32767 + x*32758；制动/左：32767 + x*32757；钳位 [10, 65525]

Signal2 横向（scale_lateral，新规格）：0~65535，32762 为中心，
  越靠近 0 越左（0 = 满左）、越大越右（65535 = 满右）。

0x1E4（RES 遥控器 → 工控机）：33.3Hz 周期电平广播，Byte1 = 遥控器状态，
  0x11 上线 / 0x13 发车按钮（瞬时脉冲）/ 0x10 急停（持续电平）。
0x301（工控机 → VCU）：上线心跳，标准帧 / DLC=1 / 20Hz，
  Data[0]=0x01 在线 / 0x00 离线（取值同 0x210 Signal3，启动即发）。
"""

import os

import yaml

# 0x1E4 Byte1 RES 状态（与 can_interface.cpp 的 kRes* 常量保持一致）
RES_ESTOP = 0x10
RES_ONLINE = 0x11
RES_START = 0x13


def scale_control(x):
    """Signal1 纵向：归一化控制量 → 16bit 定标值（镜像 C++ scaleLongitudinal，含钳位）."""
    value = 32767.0 + x * 32758.0 if x >= 0.0 else 32767.0 + x * 32757.0
    return int(min(65525.0, max(10.0, value)))


# Signal2 横向（新规格）：0~65535，32762 为中心（回正）
#   0~32762    越靠近 0 越左 → 0 = 满左；32762~65535 越大越右 → 65535 = 满右
#   左半段跨度 = center-min = 32762；右半段 = max-center = 32773（镜像 C++ scaleLateral）
LATERAL_CENTER = 32762.0
LATERAL_MIN = 0.0
LATERAL_MAX = 65535.0


def scale_lateral(steer_deg, max_steer_deg):
    """Signal2 横向：转向角（+ = 左，autoware 约定）→ 原始值.

    满量程 ±max_steer_deg 对应 0（满左）/ 65535（满右），中心 32762 = 回正。
    """
    x = (steer_deg / max_steer_deg) if max_steer_deg > 0.0 else 0.0
    span = (LATERAL_CENTER - LATERAL_MIN) if x >= 0.0 else (LATERAL_MAX - LATERAL_CENTER)
    return int(min(LATERAL_MAX, max(LATERAL_MIN, LATERAL_CENTER - x * span)))


class Protocol:
    """协议配置加载与编解码."""

    def __init__(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.path = os.path.abspath(path)
        self.tx = cfg['tx_210']
        self.rx = cfg['rx_501']
        self.res = cfg['rx_1e4']
        # 0x301 上线心跳（.get 容错：旧 protocol.yaml 无本段时用内置默认）
        self.hb = cfg.get('tx_301', {
            'id': 0x301, 'dlc': 1, 'period_ms': 50,
            'online_byte': 0, 'online_value': 0x01, 'offline_value': 0x00})
        self.max_steer_deg = float(self.tx['max_steer_deg'])

    # ---- 编码 0x210（工控机→VCU）----
    def encode_210(self, throttle_brake, angle_deg, online, finished):
        """组装 0x210 帧数据（Signal1 纵向 / Signal2 横向 / Signal3 上线 / Signal4 完成）."""
        s1 = scale_control(float(throttle_brake))
        s2 = scale_lateral(float(angle_deg), self.max_steer_deg)  # angle 正=左 → 小值
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
        """解析 0x501：返回测试模式（Byte1）.

        新协议：原 Byte1「VCU 状态」定义已取消，原 Byte2「测试模式」提升到 Byte1。
        """
        return data[self.rx['mode_byte']]

    # ---- 解析 0x301（工控机→VCU 上线心跳，DLC=1）----
    def decode_301(self, data):
        """解析 0x301：返回 Data[0]（0x01 在线 / 0x00 离线）."""
        return data[self.hb['online_byte']]

    def mode_topic(self, mode):
        """测试模式 → mission_mode_cmd 字符串；None 表示忽略（操控性=有人驾驶）."""
        return self.rx['mode_topic_map'].get(mode)

    # ---- 编解码 0x1E4（RES 遥控器 → 工控机）----
    def encode_1e4(self, state):
        """组装 0x1E4 帧数据（Byte1=RES 状态，其余补 0）."""
        data = bytearray(int(self.res.get('dlc', 3)))
        data[self.res['byte']] = state
        return bytes(data)

    def decode_1e4(self, data):
        """解析 0x1E4：返回 Byte1（RES 状态）."""
        return data[self.res['byte']]

    def res_state(self, value):
        """RES 状态字节 → 名称（online/start/estop）；未知返回 None."""
        return self.res['state_map'].get(value)

    @staticmethod
    def _put_u16(data, offset, value):
        """小端写入 16bit."""
        data[offset] = value & 0xFF
        data[offset + 1] = (value >> 8) & 0xFF

    @staticmethod
    def _get_u16(data, offset):
        """小端读出 16bit."""
        return data[offset] | (data[offset + 1] << 8)
