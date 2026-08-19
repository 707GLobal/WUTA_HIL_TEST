"""rclpy 进程内 ROS 注入与断言.

发布 FSD 输入话题（/control/command、/system/mission_state、/system/devices_inspection、
/system/mission_complete、/chcnav/velocity 台架车速源），订阅 can_interface 输出话题
（/system/start_command、/system/emergency、/system/mission_mode_cmd）供断言。

HIL 台架模式代发（车辆架起，传感器仅保在线）：
  /system/lidar_ready、/system/localization_ready、/localization/pose、
  /planning/final_waypoints（直路）——由本类代发，保证 FSD 正常在线。

需在 source 了 FSD workspace（install/setup.bash）的 ROS2 环境中运行。
"""

import math
import time


class RosInjector:
    """ROS2 注入/断言器（单节点，测试进程内使用）."""

    def __init__(self, node_name='hil_test_injector'):
        # 延迟导入：环境缺失时抛出明确错误
        try:
            import rclpy
            from autoware_msgs.msg import Command, Lane, Waypoint
            from wuta_msgs.msg import MissionState, DevicesInspection
            from geometry_msgs.msg import PoseStamped, TwistStamped
            from std_msgs.msg import Bool, String
        except ImportError as e:
            raise RuntimeError(
                'ROS2 环境不可用（需 source FSD workspace 后运行）: %s' % e) from e

        self._Command = Command
        self._Lane = Lane
        self._Waypoint = Waypoint
        self._MissionState = MissionState
        self._DevicesInspection = DevicesInspection
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._Bool = Bool
        self._String = String

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(node_name)
        self._latest = {}  # topic → (time.time, value)

        # 注入发布器
        self._pub_cmd = self._node.create_publisher(Command, '/control/command', 10)
        self._pub_state = self._node.create_publisher(
            MissionState, '/system/mission_state', 10)
        self._pub_insp = self._node.create_publisher(
            DevicesInspection, '/system/devices_inspection', 10)
        self._pub_vel = self._node.create_publisher(
            TwistStamped, '/chcnav/velocity', 10)
        self._pub_complete = self._node.create_publisher(Bool, '/system/mission_complete', 10)
        # HIL 台架代发
        self._pub_lidar_ready = self._node.create_publisher(
            Bool, '/system/lidar_ready', 10)
        self._pub_loc_ready = self._node.create_publisher(
            Bool, '/system/localization_ready', 10)
        self._pub_pose = self._node.create_publisher(
            PoseStamped, '/localization/pose', 10)
        self._pub_waypoints = self._node.create_publisher(
            Lane, '/planning/final_waypoints', 10)

        # 断言订阅器
        self._node.create_subscription(
            Bool, '/system/start_command',
            lambda m: self._on('/system/start_command', m.data), 10)
        self._node.create_subscription(
            Bool, '/system/emergency',
            lambda m: self._on('/system/emergency', m.data), 10)
        self._node.create_subscription(
            String, '/system/mission_mode_cmd',
            lambda m: self._on('/system/mission_mode_cmd', m.data), 10)
        self._node.create_subscription(
            MissionState, '/system/mission_state',
            lambda m: self._on('/system/mission_state', m.state), 10)

        time.sleep(1.0)  # 等 DDS 发现完成

    def _on(self, topic, value):
        self._latest[topic] = (time.time(), value)

    def spin_once(self):
        """处理一次订阅回调."""
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.05)

    # ---- 注入 ----
    def publish_command(self, speed=0.0, angle=0.0, throttle_brake=0.0):
        """注入 /control/command."""
        msg = self._Command()
        msg.speed = float(speed)
        msg.angle = float(angle)
        msg.throttle_brake = float(throttle_brake)
        self._pub_cmd.publish(msg)

    def publish_mission_state(self, state, mission_mode=0):
        """注入 /system/mission_state."""
        msg = self._MissionState()
        msg.state = state
        msg.mission_mode = mission_mode
        self._pub_state.publish(msg)

    def publish_devices_inspection(self, ok, failures=None):
        """注入 /system/devices_inspection（设备自检结果）."""
        msg = self._DevicesInspection()
        msg.ok = ok
        msg.failures = list(failures or [])
        self._pub_insp.publish(msg)

    def publish_velocity(self, vx):
        """注入 /chcnav/velocity（L3 台架车速源，FSD 侧不改动）."""
        msg = self._TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        self._pub_vel.publish(msg)

    def publish_mission_complete(self):
        """注入 /system/mission_complete=true."""
        msg = self._Bool()
        msg.data = True
        self._pub_complete.publish(msg)

    # ---- HIL 台架代发（传感器仅保在线） ----
    def publish_lidar_ready(self, ready=True):
        """代发 /system/lidar_ready（IDLE→READY 门控）."""
        msg = self._Bool()
        msg.data = ready
        self._pub_lidar_ready.publish(msg)

    def publish_localization_ready(self, ready=True):
        """代发 /system/localization_ready（门控 + planning 门槛）."""
        msg = self._Bool()
        msg.data = ready
        self._pub_loc_ready.publish(msg)

    def publish_pose(self, x=0.0, y=0.0, yaw=0.0):
        """代发 /localization/pose（controller 位姿 + mission_manager 计圈）."""
        msg = self._PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._pub_pose.publish(msg)

    def publish_waypoints_straight(self, length=10.0, step=1.0, speed=2.0):
        """代发 /planning/final_waypoints：原点沿 x 轴直路."""
        lane = self._Lane()
        lane.header.stamp = self._node.get_clock().now().to_msg()
        lane.header.frame_id = 'map'
        dist = 0.0
        while dist <= length:
            wp = self._Waypoint()
            wp.pose.header = lane.header
            wp.pose.pose.position.x = dist
            wp.pose.pose.orientation.w = 1.0
            wp.twist.header = lane.header
            wp.twist.twist.linear.x = float(speed)
            lane.waypoints.append(wp)
            dist += step
        self._pub_waypoints.publish(lane)

    # ---- 断言 ----
    def latest(self, topic):
        """最近收到的话题值；未收到返回 None."""
        entry = self._latest.get(topic)
        return None if entry is None else entry[1]

    def wait_for(self, topic, expected, timeout=5.0):
        """轮询等待话题值等于 expected；成功返回 True，超时 False."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.spin_once()
            if self.latest(topic) == expected:
                return True
        return False

    def graph_nodes(self):
        """ROS 图中已发现节点名（用于检查 FSD 是否运行）."""
        return sorted(n for n, _ in self._node.get_node_names_and_namespaces())

    def destroy(self):
        """销毁节点（进程内单次测试用）."""
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
