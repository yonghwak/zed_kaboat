"""
경기규정상 항상 켜져 있어야 하는 장애물 회피 안전레이어.
미션 로직(helm_node 등)은 cmd_mission으로 "이렇게 가고싶다"를 발행하고,
이 노드가 전방 depth를 실시간으로 보면서 위험하면 감속/회피로 덮어써서
cmd_final로 최종 발행한다. ThrusterOutput은 cmd_final만 구독.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Twist

from zed_common import config


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('zed_safety_supervisor_node')
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.latest_danger = None  # (forward_min_dist, left_count, right_count)

        self.create_subscription(Twist, 'cmd_mission', self.on_cmd_mission, 10)
        self.create_subscription(PointCloud2, config.POINTCLOUD_TOPIC, self.on_cloud, 5)
        self.pub = self.create_publisher(Twist, 'cmd_final', 10)
        self.create_timer(1.0 / config.HELM_RATE_HZ, self.on_tick)

        self.get_logger().info("안전 감시(상시 장애물회피) 노드 시작")

    def on_cmd_mission(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_cloud(self, msg: PointCloud2):
        pts_struct = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts_struct.size == 0:
            self.latest_danger = None
            return
        stride = config.SAFETY_POINTCLOUD_STRIDE
        x = pts_struct['x'][::stride]
        y = pts_struct['y'][::stride]
        z = pts_struct['z'][::stride]

        valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                 & (z > config.Z_MIN_M) & (z < config.Z_MAX_M) & (x > 0))
        x, y = x[valid], y[valid]
        if x.size == 0:
            self.latest_danger = None
            return

        corridor = np.abs(y) < config.SAFETY_CORRIDOR_HALF_WIDTH_M
        forward_min = float(x[corridor].min()) if np.any(corridor) else math.inf

        near = x < config.SAFETY_SLOW_DIST_M
        left_count = int(np.sum((y > 0) & near))
        right_count = int(np.sum((y < 0) & near))

        self.latest_danger = (forward_min, left_count, right_count)

    def on_tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        twist = Twist()
        if elapsed > config.SAFETY_CMD_TIMEOUT_SEC:
            self.pub.publish(twist)  # 미션 명령 끊기면 정지
            return

        linear = self.latest_cmd.linear.x
        angular = self.latest_cmd.angular.z

        if self.latest_danger is not None:
            forward_min, left_count, right_count = self.latest_danger
            turn_dir = 1.0 if left_count < right_count else -1.0

            if forward_min < config.SAFETY_HARD_STOP_DIST_M:
                # 진짜 임박 - 완전정지 + 최대 선회 (최후 수단)
                linear = 0.0
                angular = turn_dir * config.SAFETY_MAX_AVOID_TURN
                self.get_logger().warn(
                    f"장애물 매우 근접({forward_min:.1f}m) - 긴급정지+선회", throttle_duration_sec=1.0)
            elif forward_min < config.SAFETY_STOP_DIST_M:
                # 회피 구간 - 서지 않고 속도 줄이며 점점 세게 틀기
                avoid_ratio = (config.SAFETY_STOP_DIST_M - forward_min) / \
                              (config.SAFETY_STOP_DIST_M - config.SAFETY_HARD_STOP_DIST_M)
                angular = turn_dir * config.SAFETY_MAX_AVOID_TURN * avoid_ratio
                linear = min(linear, config.HELM_LINEAR_MAX * config.SAFETY_AVOID_LINEAR_SCALE)
                self.get_logger().info(
                    f"장애물 회피 중({forward_min:.1f}m)", throttle_duration_sec=1.0)
            elif forward_min < config.SAFETY_SLOW_DIST_M:
                scale = (forward_min - config.SAFETY_STOP_DIST_M) / \
                        (config.SAFETY_SLOW_DIST_M - config.SAFETY_STOP_DIST_M)
                linear *= max(0.3, scale)

        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
