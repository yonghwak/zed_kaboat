"""
현재 경로(mission_path.json)의 진행중인 게이트(mission_progress.json) 방향으로
cmd_final(Twist)을 발행한다. ThrusterOutput이 이걸 받아서 실제로 배를 움직인다.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

from zed_common import config
from zed_common import path_planner as pp


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HelmNode(Node):
    def __init__(self):
        super().__init__('zed_helm_node')
        self.latest_pose = None  # (x, y, yaw)

        self.pub = self.create_publisher(Twist, 'cmd_final', 10)
        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_timer(1.0 / config.HELM_RATE_HZ, self.on_control_tick)

        self.get_logger().info("조향 노드 시작 - 현재 게이트를 향해 cmd_final 발행")

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.latest_pose = (p.x, p.y, yaw)

    def on_control_tick(self):
        if self.latest_pose is None:
            return

        path = pp.load_path()
        progress = pp.load_progress()
        idx = progress.get("current_gate_idx", 0)

        twist = Twist()
        if not path or idx >= len(path):
            self.pub.publish(twist)  # 목표 없음/완주 -> 정지
            return

        gate = path[idx]
        x, y, yaw = self.latest_pose
        dx, dy = gate['x'] - x, gate['y'] - y
        dist = math.hypot(dx, dy)
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi

        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.HELM_KP_ANGULAR * bearing))
        # 많이 틀어져 있으면 속도 줄이고 제자리에 가깝게 회전, 정면이면 최대속도
        forward_scale = max(0.0, math.cos(bearing))
        linear = config.HELM_LINEAR_MAX * forward_scale
        if dist < config.GATE_REACHED_DIST_M:
            linear *= 0.5  # 게이트 근처에서는 속도 줄임

        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = HelmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
