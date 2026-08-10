"""
mission_mode.json의 모드에 따라 cmd_final(Twist)을 다르게 발행한다.
- gate_follow: mission_path.json의 현재 게이트를 향해 이동 (기존 동작)
- station_keep: 지정된 목표 반경(5m) 안에서 5초 연속 대기, 벗어나면 살짝 복귀
- idle: 정지
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

from zed_common import config
from zed_common import path_planner as pp
from zed_common import mission_targets as mt
from zed_common import mission_state as ms


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HelmNode(Node):
    def __init__(self):
        super().__init__('zed_helm_node')
        self.latest_pose = None  # (x, y, yaw)
        self.station_hold_start = None  # rclpy Time or None

        self.pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_timer(1.0 / config.HELM_RATE_HZ, self.on_control_tick)

        self.get_logger().info("조향 노드 시작")

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.latest_pose = (p.x, p.y, yaw)

    def _bearing_dist(self, x, y, yaw, tx, ty):
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        return bearing, dist

    def on_control_tick(self):
        if self.latest_pose is None:
            return
        x, y, yaw = self.latest_pose
        mode_data = ms.load_mode()
        mode = mode_data.get("mode", "gate_follow")

        if mode == "station_keep":
            self._tick_station_keep(x, y, yaw, mode_data.get("station_target"))
        elif mode == "gate_follow":
            self._tick_gate_follow(x, y, yaw)
        elif mode == "goto":
            self._tick_goto(x, y, yaw, mode_data.get("station_target"))
        elif mode == "scan":
            self.station_hold_start = None
            twist = Twist()
            twist.angular.z = config.MISSION_ZONE_SCAN_ANGULAR
            self.pub.publish(twist)
        else:
            self.station_hold_start = None
            self.pub.publish(Twist())

    def _tick_goto(self, x, y, yaw, target_name):
        self.station_hold_start = None
        targets = mt.load_targets()
        twist = Twist()
        if not target_name or target_name not in targets:
            self.pub.publish(twist)
            return
        t = targets[target_name]
        bearing, dist = self._bearing_dist(x, y, yaw, t['x'], t['y'])
        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.HELM_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        linear = config.HELM_LINEAR_MAX * forward_scale
        if dist < config.MISSION_ARRIVE_DIST_M:
            linear *= 0.3
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def _tick_gate_follow(self, x, y, yaw):
        self.station_hold_start = None
        path = pp.load_path()
        progress = pp.load_progress()
        idx = progress.get("current_gate_idx", 0)

        twist = Twist()
        if not path or idx >= len(path):
            self.pub.publish(twist)
            return

        gate = path[idx]
        bearing, dist = self._bearing_dist(x, y, yaw, gate['x'], gate['y'])
        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.HELM_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        linear = config.HELM_LINEAR_MAX * forward_scale
        if dist < config.GATE_REACHED_DIST_M:
            linear *= 0.5

        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def _tick_station_keep(self, x, y, yaw, target_name):
        targets = mt.load_targets()
        twist = Twist()

        if not target_name or target_name not in targets:
            self.station_hold_start = None
            ms.save_station_progress(0.0, False, target_name)
            self.pub.publish(twist)
            return

        t = targets[target_name]
        bearing, dist = self._bearing_dist(x, y, yaw, t['x'], t['y'])

        now = self.get_clock().now()
        if dist <= config.STATION_KEEP_RADIUS_M:
            if self.station_hold_start is None:
                self.station_hold_start = now
            held_sec = (now - self.station_hold_start).nanoseconds / 1e9
        else:
            self.station_hold_start = None
            held_sec = 0.0

        success = held_sec >= config.STATION_KEEP_HOLD_SEC
        ms.save_station_progress(held_sec, success, target_name)

        if dist < config.STATION_KEEP_DEADBAND_M:
            self.pub.publish(twist)  # 중심 근처면 정지, 미세보정 안 함
            return

        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.STATION_KEEP_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        twist.linear.x = config.STATION_KEEP_LINEAR_MAX * forward_scale
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
