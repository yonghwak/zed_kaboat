"""
기존 camera_node가 발행하는 camera/detections(색+모양+각도+거리, JSON)를
현재 pose와 결합해서 world 좌표로 바꾸고 mission_targets에 자동 등록한다.

- 원형(circle) + 빨강/초록/흰색  -> "buoy_<색>_<n>" (게이트/항로 표식용)
- 그 외 모양+색 조합             -> "dock_<색>_<모양>_<n>" (도킹 표식용)
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from zed_common import config
from zed_common import mission_targets as mt

COLOR_NAME = {'R': 'red', 'G': 'green', 'B': 'blue', 'O': 'orange', 'Y': 'yellow', 'W': 'white'}
GATE_BUOY_SHAPE = 'circle'
GATE_BUOY_COLORS = ('red', 'green', 'white')


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PerceptionBridgeNode(Node):
    def __init__(self):
        super().__init__('zed_perception_bridge_node')
        self.latest_pose = None  # (x, y, yaw)

        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_subscription(String, config.CAMERA_DETECTIONS_TOPIC, self.on_detections, 10)

        self.get_logger().info(f"perception bridge 시작. {config.CAMERA_DETECTIONS_TOPIC} 구독 중.")

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.latest_pose = (p.x, p.y, yaw)

    def on_detections(self, msg: String):
        if self.latest_pose is None:
            return
        try:
            detections = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not detections:
            return

        cam_x, cam_y, yaw = self.latest_pose
        targets = mt.load_targets()

        for det in detections:
            dist = det.get('distance')
            if dist is None:
                continue
            angle = det.get('angle', 0.0)
            color_code = det.get('color', '')
            shape = det.get('shape', 'circle')
            color = COLOR_NAME.get(color_code, color_code.lower())

            local_fwd = dist * math.cos(angle)
            local_left = dist * math.sin(angle)
            wx = cam_x + local_fwd * math.cos(yaw) - local_left * math.sin(yaw)
            wy = cam_y + local_fwd * math.sin(yaw) + local_left * math.cos(yaw)

            if shape == GATE_BUOY_SHAPE and color in GATE_BUOY_COLORS:
                prefix = f"buoy_{color}"
            else:
                prefix = f"dock_{color}_{shape}"

            dup = any(
                math.hypot(t['x'] - wx, t['y'] - wy) < config.BUOY_DEDUP_DIST_M
                for t in targets.values()
            )
            if dup:
                continue

            idx = sum(1 for k in targets if k.startswith(prefix)) + 1
            name = f"{prefix}_{idx}"
            targets = mt.register_target(name, wx, wy)
            self.get_logger().info(f"등록: {name} -> ({wx:.2f}, {wy:.2f})")


def main():
    rclpy.init()
    node = PerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
