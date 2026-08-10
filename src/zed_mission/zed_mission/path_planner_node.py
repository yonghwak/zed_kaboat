import os
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from zed_common import config
from zed_common import path_planner as pp


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('zed_path_planner_node')
        self.latest_pose_xy = (0.0, 0.0)
        self.prev_xy = None
        self._targets_mtime = None
        self.path = pp.load_path()
        self.current_gate_idx = pp.load_progress().get("current_gate_idx", 0)

        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_service(Trigger, 'plan_path', self.on_plan_path)
        self.create_timer(2.0, self.on_check_targets_changed)
        self.get_logger().info(
            "경로 계획 노드 시작. 부표 목록 바뀌면 자동 재계산됨. "
            "수동: ros2 service call /plan_path std_srvs/srv/Trigger")

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        cur_xy = (p.x, p.y)
        self.latest_pose_xy = cur_xy

        if self.path and self.current_gate_idx < len(self.path) and self.prev_xy is not None:
            gate = self.path[self.current_gate_idx]
            if 'left_x' in gate:
                gA = (gate['left_x'], gate['left_y'])
                gB = (gate['right_x'], gate['right_y'])
                crossed = pp.segments_intersect(self.prev_xy, cur_xy, gA, gB)
            else:
                crossed = math.hypot(gate['x'] - p.x, gate['y'] - p.y) < config.GATE_REACHED_DIST_M
            if crossed:
                self.current_gate_idx = min(self.current_gate_idx + 1, len(self.path))
                pp.save_progress(self.current_gate_idx)
                self.get_logger().info(f"게이트 통과: {gate['name']} ({self.current_gate_idx}/{len(self.path)})")

        self.prev_xy = cur_xy

    def _replan(self):
        targets = pp.load_targets()
        path = pp.build_path(self.latest_pose_xy, targets)
        pp.save_path(path)
        self.path = path
        self.current_gate_idx = 0
        self.prev_xy = None
        pp.save_progress(0)
        self.get_logger().info(f"경로 계산 완료: 게이트 {len(path)}개 (진행상황 초기화됨)")
        return path

    def on_plan_path(self, request, response):
        path = self._replan()
        response.success = True
        response.message = f"게이트 {len(path)}개로 경로 저장됨"
        return response

    def on_check_targets_changed(self):
        try:
            mtime = os.path.getmtime(config.MISSION_TARGETS_PATH)
        except OSError:
            return
        if mtime != self._targets_mtime:
            self._targets_mtime = mtime
            self._replan()


def main():
    rclpy.init()
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
