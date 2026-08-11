"""
mission_targets 중 "zone_<순번>_<종류>" 이름의 목표를 존으로 해석해서
순서대로: 이동(goto) -> 도착 후 스캔(scan, 제자리 회전) -> 해당 미션 수행 -> 다음 존.

지원 종류: stationkeep(위치유지), gate(항로추종 경로), search/circle(주회).
그 외 종류(dock 등)는 아직 미구현이라 잠깐 대기 후 건너뜀.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String as RosString

from zed_common import config
from zed_common import mission_targets as mt
from zed_common import mission_state as ms
from zed_common import path_planner as pp

MODE_QOS = QoSProfile(depth=1)
MODE_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MODE_QOS.reliability = QoSReliabilityPolicy.RELIABLE


def parse_zone(name):
    parts = name.split('_')
    if len(parts) < 3 or parts[0] != 'zone':
        return None
    try:
        seq = int(parts[1])
    except ValueError:
        return None
    return seq, '_'.join(parts[2:])


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('zed_mission_manager_node')
        self.latest_pose = None
        self.zone_index = 0
        self.state = "idle"
        self.state_since = self.get_clock().now()

        self.mode_pub = self.create_publisher(RosString, 'mission_mode', MODE_QOS)

        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_timer(0.5, self.on_tick)
        self.get_logger().info(
            "미션 매니저 시작 - zone_<순번>_<종류> 이름의 목표를 순서대로 수행")

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.latest_pose = (p.x, p.y)

    def _publish_mode(self, mode, station_target=None):
        """helm_node의 실제 제어 입력은 이 토픽. 파일은 대시보드 표시용으로만 계속 저장."""
        ms.save_mode(mode, station_target)
        msg = RosString()
        msg.data = json.dumps({"mode": mode, "station_target": station_target})
        self.mode_pub.publish(msg)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _set_state(self, state):
        self.state = state
        self.state_since = self.get_clock().now()

    def _zones(self):
        targets = mt.load_targets()
        zones = []
        for name, t in targets.items():
            parsed = parse_zone(name)
            if parsed:
                seq, mtype = parsed
                zones.append({"name": name, "seq": seq, "type": mtype, "x": t['x'], "y": t['y']})
        zones.sort(key=lambda z: z['seq'])
        return zones

    def on_tick(self):
        zones = self._zones()
        if self.latest_pose is None or not zones:
            ms.save_manager_progress(self.zone_index, None, self.state)
            return
        if self.zone_index >= len(zones):
            self._publish_mode("idle")
            ms.save_manager_progress(self.zone_index, None, "all_done")
            return

        zone = zones[self.zone_index]
        ms.save_manager_progress(self.zone_index, zone['name'], self.state)

        x, y = self.latest_pose
        dist = math.hypot(zone['x'] - x, zone['y'] - y)

        if self.state == "idle":
            self._set_state("traveling")

        elif self.state == "traveling":
            self._publish_mode("goto", station_target=zone['name'])
            if dist < config.MISSION_ZONE_RADIUS_M:
                self.get_logger().info(f"{zone['name']} 존 도착 - 스캔 시작")
                self._set_state("scanning")

        elif self.state == "scanning":
            self._publish_mode("scan")
            if self._elapsed() > config.MISSION_ZONE_SCAN_SEC:
                self._set_state("executing")

        elif self.state == "executing":
            mtype = zone['type']
            if mtype.startswith('stationkeep'):
                self._publish_mode("station_keep", station_target=zone['name'])
                prog = ms.load_station_progress()
                if prog.get('success') and prog.get('target') == zone['name']:
                    self.get_logger().info(f"{zone['name']} 위치유지 성공 - 다음 존으로")
                    self._advance()
            elif mtype.startswith('gate'):
                self._publish_mode("gate_follow")
                path = pp.load_path()
                progress = pp.load_progress()
                if path and progress.get('current_gate_idx', 0) >= len(path):
                    self.get_logger().info("게이트 경로 완료 - 다음 존으로")
                    self._advance()
            elif mtype.startswith('dock'):
                self._publish_mode("dock")
                prog = ms.load_docking_progress()
                if prog.get('docked'):
                    self.get_logger().info(f"{zone['name']} 도킹 성공 - 다음 존으로")
                    self._advance()
            elif mtype.startswith('search') or mtype.startswith('circle'):
                self._publish_mode("search", station_target=zone['name'])
                prog = ms.load_search_progress()
                if prog.get('success') and prog.get('target') == zone['name']:
                    self.get_logger().info(f"{zone['name']} 탐색(주회) 성공 - 다음 존으로")
                    self._advance()
            else:
                self._publish_mode("idle")
                if self._elapsed() > config.MISSION_ZONE_STUB_DWELL_SEC:
                    self.get_logger().warn(f"{zone['name']} ({mtype}) 은 아직 미구현 - 건너뜀")
                    self._advance()

    def _advance(self):
        self.zone_index += 1
        self._set_state("idle")


def main():
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
