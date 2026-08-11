"""
mission_targets 중 "zone_<순번>_<종류>" 이름의 목표를 존으로 해석해서
순서대로: 이동(goto) -> 도착 후 스캔(scan, 제자리 회전) -> 해당 미션 수행 -> 다음 존.

지원 종류: stationkeep(위치유지), gate(항로추종, 반응형), dock(도킹), search/circle(주회),
obstacle/avoid(장애물회피, 목표점까지 gap follower로 회피 주행).

장애물회피(obstacle/avoid)는 다른 타입처럼 도착->스캔->수행 3단계가 아니라, zone
좌표(목표점) 방향으로 "회피하며 이동" 자체가 미션이라 도착하면 바로 완료 처리.

게이트(gate)는 사전계획 경로가 아니라 helm_node가 매 tick 실시간으로 전방 빨강/초록
쌍을 찾아 중앙선 추종하는 반응형 방식 - 여기서는 그냥 모드만 켜두고, 완료 판정은
gate_progress.json(helm_node가 "더 이상 게이트 안 보임" 감지 시 success=True 기록)으로 확인.

위치유지/탐색은 zone 좌표를 그대로 쓰지 않고, zone 근처(ZONE_TARGET_MATCH_RADIUS_M
이내)에서 실시간 감지된 buoy_*가 있으면 그걸 우선 사용한다(_resolve_live_target).
못 찾으면 zone 좌표 자체로 폴백 - 매 tick 재확인하므로 스캔 중 부표가 잡히는
순간 자동으로 실시간 타겟으로 전환된다.
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
        self.mission_params = {"docking_color": None, "docking_shape": None, "search_color": "red"}

        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_subscription(RosString, 'mission_params', self.on_params, MODE_QOS)
        self.create_timer(0.5, self.on_tick)
        self.get_logger().info(
            "미션 매니저 시작 - zone_<순번>_<종류> 이름의 목표를 순서대로 수행")

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.latest_pose = (p.x, p.y)

    def on_params(self, msg: RosString):
        try:
            parsed = json.loads(msg.data)
            self.mission_params.update(parsed)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("mission_params 메시지 파싱 실패, 이전 값 유지")

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

    def _resolve_live_target(self, zone, required_color=None):
        """zone 좌표(매핑 때 등록한 대략적 위치) 근처에서 실시간 감지된 buoy_*가 있으면
        그 이름을 반환(우선), 없으면 zone 자체 이름을 반환(폴백 - 이미 mission_targets에
        좌표가 있는 유효한 타겟이라 그대로 station_target으로 써도 동작함).
        required_color가 주어지면 그 색상의 buoy만 후보로 본다 (예: 탐색 존에 빨강/초록/흰색
        3개가 다 있을 때, 대회측이 공지한 색만 골라야 하므로)."""
        targets = mt.load_targets()
        best_name, best_d = None, None
        for name, t in targets.items():
            if not name.startswith("buoy_"):
                continue
            if required_color is not None and not name.startswith(f"buoy_{required_color}_"):
                continue
            d = math.hypot(t['x'] - zone['x'], t['y'] - zone['y'])
            if d <= config.ZONE_TARGET_MATCH_RADIUS_M and (best_d is None or d < best_d):
                best_name, best_d = name, d
        return best_name or zone['name']

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

        # 장애물회피(미션5)는 존형태(도착->스캔->수행)가 아니라 "목표점까지 회피하며
        # 이동" 자체가 미션이라, traveling/scanning/executing 상태분기 없이 바로 처리.
        # 도착=미션 완료.
        if zone['type'].startswith('obstacle') or zone['type'].startswith('avoid'):
            self._publish_mode("avoid_to_goal", station_target=zone['name'])
            if dist < config.MISSION_ARRIVE_DIST_M:
                self.get_logger().info(f"{zone['name']} 장애물 구간 통과 완료 - 다음 존으로")
                self._advance()
            return

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
                live_target = self._resolve_live_target(zone)
                self._publish_mode("station_keep", station_target=live_target)
                prog = ms.load_station_progress()
                if prog.get('success') and prog.get('target') == live_target:
                    self.get_logger().info(f"{zone['name']} 위치유지 성공 - 다음 존으로")
                    self._advance()
            elif mtype.startswith('gate'):
                self._publish_mode("gate_follow")
                prog = ms.load_gate_progress()
                if prog.get('success'):
                    self.get_logger().info(f"{zone['name']} 게이트 구간 종료(더 이상 안 보임) - 다음 존으로")
                    self._advance()
            elif mtype.startswith('dock'):
                self._publish_mode("dock", station_target=zone['name'])
                prog = ms.load_docking_progress()
                if prog.get('docked'):
                    self.get_logger().info(f"{zone['name']} 도킹 성공 - 다음 존으로")
                    self._advance()
            elif mtype.startswith('search') or mtype.startswith('circle'):
                search_color = self.mission_params.get("search_color")
                live_target = self._resolve_live_target(zone, required_color=search_color)
                self._publish_mode("search", station_target=live_target)
                prog = ms.load_search_progress()
                if prog.get('success') and prog.get('target') == live_target:
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
