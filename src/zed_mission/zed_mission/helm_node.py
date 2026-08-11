"""
mission_mode 토픽(transient_local)으로 받은 모드에 따라 cmd_final(Twist)을 다르게 발행한다.
- gate_follow: mission_path.json의 현재 게이트를 향해 이동 (기존 동작)
- station_keep: 지정된 목표 반경(5m) 안에서 5초 연속 대기, 벗어나면 살짝 복귀
- search: 지정된 목표를 반경만큼 거리를 두고 원형으로 주회, 1바퀴 채우면 성공
- idle: 정지

모드값은 더 이상 제어 루프에서 파일로 폴링하지 않고 mission_mode 토픽 구독으로 받는다
(mission_manager_node / dashboard_node가 publish). 대시보드 표시용으로 파일에도
같이 저장되지만, helm_node는 그 파일을 읽지 않는다.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String as RosString

from zed_common import config
from zed_common import path_planner as pp
from zed_common import mission_targets as mt
from zed_common import mission_state as ms

MODE_QOS = QoSProfile(depth=1)
MODE_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MODE_QOS.reliability = QoSReliabilityPolicy.RELIABLE


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HelmNode(Node):
    def __init__(self):
        super().__init__('zed_helm_node')
        self.latest_pose = None  # (x, y, yaw)
        self.last_pose_time = None  # rclpy Time or None - pose 최신성 체크용
        self.station_hold_start = None  # rclpy Time or None
        self.dock_hold_start = None  # rclpy Time or None
        self.mode_data = {"mode": "gate_follow", "station_target": None}
        self.mission_params = {"docking_color": None, "docking_shape": None, "search_color": "red"}

        self.pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_subscription(RosString, 'mission_mode', self.on_mode, MODE_QOS)
        self.create_subscription(RosString, 'mission_params', self.on_params, MODE_QOS)
        self.create_timer(1.0 / config.HELM_RATE_HZ, self.on_control_tick)

        self.get_logger().info("조향 노드 시작")

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.latest_pose = (p.x, p.y, yaw)
        self.last_pose_time = self.get_clock().now()

    def on_mode(self, msg: RosString):
        try:
            self.mode_data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("mission_mode 메시지 파싱 실패, 이전 모드 유지")

    def on_params(self, msg: RosString):
        try:
            parsed = json.loads(msg.data)
            self.mission_params.update(parsed)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("mission_params 메시지 파싱 실패, 이전 값 유지")

    def _bearing_dist(self, x, y, yaw, tx, ty):
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        return bearing, dist

    def on_control_tick(self):
        if self.latest_pose is None:
            return
        # VIO pose가 최근에 갱신 안 됐으면(예: LOST) 낡은 위치로 계속 조향하지 말고 정지.
        # 재수신되면 last_pose_time이 다시 최신화되니 자동 복귀됨.
        elapsed_since_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if elapsed_since_pose > config.POSE_STALE_TIMEOUT_SEC:
            self.station_hold_start = None
            self.dock_hold_start = None
            self.pub.publish(Twist())
            self.get_logger().warn("pose 갱신 끊김(VIO 문제 의심) - 정지", throttle_duration_sec=1.0)
            return
        x, y, yaw = self.latest_pose
        mode_data = self.mode_data
        mode = mode_data.get("mode", "gate_follow")

        if mode == "station_keep":
            self._tick_station_keep(x, y, yaw, mode_data.get("station_target"))
        elif mode == "gate_follow":
            self._tick_gate_follow(x, y, yaw)
        elif mode == "goto":
            self._tick_goto(x, y, yaw, mode_data.get("station_target"))
        elif mode == "search":
            self._tick_search(x, y, yaw, mode_data.get("station_target"))
        elif mode == "dock":
            self._tick_dock(x, y, yaw, mode_data.get("station_target"))
        elif mode == "scan":
            self.station_hold_start = None
            self.dock_hold_start = None
            twist = Twist()
            twist.angular.z = config.MISSION_ZONE_SCAN_ANGULAR
            self.pub.publish(twist)
        else:
            self.station_hold_start = None
            self.dock_hold_start = None
            self.pub.publish(Twist())

    def _tick_goto(self, x, y, yaw, target_name):
        self.station_hold_start = None
        self.dock_hold_start = None
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
        self.dock_hold_start = None
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
        self.dock_hold_start = None
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

    def _tick_search(self, x, y, yaw, target_name):
        """target_name을 중심으로 SEARCH_RADIUS_M 거리를 두고 원형 주회.
        진행한 각도 누적이 360도(1랩)를 넘기면 success=True.
        회전 방향: 규정상 빨강/초록 부표=시계방향, 흰색 부표=반시계방향.
        target_name이 perception_bridge가 등록한 "buoy_<색>_<n>"이면 색상을 그대로 쓰고,
        아니면(수동 등록된 zone 마커 등) 대시보드에서 지정한 search_color를 씀.
        cw일 때 아래 base_tangent 부호가 실제로 시계방향이 맞는지 실측 필수 —
        반대로 돌면 SEARCH_DIRECTION_BY_COLOR 값을 뒤집거나 부호를 반전할 것."""
        self.station_hold_start = None
        self.dock_hold_start = None
        targets = mt.load_targets()
        twist = Twist()

        if not target_name or target_name not in targets:
            ms.save_search_progress(0.0, 0.0, False, target_name)
            self.pub.publish(twist)
            return

        if target_name.startswith("buoy_"):
            color = target_name.split('_')[1]
        else:
            color = self.mission_params.get("search_color", "red")
        direction = config.SEARCH_DIRECTION_BY_COLOR.get(color, "cw")

        t = targets[target_name]
        bearing, dist = self._bearing_dist(x, y, yaw, t['x'], t['y'])
        # 목표 기준으로 배가 지금 어느 각위치에 있는지 (월드 프레임 기준)
        raw_angle = math.degrees(math.atan2(y - t['y'], x - t['x']))

        prog = ms.load_search_progress()
        if prog.get('target') != target_name:
            prog = {"angle_deg": raw_angle, "laps": 0.0, "success": False, "target": target_name}

        delta = (raw_angle - prog.get('angle_deg', raw_angle) + 180) % 360 - 180
        laps = prog.get('laps', 0.0) + abs(delta) / 360.0
        success = laps >= config.SEARCH_LAPS_TARGET
        ms.save_search_progress(raw_angle, laps, success, target_name)

        radial_error = dist - config.SEARCH_RADIUS_M
        base_tangent = bearing - math.pi / 2.0 + config.SEARCH_RADIAL_KP * radial_error
        tangent_bearing = base_tangent if direction == "cw" else -base_tangent

        twist.linear.x = config.SEARCH_LINEAR_MAX
        twist.angular.z = max(-config.HELM_ANGULAR_MAX,
                               min(config.HELM_ANGULAR_MAX,
                                   config.SEARCH_KP_ANGULAR * tangent_bearing))
        self.pub.publish(twist)

    def _tick_dock(self, x, y, yaw, zone_hint=None):
        """docking_target.json(대시보드에서 대회 당일 공지된 색/모양으로 지정)과 일치하는 표식에 접근.
        모양이 circle이면 perception_bridge가 buoy_<색>_<n>으로 등록하므로 그쪽도 같이 찾는다
        (그 외 모양은 dock_<색>_<모양>_<n>). zone_hint(존 이름)가 주어지면 그 존 좌표에 가장 가까운
        후보를 우선 선택 (여러 도킹 스테이션 중 지금 이 존의 것을 정확히 집기 위함), 없으면 배 현재
        위치 기준 최근접. DOCKING_ARRIVE_DIST_M 이내에서 DOCKING_HOLD_SEC 이상 연속 정지하면 도킹 완료."""
        self.station_hold_start = None
        twist = Twist()

        dt = self.mission_params
        color, shape = dt.get('docking_color'), dt.get('docking_shape')
        if not color or not shape:
            self.dock_hold_start = None
            ms.save_docking_progress(False, None)
            self.pub.publish(twist)
            return

        targets = mt.load_targets()
        prefix = f"buoy_{color}" if shape == "circle" else f"dock_{color}_{shape}"
        candidates = {n: t for n, t in targets.items() if n.startswith(prefix)}
        if not candidates:
            self.dock_hold_start = None
            ms.save_docking_progress(False, None)
            self.pub.publish(twist)
            return

        hint = targets.get(zone_hint) if zone_hint else None
        if hint is not None:
            ref_x, ref_y = hint['x'], hint['y']
        else:
            ref_x, ref_y = x, y
        name, t = min(candidates.items(), key=lambda kv: math.hypot(kv[1]['x'] - ref_x, kv[1]['y'] - ref_y))
        bearing, dist = self._bearing_dist(x, y, yaw, t['x'], t['y'])

        now = self.get_clock().now()
        if dist <= config.DOCKING_ARRIVE_DIST_M:
            if self.dock_hold_start is None:
                self.dock_hold_start = now
            held_sec = (now - self.dock_hold_start).nanoseconds / 1e9
        else:
            self.dock_hold_start = None
            held_sec = 0.0

        docked = held_sec >= config.DOCKING_HOLD_SEC
        ms.save_docking_progress(docked, name)

        if docked or dist < config.DOCKING_ARRIVE_DIST_M * 0.5:
            self.pub.publish(twist)  # 표식 바로 앞이면 정지 유지 (과충돌 방지)
            return

        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.DOCKING_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        twist.linear.x = config.DOCKING_LINEAR_MAX * forward_scale
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
