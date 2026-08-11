"""
mission_mode 토픽(transient_local)으로 받은 모드에 따라 cmd_final(Twist)을 다르게 발행한다.
- gate_follow: 매 tick 실시간으로 전방 빨강/초록 부표쌍을 찾아 중앙선 추종(반응형).
  더 이상 유효 쌍이 안 보이면(GATE_END_TIMEOUT_SEC) 미션 종료로 판정.
- station_keep: 지정된 목표 반경(5m) 안에서 5초 연속 대기, 벗어나면 살짝 복귀
- search: 지정된 목표를 반경만큼 거리를 두고 원형으로 주회, 1바퀴 채우면 성공
- dock: 도킹 표식(색/모양)에 접근, 근접거리에서 일정시간 정지하면 도킹 완료
- avoid_to_goal: 목표점으로 향하되 전방 gap(뚫린 방향) 중 목표방향에 가까운 쪽으로 조향
- idle: 정지

모드값은 더 이상 제어 루프에서 파일로 폴링하지 않고 mission_mode 토픽 구독으로 받는다
(mission_manager_node / dashboard_node가 publish). 대시보드 표시용으로 파일에도
같이 저장되지만, helm_node는 그 파일을 읽지 않는다.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String as RosString
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from zed_common import config
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
        self.gate_last_seen_time = None  # rclpy Time or None - 마지막으로 유효 게이트쌍 본 시각
        self.latest_sectors = None  # 미션5용 전방 클리어런스 섹터 리스트 (or None)

        self.pub = self.create_publisher(Twist, 'cmd_mission', 10)
        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_subscription(RosString, 'mission_mode', self.on_mode, MODE_QOS)
        self.create_subscription(RosString, 'mission_params', self.on_params, MODE_QOS)
        self.create_subscription(PointCloud2, config.POINTCLOUD_TOPIC, self.on_cloud, 5)
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

    def on_cloud(self, msg: PointCloud2):
        """미션5(avoid_to_goal)용 전방 클리어런스 섹터 계산.
        safety_supervisor와 별개로 구독 - 여기서는 gap(뚫린 방향) 탐색용이고,
        safety_supervisor는 최종 안전망으로 그쪽에서 따로 판단해서 cmd_final을 덮어씀."""
        pts_struct = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts_struct.size == 0:
            self.latest_sectors = None
            return
        stride = config.SAFETY_POINTCLOUD_STRIDE
        x_ = pts_struct['x'][::stride]
        y_ = pts_struct['y'][::stride]
        z_ = pts_struct['z'][::stride]
        valid = (np.isfinite(x_) & np.isfinite(y_) & np.isfinite(z_)
                 & (z_ > config.Z_MIN_M) & (z_ < config.Z_MAX_M)
                 & (x_ > 0) & (x_ < config.AVOID_MAX_RANGE_M))
        x_, y_ = x_[valid], y_[valid]
        n = config.AVOID_SECTORS
        if x_.size == 0:
            self.latest_sectors = [config.AVOID_MAX_RANGE_M] * n
            return
        # x=전방, y=좌측(양수) 기준 (safety_supervisor와 동일 관례) - atan2(y,x) 양수=왼쪽
        angles = np.arctan2(y_, x_)
        ranges = np.hypot(x_, y_)
        half_fov = math.radians(config.AVOID_FOV_DEG / 2.0)
        edges = np.linspace(-half_fov, half_fov, n + 1)
        sectors = []
        for i in range(n):
            in_sector = (angles >= edges[i]) & (angles < edges[i + 1])
            sectors.append(float(ranges[in_sector].min()) if np.any(in_sector) else config.AVOID_MAX_RANGE_M)
        self.latest_sectors = sectors

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
        elif mode == "avoid_to_goal":
            self._tick_avoid_to_goal(x, y, yaw, mode_data.get("station_target"))
        elif mode == "scan":
            self.station_hold_start = None
            self.dock_hold_start = None
            self.gate_last_seen_time = None
            twist = Twist()
            twist.angular.z = config.MISSION_ZONE_SCAN_ANGULAR
            self.pub.publish(twist)
        else:
            self.station_hold_start = None
            self.dock_hold_start = None
            self.gate_last_seen_time = None
            self.pub.publish(Twist())

    def _tick_goto(self, x, y, yaw, target_name):
        self.station_hold_start = None
        self.dock_hold_start = None
        self.gate_last_seen_time = None
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
        """path_planner 기반 사전계획 대신, 매 tick마다 등록된 buoy_red_*/buoy_green_* 중
        전방(GATE_FRONT_CONE_DEG 안)에 있고 서로 GATE_MAX_PAIR_DIST_M 이내로 가까운 쌍을
        실시간으로 찾아 그 중앙점으로 조향한다. 가장 가까운(중앙점 거리 기준) 쌍을 선택.
        GATE_END_TIMEOUT_SEC 동안 연속으로 유효 쌍이 안 보이면 미션 종료(success)로 판정 —
        바다처럼 넓은 구간에서 게이트가 멀리서부터 안 보일 수 있어서, 사전 경로 대신
        실시간 인식 기반으로 동작하는 게 더 맞음."""
        self.station_hold_start = None
        self.dock_hold_start = None
        targets = mt.load_targets()
        twist = Twist()

        reds = {n: t for n, t in targets.items() if n.startswith("buoy_red_")}
        greens = {n: t for n, t in targets.items() if n.startswith("buoy_green_")}

        half_cone = math.radians(config.GATE_FRONT_CONE_DEG / 2.0)
        best_pair = None
        best_mid_dist = None
        for rn, rt in reds.items():
            r_bearing, r_dist = self._bearing_dist(x, y, yaw, rt['x'], rt['y'])
            if r_dist > config.GATE_MAX_CONSIDER_DIST_M or abs(r_bearing) > half_cone:
                continue
            for gn, gt in greens.items():
                g_bearing, g_dist = self._bearing_dist(x, y, yaw, gt['x'], gt['y'])
                if g_dist > config.GATE_MAX_CONSIDER_DIST_M or abs(g_bearing) > half_cone:
                    continue
                pair_width = math.hypot(rt['x'] - gt['x'], rt['y'] - gt['y'])
                if pair_width > config.GATE_MAX_PAIR_DIST_M:
                    continue
                mid_dist = (r_dist + g_dist) / 2.0
                if best_mid_dist is None or mid_dist < best_mid_dist:
                    best_mid_dist = mid_dist
                    best_pair = (rt, gt)

        now = self.get_clock().now()
        if best_pair is None:
            if self.gate_last_seen_time is None:
                self.gate_last_seen_time = now  # 시작부터 안 보였으면 지금부터 카운트 시작
            no_gate_sec = (now - self.gate_last_seen_time).nanoseconds / 1e9
            success = no_gate_sec >= config.GATE_END_TIMEOUT_SEC
            ms.save_gate_progress(success)
            if success:
                self.pub.publish(twist)  # 미션 종료 판정 - 정지, 다음 미션 전환은 mission_manager가 처리
                return
            # 타임아웃 전이면 마지막 알던 방향(직전 twist 유지 대신) 감속 직진하며 재탐색 대기
            twist.linear.x = config.HELM_LINEAR_MAX * 0.3
            self.pub.publish(twist)
            return

        self.gate_last_seen_time = now
        ms.save_gate_progress(False)
        rt, gt = best_pair
        mid_x = (rt['x'] + gt['x']) / 2.0
        mid_y = (rt['y'] + gt['y']) / 2.0
        bearing, dist = self._bearing_dist(x, y, yaw, mid_x, mid_y)
        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.HELM_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        twist.linear.x = config.HELM_LINEAR_MAX * forward_scale
        twist.angular.z = angular
        self.pub.publish(twist)

    def _tick_station_keep(self, x, y, yaw, target_name):
        self.dock_hold_start = None
        self.gate_last_seen_time = None
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
        self.gate_last_seen_time = None
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
        self.gate_last_seen_time = None
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

    def _find_gaps(self, sectors):
        """섹터 클리어런스 리스트에서 연속으로 열려있는(AVOID_MIN_GAP_CLEARANCE_M 이상)
        구간들을 찾아 각 구간의 중심 각도를 반환. 사방이 막혀있으면 빈 리스트."""
        n = config.AVOID_SECTORS
        half_fov = math.radians(config.AVOID_FOV_DEG / 2.0)
        edges = [-half_fov + i * (2 * half_fov / n) for i in range(n + 1)]
        centers = [(edges[i] + edges[i + 1]) / 2.0 for i in range(n)]
        open_flags = [s >= config.AVOID_MIN_GAP_CLEARANCE_M for s in sectors]

        gaps = []
        i = 0
        while i < n:
            if open_flags[i]:
                j = i
                while j < n and open_flags[j]:
                    j += 1
                gaps.append({'start': i, 'end': j - 1, 'center': (centers[i] + centers[j - 1]) / 2.0})
                i = j
            else:
                i += 1
        return gaps

    def _tick_avoid_to_goal(self, x, y, yaw, target_name):
        """target_name(장애물구간 출구/최종 목표 좌표)으로 향하되, 전방에 막힌 구간이 있으면
        열린 gap(뚫린 방향) 중 목표방향에 가장 가까운 쪽으로 조향(gap follower).
        safety_supervisor가 이 뒤에서 항상 최종 안전망으로 급박한 상황을 따로 덮어쓰니,
        여기서는 "어느 쪽으로 가는 게 그럴듯한가"만 판단하면 됨 - 완벽한 안전 판단은
        safety_supervisor 몫."""
        self.station_hold_start = None
        self.dock_hold_start = None
        self.gate_last_seen_time = None
        targets = mt.load_targets()
        twist = Twist()

        if not target_name or target_name not in targets:
            self.pub.publish(twist)
            return

        t = targets[target_name]
        goal_bearing, dist = self._bearing_dist(x, y, yaw, t['x'], t['y'])

        chosen_bearing = goal_bearing
        if self.latest_sectors:
            gaps = self._find_gaps(self.latest_sectors)
            if gaps:
                # 목표 방향과 각도차가 가장 작은 gap을 선택 (목표방향이 이미 열려있으면 그대로 감)
                best_gap = min(gaps, key=lambda g: abs(
                    (g['center'] - goal_bearing + math.pi) % (2 * math.pi) - math.pi))
                chosen_bearing = best_gap['center']
            # gaps가 비어있으면(사방이 막힘) 목표방향 유지 - safety_supervisor가 감속/급선회 처리

        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.AVOID_KP_ANGULAR * chosen_bearing))
        forward_scale = max(0.0, math.cos(chosen_bearing))
        linear = config.AVOID_LINEAR_MAX * forward_scale
        if dist < config.MISSION_ARRIVE_DIST_M:
            linear *= 0.3
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

class HelmNode(Node):
    def __init__(self):
        super().__init__('zed_helm_node')
        self.latest_pose = None  # (x, y, yaw)
        self.last_pose_time = None  # rclpy Time or None - pose 최신성 체크용
        self.station_hold_start = None  # rclpy Time or None
        self.dock_hold_start = None  # rclpy Time or None
        self.mode_data = {"mode": "gate_follow", "station_target": None}
        self.mission_params = {"docking_color": None, "docking_shape": None, "search_color": "red"}
        self.gate_last_seen_time = None  # rclpy Time or None - 마지막으로 유효 게이트쌍 본 시각

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
            self.gate_last_seen_time = None
            twist = Twist()
            twist.angular.z = config.MISSION_ZONE_SCAN_ANGULAR
            self.pub.publish(twist)
        else:
            self.station_hold_start = None
            self.dock_hold_start = None
            self.gate_last_seen_time = None
            self.pub.publish(Twist())

    def _tick_goto(self, x, y, yaw, target_name):
        self.station_hold_start = None
        self.dock_hold_start = None
        self.gate_last_seen_time = None
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
        """path_planner 기반 사전계획 대신, 매 tick마다 등록된 buoy_red_*/buoy_green_* 중
        전방(GATE_FRONT_CONE_DEG 안)에 있고 서로 GATE_MAX_PAIR_DIST_M 이내로 가까운 쌍을
        실시간으로 찾아 그 중앙점으로 조향한다. 가장 가까운(중앙점 거리 기준) 쌍을 선택.
        GATE_END_TIMEOUT_SEC 동안 연속으로 유효 쌍이 안 보이면 미션 종료(success)로 판정 —
        바다처럼 넓은 구간에서 게이트가 멀리서부터 안 보일 수 있어서, 사전 경로 대신
        실시간 인식 기반으로 동작하는 게 더 맞음."""
        self.station_hold_start = None
        self.dock_hold_start = None
        targets = mt.load_targets()
        twist = Twist()

        reds = {n: t for n, t in targets.items() if n.startswith("buoy_red_")}
        greens = {n: t for n, t in targets.items() if n.startswith("buoy_green_")}

        half_cone = math.radians(config.GATE_FRONT_CONE_DEG / 2.0)
        best_pair = None
        best_mid_dist = None
        for rn, rt in reds.items():
            r_bearing, r_dist = self._bearing_dist(x, y, yaw, rt['x'], rt['y'])
            if r_dist > config.GATE_MAX_CONSIDER_DIST_M or abs(r_bearing) > half_cone:
                continue
            for gn, gt in greens.items():
                g_bearing, g_dist = self._bearing_dist(x, y, yaw, gt['x'], gt['y'])
                if g_dist > config.GATE_MAX_CONSIDER_DIST_M or abs(g_bearing) > half_cone:
                    continue
                pair_width = math.hypot(rt['x'] - gt['x'], rt['y'] - gt['y'])
                if pair_width > config.GATE_MAX_PAIR_DIST_M:
                    continue
                mid_dist = (r_dist + g_dist) / 2.0
                if best_mid_dist is None or mid_dist < best_mid_dist:
                    best_mid_dist = mid_dist
                    best_pair = (rt, gt)

        now = self.get_clock().now()
        if best_pair is None:
            if self.gate_last_seen_time is None:
                self.gate_last_seen_time = now  # 시작부터 안 보였으면 지금부터 카운트 시작
            no_gate_sec = (now - self.gate_last_seen_time).nanoseconds / 1e9
            success = no_gate_sec >= config.GATE_END_TIMEOUT_SEC
            ms.save_gate_progress(success)
            if success:
                self.pub.publish(twist)  # 미션 종료 판정 - 정지, 다음 미션 전환은 mission_manager가 처리
                return
            # 타임아웃 전이면 마지막 알던 방향(직전 twist 유지 대신) 감속 직진하며 재탐색 대기
            twist.linear.x = config.HELM_LINEAR_MAX * 0.3
            self.pub.publish(twist)
            return

        self.gate_last_seen_time = now
        ms.save_gate_progress(False)
        rt, gt = best_pair
        mid_x = (rt['x'] + gt['x']) / 2.0
        mid_y = (rt['y'] + gt['y']) / 2.0
        bearing, dist = self._bearing_dist(x, y, yaw, mid_x, mid_y)
        angular = max(-config.HELM_ANGULAR_MAX, min(config.HELM_ANGULAR_MAX,
                                                      config.HELM_KP_ANGULAR * bearing))
        forward_scale = max(0.0, math.cos(bearing))
        twist.linear.x = config.HELM_LINEAR_MAX * forward_scale
        twist.angular.z = angular
        self.pub.publish(twist)

    def _tick_station_keep(self, x, y, yaw, target_name):
        self.dock_hold_start = None
        self.gate_last_seen_time = None
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
        self.gate_last_seen_time = None
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
        self.gate_last_seen_time = None
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
