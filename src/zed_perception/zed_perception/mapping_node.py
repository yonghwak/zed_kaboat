import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool
from collections import defaultdict

from zed_common import config
from zed_common.coord_utils import FloorPlanMeta


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w),     2*(x*z+y*w)],
        [2*(x*y+z*w),     1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w),     1 - 2*(x*x+y*y)],
    ])


class FloorMapper:
    def __init__(self, resolution=config.MAP_RESOLUTION_M_PER_PX):
        self.resolution = resolution
        self._hits = defaultdict(int)
        self._bounds = None

    def add_points(self, xyz_world):
        if xyz_world.size == 0:
            return
        x, y, z = xyz_world[:, 0], xyz_world[:, 1], xyz_world[:, 2]
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) \
            & (z > config.Z_MIN_M) & (z < config.Z_MAX_M)
        x, y = x[valid], y[valid]
        if x.size == 0:
            return
        gu = np.floor(x / self.resolution).astype(np.int64)
        gv = np.floor(y / self.resolution).astype(np.int64)
        for a, b in zip(gu, gv):
            self._hits[(int(a), int(b))] += 1
        b_ = [int(gu.min()), int(gu.max()), int(gv.min()), int(gv.max())]
        if self._bounds is None:
            self._bounds = b_
        else:
            self._bounds[0] = min(self._bounds[0], b_[0])
            self._bounds[1] = max(self._bounds[1], b_[1])
            self._bounds[2] = min(self._bounds[2], b_[2])
            self._bounds[3] = max(self._bounds[3], b_[3])

    def has_data(self):
        return len(self._hits) > 0

    def save_hits(self, path):
        """히트그리드 원본을 저장 - 렌더링된 png/meta만으로는 이어서 매핑할 수 없어서
        (다시 로드해도 어디에 몇 번 찍혔는지 정보가 없음) 원본 카운트를 따로 보관."""
        data = {
            "resolution": self.resolution,
            "bounds": self._bounds,
            "hits": {f"{u},{v}": c for (u, v), c in self._hits.items()},
        }
        with open(path, "w") as f:
            json.dump(data, f)

    def load_hits(self, path):
        """이전 세션(베스트맵 등)의 히트그리드를 불러와 이어서 누적할 수 있게 시딩.
        해상도가 다르면 좌표계가 어긋나므로 로드하지 않음."""
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return False
        if abs(data.get("resolution", -1) - self.resolution) > 1e-9:
            return False
        self._hits = defaultdict(int)
        for key, c in data.get("hits", {}).items():
            u_str, v_str = key.split(",")
            self._hits[(int(u_str), int(v_str))] = c
        self._bounds = data.get("bounds")
        return len(self._hits) > 0

    def build(self):
        margin = int(round(config.MAP_MARGIN_M / self.resolution))
        gu_min, gu_max, gv_min, gv_max = self._bounds
        gu_min -= margin; gu_max += margin; gv_min -= margin; gv_max += margin
        w, h = gu_max - gu_min + 1, gv_max - gv_min + 1
        grid = np.zeros((h, w), dtype=np.float32)
        for (gu, gv), c in self._hits.items():
            grid[gv - gv_min, gu - gu_min] += c
        # 적은 hit(작은 장애물)도 확실히 보이게 cap 값으로 클램프
        cap = float(config.OBSTACLE_HIT_CAP)
        intensity = np.minimum(grid, cap) / cap
        img = 255 - (intensity * 255).astype(np.uint8)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        meta = FloorPlanMeta(gu_min * self.resolution, gv_min * self.resolution,
                              self.resolution, w, h)
        return img_bgr, meta


class MappingNode(Node):
    def __init__(self):
        super().__init__('zed_floor_mapping_node')
        self.mapper = FloorMapper()
        self.latest_R = None
        self.latest_T = None
        self.active = True

        if self.mapper.load_hits(config.FLOOR_PLAN_HITS_PATH):
            self.get_logger().info(
                "기존 히트그리드(베스트맵 등) 로드 완료 - 이어서 누적합니다.")

        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)
        self.create_subscription(PointCloud2, config.POINTCLOUD_TOPIC, self.on_cloud, 5)
        self.create_service(Trigger, 'save_floor_plan', self.on_save_service)
        self.create_service(SetBool, 'set_mapping_active', self.on_set_active)
        self.create_timer(config.AUTOSAVE_INTERVAL_SEC, self.on_autosave_timer)
        self.get_logger().info(
            f"매핑 시작. {config.POINTCLOUD_TOPIC} 구독 중. "
            f"{config.AUTOSAVE_INTERVAL_SEC}초마다 자동 저장됩니다. "
            f"수동 저장: ros2 service call /save_floor_plan std_srvs/srv/Trigger")

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        self.latest_R = quat_to_rot(o.x, o.y, o.z, o.w)
        self.latest_T = np.array([p.x, p.y, p.z])

    def on_cloud(self, msg: PointCloud2):
        if not self.active:
            return
        if self.latest_R is None:
            self.get_logger().warn("아직 pose를 못 받았습니다.", throttle_duration_sec=2.0)
            return

        pts_struct = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts_struct.size == 0:
            return
        pts = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=-1).astype(np.float32)
        pts = pts[::4]
        world = pts @ self.latest_R.T + self.latest_T
        self.mapper.add_points(world.astype(np.float32))

    def _save(self):
        if not self.mapper.has_data():
            return False
        img, meta = self.mapper.build()
        cv2.imwrite(config.FLOOR_PLAN_IMAGE_PATH, img)
        meta.save(config.FLOOR_PLAN_META_PATH)
        self.mapper.save_hits(config.FLOOR_PLAN_HITS_PATH)
        return True

    def on_autosave_timer(self):
        if self.active:
            self._save()

    def on_set_active(self, request, response):
        self.active = request.data
        response.success = True
        response.message = "매핑 재개" if self.active else "매핑 일시정지 (더 이상 점 안 쌓임)"
        self.get_logger().info(response.message)
        return response

    def on_save_service(self, request, response):
        if self._save():
            response.success = True
            response.message = f"저장 완료: {config.FLOOR_PLAN_IMAGE_PATH}"
        else:
            response.success = False
            response.message = "누적된 포인트가 없습니다."
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
