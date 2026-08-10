import os

DATA_DIR = os.path.expanduser("~/zed_dashboard_data")
os.makedirs(DATA_DIR, exist_ok=True)

FLOOR_PLAN_IMAGE_PATH = os.path.join(DATA_DIR, "floor_plan.png")
FLOOR_PLAN_META_PATH = os.path.join(DATA_DIR, "floor_plan_meta.json")
MISSION_TARGETS_PATH = os.path.join(DATA_DIR, "mission_targets.json")

MAP_RESOLUTION_M_PER_PX = 0.02
MAP_MARGIN_M = 1.0
Z_MIN_M = 0.05
Z_MAX_M = 2.0
AUTOSAVE_INTERVAL_SEC = 3.0
OBSTACLE_HIT_CAP = 6
POSE_HEADING_LEN_M = 0.3

POSE_DOT_RADIUS_PX = 5
PATH_TRAIL_MAX_POINTS = 2000
TARGET_MARKER_COLOR = (0, 255, 0)
ARROW_COLOR = (0, 0, 255)
DEFAULT_TARGET_HEIGHT_M = 0.0

POINTCLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"
POSE_TOPIC = "/zed/zed_node/pose"
MAP_FRAME = "map"

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080
STREAM_FPS = 10

# ---- 부표 탐지 (색깔/모양 기반) ----
# HSV 범위는 실제 부표 색 보면서 튜닝 필요. 빨강은 Hue가 0 근처와 180 근처 둘 다 걸쳐서 두 개로 나눠둠.
BUOY_COLOR_RANGES = {
    "buoy_red_a": ((0, 120, 70), (10, 255, 255)),
    "buoy_red_b": ((170, 120, 70), (180, 255, 255)),
    "buoy_green": ((40, 70, 70), (85, 255, 255)),
    "buoy_yellow": ((20, 100, 100), (32, 255, 255)),
}
BUOY_MIN_AREA_PX = 150       # 이보다 작은 덩어리는 무시
BUOY_MIN_CIRCULARITY = 0.55  # 1.0에 가까울수록 원에 가까움 (부표 모양 필터)
BUOY_DEDUP_DIST_M = 1.5      # 이 거리 안에 이미 등록된 부표 있으면 중복 등록 안 함

# ---- 경로 계획 (부표 게이트) ----
MISSION_PATH_PATH = os.path.join(DATA_DIR, "mission_path.json")
GATE_LEFT_COLOR = "red"
GATE_RIGHT_COLOR = "green"
GATE_MAX_PAIR_DIST_M = 8.0    # 이보다 멀면 같은 게이트로 안 묶음
GATE_REACHED_DIST_M = 2.0     # 이 거리 안으로 들어오면 다음 게이트로 진행
GATE_LINE_COLOR = (255, 0, 255)
GATE_CURRENT_COLOR = (0, 140, 255)
GATE_DONE_COLOR = (160, 160, 160)

# ---- 기존 camera_node 연동 ----
CAMERA_DETECTIONS_TOPIC = "/camera/detections"

# ---- 진행상황 공유 파일 ----
MISSION_PROGRESS_PATH = os.path.join(DATA_DIR, "mission_progress.json")

# ---- 조향(helm) ----
HELM_LINEAR_MAX = 0.6
HELM_ANGULAR_MAX = 1.0
HELM_KP_ANGULAR = 1.2
HELM_RATE_HZ = 10.0
