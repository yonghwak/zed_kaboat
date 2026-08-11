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
BUOY_DEDUP_DIST_M = 3.0  # 좁은 실내 테스트에서 중복 등록 줄이려고 상향 (야외 대회장에선 재조정 필요할 수 있음)      # 이 거리 안에 이미 등록된 부표 있으면 중복 등록 안 함

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

# ---- 위치유지 (Station Keeping) ----
MISSION_MODE_PATH = os.path.join(DATA_DIR, "mission_mode.json")
STATION_KEEP_PROGRESS_PATH = os.path.join(DATA_DIR, "station_keep_progress.json")
STATION_KEEP_RADIUS_M = 5.0       # 이 반경 안에 있어야 유지 중으로 인정
STATION_KEEP_HOLD_SEC = 5.0       # 연속으로 이만큼 버티면 성공
STATION_KEEP_DEADBAND_M = 1.0     # 이 안쪽이면 그냥 정지(미세 보정 안 함)
STATION_KEEP_LINEAR_MAX = 0.3
STATION_KEEP_KP_ANGULAR = 1.0

# ---- 안전 감시(상시 장애물 회피) ----
SAFETY_HARD_STOP_DIST_M = 0.8   # 이 안쪽이면 진짜 위급 - 완전정지+급선회
SAFETY_STOP_DIST_M = 2.0        # 이 안쪽부터 회피 조향 시작 (전진은 유지)
SAFETY_SLOW_DIST_M = 4.0        # 이 안쪽부터 감속만
SAFETY_CORRIDOR_HALF_WIDTH_M = 1.0
SAFETY_MAX_AVOID_TURN = 1.0
SAFETY_AVOID_LINEAR_SCALE = 0.4  # 회피 중 전진속도 비율
SAFETY_CMD_TIMEOUT_SEC = 0.5
SAFETY_POINTCLOUD_STRIDE = 8

# ---- 미션 매니저 (존 기반) ----
MISSION_ZONE_RADIUS_M = 6.0
MISSION_ARRIVE_DIST_M = 2.0
MISSION_ZONE_SCAN_SEC = 5.0
MISSION_ZONE_SCAN_ANGULAR = 0.4
MISSION_ZONE_STUB_DWELL_SEC = 3.0
MISSION_MANAGER_PROGRESS_PATH = os.path.join(DATA_DIR, "mission_manager_progress.json")

# ---- 도킹 (지정된 색/모양 마커에 접근) ----
DOCKING_TARGET_PATH = os.path.join(DATA_DIR, "docking_target.json")
DOCKING_PROGRESS_PATH = os.path.join(DATA_DIR, "docking_progress.json")
DOCKING_ARRIVE_DIST_M = 0.8
DOCKING_LINEAR_MAX = 0.25
DOCKING_KP_ANGULAR = 1.5

# ---- 탐색 (부표 주회) ----
SEARCH_PROGRESS_PATH = os.path.join(DATA_DIR, "search_progress.json")
SEARCH_RADIUS_M = 3.0
SEARCH_LINEAR_MAX = 0.3
SEARCH_KP_ANGULAR = 1.2
SEARCH_RADIAL_KP = 0.3
SEARCH_LAPS_TARGET = 1.0

# ---- 타겟 오탐 필터링 (연속 감지 확정) ----
TARGET_CONFIRM_HITS = 3          # 같은 위치에서 이만큼 연속 감지되어야 실제 등록
TARGET_CANDIDATE_TIMEOUT_SEC = 4.0  # 이 시간 동안 재감지 안 되면 후보 폐기
