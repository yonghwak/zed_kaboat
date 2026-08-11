import os

DATA_DIR = os.path.expanduser("~/zed_dashboard_data")
os.makedirs(DATA_DIR, exist_ok=True)

FLOOR_PLAN_IMAGE_PATH = os.path.join(DATA_DIR, "floor_plan.png")
FLOOR_PLAN_META_PATH = os.path.join(DATA_DIR, "floor_plan_meta.json")
FLOOR_PLAN_HITS_PATH = os.path.join(DATA_DIR, "floor_plan_hits.json")
MISSION_TARGETS_PATH = os.path.join(DATA_DIR, "mission_targets.json")

# ---- 베스트맵 저장/불러오기 ----
SAVED_MAPS_DIR = os.path.join(DATA_DIR, "saved_maps")
os.makedirs(SAVED_MAPS_DIR, exist_ok=True)
# 저장/로드 시 함께 다루는 파일 4종 (히트그리드 원본이 있어야 로드 후 이어서 매핑 가능)
BEST_MAP_FILES = ["floor_plan.png", "floor_plan_meta.json", "floor_plan_hits.json", "mission_targets.json"]
# 이미 확정등록된 타겟(베스트맵에서 불러온 기대위치 포함) 근처에서 재감지되면
# 새로 만들지 않고 이 비율만큼 실측값 쪽으로 이동평균 보정
TARGET_UPDATE_ALPHA = 0.3

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
PARAMS_HTTP_PORT = 8081
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
# ---- 미션1 반응형 게이트 추종 ----
GATE_PROGRESS_PATH = os.path.join(DATA_DIR, "gate_progress.json")
GATE_FRONT_CONE_DEG = 140.0     # 이 각도(전방 기준 좌우 절반씩) 안의 부표쌍만 후보로 봄
GATE_MAX_CONSIDER_DIST_M = 15.0  # 이보다 먼 부표는 무시(오래된/먼 등록값 배제)
GATE_END_TIMEOUT_SEC = 4.0      # 이만큼 연속으로 유효 게이트쌍이 안 보이면 미션 종료로 판정

# ---- 기존 camera_node 연동 ----
CAMERA_DETECTIONS_TOPIC = "/camera/detections"

# ---- 진행상황 공유 파일 ----
MISSION_PROGRESS_PATH = os.path.join(DATA_DIR, "mission_progress.json")

# ---- 조향(helm) ----
HELM_LINEAR_MAX = 0.6
HELM_ANGULAR_MAX = 1.0
HELM_KP_ANGULAR = 1.2
HELM_RATE_HZ = 10.0
POSE_STALE_TIMEOUT_SEC = 1.0  # 이만큼 pose 갱신이 없으면 VIO 문제로 보고 정지

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
# zone 좌표(매핑 때 등록한 대략적 위치) 근처 이 반경 안에서 실시간 감지된 buoy_*가
# 있으면 그걸 실제 미션 대상으로 우선 사용, 없으면 zone 좌표 자체로 폴백.
ZONE_TARGET_MATCH_RADIUS_M = 5.0

# ---- 도킹 (지정된 색/모양 마커에 접근) ----
DOCKING_TARGET_PATH = os.path.join(DATA_DIR, "docking_target.json")
DOCKING_PROGRESS_PATH = os.path.join(DATA_DIR, "docking_progress.json")
DOCKING_ARRIVE_DIST_M = 0.8
DOCKING_LINEAR_MAX = 0.25
DOCKING_KP_ANGULAR = 1.5
DOCKING_HOLD_SEC = 3.0   # 표식 근처에서 이만큼 연속 정지해야 도킹 완료로 판정

# ---- 탐색 (부표 주회) ----
SEARCH_PROGRESS_PATH = os.path.join(DATA_DIR, "search_progress.json")
SEARCH_COLOR_PATH = os.path.join(DATA_DIR, "search_color.json")
SEARCH_RADIUS_M = 3.0
SEARCH_LINEAR_MAX = 0.3
SEARCH_KP_ANGULAR = 1.2
SEARCH_RADIAL_KP = 0.3
SEARCH_LAPS_TARGET = 1.5  # 1바퀴(1.0)보다 여유있게 1.5바퀴(540도) 채워야 성공 - 안전마진
# 규정: 빨강/초록 부표는 시계방향, 흰색 부표는 반시계방향
SEARCH_DIRECTION_BY_COLOR = {"red": "cw", "green": "cw", "white": "ccw"}

# ---- 타겟 오탐 필터링 (연속 감지 확정) ----
TARGET_CONFIRM_HITS = 3          # 같은 위치에서 이만큼 연속 감지되어야 실제 등록
TARGET_CANDIDATE_TIMEOUT_SEC = 4.0  # 이 시간 동안 재감지 안 되면 후보 폐기
