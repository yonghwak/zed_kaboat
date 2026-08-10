"""
helm_node가 지금 어떤 모드로 동작해야 하는지(경로추종 / 위치유지 / 정지),
그리고 위치유지 진행상황을 공유하는 파일 기반 상태.
대시보드(웹)에서 모드를 바꾸고, helm_node가 그걸 읽어서 동작한다.
"""

import json
import os

from . import config


def load_mode(path=config.MISSION_MODE_PATH):
    if not os.path.exists(path):
        return {"mode": "gate_follow", "station_target": None}
    with open(path) as f:
        return json.load(f)


def save_mode(mode, station_target=None, path=config.MISSION_MODE_PATH):
    with open(path, "w") as f:
        json.dump({"mode": mode, "station_target": station_target}, f)


def load_station_progress(path=config.STATION_KEEP_PROGRESS_PATH):
    if not os.path.exists(path):
        return {"held_seconds": 0.0, "success": False, "target": None}
    with open(path) as f:
        return json.load(f)


def save_station_progress(held_seconds, success, target, path=config.STATION_KEEP_PROGRESS_PATH):
    with open(path, "w") as f:
        json.dump({"held_seconds": round(held_seconds, 1), "success": success, "target": target}, f)


def load_manager_progress(path=config.MISSION_MANAGER_PROGRESS_PATH):
    if not os.path.exists(path):
        return {"zone_index": 0, "zone_name": None, "state": "idle"}
    with open(path) as f:
        return json.load(f)


def save_manager_progress(zone_index, zone_name, state, path=config.MISSION_MANAGER_PROGRESS_PATH):
    with open(path, "w") as f:
        json.dump({"zone_index": zone_index, "zone_name": zone_name, "state": state}, f)


def load_docking_target(path=config.DOCKING_TARGET_PATH):
    if not os.path.exists(path):
        return {"color": None, "shape": None}
    with open(path) as f:
        return json.load(f)


def save_docking_target(color, shape, path=config.DOCKING_TARGET_PATH):
    with open(path, "w") as f:
        json.dump({"color": color, "shape": shape}, f)


def load_docking_progress(path=config.DOCKING_PROGRESS_PATH):
    if not os.path.exists(path):
        return {"docked": False, "target": None}
    with open(path) as f:
        return json.load(f)


def save_docking_progress(docked, target, path=config.DOCKING_PROGRESS_PATH):
    with open(path, "w") as f:
        json.dump({"docked": docked, "target": target}, f)


def load_search_progress(path=config.SEARCH_PROGRESS_PATH):
    if not os.path.exists(path):
        return {"angle_deg": 0.0, "laps": 0.0, "success": False, "target": None}
    with open(path) as f:
        return json.load(f)


def save_search_progress(angle_deg, laps, success, target, path=config.SEARCH_PROGRESS_PATH):
    with open(path, "w") as f:
        json.dump({"angle_deg": round(angle_deg, 1), "laps": round(laps, 2),
                   "success": success, "target": target}, f)
