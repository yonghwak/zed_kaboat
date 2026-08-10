import json
import os
from . import config


def load_targets(path=config.MISSION_TARGETS_PATH) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def save_targets(targets, path=config.MISSION_TARGETS_PATH):
    with open(path, "w") as f:
        json.dump(targets, f, indent=2, ensure_ascii=False)


def register_target(name, x, y, height=config.DEFAULT_TARGET_HEIGHT_M,
                     path=config.MISSION_TARGETS_PATH):
    targets = load_targets(path)
    targets[name] = {"x": round(x, 3), "y": round(y, 3), "height": round(height, 3)}
    save_targets(targets, path)
    return targets


def remove_target(name, path=config.MISSION_TARGETS_PATH):
    targets = load_targets(path)
    targets.pop(name, None)
    save_targets(targets, path)
    return targets
