"""
빨강/초록 부표를 한 쌍씩 게이트로 묶고, 현재 위치에서 가까운 게이트부터
순서를 정해 지나갈 경로(mission_path.json)를 만든다.
"""

import json
import math
import os

from . import config


def _color_of(name: str):
    parts = name.split('_')
    if len(parts) >= 2 and parts[0] == 'buoy':
        return parts[1]
    return None


def load_targets(path=config.MISSION_TARGETS_PATH) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def pair_gates(targets: dict, left_color=config.GATE_LEFT_COLOR,
               right_color=config.GATE_RIGHT_COLOR,
               max_pair_dist=config.GATE_MAX_PAIR_DIST_M):
    left = {n: t for n, t in targets.items() if _color_of(n) == left_color}
    right = {n: t for n, t in targets.items() if _color_of(n) == right_color}

    gates = []
    used_right = set()
    for ln, lt in left.items():
        best_name, best_d = None, None
        for rn, rt in right.items():
            if rn in used_right:
                continue
            d = math.hypot(lt['x'] - rt['x'], lt['y'] - rt['y'])
            if max_pair_dist is not None and d > max_pair_dist:
                continue
            if best_name is None or d < best_d:
                best_name, best_d = rn, d
        if best_name is not None:
            used_right.add(best_name)
            rt = right[best_name]
            gates.append({
                "members": [ln, best_name],
                "x": (lt['x'] + rt['x']) / 2.0,
                "y": (lt['y'] + rt['y']) / 2.0,
                "left_x": lt['x'], "left_y": lt['y'],
                "right_x": rt['x'], "right_y": rt['y'],
                "width_m": round(best_d, 2),
            })
    return gates


def order_gates_nearest(gates, start_xy):
    remaining = list(gates)
    ordered = []
    cur = start_xy
    while remaining:
        remaining.sort(key=lambda g: math.hypot(g['x'] - cur[0], g['y'] - cur[1]))
        nxt = remaining.pop(0)
        ordered.append(nxt)
        cur = (nxt['x'], nxt['y'])
    return ordered


def build_path(start_xy, targets=None, left_color=config.GATE_LEFT_COLOR,
                right_color=config.GATE_RIGHT_COLOR,
                max_pair_dist=config.GATE_MAX_PAIR_DIST_M):
    if targets is None:
        targets = load_targets()
    gates = pair_gates(targets, left_color, right_color, max_pair_dist)
    ordered = order_gates_nearest(gates, start_xy)
    for i, g in enumerate(ordered):
        g["name"] = f"gate_{i + 1}"
    return ordered


def save_path(path_list, out_path=config.MISSION_PATH_PATH):
    with open(out_path, "w") as f:
        json.dump(path_list, f, indent=2, ensure_ascii=False)


def load_path(path=config.MISSION_PATH_PATH):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return json.load(f)


def _orient(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def _on_segment(a, b, c):
    return (min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9 and
            min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9)


def segments_intersect(p1, p2, p3, p4):
    """이동 선분(p1->p2)이 게이트 선분(p3->p4)을 실제로 가로질렀는지 판정."""
    d1, d2 = _orient(p3, p4, p1), _orient(p3, p4, p2)
    d3, d4 = _orient(p1, p2, p3), _orient(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


def save_progress(current_gate_idx, out_path=config.MISSION_PROGRESS_PATH):
    with open(out_path, "w") as f:
        json.dump({"current_gate_idx": current_gate_idx}, f)


def load_progress(path=config.MISSION_PROGRESS_PATH):
    if not os.path.exists(path):
        return {"current_gate_idx": 0}
    with open(path) as f:
        return json.load(f)
