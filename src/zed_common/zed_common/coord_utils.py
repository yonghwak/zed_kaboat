import json
from dataclasses import dataclass, asdict


@dataclass
class FloorPlanMeta:
    origin_x: float
    origin_y: float
    resolution: float
    width_px: int
    height_px: int

    def save(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @staticmethod
    def load(path) -> "FloorPlanMeta":
        with open(path) as f:
            return FloorPlanMeta(**json.load(f))


def world_to_pixel(x, y, meta: FloorPlanMeta):
    return (int(round((x - meta.origin_x) / meta.resolution)),
            int(round((y - meta.origin_y) / meta.resolution)))


def pixel_to_world(u, v, meta: FloorPlanMeta):
    return (meta.origin_x + u * meta.resolution,
            meta.origin_y + v * meta.resolution)


def in_bounds(u, v, meta: FloorPlanMeta) -> bool:
    return 0 <= u < meta.width_px and 0 <= v < meta.height_px
