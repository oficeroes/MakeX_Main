"""JSON 存/读 + 自增文件名"""
import json
import re
from datetime import datetime
from pathlib import Path

from .config import SCHEMA_VERSION, TRAJECTORIES_DIR


_NAME_RE = re.compile(r"trajectory_(\d+)\.json$", re.IGNORECASE)


def next_filename(folder=TRAJECTORIES_DIR):
    """扫描 folder 下 trajectory_NNN.json，返回下一个可用文件路径"""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    max_n = 0
    for p in folder.glob("trajectory_*.json"):
        m = _NAME_RE.search(p.name)
        if m:
            try:
                max_n = max(max_n, int(m.group(1)))
            except ValueError:
                pass
    next_n = max_n + 1
    width = max(3, len(str(next_n)))
    return folder / ("trajectory_%0*d.json" % (width, next_n))


def build_payload(field_w, field_h, calibration, settings,
                  raw_points, smoothed_points, obstacles):
    """组装要存盘的数据结构"""
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "field": {"width_cm": float(field_w), "height_cm": float(field_h)},
        "calibration": dict(calibration),
        "settings": dict(settings),
        "path": {
            "raw_points_cm": [[float(x), float(y)] for x, y in raw_points],
            "smoothed_points_cm": [[float(x), float(y)] for x, y in smoothed_points],
        },
        "obstacles": [dict(o) for o in obstacles],
    }


def save_trajectory(payload, target_path=None):
    """写 JSON。target_path 为 None 时自动取下一个编号"""
    if target_path is None:
        target_path = next_filename()
    target_path = Path(target_path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    with target_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return target_path


def load_trajectory(path):
    """读取并返回 payload（字典）"""
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)
