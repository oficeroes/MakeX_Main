"""JSON 轨迹文件的存读 + 自增文件名管理

目的
====
把绘制会话（场地参数、标定值、设置、原始点、平滑后点、障碍物）序列化为 JSON，
并保证每次保存自动产生新文件编号（trajectory_001.json → trajectory_002.json …）。

文件格式（schema_version 4）
============================
{
  "schema_version": 4,
  "created_at": "2026-06-27T14:32:11",
  "field": {"width_cm": 300, "height_cm": 300},
  "calibration": {
    "cm_per_second_at_power_50": 30.0,
    "deg_per_second_at_omega_50": 90.0,
    "auto_power": 50,
    "omega_power": 40,
    "drift_left_omega": 0,
    "drift_right_omega": 0
  },
  "settings": {
    "mode": "translation",
    "smoothing_iterations": 3,
    "resample_step_cm": 5.0,
    "invert_x": false,
    "invert_y": false,
    "ramp_ms": 100,
    "chassis_profile_id": "omni3"
  },
  "path": {
    "raw_points_cm":      [[x, y], ...],
    "smoothed_points_cm": [[x, y], ...]
  },
  "obstacles": [{"x_cm":100,"y_cm":50,"width_cm":30,"height_cm":40,"label":"obs1"}]
}

同时保存 raw + smoothed，导入后可用新参数重新平滑。

注意
====
- schema_version 在 config.SCHEMA_VERSION 中维护；每次增字段时递增。
- chassis_profile_id 在 settings 下（v4 新增），导入时缺省 → "omni3"。
- load_trajectory 不做版本迁移，老文件字段缺失时依赖调用方的 .get(key, default)。

API
===
  next_filename(folder=TRAJECTORIES_DIR) -> Path
  build_payload(field_w, field_h, calibration, settings,
                raw_points, smoothed_points, obstacles) -> dict
  save_trajectory(payload, target_path=None) -> Path
  load_trajectory(path) -> dict
"""
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
