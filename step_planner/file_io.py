"""step_planner.file_io — JSON 存读（schema v1）

save_steps   — 序列化到 JSON 文件
load_steps   — 从 JSON 文件反序列化
next_filename — 自增文件名（step_sequence_001.json, 002, ...）
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Optional

from .config import (
    DEFAULT_FIELD_HEIGHT_CM,
    DEFAULT_FIELD_WIDTH_CM,
    DEFAULT_PROFILE_ID,
    STEP_SCHEMA_VERSION,
    STEP_SEQUENCES_DIR,
    default_calibration,
)
from .models import CalibrationState, Pose, Step, StepType


# =========================================================
#  save_steps
# =========================================================
def save_steps(
    steps: list[Step],
    cal: CalibrationState,
    start_pose: Pose,
    profile_id: str,
    field_w: float = DEFAULT_FIELD_WIDTH_CM,
    field_h: float = DEFAULT_FIELD_HEIGHT_CM,
    target_path: Optional[Path] = None,
) -> Path:
    """把当前步骤序列序列化为 JSON，返回写入路径。

    若 target_path 为 None，自动使用 next_filename() 生成新文件。
    """
    if target_path is None:
        target_path = next_filename()

    target_path = Path(target_path)
    target_path.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "schema_version": STEP_SCHEMA_VERSION,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "chassis_profile_id": profile_id,
        "field": {"width_cm": field_w, "height_cm": field_h},
        "start_pose": start_pose.to_dict(),
        "calibration": cal.to_dict(),
        "steps": [s.to_dict() for s in steps],
    }

    with open(target_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    return target_path


# =========================================================
#  load_steps
# =========================================================
def load_steps(
    path: Path,
) -> tuple[list[Step], CalibrationState, Pose, str, float, float]:
    """从 JSON 文件读取步骤序列。

    返回 (steps, calibration, start_pose, profile_id, field_w, field_h)。
    对未知或较旧的 schema 尽量向前兼容（缺字段用默认值）。
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    schema_ver = data.get("schema_version", 1)

    # 底盘
    profile_id = data.get("chassis_profile_id", DEFAULT_PROFILE_ID)

    # 场地尺寸
    field = data.get("field", {})
    field_w = float(field.get("width_cm",  DEFAULT_FIELD_WIDTH_CM))
    field_h = float(field.get("height_cm", DEFAULT_FIELD_HEIGHT_CM))

    # 起始位姿
    sp_raw = data.get("start_pose", {})
    start_pose = Pose.from_dict(sp_raw) if sp_raw else Pose()

    # 标定
    cal_raw = data.get("calibration", {})
    if cal_raw:
        cal = CalibrationState.from_dict(cal_raw)
    else:
        cal = default_calibration(profile_id)

    # 步骤
    steps_raw = data.get("steps", [])
    steps: list[Step] = []
    for s in steps_raw:
        try:
            steps.append(Step.from_dict(s))
        except Exception:
            pass  # 跳过无法解析的步骤（前向兼容）

    return steps, cal, start_pose, profile_id, field_w, field_h


# =========================================================
#  next_filename
# =========================================================
def next_filename(folder: Optional[Path] = None) -> Path:
    """返回下一个未占用的 step_sequence_NNN.json 路径。

    若文件夹不存在，先创建。
    """
    folder = Path(folder) if folder else STEP_SEQUENCES_DIR
    folder.mkdir(parents=True, exist_ok=True)

    existing = sorted(folder.glob("step_sequence_???.json"))
    if not existing:
        return folder / "step_sequence_001.json"

    last = existing[-1]
    # 从文件名提取序号
    try:
        num = int(last.stem.rsplit("_", 1)[-1])
    except ValueError:
        num = 0
    return folder / ("step_sequence_%03d.json" % (num + 1))
