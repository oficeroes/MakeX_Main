"""step_planner.config — ChassisProfile 注册表 + 标定默认值

直接复用 trajectory_planner.config 里的 ChassisProfile 和注册表，
在此模块暴露 step_planner 专用的额外常量。

新增字段
========
  _CAL_DEFAULTS: 每个底盘的初始标定值
    - encoder_ticks_per_cm: 线性运动，来自各自固件文件的实测值
    - turn_ticks_per_deg:   旋转标定，初始值 = encoder_ticks_per_cm * 0.5
                            （旧系统的 0.5 魔法常数被显式化为可标定量）

  default_calibration(profile_id) -> CalibrationState
    按底盘 ID 返回默认标定值；用户标定后会覆盖。

STEP_SCHEMA_VERSION = 1
STEP_SEQUENCES_DIR  = PROJECT_ROOT / "step_sequences"
"""

from pathlib import Path

from trajectory_planner.config import (
    CHASSIS_PROFILES,
    DEFAULT_FIELD_HEIGHT_CM,
    DEFAULT_FIELD_WIDTH_CM,
    DEFAULT_PROFILE_ID,
    PROJECT_ROOT,
    ChassisProfile,
    get_profile,
)

from .models import CalibrationState

# ── 新增常量 ──────────────────────────────────────────────
STEP_SCHEMA_VERSION = 1
STEP_SEQUENCES_DIR  = PROJECT_ROOT / "step_sequences"

# 每个底盘的默认标定值
#   encoder_ticks_per_cm: 从各底盘 firmware 文件实测常量读取
#     三轮: Mecanum_forward_能用的.py line 96  ENCODER_TICKS_PER_CM = 9.4760
#     四轮: mecanum_drive.py line 221          ENCODER_TICKS_PER_CM = 12.2870
#   turn_ticks_per_deg: 旧系统用 0.5 魔法系数，这里显式列出作为初始估计
#     用户应通过「旋转标定」流程替换为实测值
_CAL_DEFAULTS: dict[str, dict] = {
    "omni3": {
        "encoder_ticks_per_cm":    9.4760,
        "turn_ticks_per_deg":      4.7380,
        "invert_vx":               False,
        "invert_vy":               True,
        "invert_omega":            False,
        "front_back_compensation": 0.0,
    },
    "mecanum_4w": {
        "encoder_ticks_per_cm":    12.2870,
        "turn_ticks_per_deg":       6.1435,
        "invert_vx":               False,
        "invert_vy":               True,
        "invert_omega":            True,
        "front_back_compensation": 0.0,
    },
}


def default_calibration(profile_id: str) -> CalibrationState:
    """按底盘 ID 返回默认标定值；未知 ID 回退到 omni3。"""
    d = _CAL_DEFAULTS.get(profile_id, _CAL_DEFAULTS["omni3"])
    return CalibrationState(**d)


# ── 重导出（让调用方只 import step_planner.config 即可）────
__all__ = [
    "CHASSIS_PROFILES",
    "DEFAULT_FIELD_WIDTH_CM",
    "DEFAULT_FIELD_HEIGHT_CM",
    "DEFAULT_PROFILE_ID",
    "PROJECT_ROOT",
    "STEP_SCHEMA_VERSION",
    "STEP_SEQUENCES_DIR",
    "ChassisProfile",
    "CalibrationState",
    "get_profile",
    "default_calibration",
    "_CAL_DEFAULTS",
]
