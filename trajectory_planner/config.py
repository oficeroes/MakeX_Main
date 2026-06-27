"""默认配置常量"""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ROBOT_FILE = PROJECT_ROOT / "mecanum_forward.py"
TRAJECTORIES_DIR = PROJECT_ROOT / "trajectories"

DEFAULT_FIELD_WIDTH_CM = 300.0
DEFAULT_FIELD_HEIGHT_CM = 300.0

DEFAULT_CM_PER_SEC_AT_P50 = 30.0
DEFAULT_DEG_PER_SEC_AT_OMEGA50 = 90.0
DEFAULT_AUTO_POWER = 50
DEFAULT_OMEGA_POWER = 40

POWER_MIN = 20
POWER_MAX = 90

DEFAULT_SMOOTH_ITER = 3
DEFAULT_RESAMPLE_CM = 5.0

MODE_TRANSLATION = "translation"
MODE_HEADING = "heading_follows"

ROTATION_DEADBAND_DEG = 3.0
MERGE_ANGLE_DEG = 5.0
MERGE_MAX_DURATION = 4.0
STOP_BUFFER = (0.1, 0, 0, 0)

SCHEMA_VERSION = 3  # v3: + invert_x

# ===== 机器人执行端参数（导出时一并写入 mecanum_forward.py 顶部）=====
DEFAULT_INVERT_X = False       # True = 所有 Vx 取反（机器人左右装反了）
DEFAULT_INVERT_Y = False       # True = 所有 Vy 取反（机器人前后装反了）
DEFAULT_RAMP_MS = 100          # 步间速度线性插值时长（毫秒）
DEFAULT_DRIFT_LEFT_OMEGA = 0   # 向左平移（GUI 视角 Vx<0）时叠加的 omega 补偿
DEFAULT_DRIFT_RIGHT_OMEGA = 0  # 向右平移（GUI 视角 Vx>0）时叠加的 omega 补偿

GRID_SPACING_CM = 10.0
PATH_PEN_WIDTH_CM = 1.5
