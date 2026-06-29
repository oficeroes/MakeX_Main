"""默认配置常量 + 底盘配置（ChassisProfile）注册表

设计思路
========
"轨迹绘制 → AUTO_SEQUENCE 元组列表"这件事跟底盘是几个轮子无关 ——
GUI 输出 (Vx, Vy, omega) 速度向量，由机器人侧的 kinematics() 函数负责
转成各轮功率。所以底盘扩展只需要：
  1) 一个新的 .py 模板文件（机器人侧 kinematics + AUTO_SEQUENCE 块）
  2) 在 CHASSIS_PROFILES 注册表里加一项

GUI 不需要改运动学代码，只需要根据当前选中的 profile 路由到正确的目标文件。

新增底盘时的步骤详见 docs/EXTENDING_FOR_NEW_CHASSIS.md。

常量分类
========
  画布默认值        DEFAULT_FIELD_*
  速度标定默认值    DEFAULT_CM_PER_SEC_AT_P50, DEFAULT_DEG_PER_SEC_AT_OMEGA50
  运动参数默认值    DEFAULT_AUTO_POWER, DEFAULT_OMEGA_POWER, POWER_MIN/MAX
  平滑参数默认值    DEFAULT_SMOOTH_ITER, DEFAULT_RESAMPLE_CM
  运动模式字符串    MODE_TRANSLATION, MODE_HEADING
  轨迹生成参数      ROTATION_DEADBAND_DEG, MERGE_ANGLE_DEG, MERGE_MAX_DURATION, STOP_BUFFER
  数据结构版本      SCHEMA_VERSION（每次 JSON 格式变化时递增）
  机器人执行端参数  DEFAULT_INVERT_X/Y, DEFAULT_RAMP_MS, DEFAULT_DRIFT_*

ChassisProfile 注册表
=====================
  OMNI3_PROFILE    三轮全向（mecanum_forward.py），has_face_concept=True
  MECANUM_X_PROFILE  X 型麦克纳姆（mecanum_X_forward.py），has_face_concept=False
  CHASSIS_PROFILES   有序列表，GUI combo_chassis 按此顺序填充
  DEFAULT_PROFILE_ID 默认选中的 profile_id（= "omni3"）

向后兼容
========
  ROBOT_FILE = OMNI3_PROFILE.file_path  ← 旧代码可继续用这个名字

API
===
  get_profile(profile_id: str) -> ChassisProfile   # 找不到时返回 OMNI3_PROFILE
"""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TRAJECTORIES_DIR = PROJECT_ROOT / "trajectories"

# ===== 画布默认值 =====
DEFAULT_FIELD_WIDTH_CM = 300.0
DEFAULT_FIELD_HEIGHT_CM = 300.0

# ===== 速度标定默认值 =====
DEFAULT_CM_PER_SEC_AT_P50 = 30.0      # 功率=50 时机器人沿轴速度（cm/s）
DEFAULT_DEG_PER_SEC_AT_OMEGA50 = 90.0 # omega=50 时机器人自转角速度（deg/s）
DEFAULT_AUTO_POWER = 50                # 平移功率默认值
DEFAULT_OMEGA_POWER = 40               # 旋转功率默认值

# 安全限制：低于 POWER_MIN 时机器人会因静摩擦不动；高于 POWER_MAX 时易过载
POWER_MIN = 20
POWER_MAX = 90

# ===== 平滑参数默认值 =====
DEFAULT_SMOOTH_ITER = 3       # Chaikin 迭代次数
DEFAULT_RESAMPLE_CM = 5.0     # 等弧长重采样间隔

# ===== 运动模式 =====
MODE_TRANSLATION = "translation"     # 纯平移（车头不变）
MODE_HEADING = "heading_follows"     # 车头跟随路径切线

# ===== 轨迹生成参数 =====
ROTATION_DEADBAND_DEG = 3.0   # 模式 B 中 |Δθ|< 此值跳过旋转
MERGE_ANGLE_DEG = 5.0          # 相邻段方向夹角小于此值则合并
MERGE_MAX_DURATION = 4.0       # 合并后单步最长 4 秒（防止误合并）
STOP_BUFFER = (0.1, 0, 0, 0)   # 自动序列末尾的停止缓冲

# ===== 数据结构版本 =====
SCHEMA_VERSION = 4  # v4: + chassis profile id

# ===== 机器人执行端参数（导出时写入机器人代码顶部）=====
DEFAULT_INVERT_X = False       # True = 所有 Vx 取反（左右装反）
DEFAULT_INVERT_Y = False       # True = 所有 Vy 取反（前后装反）
DEFAULT_RAMP_MS = 100          # 步间速度线性插值时长（毫秒），0=立即切换
DEFAULT_DRIFT_LEFT_OMEGA = 0   # GUI 视角向左平移（Vx<0）的 omega 补偿
DEFAULT_DRIFT_RIGHT_OMEGA = 0  # GUI 视角向右平移（Vx>0）的 omega 补偿

# ===== 显示样式 =====
GRID_SPACING_CM = 10.0
PATH_PEN_WIDTH_CM = 1.5


# ================================================================
#  ChassisProfile — 底盘配置
# ================================================================
class ChassisProfile:
    """一个底盘 = 一个机器人源文件 + 一组默认值 + 是否启用某些功能开关

    用 ChassisProfile 把"导出到哪个文件"和"GUI 控件可见性"解耦。
    GUI 不知道、也不关心底盘是 3 轮还是 4 轮 —— 只知道当前 profile
    指向哪个文件、有没有 face 概念等。
    """

    def __init__(self, profile_id, display_name, file_name, wheel_count,
                 has_face_concept=False, description=""):
        # 唯一 ID：写入 JSON 用，不显示给用户。保持小写蛇形命名
        self.profile_id = profile_id
        # 在 GUI 下拉框里显示的名称
        self.display_name = display_name
        # 机器人源文件名（相对 PROJECT_ROOT）
        self.file_name = file_name
        # 轮子数量（仅用于显示统计信息）
        self.wheel_count = wheel_count
        # 三轮全向有 3 个对称正面（按 R1/L1 切换）；麦克纳姆没有
        self.has_face_concept = has_face_concept
        # 简要描述，显示在 GUI 提示区
        self.description = description

    @property
    def file_path(self):
        return PROJECT_ROOT / self.file_name

    def __repr__(self):
        return "ChassisProfile(%s -> %s)" % (self.profile_id, self.file_name)


# ===== 注册表：在这里加 profile 即可在 GUI 出现 =====
OMNI3_PROFILE = ChassisProfile(
    profile_id="omni3",
    display_name="三轮全向（120° 对称）",
    file_name="mecanum_forward.py",  # 历史命名保留，不改避免破坏现有引用
    wheel_count=3,
    has_face_concept=True,
    description=(
        "M1（前左 150°）+ M2（前右 30°）+ M3（尾部 270°）。"
        "通过 R1/L1 切换正面方向。"
    ),
)

MECANUM_X_PROFILE = ChassisProfile(
    profile_id="mecanum_x",
    display_name="X 型麦克纳姆（4 轮）",
    file_name="mecanum_X_forward.py",
    wheel_count=4,
    has_face_concept=False,
    description=(
        "M1（前左）+ M2（前右）+ M3（后左）+ M4（后右），"
        "滚轮 ±45° 形成 X 形。前后/左右对称，没有 face 概念。"
    ),
)

MECANUM_DRIVE_PROFILE = ChassisProfile(
    profile_id="mecanum_drive",
    display_name="麦克纳姆竞赛车（含收球 / 滚球）",
    file_name="mecanum_drive.py",
    wheel_count=4,
    has_face_concept=False,
    description=(
        "X 型麦克纳姆底盘 + M5 滚球电机 + DC1/DC2 收球电机。"
        "运动学与 mecanum_x 相同；N1-N4 键用于执行机构，+ 键触发自动程序。"
    ),
)

CHASSIS_PROFILES = [OMNI3_PROFILE, MECANUM_X_PROFILE, MECANUM_DRIVE_PROFILE]

# 默认选中第一个
DEFAULT_PROFILE_ID = OMNI3_PROFILE.profile_id


def get_profile(profile_id):
    """根据 ID 查找 profile；找不到时返回默认"""
    for p in CHASSIS_PROFILES:
        if p.profile_id == profile_id:
            return p
    return OMNI3_PROFILE


# 向后兼容：旧代码用 ROBOT_FILE 直接拿默认机器人路径
ROBOT_FILE = OMNI3_PROFILE.file_path
