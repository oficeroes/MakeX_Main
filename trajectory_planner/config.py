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
  MECANUM_4W_PROFILE  四轮麦克纳姆（mecanum_drive.py），has_face_concept=False
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

# ===== 画布默认值（固定场地：4655mm × 3055mm）=====
DEFAULT_FIELD_WIDTH_CM = 465.5
DEFAULT_FIELD_HEIGHT_CM = 305.5

# ===== 机器人物理参数 =====
ROBOT_LENGTH_CM = 49.0            # 默认小车前后长度（cm）
ROBOT_WIDTH_CM = 50.0             # 默认小车左右宽度（cm）
WHEEL_DIAMETER_CM = 10.0          # 轮子直径（cm）
WHEEL_CIRCUMFERENCE_CM = 31.4159  # π × 10cm，轮子转一圈走的距离
ENCODER_PPR = None                # 编码器每转脉冲数（待标定，填了才启用闭环）
ENCODER_PULSES_PER_CM = None      # 1 cm = 多少编码脉冲（PPR / 周长，自动推算）

# ===== 闭环控制参数（编码器 PID） =====
# 编码器 → 距离换算：1 cm = 多少 encoder ticks（get_count() 返回值）
# 用户自行标定后填入，填了才启用闭环模式
ENCODER_TICKS_PER_CM = 18.0       # 默认值（约 1000°/58cm ≈ 17.2°，用 get_value("angle") 的度/cm）
DEFAULT_DISTANCE_SCALE = 1.0      # 导出距离倍率；实车偏小可临时调大，标定后应回到 1.0

# PID 控制器增益
PID_KP = 0.35                     # 比例系数：位置误差 → 功率修正
PID_KI = 0.02                     # 积分系数：消除稳态误差
PID_KD = 0.08                     # 微分系数：抑制超调
PID_INTEGRAL_MAX = 30.0           # 积分上限（防止积分饱和）

# 梯形速度曲线参数（机器人端执行）
PROFILE_ACCEL_TICKS = 80          # 加速段长度（encoder ticks），越大越柔和
PROFILE_CRUISE_MIN = 30           # 最低巡航速度（power %），防止静摩擦
PROFILE_DEADBAND_TICKS = 8        # 到位死区（ticks），小于此值视为到达

# ===== 速度曲线参数（用户可调） =====
# 整个路径分为三段：加速段 → 匀速段 → 减速段
# 加速段和减速段按"距离"定义（cm），而非时间，确保物理一致性
PROFILE_ACCEL_CM = 0.0            # 路径开头多少 cm 用于加速（0=禁用）
PROFILE_DECEL_CM = 0.0            # 路径结尾多少 cm 用于减速（0=禁用）
PROFILE_MAX_SPEED = 100.0         # 匀速段最高速度（cm/s）
PROFILE_MIN_SPEED = 30.0          # 加速起点 / 减速终点的最低速度（cm/s），克服静摩擦
PROFILE_ROT_SPEED = 150.0         # 旋转最高角速度（°/s）
PROFILE_ROT_ACCEL_DEG = 15.0      # 旋转加速段长度（度）
PROFILE_ROT_DECEL_DEG = 15.0      # 旋转减速段长度（度）
PROFILE_CURVE_ADAPTIVE = False    # 是否启用曲率自适应（弯减速 10%，直加速 5%）

# 曲率自适应倍率（内部使用）
CURVE_SLOWDOWN_FACTOR = 0.95
STRAIGHT_BOOST_FACTOR = 1.15
CURVATURE_THRESHOLD = 0.08

# ===== 编码器序列合并参数（v5.1：解决"一格一格走"） =====
MIN_ENC_STEP_TICKS = 15             # 最小步长（encoder ticks），小于此值的步被合并到相邻步
ENC_MERGE_POWER_RATIO = 0.30        # 合并容许功率偏差（0.30 = ±30%，即功率在 70%~130% 之间可合并）

# 向后兼容别名
BASE_SPEED_CM_PER_SEC = PROFILE_MAX_SPEED
BASE_ROT_DEG_PER_SEC = PROFILE_ROT_SPEED

# ===== 速度标定默认值 =====
DEFAULT_CM_PER_SEC_AT_P50 = 30.0      # 功率=50 时机器人实测沿轴速度（cm/s），待标定
DEFAULT_STRAFE_CM_PER_SEC_AT_P50 = 30.0  # 功率=50 时机器人实测横移速度（cm/s），四轮曲线用
DEFAULT_DEG_PER_SEC_AT_OMEGA50 = 90.0 # 功率=50 时机器人实测自转角速度（°/s），待标定
DEFAULT_AUTO_POWER = 85                # 平移功率默认值
DEFAULT_OMEGA_POWER = 70               # 旋转功率默认值

# 安全限制：低于 POWER_MIN 时机器人会因静摩擦不动；高于 POWER_MAX 时易过载
POWER_MIN = 25
POWER_MAX = 95

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

# ===== 缓升缓降（速度斜坡）默认值 =====
# 每个路段开头/结尾插入加速/减速子步骤，防止高功率起步打滑
DEFAULT_RAMP_TIME = 0.25       # 每段加速 / 减速各占多少秒（总过渡 = RAMP_TIME × 2）
DEFAULT_RAMP_STEPS = 5         # 加速 / 减速各切成多少个子步骤（越多越平滑）
DEFAULT_RAMP_MIN_RATIO = 0.15  # 起始 / 结束功率比例（0.15 = 15%，克服静摩擦最低值）
RAMP_ENABLED = False           # 是否启用缓升缓降（导出时生效）

# ===== 连续轨迹播放参数（四轮曲线止血版） =====
DEFAULT_TRAJ_SAMPLE_DT = 0.05   # 每个速度采样保持时长（秒），0.05=20Hz 轨迹点
CENTER_SAFETY_MARGIN_CM = 0.0   # 中心点额外安全边距；先按真实车身半尺寸，不额外膨胀

# ===== 数据结构版本 =====
SCHEMA_VERSION = 6  # v6: per-chassis calibration/settings panels

# ===== 机器人执行端参数（导出时写入机器人代码顶部）=====
DEFAULT_INVERT_X = False       # True = 所有 Vx 取反（左右装反）
DEFAULT_INVERT_Y = False       # True = 所有 Vy 取反（前后装反）
DEFAULT_RAMP_MS = 0            # 步间速度线性插值时长（毫秒），0=立即切换
# 前后重量补偿：尾重头轻 → 前轮加力（写入机器人 FRONT_BACK_COMPENSATION 常量）
# 范围 0.0 ~ 0.50，0 = 不补偿，值越大前轮出力越多
DEFAULT_FRONT_BACK_COMPENSATION = 0.0

# 旋转均衡补偿：尾重导致旋转时前后出力不均（写入机器人 ROTATION_BALANCE）
# 范围 0.0 ~ 0.50，0 = 不补偿，独立于前后补偿
DEFAULT_ROTATION_BALANCE = 0.0

# 平移纵向耦合补偿：平移时向前漂移 → 按 |Vx| 减 Vy（写入机器人 STRAFE_VY_COUPLING）
# 范围 0.0 ~ 0.30，0 = 不补偿，值越大反向拉回越强
DEFAULT_STRAFE_VY_COUPLING = 0.0

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
                 has_face_concept=False, description="", color="#2878D0",
                 supports_oscillate=False,
                 robot_length_cm=ROBOT_LENGTH_CM, robot_width_cm=ROBOT_WIDTH_CM):
        self.profile_id = profile_id
        self.display_name = display_name
        self.file_name = file_name
        self.wheel_count = wheel_count
        self.has_face_concept = has_face_concept
        self.description = description
        # 多车叠加视图中该车型的专属颜色（HTML 十六进制）
        self.color = color
        # 四轮专用功能开关
        self.supports_oscillate = supports_oscillate
        # 机器人外形尺寸，用于中心点禁入区
        self.robot_length_cm = float(robot_length_cm)
        self.robot_width_cm = float(robot_width_cm)

    @property
    def file_path(self):
        return PROJECT_ROOT / self.file_name

    def __repr__(self):
        return "ChassisProfile(%s -> %s)" % (self.profile_id, self.file_name)


# ===== 注册表：在这里加 profile 即可在 GUI 出现 =====
OMNI3_PROFILE = ChassisProfile(
    profile_id="omni3",
    display_name="三轮全向（120° 对称）",
    file_name="mecanum_forward.py",
    wheel_count=3,
    has_face_concept=True,
    color="#2878D0",   # 蓝色
    description=(
        "M1（前左 150°）+ M2（前右 30°）+ M3（尾部 270°）。"
        "通过 R1/L1 切换正面方向。"
    ),
)

MECANUM_4W_PROFILE = ChassisProfile(
    profile_id="mecanum_4w",
    display_name="四轮麦克纳姆（含收球 / 滚球）",
    file_name="mecanum_drive.py",
    wheel_count=4,
    has_face_concept=False,
    supports_oscillate=True,
    robot_length_cm=50.0,
    robot_width_cm=48.0,
    color="#2E7D32",   # 绿色
    description=(
        "X 型麦克纳姆底盘（M1–M4）+ M5 滚球电机 + DC1/DC2 收球电机。"
        "前后左右对称，没有 face 概念；N1-N4 键用于执行机构，+ 键触发自动程序。"
    ),
)

CHASSIS_PROFILES = [OMNI3_PROFILE, MECANUM_4W_PROFILE]

# 默认选中第一个
DEFAULT_PROFILE_ID = OMNI3_PROFILE.profile_id

# 旧 JSON 兼容：早期四轮底盘曾使用过这些 ID
PROFILE_ID_ALIASES = {
    "mecanum_x": MECANUM_4W_PROFILE.profile_id,
    "mecanum_drive": MECANUM_4W_PROFILE.profile_id,
}


def normalize_profile_id(profile_id):
    return PROFILE_ID_ALIASES.get(profile_id, profile_id)


def get_profile(profile_id):
    """根据 ID 查找 profile；找不到时返回默认"""
    profile_id = normalize_profile_id(profile_id)
    for p in CHASSIS_PROFILES:
        if p.profile_id == profile_id:
            return p
    return OMNI3_PROFILE


# 向后兼容：旧代码用 ROBOT_FILE 直接拿默认机器人路径
ROBOT_FILE = OMNI3_PROFILE.file_path
