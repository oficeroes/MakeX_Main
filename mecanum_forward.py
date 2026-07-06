"""
三轮全向机器人 — 遥控操控程序 (v2.0 面映射版)
================================================
描述：基于 Novapi 平台的三轮全向底盘遥控程序。
      左摇杆控制全向移动（前进/后退/左右横移），
      右摇杆左右控制原地自旋，
      两个摇杆可同时操作实现复合运动（如边前进边转圈）。
      🆕 L1 换面后功能键自动重映射为当前面搭载设备的功能。

硬件需求：
  - 编码电机 ×5：M1（前左轮）、M2（前右轮）、M3（尾部轮）、M4（一号臂升降）、M5（二号臂升降）
  - 直流电机 ×1：DC1（收球履带）
  - 无刷电机 ×2：BL1 + BL2（发射小球，动力扩展板）
  - 智能舵机 ×2：M6-INDEX1（一号臂夹取）、M6-INDEX2（二号臂夹取）
  - 8×16 LED 点阵屏 ×1：PORT2-INDEX1

操控速查（v2.0 — 面映射版）：
  ┌─────────────────┬──────────────────────────────────────────────┐
  │    你想做的      │                 手柄操作                      │
  ├─────────────────┼──────────────────────────────────────────────┤
  │  前进/后退      │  左摇杆 ↑↓                                   │
  │  左右横移       │  左摇杆 ←→                                   │
  │  原地自旋       │  右摇杆 ←→                                   │
  │  顺时针换面     │  按 L1（车身旋转120°+功能键重映射）           │
  ├─────────────────┼──────────┬──────────┬────────────────────────┤
  │  按键 \\ 当前面  │  Face 0  │  Face 1  │  Face 2               │
  │                 │  (收球面) │ (二号臂) │  (一号臂)              │
  ├─────────────────┼──────────┼──────────┼────────────────────────┤
  │  D-pad ↑↓       │ BL调速±5 │ M5升降档 │ M4升降档               │
  │  D-pad ←→       │   闲置   │   闲置   │   闲置                 │
  │  键A (N2)       │ DC1收球  │ M5升降档 │ M4升降档               │
  │  键B (R2)       │ BL发射   │ SV2夹取  │ SV1夹取                │
  │  N3             │   闲置   │ M5回零   │ M4回零                 │
  ├─────────────────┼──────────┼──────────┼────────────────────────┤
  │  自动程序       │  按 +（加号键）                              │
  │  调试模式       │  按 ≡（菜单键切换）                          │
  └─────────────────┴──────────────────────────────────────────────┘

🆕 v2.0 更新：
  - L1 换面后 D-pad↑↓、键A(N2)、键B(R2)、N3 自动重映射
  - BL 无刷电机：换面时强制停止（安全），D-pad↑↓ 调速（Face 0 专用）
  - DC1 收球：键A(N2) 切换，换面保持状态不中断
  - M4/M5 双升降臂：各自独立 PID，换面不中断对方运动
  - SV1/SV2 双夹爪：键B(R2) 触发，各自独立自适应夹取

L1 换面说明：
  机器人有三个面（三角形三条边），默认 M1-M2 边为正面。
  按 L1 → 顺时针切换一个面（0→1→2→0），同时车身 PID 闭环旋转 120°。
  换面时：DC1 保持当前状态 ✅ | BL1/BL2 强制停止 🚨 | M4/M5 PID 各自继续

升降臂说明：
  一号臂：M4（升降）+ M6-INDEX1（夹取）— PID 控制 ✅
  二号臂：M5（升降）+ M6-INDEX2（夹取）— PID 控制 ✅
  两个臂完全独立，换面不中断对方正在进行的 PID 运动。

自适应夹爪说明（键B=R2，面映射）：
  Face 1 → SV2（二号臂）夹取 | Face 2 → SV1（一号臂）夹取
  按一下→缓慢夹紧 → 速度归零堵转=夹到 → 自动停电保持。
  再按→释放回全开位 0°。角度硬限位 300° 保护机械结构。

无刷电机说明：
  Face 0 → 键B(R2) 切换 BL1+BL2 发射，D-pad↑↓ 调速（10~100）。
  换面离开 Face 0 → BL 强制停止。

舵机铁律：
  🚨 servo_dir = -1 写死（负=夹紧，正=撑爆）
  🚨 夹紧幅度 ≤ 300°（机械极限）
  🚨 机器人无终端输出，LED 点阵屏是唯一用户界面

自动程序说明：
  按 + 键触发预设自动程序（运行期间遥控被禁用，程序结束后自动恢复）。

运动学模型：三轮全向 120° 对称布局
===================================
         前进方向 (+Vy)
              ↑
       M1 ←──┼──→ M2      (前左 M1, 前右 M2)
              │
              │
             M3            (尾部 M3)

  逆运动学方程（顺时针旋转为正）：
    M1_power = -0.5·Vx - 0.866·Vy - ω
    M2_power = -0.5·Vx + 0.866·Vy - ω
    M3_power =  1.0·Vx            - ω

  其中 Vx=横向速度(+右), Vy=纵向速度(+前), ω=旋转速度(+顺时针)
"""
import novapi
import time
import math
from mbuild import gamepad
from mbuild import power_expand_board
from mbuild.encoder_motor import encoder_motor_class
from mbuild.led_matrix import led_matrix_class

# ---- 智能舵机：可选硬件，机器人可能未安装 ----
try:
    from mbuild.smartservo import smartservo_class
    _HAS_SERVO = True
except ImportError:
    _HAS_SERVO = False
    smartservo_class = None

# ==================== 配置常量 ====================
SPEED_SCALE = 0.6       # 全局速度倍率 (0~1)，安全起见默认 60%
DEAD_ZONE = 8           # 摇杆死区阈值，小于此值的输入视为 0（防误触）
LOOP_DELAY = 0.02       # 主循环周期（秒），20ms = 50Hz

# 运动学系数（√3/2 ≈ 0.866）
SQRT3_OVER_2 = 0.866
HALF = 0.5

# 自动程序配置
AUTO_SPEED = 50            # 自动程序默认速度（0~100）
AUTO_STEP_DELAY = 0.02     # 自动程序步骤循环周期（秒）

# 收球直流电机配置
DC_COLLECTOR_PORT = "DC1"  # 动力扩展板通道（1号口）
DC_COLLECTOR_SPEED = -100  # 收球最大速度（负转收球，已翻转方向）

# 无刷电机配置（发射小球用，动力扩展板 BL1/BL2 通道）
BL_ACTION_SPEED = 80        # 无刷电机发射速度（10~100，R1 开关 / D-pad ←→ 调速）
BL_SPINUP_DELAY = 0.1       # 无刷电机启动延迟（秒）

# 8x16 LED 点阵屏配置（横屏：16列 × 8行）
LED_PORT = "PORT2"          # 点阵屏连接的 PORT 口（PORT1~PORT4）
LED_INDEX = "INDEX1"        # 端口链上的序号

# 调试模式配置
DEBUG_TEST_SPEED = 50        # 调试模式中电机测试速度
DEBUG_MOTOR_TYPES = ["编码电机", "直流电机", "升降臂", "舵机", "无刷电机"]
DEBUG_MOTOR_NAMES = ["M1", "M2", "M3", "M4"]       # 电机编号名称
DEBUG_DC_NAMES = ["DC1"]                           # 直流电机编号
DEBUG_BLDC_NAMES = ["BL1", "BL2", "BL12"]            # 无刷电机编号（BL12=双电机同时）
# 升降臂调试档位（编码器角度，1号臂M4和2号臂M5通用）
DEBUG_LIFT_GEARS = [0, 330, 660]     # 实测档位：L0=底部, L1=330°, L2=660°
DEBUG_LIFT_NAMES = ["L0", "L1", "L2"]
DEBUG_LIFT_SPEED = 80           # 升降移动速度 (rpm)
# 舵机调试（M6端口链 INDEX1/INDEX2）
# ╔══════════════════════════════════════════════════════════════╗
# ║  🚨 铁律 1：收爪必须用负角度！0°=全开零点，负=夹紧！   ║
# ║  🚨 铁律 2：夹紧幅度严禁超过 300°！超过→齿轮扫齿报废！ ║
# ║  servo_dir 写死 -1，任何时候都不要改成 +1！              ║
# ╚══════════════════════════════════════════════════════════════╝
DEBUG_SERVO_NAMES = ["SV1", "SV2"]                # 舵机编号
DEBUG_SERVO_ANGLE = 40              # 舵机测试角度（仅幅度，方向固定为负=夹紧）
SERVO_MAX_ANGLE = 300               # 🚨 机械极限！实测夹爪最大300°，超过报废
SERVO_MOVE_SPEED = 30               # 舵机移动速度
# 自适应夹爪参数（一号臂舵机 M6-INDEX1 + 二号臂舵机 M6-INDEX2，键B 触发）
# 检测策略：速度归零=堵转（夹到东西或到机械限位），比电流检测更抗摩擦干扰
GRIP_SPEED_THRESHOLD = 5    # 堵转速度阈值 (rpm)，低于此值=已夹到
GRIP_CLOSE_POWER = -25         # 夹紧功率（负=夹紧，慢速）
GRIP_TIMEOUT = 5.0             # 夹取超时（秒）
GRIP_SETTLE_TIME = 0.3         # 启动延迟（秒），避开电机启动阶段

# 功能键配置（可替换为任意物理按键，后续统一调整）
KEY_FUNC_A = "N2"           # 键A: Face0=DC1收球, Face1/2=升降档位切换
KEY_FUNC_B = "R2"           # 键B: Face0=BL发射, Face1/2=自适应夹爪
# 面→硬件映射（按 face 查表分发按键功能）
#   FACE_LIFT_MOTOR[face]: 当前面升降臂电机 (0=M4, 1=M5, None=无)
#   FACE_GRIP_SERVO[face]: 当前面夹爪舵机   (0=SV1, 1=SV2, None=无)
FACE_LIFT_MOTOR = [None, 1, 0]   # Face0=无, Face1=M5(idx1), Face2=M4(idx0)
FACE_GRIP_SERVO = [None, 1, 0]   # Face0=无, Face1=SV2(idx1), Face2=SV1(idx0)
DEBUG_ALL_NAMES = [DEBUG_MOTOR_NAMES, DEBUG_DC_NAMES, DEBUG_LIFT_NAMES, DEBUG_SERVO_NAMES, DEBUG_BLDC_NAMES]

# 升降臂 PID 控制参数（到位自动停电，机械张力维持位置）
LIFT_PID_KP = 0.35          # 比例增益
LIFT_PID_KI = 0.002         # 积分增益（消除静差）
LIFT_PID_KD = 0.05          # 微分增益（抑制过冲）
LIFT_MAX_SPEED = 100        # 最大输出功率 %（比之前更快）
LIFT_MIN_SPEED = 22         # 最小功率 %（克服静摩擦）
LIFT_DEADBAND = 6           # 到位死区（度）
LIFT_SETTLE_TIME = 0.3      # 稳定确认时间（秒），确认后停电
LIFT_TIMEOUT = 3.0          # 超时保护（秒）
LIFT_DECEL_ZONE = 0.35      # 余弦减速区间占比

# 标定模式配置（调试模式下 L1+R1 进入）
# 已测：编码 1000° = 直走 58 cm
CAL_DEG_PER_CM = 1000 / 58   # ≈ 17.24 编码度/厘米（改这个值来校准精度）
CAL_TARGET_CM = 20           # ← 你要机器人走多少厘米，改这里！
CAL_ANGLE = int(CAL_TARGET_CM * CAL_DEG_PER_CM)  # 自动换算成编码角度
CAL_MOVE_SPEED = 20          # 标定移动最大转速（rpm）

# S 曲线加速参数
CAL_RAMP_UP = 0.4            # 加速段占比（sin 加速）
CAL_RAMP_DOWN = 0.4          # 减速段占比（cos 减速）

# 左右 PID 补偿参数
CAL_PID_KP = 0.16             # 比例系数（左右进度差 → 速度修正）

# 正面切换 — 旋转矩阵常量（cos/sin of ±120°）
COS120 = -0.5
SIN120 = 0.866          # sin(120°) = √3/2

# 三个正面的速度旋转预设（预计算避免循环中重复运算）
# face_rotation[face] = (cos, -sin, sin, cos) → (Vx', Vy') = (cos*Vx - sin*Vy, sin*Vx + cos*Vy)
FACE_ROTATIONS = [
    (1.0,  0.0,  0.0,  1.0),   # Face 0: 0°   — M1-M2 边为正面（默认）
    (COS120, -SIN120, SIN120, COS120),  # Face 1: +120° — M2-M3 边为正面（右切）
    (COS120,  SIN120, -SIN120, COS120), # Face 2: -120° — M3-M1 边为正面（左切）
]

FACE_NAMES = [
    "Face0: M1-M2 正面",
    "Face1: M2-M3 正面",
    "Face2: M3-M1 正面",
]

# ==================== 硬件初始化 ====================
# M1: 前左轮（角度位置 150°）
# M2: 前右轮（角度位置 30°）
# M3: 尾部轮（角度位置 270° = -90°）
__motor_M1 = encoder_motor_class("M1", "INDEX1")
__motor_M2 = encoder_motor_class("M2", "INDEX1")
__motor_M3 = encoder_motor_class("M3", "INDEX1")
__motor_M4 = encoder_motor_class("M4", "INDEX1")  # M4: 一号臂升降编码电机
__motor_M5 = encoder_motor_class("M5", "INDEX1")  # M5: 二号臂升降编码电机

# 智能舵机（M6 端口链：INDEX1=一号臂夹取, INDEX2=二号臂夹取）
if _HAS_SERVO:
    try:
        __servo_1 = smartservo_class("M6", "INDEX1")  # 一号臂夹取
        __servo_2 = smartservo_class("M6", "INDEX2")  # 二号臂夹取
    except Exception:
        _HAS_SERVO = False

__led = led_matrix_class(LED_PORT, LED_INDEX)     # 8x16 点阵屏

# 开机时舵机归零位（全开安全位，避免之前测试残留角度）
if _HAS_SERVO:
    try:
        __servo_1.move_to(0, 30)
        print(">>> 舵机开机归零: SV1 → 0° (全开)")
    except Exception:
        pass
    try:
        __servo_2.move_to(0, 30)
        print(">>> 舵机开机归零: SV2 → 0° (全开)")
    except Exception:
        pass

# ==================== 正面状态 ====================
face = 0  # 当前正面: 0=M1-M2, 1=M2-M3, 2=M3-M1

# ==================== 自动程序 ====================
# 自动程序步骤列表：每步 = (持续时间_秒, Vx, Vy, omega)
#   Vx: 横向速度 (+右)，Vy: 纵向速度 (+前)，omega: 旋转速度 (+顺时针)
#   最后一个步骤建议设为 (0.1, 0, 0, 0) 作为停止缓冲
#
# AUTO_RAMP_MS：步间速度线性插值时长（毫秒）
#   每进入新步骤时，前 RAMP_MS 毫秒内速度从上一步线性插值到当前步，
#   避免瞬间跳变带来的电流冲击 / 轮子打滑 / 机身震动。
#   0 = 立即切换（旧行为）；100ms 是常用值；200ms 最软但路径偏差稍大。
#   该常量会被 trajectory_planner 导出时覆盖。
AUTO_RAMP_MS = 0
AUTO_ACCEL_MS = 0               # ms: within-step 0→100→0 acceleration ramp (防打滑)
ENCODER_TICKS_PER_CM = 18.0000

# 摇杆指数曲线: 1.0=线性, 2.0=二次, 3.0=三次 (推荐1.5~2.5)
JOYSTICK_EXPO = 2.0
OMEGA_EXPO = 2.0

# 方向取反常量（若电机实际转向与预期相反，改这里）
INVERT_VX = False
INVERT_VY = True
INVERT_OMEGA = False

# ==================== 编码器闭环参数（基于 novapi get_value("angle") 角度制）====================
# Novapi move() 使用编码器角度。这里必须用实测角度/cm，
# 不能用理想轮周 360/(πD)，否则实际距离会明显偏小。
_DEG_PER_CM = ENCODER_TICKS_PER_CM
_TICKS_TO_DEG = 1.0
# 到位死区（度）：电机角度误差小于此值视为到达
PROFILE_DEADBAND_DEG = 8
# 每步最大等待时间（秒），防止卡死
_ENC_STEP_TIMEOUT = 8.0
# 功率 → rpm 映射（move(deg, rpm) 的速度参数）
_RPM_PER_POWER = 8.0

# ==================== M4/M5 升降臂配置 ====================
# M4: 一号臂升降编码电机（PID 控制待接入）
# M5: 二号臂升降编码电机（PID 控制已接入）

# ==================== 面切换旋转配置（编码器闭环 + PID）====================
# FACE_ROTATE_WHEEL_DEG: 120° 底盘旋转 → 每个轮子需要转动的角度（度）
#   理论值 = 120° × (机器人半径 ÷ 轮子半径)
#   实测校准方法：
#     1. 设一个初始值（如 360），上传程序
#     2. 按 L1 触发换面旋转
#     3. 观察机器人实际转了多少度
#     4. 按比例调整：新值 = 旧值 × (120° ÷ 实际转角)
#     5. 重复直到精准
FACE_ROTATE_WHEEL_DEG = 940       # ← 120°旋转对应轮子角度，需实测校准！

# PID 控制器参数（面切换旋转专用）
FACE_ROTATE_PID_KP = 0.2        # 比例系数：快速响应主力
FACE_ROTATE_PID_KI = 0.001       # 积分系数：消除末尾静差（0=无积分）
FACE_ROTATE_PID_KD = 0.02       # 微分系数：抑制过冲抖动
FACE_ROTATE_MAX_SPEED = 78       # 旋转最大速度（-100~100）
FACE_ROTATE_MIN_SPEED = 12       # 旋转最小速度（克服静摩擦，太低→转不动）
FACE_ROTATE_DEADBAND = 8         # 到位死区（编码角度），越小越准
FACE_ROTATE_TIMEOUT = 0.6        # 最大旋转时间（秒），给足时间让它跑完
FACE_ROTATE_SETTLE_TIME = 0.3   # 到位后稳定确认（秒）
FACE_ROTATE_DECEL_ZONE = 0.40    # 余弦减速区间：最后30%误差时开始丝滑降速

# 面切换后 LED 显示时长
FACE_SHOW_MS = 800                # 换面完成后显示面名称的毫秒数

FRONT_BACK_COMPENSATION = 0.0000
ROTATION_BALANCE = 0.0000
STRAFE_VY_COUPLING = 0.0000
AUTO_SEQUENCE = [
    # === Auto-generated by trajectory_planner on 2026-07-06 15:29 ===
    # Source: (unsaved)  |  Format: v5-encoder
    # Mode: translation | speed: 100.0 cm/s | power: 85
    ('enc_move', 90, 29, 33),
    ('enc_move', 630, 54, 63),
    ('enc_move', 720, 37, 76),
    ('enc_move', 88, 45, 72),
    ('enc_move', 89, 85, -1),
    ('enc_move', 360, 85, -9),
    ('enc_move', 48, 77, 37),
    ('enc_move', 539, -2, 85),
    ('enc_move', 179, -7, -85),
    ('enc_move', 360, -31, -79),
    ('enc_move', 180, -6, -85),
    ('enc_move', 55, 50, -68),
    ('enc_move', 540, 66, 52),
    ('enc_move', 180, 78, 33),
    ('enc_move', 159, -79, 32),
    ('enc_move', 540, -72, 45),
    ('enc_move', 34, -46, -72),
    ('enc_move', 630, 38, -74),
    ('enc_move', 67, 18, -35),
    ('enc_stop', 0, 0, 0),
]

# 自动程序运行时状态
auto_mode = False        # 当前是否在自动模式
auto_step = 0            # 当前步骤索引
auto_step_start = 0.0    # 当前步骤开始时间

# 速度过渡插值：记录"上一步结束时"实际输出的速度，作为新步骤的插值起点
auto_prev_Vx = 0.0       # 上一步的 Vx（已经经过 face 旋转后的值）
auto_prev_Vy = 0.0       # 上一步的 Vy
auto_prev_omega = 0.0    # 上一步的 omega

# 编码器闭环状态（用于 enc_move / enc_rot 标签）
_motors = None                     # 延迟初始化，第一次调用时赋值
_enc_targets = [0, 0, 0]           # M1-M3 目标角度（绝对角度，度）
_enc_step_initialized = -1          # 已初始化的 auto_step 索引
_enc_step_start_time = 0.0          # 当前步开始时刻

# ==================== 收球开关状态 ====================
collector_on = False     # DC1 收球电机当前状态: False=关, True=开（换面保持）
lift_gear = [0, 0]       # [M4一号臂, M5二号臂] 升降档位: 0=L0(底部), 1=L1(330°), 2=L2(660°)

# 无刷电机状态（BL1/BL2 发射小球）
bl1_on = False            # BL1 无刷电机开关
bl2_on = False            # BL2 无刷电机开关

# 自适应夹爪状态 [0]=SV1一号臂, [1]=SV2二号臂（键B 触发）
_grip_active = [False, False]    # 切换开关
_grip_closing = [False, False]   # 正在夹紧中（监测速度）
_grip_hold = [False, False]      # 已夹到东西，保持中
_grip_start_time = [0.0, 0.0]    # 夹取开始时刻
_grip_settled = [False, False]   # 启动延迟完成
_grip_last_print = [0.0, 0.0]    # 上次打印速度的时刻

# 升降臂 PID 状态 [0]=M4一号臂, [1]=M5二号臂（到位后自动停电，避免电机过热）
# 两个臂完全独立：换面不中断对方运动，各自到位各自停电
_lift_pid_active = [False, False]
_lift_pid_target = [0.0, 0.0]
_lift_pid_integral = [0.0, 0.0]
_lift_pid_last_error = [0.0, 0.0]
_lift_pid_last_time = [0.0, 0.0]
_lift_pid_settle_start = [0.0, 0.0]
_lift_pid_settled = [False, False]
_lift_pid_start_time = [0.0, 0.0]

# ==================== 面切换旋转状态（L1 触发，编码器闭环 + PID）====================
face_rotating = False           # True = 正在执行面切换旋转
face_rot_start_enc = [0, 0, 0]  # 旋转起始编码值（M1, M2, M3）
face_rot_target_deg = 0.0       # 目标编码增量（度）
face_rot_pid_integral = 0.0     # PID 积分累加
face_rot_last_error = 0.0       # PID 上次误差
face_rot_last_time = 0.0        # PID 上次时间
face_rot_settle_start = 0.0     # 到位稳定计时起点
face_rot_settled = False        # 是否已进入稳定阶段
face_rot_start_time = 0.0       # 旋转开始时刻（超时用）
face_show_until = 0.0           # 换面完成 LED 显示保持时刻

# ==================== 调试模式状态 ====================
debug_mode = False            # 当前是否在调试模式（≡ 键切换）
debug_motor_type = 0          # 0=编码电机, 1=直流电机, 2=升降臂, 3=舵机
debug_motor_index = 0         # 当前子项索引
debug_show_speed_until = 0.0  # 速度显示保持时刻
debug_show_ok = False         # OK 显示状态
debug_cal_mode = False        # 是否在标定子模式
debug_cal_running = False     # 标定移动是否正在执行
debug_cal_M1_start = 0.0      # M1 起始编码角
debug_cal_M2_start = 0.0      # M2 起始编码角
debug_cal_M1_target = 0.0     # M1 目标编码增量
debug_cal_M2_target = 0.0     # M2 目标编码增量
debug_cal_done_time = 0.0     # 标定完成时刻（显示最终值用）
debug_cal_auto_start = False   # 是否等待自动启动
debug_cal_auto_time = 0.0      # 自动启动倒计时起始时刻
debug_lift_gear_index = 0       # 当前选中的升降档位
servo_dir = -1                 # 🚨 写死 -1（夹紧）！改 +1 会撑爆夹爪！

# D-pad 冲突保护（调试模式）
_dpad_lr_conflict = 0           # 左右冲突连续计数
_dpad_ud_conflict = 0           # 上下冲突连续计数

# ==================== 编码器标定（GUI 导出 'encoder_cal' 触发） ====================
_cal_delta_M1 = None            # 标定后 M1 编码增量
_cal_delta_M2 = None            # 标定后 M2 编码增量
_cal_delta_M3 = None            # 标定后 M3 编码增量
_debug_cal_done = False         # True = 标定完成，N1/N2/N3 可查询增量

# 自动模式编码器标定状态（全局变量，避免 MicroPython 对象属性兼容问题）
_enc_cal_auto_started = False   # True = 首帧已完成
_enc_cal_M1_start = 0.0         # M1 起始角度快照
_enc_cal_M2_start = 0.0         # M2 起始角度快照
_enc_cal_M3_start = 0.0         # M3 起始角度快照

# ==================== 摇杆响应曲线 ====================
def apply_response_curve(value, exponent=JOYSTICK_EXPO, max_input=100.0):
    """WPILib 风格的指数响应曲线。

    output = sign(value) * (|value| / max_input)^exponent * max_input

    示例 (exponent=2.0, max_input=100):
      摇杆 25% → 6.25%  (极细腻)
      摇杆 50% → 25%    (柔和)
      摇杆 75% → 56.25%
      摇杆 100% → 100%  (满功率)
    """
    if value == 0:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    normalized = abs(value) / max_input
    curved = normalized ** exponent
    return sign * curved * max_input


# ==================== 运动学函数 ====================
def rotate_velocity(Vx, Vy, face_index):
    """
    根据当前正面，将摇杆速度向量旋转到对应坐标系

    参数:
        Vx, Vy: 摇杆原始速度（已缩放、取反）
        face_index: 当前正面编号 (0/1/2)

    返回:
        (Vx', Vy'): 旋转后的速度向量
    """
    c, ns, s, nc = FACE_ROTATIONS[face_index]
    # Vx' = c*Vx + ns*Vy  (ns = -sinθ)
    # Vy' = s*Vx + nc*Vy  (nc = cosθ)
    return c * Vx + ns * Vy, s * Vx + nc * Vy


def omni_kinematics(Vx, Vy, omega):
    """
    三轮全向逆运动学计算

    参数:
        Vx  (float): 横向目标速度（+右, -左），范围 -100~100
        Vy  (float): 纵向目标速度（+前, -后），范   围 -100~100
                     注意：调用方需根据实际电机接线决定是否对 Vy 取反
        omega (float): 旋转目标速度（+顺时针, -逆时针），范围 -100~100

    返回:
        tuple: (M1_power, M2_power, M3_power)，范围 -100~100
    """
    M1 = -HALF * Vx - SQRT3_OVER_2 * Vy - omega
    M2 = -HALF * Vx + SQRT3_OVER_2 * Vy - omega
    M3 =  Vx                            - omega

    # 向量等比缩放：保持方向不变，速度等比降低
    max_abs = max(abs(M1), abs(M2), abs(M3))
    if max_abs > 100:
        scale = 100.0 / max_abs
        M1 *= scale
        M2 *= scale
        M3 *= scale

    return M1, M2, M3


def apply_dead_zone(value, threshold=DEAD_ZONE):
    """摇杆死区过滤：绝对值小于阈值的值置零"""
    if abs(value) < threshold:
        return 0
    return value


def stop_all_motors():
    """紧急停止所有电机（含 M4 收球、M5 升降臂、舵机、无刷电机）"""
    __motor_M1.set_power(0)
    __motor_M2.set_power(0)
    __motor_M3.set_power(0)
    __motor_M4.set_power(0)
    __motor_M5.set_power(0)
    power_expand_board.stop("BL1")
    power_expand_board.stop("BL2")
    if _HAS_SERVO:
        try:
            __servo_1.set_power(0)
            __servo_2.set_power(0)
        except Exception:
            pass


def debug_stop_motor():
    """停止调试模式中当前选中的测试电机"""
    if debug_motor_type == 0:  # 编码电机
        motor = [__motor_M1, __motor_M2, __motor_M3, __motor_M4][debug_motor_index]
        motor.set_power(0)
    elif debug_motor_type == 1:  # 直流电机
        power_expand_board.set_power(DC_COLLECTOR_PORT, 0)
    elif debug_motor_type == 2:  # 升降臂
        __motor_M5.set_power(0)
    elif debug_motor_type == 3:  # 舵机
        if _HAS_SERVO:
            try:
                servo = [__servo_1, __servo_2][debug_motor_index]
                servo.set_power(0)
            except Exception:
                pass
    elif debug_motor_type == 4:  # 无刷电机
        if debug_motor_index == 2:  # BL12：双电机同时
            power_expand_board.stop("BL1")
            power_expand_board.stop("BL2")
            power_expand_board.set_power(DC_COLLECTOR_PORT, 0)  # DC1 也停
        else:
            port = ["BL1", "BL2"][debug_motor_index]
            power_expand_board.stop(port)


def debug_cal_start(angle):
    """
    启动标定移动：记录起始编码值并计算目标增量（非阻塞）

    运动学分配:
        M1(前左): 反转 0.866*angle
        M2(前右): 正转 0.866*angle
        M3(尾部): 不动
    """
    global debug_cal_running, debug_cal_M1_start, debug_cal_M2_start
    global debug_cal_M1_target, debug_cal_M2_target

    debug_cal_M1_start = __motor_M1.get_value("angle")
    debug_cal_M2_start = __motor_M2.get_value("angle")
    debug_cal_M1_target = -SQRT3_OVER_2 * angle
    debug_cal_M2_target =  SQRT3_OVER_2 * angle
    debug_cal_running = True

    print(">>> 标定移动: M1=%.0f° M2=%.0f° M3=0° (max_speed=%d rpm)" %
          (debug_cal_M1_target, debug_cal_M2_target, CAL_MOVE_SPEED))


def debug_cal_tick():
    """
    标定移动单步更新（每循环调用一次，非阻塞）
    返回: True=仍在运行, False=已完成
    """
    global debug_cal_running, debug_cal_done_time

    # 读取当前编码位置
    M1_cur = __motor_M1.get_value("angle") - debug_cal_M1_start
    M2_cur = __motor_M2.get_value("angle") - debug_cal_M2_start

    # 计算进度（0→1）
    if abs(debug_cal_M1_target) > 0.1:
        progress_M1 = abs(M1_cur / debug_cal_M1_target)
    else:
        progress_M1 = 1.0

    if abs(debug_cal_M2_target) > 0.1:
        progress_M2 = abs(M2_cur / debug_cal_M2_target)
    else:
        progress_M2 = 1.0

    progress = min(progress_M1, progress_M2)

    # 检查是否完成
    if progress >= 0.995:
        __motor_M1.set_power(0)
        __motor_M2.set_power(0)
        debug_cal_running = False
        debug_cal_done_time = novapi.timer()
        M1_final = __motor_M1.get_value("angle") - debug_cal_M1_start
        M2_final = __motor_M2.get_value("angle") - debug_cal_M2_start
        # 存储标定结果到统一变量，供 N1/N2/N3 查询
        _cal_delta_M1 = M1_final
        _cal_delta_M2 = M2_final
        _cal_delta_M3 = __motor_M3.get_value("angle") - 0  # M3 不动
        _debug_cal_done = True
        print(">>> 标定完成! M1实际=%.0f° M2实际=%.0f°" % (M1_final, M2_final))
        return False

    # ==== S 曲线速度因子 ====
    if progress < CAL_RAMP_UP:
        # sin 加速: 0.15 → 1（最低 15% 克服静摩擦）
        factor = 0.15 + 0.85 * math.sin(progress / CAL_RAMP_UP * math.pi / 2.0)
    elif progress < 1.0 - CAL_RAMP_DOWN:
        # 匀速段
        factor = 1.0
    else:
        # cos 减速: 1 → 0.15
        p_dec = (progress - (1.0 - CAL_RAMP_DOWN)) / CAL_RAMP_DOWN
        factor = 0.15 + 0.85 * math.cos(p_dec * math.pi / 2.0)

    base_speed = CAL_MOVE_SPEED * factor

    # ==== 左右 PID 补偿 ====
    error = progress_M1 - progress_M2   # M1超前 → 正值 → 减速M1加速M2
    correction = CAL_PID_KP * error * CAL_MOVE_SPEED

    M1_speed = base_speed - correction
    M2_speed = base_speed + correction

    # 限幅并确定方向
    M1_power = max(-100, min(100, M1_speed if debug_cal_M1_target > 0 else -M1_speed))
    M2_power = max(-100, min(100, M2_speed if debug_cal_M2_target > 0 else -M2_speed))

    # 最低速度兜底（避免完全停转，15% 克服静摩擦）
    if abs(M1_power) < 15 and progress < 0.9:
        M1_power = 15 if debug_cal_M1_target > 0 else -15
    if abs(M2_power) < 15 and progress < 0.9:
        M2_power = 15 if debug_cal_M2_target > 0 else -15

    __motor_M1.set_power(M1_power)
    __motor_M2.set_power(M2_power)

    return True


# ==================== 编码器闭环函数（基于 novapi move() + get_value("angle")）====================
# novapi 编码电机有内置位置闭环 move(degrees, rpm)，无需自己写 PID！
# get_value("angle") 返回当前角度（度），move() 是非阻塞的相对角度转动。

def _get_motors():
    """返回三轮电机列表（延迟初始化，避免模块加载顺序问题）"""
    global _motors
    if _motors is None:
        _motors = [__motor_M1, __motor_M2, __motor_M3]
    return _motors


def _enc_move_start(total_ticks, vy_power):
    """初始化编码器直走：计算目标角度 → 调用 novapi move()

    total_ticks: GUI 导出的 ticks 值（ticks = cm × ENCODER_TICKS_PER_CM）
    vy_power:   前进功率百分比（用于确定方向和速度）
    """
    global _enc_targets, _enc_step_start_time

    motors = _get_motors()

    # ticks → cm → 度
    distance_cm = abs(total_ticks) / ENCODER_TICKS_PER_CM
    total_degrees = distance_cm * _DEG_PER_CM  # 每个轮子转动的角度

    # 运动方向由 total_ticks 符号决定（正=前进，负=后退）
    direction = 1 if total_ticks >= 0 else -1

    # 用运动学计算各轮方向
    vy_dir = abs(vy_power) * direction  # 带方向的 Vy 功率
    M1p, M2p, M3p = omni_kinematics(0, vy_dir, 0)
    signs = [1 if M1p >= 0 else -1, 1 if M2p >= 0 else -1,
             1 if M3p >= 0 else -1]

    # 记录起始位置 → 计算目标绝对角度
    starts = [m.get_value("angle") for m in motors]
    _enc_targets = [int(starts[i] + signs[i] * total_degrees) for i in range(3)]

    # 速度：功率% → rpm
    motor_rpm = max(30, int(abs(vy_power) * _RPM_PER_POWER))

    # 调用 novapi 内置位置闭环：非阻塞，电机会自动停到目标
    for i in range(3):
        try:
            motors[i].move(int(signs[i] * total_degrees), motor_rpm)
        except Exception:
            pass

    _enc_step_start_time = novapi.timer()
    print(">>> enc_move: %.1fcm (%d°) @%drpm  dir=%d" %
          (distance_cm, int(total_degrees), motor_rpm, direction))


def _enc_move_tick(max_power, stop_motors=True):
    """编码器直走单步检测 → 返回 True=已到达目标

    novapi 的 move() 会自动减速停在目标角度，这里只需检查是否到位。
    加入超时保护防止卡死。

    stop_motors=False 时到达目标不刹停，用于连续 enc_move 衔接。
    """
    motors = _get_motors()

    # 超时保护
    if novapi.timer() - _enc_step_start_time > _ENC_STEP_TIMEOUT:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        print(">>> enc_move: 超时，强制进入下一步")
        return True

    # 检查三轮是否到位
    all_done = True
    for i in range(3):
        try:
            cur = motors[i].get_value("angle")
            if abs(_enc_targets[i] - cur) > PROFILE_DEADBAND_DEG:
                all_done = False
                break
        except Exception:
            pass

    if all_done:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True

    return False


def _enc_rot_start(total_ticks, omega_power):
    """初始化编码器自旋：计算目标角度 → 调用 novapi move()"""
    global _enc_targets, _enc_step_start_time

    motors = _get_motors()

    # ticks → 旋转角度（度）
    distance_cm = abs(total_ticks) / ENCODER_TICKS_PER_CM
    total_degrees = distance_cm * _DEG_PER_CM

    direction = 1 if total_ticks >= 0 else -1

    # 自旋运动学
    w_dir = abs(omega_power) * direction
    M1p, M2p, M3p = omni_kinematics(0, 0, w_dir)
    signs = [1 if M1p >= 0 else -1, 1 if M2p >= 0 else -1,
             1 if M3p >= 0 else -1]

    starts = [m.get_value("angle") for m in motors]
    _enc_targets = [int(starts[i] + signs[i] * total_degrees) for i in range(3)]

    motor_rpm = max(30, int(abs(omega_power) * _RPM_PER_POWER))

    for i in range(3):
        try:
            motors[i].move(int(signs[i] * total_degrees), motor_rpm)
        except Exception:
            pass

    _enc_step_start_time = novapi.timer()
    print(">>> enc_rot: %.1f° @%drpm" % (total_degrees, motor_rpm))


def _enc_rot_tick(max_power, stop_motors=True):
    """编码器自旋单步检测 → 返回 True=已到达目标（支持连续衔接）"""
    return _enc_move_tick(max_power, stop_motors=stop_motors)


# ==================== 面切换旋转（编码器闭环 + PID 控制）====================
def face_rotate_start():
    """
    启动面切换旋转：记录起始编码值，初始化 PID 状态。
    顺时针旋转 120°（对应一个面的切换）。

    运动学：Vx=0, Vy=0，omega>0（顺时针），三轮等速反转。
    """
    global face_rotating, face_rot_start_enc, face_rot_target_deg
    global face_rot_pid_integral, face_rot_last_error, face_rot_last_time
    global face_rot_settle_start, face_rot_settled, face_rot_start_time

    face_rot_start_enc = [
        __motor_M1.get_value("angle"),
        __motor_M2.get_value("angle"),
        __motor_M3.get_value("angle"),
    ]
    face_rot_target_deg = FACE_ROTATE_WHEEL_DEG
    face_rot_pid_integral = 0.0
    face_rot_last_error = 0.0
    face_rot_last_time = novapi.timer()
    face_rot_settle_start = 0.0
    face_rot_settled = False
    face_rot_start_time = novapi.timer()
    face_rotating = True

    print(">>> 面切换旋转启动！目标=%.0f°/轮 (120°底盘)" % face_rot_target_deg)


def face_rotate_tick():
    """
    面切换旋转 PID 单步更新（每主循环调用一次）。
    返回: True=旋转已完成, False=仍在旋转中。

    PID 控制器调节 omega（旋转速度），三轮等速驱动。
    接近目标时余弦减速曲线平滑降速 → 到位后稳定确认 → 完成。
    """
    global face_rotating, face_rot_pid_integral, face_rot_last_error
    global face_rot_last_time, face_rot_settle_start, face_rot_settled

    now = novapi.timer()

    # --- 超时保护 ---
    if now - face_rot_start_time > FACE_ROTATE_TIMEOUT:
        stop_all_motors()
        face_rotating = False
        print(">>> 面切换旋转超时！强制停止")
        return True

    # --- 读取当前编码增量（取三轮绝对增量的平均值）---
    M1_delta = abs(__motor_M1.get_value("angle") - face_rot_start_enc[0])
    M2_delta = abs(__motor_M2.get_value("angle") - face_rot_start_enc[1])
    M3_delta = abs(__motor_M3.get_value("angle") - face_rot_start_enc[2])
    avg_delta = (M1_delta + M2_delta + M3_delta) / 3.0

    # --- 到位检测 + 稳定阶段 ---
    error = face_rot_target_deg - avg_delta

    if abs(error) <= FACE_ROTATE_DEADBAND:
        if not face_rot_settled:
            face_rot_settled = True
            face_rot_settle_start = now
        elif now - face_rot_settle_start >= FACE_ROTATE_SETTLE_TIME:
            stop_all_motors()
            face_rotating = False
            print(">>> 面切换旋转完成！轮均=%.0f° (误差=%.1f°)" % (avg_delta, error))
            return True
        # 稳定中：保持停转，清零积分防止积压
        stop_all_motors()
        face_rot_pid_integral = 0.0
        face_rot_last_error = 0.0
        return False
    else:
        face_rot_settled = False
        face_rot_settle_start = 0.0

    # --- PID 控制 ---
    dt = now - face_rot_last_time
    if dt <= 0:
        dt = LOOP_DELAY
    face_rot_last_time = now

    # 积分项（带限幅防饱和，KI=0 时跳过）
    if FACE_ROTATE_PID_KI > 0:
        face_rot_pid_integral += error * dt
        max_integral = FACE_ROTATE_MAX_SPEED / FACE_ROTATE_PID_KI
        face_rot_pid_integral = max(-max_integral, min(max_integral, face_rot_pid_integral))

    # 微分项
    derivative = (error - face_rot_last_error) / dt if dt > 0 else 0
    face_rot_last_error = error

    # PID 输出
    omega = (FACE_ROTATE_PID_KP * error
             + FACE_ROTATE_PID_KI * face_rot_pid_integral
             + FACE_ROTATE_PID_KD * derivative)

    # --- 余弦减速曲线：接近目标时丝滑降速 ---
    decel_threshold = face_rot_target_deg * FACE_ROTATE_DECEL_ZONE
    if 0 < error < decel_threshold:
        # 在最后 30% 误差区间内，用 cos 曲线从 max 平滑降到 min
        decel_progress = 1.0 - (error / decel_threshold)  # 0(刚进入) → 1(到达)
        speed_limit = (FACE_ROTATE_MIN_SPEED
                       + (FACE_ROTATE_MAX_SPEED - FACE_ROTATE_MIN_SPEED)
                       * math.cos(decel_progress * math.pi / 2.0))
        omega = max(-speed_limit, min(speed_limit, omega))

    # 硬限幅（常规阶段用 MAX_SPEED）
    omega = max(-FACE_ROTATE_MAX_SPEED, min(FACE_ROTATE_MAX_SPEED, omega))

    # 最小速度兜底（克服静摩擦，方向正确）
    if abs(omega) < FACE_ROTATE_MIN_SPEED and abs(error) > FACE_ROTATE_DEADBAND:
        omega = FACE_ROTATE_MIN_SPEED if error > 0 else -FACE_ROTATE_MIN_SPEED

    # 方向校准：面切换需要顺时针（右转），但本车电机正 omega = 左转，故取反
    omega = -omega

    # --- 应用旋转：Vx=0, Vy=0, omega=PID输出 ---
    M1p, M2p, M3p = omni_kinematics(0, 0, omega)
    __motor_M1.set_power(M1p)
    __motor_M2.set_power(M2p)
    __motor_M3.set_power(M3p)

    return False


# ==================== 升降臂 PID 控制（双通道：M4一号臂 + M5二号臂）====================
# 两个臂完全独立：各自 PID、各自到位停电、换面不中断对方运动
# 到位后自动停电，依靠机械蜗杆自锁维持位置，避免电机过热

# 电机引用表（与 PID 状态数组同索引）
_LIFT_MOTORS = [None, None]  # 延迟初始化，避免模块加载顺序问题

def _init_lift_motors():
    """延迟初始化升降臂电机引用（避免硬件未就绪时访问）"""
    global _LIFT_MOTORS
    if _LIFT_MOTORS[0] is None:
        _LIFT_MOTORS = [__motor_M4, __motor_M5]

def lift_pid_start(motor_idx, target_deg):
    """启动升降臂 PID 控制
    motor_idx: 0=M4(一号臂), 1=M5(二号臂)
    target_deg: 目标编码器角度（度）
    """
    global _lift_pid_active, _lift_pid_target
    global _lift_pid_integral, _lift_pid_last_error, _lift_pid_last_time
    global _lift_pid_settle_start, _lift_pid_settled, _lift_pid_start_time

    _init_lift_motors()
    _lift_pid_target[motor_idx] = target_deg
    _lift_pid_integral[motor_idx] = 0.0
    _lift_pid_last_error[motor_idx] = 0.0
    _lift_pid_last_time[motor_idx] = novapi.timer()
    _lift_pid_settle_start[motor_idx] = 0.0
    _lift_pid_settled[motor_idx] = False
    _lift_pid_start_time[motor_idx] = novapi.timer()
    _lift_pid_active[motor_idx] = True


def lift_pid_tick(motor_idx):
    """升降臂 PID 单步更新。返回 True=已到位停电, False=仍在控制中
    motor_idx: 0=M4(一号臂), 1=M5(二号臂)
    """
    global _lift_pid_integral, _lift_pid_last_error, _lift_pid_last_time
    global _lift_pid_settle_start, _lift_pid_settled, _lift_pid_active

    _init_lift_motors()
    motor = _LIFT_MOTORS[motor_idx]

    if not _lift_pid_active[motor_idx]:
        return True

    now = novapi.timer()

    # 超时保护
    if now - _lift_pid_start_time[motor_idx] > LIFT_TIMEOUT:
        motor.set_power(0)
        _lift_pid_active[motor_idx] = False
        arm_name = "M4一号臂" if motor_idx == 0 else "M5二号臂"
        print(">>> 升降臂PID(%s): 超时强制停电" % arm_name)
        return True

    cur_deg = motor.get_value("angle")
    error = _lift_pid_target[motor_idx] - cur_deg

    # 到位检测 + 稳定确认
    if abs(error) <= LIFT_DEADBAND:
        if not _lift_pid_settled[motor_idx]:
            _lift_pid_settled[motor_idx] = True
            _lift_pid_settle_start[motor_idx] = now
        elif now - _lift_pid_settle_start[motor_idx] >= LIFT_SETTLE_TIME:
            motor.set_power(0)
            _lift_pid_active[motor_idx] = False
            _lift_pid_settled[motor_idx] = False
            arm_name = "M4" if motor_idx == 0 else "M5"
            print(">>> 升降臂PID(%s): 到位停电 (误差=%.1f°)" % (arm_name, error))
            return True
        # 稳定中：保持零功率，清零积分
        motor.set_power(0)
        _lift_pid_integral[motor_idx] = 0.0
        _lift_pid_last_error[motor_idx] = 0.0
        return False
    else:
        _lift_pid_settled[motor_idx] = False
        _lift_pid_settle_start[motor_idx] = 0.0

    # PID 计算
    dt = now - _lift_pid_last_time[motor_idx]
    if dt <= 0:
        dt = LOOP_DELAY
    _lift_pid_last_time[motor_idx] = now

    if LIFT_PID_KI > 0:
        _lift_pid_integral[motor_idx] += error * dt
        max_integral = LIFT_MAX_SPEED / LIFT_PID_KI
        _lift_pid_integral[motor_idx] = max(-max_integral, min(max_integral, _lift_pid_integral[motor_idx]))

    derivative = (error - _lift_pid_last_error[motor_idx]) / dt if dt > 0 else 0
    _lift_pid_last_error[motor_idx] = error

    power = (LIFT_PID_KP * error
             + LIFT_PID_KI * _lift_pid_integral[motor_idx]
             + LIFT_PID_KD * derivative)

    # 余弦减速
    decel_threshold = max(abs(_lift_pid_target[motor_idx]) * LIFT_DECEL_ZONE, LIFT_DEADBAND * 4)
    if abs(error) < decel_threshold:
        decel_progress = 1.0 - (abs(error) / max(decel_threshold, 1.0))
        speed_limit = LIFT_MIN_SPEED + (LIFT_MAX_SPEED - LIFT_MIN_SPEED) * math.cos(decel_progress * math.pi / 2.0)
        power = max(-speed_limit, min(speed_limit, power))

    # 硬限幅
    power = max(-LIFT_MAX_SPEED, min(LIFT_MAX_SPEED, power))

    # 最小速度兜底
    if abs(power) < LIFT_MIN_SPEED and abs(error) > LIFT_DEADBAND:
        power = LIFT_MIN_SPEED if error > 0 else -LIFT_MIN_SPEED

    motor.set_power(power)
    return False


def lift_pid_tick_all():
    """更新所有升降臂 PID（每帧调用一次）"""
    lift_pid_tick(0)  # M4 一号臂
    lift_pid_tick(1)  # M5 二号臂


# ==================== 边沿触发辅助变量 ====================
last_R1 = False
last_L1 = False
last_Plus = False
last_Menu = False
last_N1 = False

# 调试模式按键边沿变量
last_Up = False
last_Down = False
last_Left = False
last_Right = False
last_N2 = False
last_N3 = False
last_N4 = False
last_R2 = False           # 正常模式 R2 边沿
last_L2 = False           # 正常模式 L2 边沿
last_L1_debug = False
last_R1_debug = False

# 正常模式 D-pad 边沿
last_Dpad_Up_norm = False
last_Dpad_Down_norm = False
last_Dpad_Left_norm = False
last_Dpad_Right_norm = False

# ==================== 启动确认 ====================
print("=" * 40)
print("  三轮全向机器人已启动！")
print("  左摇杆 → 全向移动")
print("  右摇杆 ←→ 自旋")
print("  L1 → 顺时针换面（PID旋转120°+功能键重映射）")
print("  Face 0(收球面): D-pad ↑↓=BL调速 键A(N2)=DC1收球 键B(R2)=BL发射")
print("  Face 1(二号臂): D-pad ↑↓=M5升降 键A(N2)=档位 键B(R2)=夹取 N3=回零")
print("  Face 2(一号臂): D-pad ↑↓=M4升降 键A(N2)=档位 键B(R2)=夹取 N3=回零")
print("  + 键 → 自动程序")
print("  ≡ 键 → 调试模式开关")
print("=" * 40)

# 点阵屏开机显示
novapi.reset_timer()

# ==================== 主循环 ====================
while True:
    # ================================================================
    #  自动模式
    # ================================================================
    if auto_mode:
        # 获取当前步骤（可能为标准 4 元组或特殊标签元组）
        step = AUTO_SEQUENCE[auto_step]
        elapsed = novapi.timer() - auto_step_start

        # --- 特殊标签步骤（第一个元素是字符串） ---
        if isinstance(step[0], str):
            tag = step[0]

            if tag == 'encoder_cal':
                # 编码器标定：记录编码值 → 直走 → 计算增量
                _, cal_dur, cal_Vx, cal_Vy, cal_omega = step  # 5 元组
                if not _enc_cal_auto_started:
                    _enc_cal_M1_start = __motor_M1.get_value("angle")
                    _enc_cal_M2_start = __motor_M2.get_value("angle")
                    _enc_cal_M3_start = __motor_M3.get_value("angle")
                    auto_step_start = novapi.timer()
                    _enc_cal_auto_started = True
                    print(">>> 编码器标定开始！记录初始编码 M1=%.0f M2=%.0f M3=%.0f" %
                          (_enc_cal_M1_start, _enc_cal_M2_start, _enc_cal_M3_start))
                if elapsed >= cal_dur:
                    M1_end = __motor_M1.get_value("angle")
                    M2_end = __motor_M2.get_value("angle")
                    M3_end = __motor_M3.get_value("angle")
                    _cal_delta_M1 = M1_end - _enc_cal_M1_start
                    _cal_delta_M2 = M2_end - _enc_cal_M2_start
                    _cal_delta_M3 = M3_end - _enc_cal_M3_start
                    _debug_cal_done = True
                    _enc_cal_auto_started = False
                    stop_all_motors()
                    auto_mode = False
                    auto_step = 0
                    print(">>> 编码器标定完成！增量: M1=%d M2=%d M3=%d" %
                          (_cal_delta_M1, _cal_delta_M2, _cal_delta_M3))
                    print("    按 N1/N2/N3 在 LED 查看各电机增量（E####）")
                else:
                    _cvx, _cvy = rotate_velocity(cal_Vx, cal_Vy, face)
                    if INVERT_VX: _cvx = -_cvx
                    if INVERT_VY: _cvy = -_cvy
                    _cw = cal_omega
                    if INVERT_OMEGA: _cw = -_cw
                    M1p, M2p, M3p = omni_kinematics(_cvx, _cvy, _cw)
                    __motor_M1.set_power(M1p)
                    __motor_M2.set_power(M2p)
                    __motor_M3.set_power(M3p)

            elif tag == 'enc_move':
                # v5.1 编码器闭环直走：使用 novapi 内置 move() 位置控制
                # 连续 enc_move 时不刹停，直接衔接
                _, total_ticks, vy_power, _w = step
                if _enc_step_initialized != auto_step:
                    _enc_move_start(total_ticks, vy_power)
                    _enc_step_initialized = auto_step

                next_step = AUTO_SEQUENCE[auto_step + 1] if auto_step + 1 < len(AUTO_SEQUENCE) else None
                next_is_enc_move = (next_step is not None
                                    and isinstance(next_step[0], str)
                                    and next_step[0] == 'enc_move')

                if _enc_move_tick(abs(vy_power), stop_motors=(not next_is_enc_move)):
                    auto_step += 1
                    if auto_step >= len(AUTO_SEQUENCE):
                        auto_mode = False; auto_step = 0
                        stop_all_motors()
                    else:
                        auto_step_start = novapi.timer()

            elif tag == 'enc_rot':
                # v5.1 编码器闭环自旋
                _, total_ticks, _vy, omega_power = step
                if _enc_step_initialized != auto_step:
                    _enc_rot_start(total_ticks, omega_power)
                    _enc_step_initialized = auto_step

                next_step = AUTO_SEQUENCE[auto_step + 1] if auto_step + 1 < len(AUTO_SEQUENCE) else None
                next_is_enc_rot = (next_step is not None
                                   and isinstance(next_step[0], str)
                                   and next_step[0] == 'enc_rot')

                if _enc_rot_tick(abs(omega_power), stop_motors=(not next_is_enc_rot)):
                    auto_step += 1
                    if auto_step >= len(AUTO_SEQUENCE):
                        auto_mode = False; auto_step = 0
                        stop_all_motors()
                    else:
                        auto_step_start = novapi.timer()

            elif tag == 'enc_stop':
                # 停止缓冲：短暂停顿后进入下一步（不终止整个序列）
                stop_all_motors()
                auto_step += 1
                if auto_step >= len(AUTO_SEQUENCE):
                    auto_mode = False
                    auto_step = 0
                    print(">>> 编码器序列完成")
                else:
                    auto_step_start = novapi.timer()
                    print(">>> enc_stop → 步骤 %d/%d" % (auto_step + 1, len(AUTO_SEQUENCE)))

            elif tag == 'spin':
                # 时间驱动原地自旋: ("spin", degrees, omega_power)
                _, deg, _w = step
                rot_speed = 90.0 * abs(_w) / 50.0
                need_dur = abs(deg) / max(rot_speed, 1.0)
                w_sign = 1 if deg > 0 else -1
                w_out = w_sign * abs(_w)
                if INVERT_OMEGA: w_out = -w_out
                M1p, M2p, M3p = omni_kinematics(0, 0, w_out)
                __motor_M1.set_power(M1p)
                __motor_M2.set_power(M2p)
                __motor_M3.set_power(M3p)
                if elapsed >= need_dur:
                    stop_all_motors()
                    auto_step += 1
                    if auto_step >= len(AUTO_SEQUENCE):
                        auto_mode = False
                        auto_step = 0
                    else:
                        auto_step_start = novapi.timer()

            elif tag == 'delay':
                _, delay_dur = step[0], step[1]
                if elapsed >= delay_dur:
                    auto_step += 1
                    auto_step_start = novapi.timer()
                    print(">>> 延时 %.1fs 完成" % delay_dur)

            elif tag == 'motor':
                # 设置编码电机持续转动（立即执行，不等待）
                _, motor_id, power = step
                motor_map = {"M1": __motor_M1, "M2": __motor_M2, "M3": __motor_M3}
                m = motor_map.get(str(motor_id))
                if m is not None:
                    m.set_power(int(power))
                    print(">>> motor %s → %d%%" % (motor_id, power))
                auto_step += 1
                if auto_step >= len(AUTO_SEQUENCE):
                    auto_mode = False; auto_step = 0
                else:
                    auto_step_start = novapi.timer()

            elif tag == 'dc_motor':
                # 设置直流电机持续转动（立即执行，不等待）
                _, dc_port, power = step
                power_expand_board.set_power(str(dc_port), int(power))
                print(">>> dc_motor %s → %d%%" % (dc_port, power))
                auto_step += 1
                if auto_step >= len(AUTO_SEQUENCE):
                    auto_mode = False; auto_step = 0
                else:
                    auto_step_start = novapi.timer()

            elif tag == 'servo':
                # 舵机步骤（占位，三轮车无舵机）
                print(">>> 舵机步骤（跳过）: %r" % (step,))
                auto_step += 1
                auto_step_start = novapi.timer()

            else:
                # 未知标签，跳过
                print(">>> 未知步骤标签 %s，跳过" % tag)
                auto_step += 1
                auto_step_start = novapi.timer()

            # 检查是否所有步骤完成
            if auto_step >= len(AUTO_SEQUENCE):
                stop_all_motors()
                auto_mode = False
                auto_step = 0
                auto_prev_Vx = 0.0
                auto_prev_Vy = 0.0
                auto_prev_omega = 0.0
                print(">>> 自动程序完成")

            __led.show("S%d" % BL_ACTION_SPEED)
            time.sleep(AUTO_STEP_DELAY)
            continue

        # --- 标准运动步骤（4 元组） ---
        step_duration, step_Vx, step_Vy, step_omega = step

        # 方向取反
        if INVERT_VX: step_Vx = -step_Vx
        if INVERT_VY: step_Vy = -step_Vy
        if INVERT_OMEGA: step_omega = -step_omega

        if elapsed >= step_duration:
            # 记录本步结束时的稳态速度作为下次插值起点
            Vx_end, Vy_end = rotate_velocity(step_Vx, step_Vy, face)
            auto_prev_Vx = Vx_end
            auto_prev_Vy = Vy_end
            auto_prev_omega = step_omega

            auto_step += 1
            if auto_step >= len(AUTO_SEQUENCE):
                stop_all_motors()
                auto_mode = False
                auto_step = 0
                auto_prev_Vx = 0.0
                auto_prev_Vy = 0.0
                auto_prev_omega = 0.0
                print(">>> 自动程序完成，恢复遥控")
            else:
                auto_step_start = novapi.timer()
                next_step = AUTO_SEQUENCE[auto_step]
                if isinstance(next_step[0], str):
                    print(">>> 自动步骤 %d/%d: %s" %
                          (auto_step + 1, len(AUTO_SEQUENCE), next_step[0]))
                else:
                    next_dur, next_Vx, next_Vy, next_omega = next_step
                    print(">>> 自动步骤 %d/%d: Vx=%d Vy=%d ω=%d (%.1fs)" %
                          (auto_step + 1, len(AUTO_SEQUENCE), next_Vx, next_Vy, next_omega, next_dur))
        else:
            # 执行当前步骤：根据当前正面旋转速度，再经运动学输出
            Vx_target, Vy_target = rotate_velocity(step_Vx, step_Vy, face)
            omega_target = step_omega

            # 步间速度线性插值（前 AUTO_RAMP_MS 毫秒）
            if AUTO_RAMP_MS > 0 and elapsed * 1000.0 < AUTO_RAMP_MS:
                alpha = elapsed * 1000.0 / AUTO_RAMP_MS  # 0 → 1
                Vx_out = auto_prev_Vx + (Vx_target - auto_prev_Vx) * alpha
                Vy_out = auto_prev_Vy + (Vy_target - auto_prev_Vy) * alpha
                omega_out = auto_prev_omega + (omega_target - auto_prev_omega) * alpha
            else:
                Vx_out = Vx_target
                Vy_out = Vy_target
                omega_out = omega_target

            # 步内加速斜坡（0→100→0），防打滑
            step_ms = step_duration * 1000.0
            elapsed_ms = elapsed * 1000.0
            if AUTO_ACCEL_MS > 0 and step_ms > AUTO_ACCEL_MS * 2:
                if elapsed_ms < AUTO_ACCEL_MS:
                    accel = elapsed_ms / AUTO_ACCEL_MS
                elif elapsed_ms > step_ms - AUTO_ACCEL_MS:
                    accel = (step_ms - elapsed_ms) / AUTO_ACCEL_MS
                else:
                    accel = 1.0
            else:
                accel = 1.0

            Vx_out *= accel
            Vy_out *= accel
            omega_out *= accel

            M1_power, M2_power, M3_power = omni_kinematics(Vx_out, Vy_out, omega_out)
            __motor_M1.set_power(M1_power)
            __motor_M2.set_power(M2_power)
            __motor_M3.set_power(M3_power)

        __led.show("S%d" % BL_ACTION_SPEED)
        time.sleep(AUTO_STEP_DELAY)
        continue  # 跳过手动/调试逻辑

    # ================================================================
    #  调试模式
    # ================================================================
    if debug_mode:
        # --- 读取按键 ---
        cur_Up = gamepad.is_key_pressed("Up")
        cur_Down = gamepad.is_key_pressed("Down")
        cur_Left = gamepad.is_key_pressed("Left")
        cur_Right = gamepad.is_key_pressed("Right")
        cur_N1 = gamepad.is_key_pressed("N1")
        cur_N2 = gamepad.is_key_pressed("N2")
        cur_N3 = gamepad.is_key_pressed("N3")
        cur_L1 = gamepad.is_key_pressed("L1")
        cur_R1 = gamepad.is_key_pressed("R1")
        cur_N4 = gamepad.is_key_pressed("N4")
        cur_Menu = gamepad.is_key_pressed("≡")
        cur_Plus = gamepad.is_key_pressed("+")

        # D-pad 冲突保护：连续 3 次同时按下才拦截
        if cur_Left and cur_Right:
            _dpad_lr_conflict += 1
        else:
            _dpad_lr_conflict = 0
        if _dpad_lr_conflict >= 3:
            cur_Left = False
            cur_Right = False

        if cur_Up and cur_Down:
            _dpad_ud_conflict += 1
        else:
            _dpad_ud_conflict = 0
        if _dpad_ud_conflict >= 3:
            cur_Up = False
            cur_Down = False

        # --- ≡ 键切换：退出调试模式 ---
        if cur_Menu and not last_Menu:
            debug_stop_motor()
            debug_mode = False
            debug_cal_mode = False
            debug_cal_running = False
            debug_cal_done_time = 0.0
            stop_all_motors()
            print(">>> ≡ 退出调试模式，恢复正常操控")
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = last_N4 = False
            last_L1_debug = last_R1_debug = False
            last_Menu = cur_Menu
            time.sleep(0.3)
            continue

        # --- + 键：退出调试并启动自动程序（编码器标定等） ---
        if cur_Plus and not last_Plus:
            debug_stop_motor()
            debug_mode = False
            debug_cal_mode = False
            debug_cal_running = False
            debug_cal_done_time = 0.0
            stop_all_motors()
            # 清除编码器标定旧数据 + 重置编码器步状态
            _debug_cal_done = False
            _enc_cal_auto_started = False
            _enc_step_initialized = -1
            _cal_delta_M1 = _cal_delta_M2 = _cal_delta_M3 = None
            # 启动自动程序
            auto_mode = True
            auto_step = 0
            auto_step_start = novapi.timer()
            auto_prev_Vx = 0.0
            auto_prev_Vy = 0.0
            auto_prev_omega = 0.0
            step0 = AUTO_SEQUENCE[0]
            if isinstance(step0[0], str):
                print(">>> 自动程序启动（从调试模式）！步骤 1/%d: %s" %
                      (len(AUTO_SEQUENCE), step0[0]))
            else:
                dur0, Vx0, Vy0, w0 = step0
                print(">>> 自动程序启动（从调试模式）！步骤 1/%d: Vx=%d Vy=%d ω=%d (%.1fs)" %
                      (len(AUTO_SEQUENCE), Vx0, Vy0, w0, dur0))
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = last_N4 = False
            last_L1_debug = last_R1_debug = False
            last_Menu = cur_Menu
            last_Plus = cur_Plus
            time.sleep(0.3)
            continue

        # ==== 标定子模式 ====
        if debug_cal_mode:

            # --- 标定移动执行中（非阻塞）---
            if debug_cal_running:
                still_running = debug_cal_tick()
                if not still_running:
                    # 刚完成：显示最终编码值（保持 2 秒后恢复角度显示）
                    pass
                # 更新边沿（防止移动期间按键穿透）
                last_Up = cur_Up
                last_Down = cur_Down
                last_Left = cur_Left
                last_Right = cur_Right
                last_N1 = cur_N1
                last_N2 = cur_N2
                last_N3 = cur_N3
                last_L1_debug = cur_L1
                last_R1_debug = cur_R1
                last_Menu = cur_Menu
                time.sleep(LOOP_DELAY)
                continue

            # --- 标定完成后的最终值展示（2 秒）---
            if debug_cal_done_time > 0:
                if novapi.timer() - debug_cal_done_time < 2.0:
                    last_Up = cur_Up
                    last_Down = cur_Down
                    last_Left = cur_Left
                    last_Right = cur_Right
                    last_N1 = cur_N1
                    last_N2 = cur_N2
                    last_N3 = cur_N3
                    last_L1_debug = cur_L1
                    last_R1_debug = cur_R1
                    last_Menu = cur_Menu
                    time.sleep(LOOP_DELAY)
                    continue
                else:
                    debug_cal_done_time = 0.0

            # --- 空闲状态：等待自动启动或手动触发 ---

            # 自动启动：进入标定模式 1 秒后自动执行
            if debug_cal_auto_start and novapi.timer() - debug_cal_auto_time >= 1.0:
                debug_cal_auto_start = False
                debug_cal_start(CAL_ANGLE)
                last_Up = cur_Up
                last_Down = cur_Down
                last_Left = cur_Left
                last_Right = cur_Right
                last_N1 = cur_N1
                last_N2 = cur_N2
                last_N3 = cur_N3
                last_L1_debug = cur_L1
                last_R1_debug = cur_R1
                last_Menu = cur_Menu
                time.sleep(LOOP_DELAY)
                continue

            # N1：手动启动标定移动
            if cur_N1 and not last_N1:
                debug_cal_auto_start = False
                debug_cal_start(CAL_ANGLE)

            # L1+R1 再次按下 → 退出标定模式
            if cur_L1 and cur_R1 and (not last_L1_debug or not last_R1_debug):
                debug_cal_mode = False
                debug_cal_running = False
                debug_cal_done_time = 0.0
                debug_cal_auto_start = False
                stop_all_motors()
                print(">>> 退出标定模式，返回调试模式")
                last_Up = last_Down = last_Left = last_Right = False
                last_N1 = last_N2 = last_N3 = False
                last_L1_debug = last_R1_debug = False
                last_Menu = cur_Menu
                time.sleep(0.3)
                continue

            # 更新边沿
            last_Up = cur_Up
            last_Down = cur_Down
            last_Left = cur_Left
            last_Right = cur_Right
            last_N1 = cur_N1
            last_N2 = cur_N2
            last_N3 = cur_N3
            last_L1_debug = cur_L1
            last_R1_debug = cur_R1
            last_Menu = cur_Menu
            time.sleep(LOOP_DELAY)
            continue

        # ==== 主调试模式 ====
        # --- L1+R1 同时按下 → 进入标定模式（1秒后自动执行）---
        if cur_L1 and cur_R1 and (not last_L1_debug or not last_R1_debug):
            debug_cal_mode = True
            debug_cal_running = False
            debug_cal_done_time = 0.0
            debug_cal_auto_start = True
            debug_cal_auto_time = novapi.timer()
            print("=" * 40)
            print("  >>> 进入标定模式！1秒后自动执行...")
            print("  目标: %d cm | 编码: %d°" % (CAL_TARGET_CM, CAL_ANGLE))
            print("  S曲线+PID | N1=手动执行 | L1+R1=退出")
            print("  显示屏: 实时编码 → 完成后最终编码(Fxxx)")
            print("=" * 40)
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = False
            last_L1_debug = cur_L1
            last_R1_debug = cur_R1
            time.sleep(0.3)
            continue

        # --- 十字键 ↑/↓：切换电机类型（所有类型统一）---
        if cur_Up and not last_Up:
            debug_motor_type = (debug_motor_type + 1) % len(DEBUG_MOTOR_TYPES)
            max_idx = len(DEBUG_ALL_NAMES[debug_motor_type]) - 1
            if debug_motor_index > max_idx:
                debug_motor_index = 0
            if debug_motor_type == 2:
                # 进入升降臂时显示当前档位名
                pass
            else:
                name = DEBUG_ALL_NAMES[debug_motor_type][debug_motor_index]
                pass
            print(">>> 电机类型: %s" % DEBUG_MOTOR_TYPES[debug_motor_type])
        if cur_Down and not last_Down:
            debug_motor_type = (debug_motor_type - 1) % len(DEBUG_MOTOR_TYPES)
            max_idx = len(DEBUG_ALL_NAMES[debug_motor_type]) - 1
            if debug_motor_index > max_idx:
                debug_motor_index = 0
            if debug_motor_type == 2:
                pass
            else:
                name = DEBUG_ALL_NAMES[debug_motor_type][debug_motor_index]
                pass
            print(">>> 电机类型: %s" % DEBUG_MOTOR_TYPES[debug_motor_type])

        # --- 十字键 ←/→：切换子项（升降臂模式切档位，其他模式切编号）---
        if debug_motor_type == 2:
            # 升降臂模式：←/→ 切换档位
            if cur_Left and not last_Left:
                debug_lift_gear_index = (debug_lift_gear_index - 1) % len(DEBUG_LIFT_GEARS)
                print(">>> 升降臂档位: %s (%d°)" % (DEBUG_LIFT_NAMES[debug_lift_gear_index], DEBUG_LIFT_GEARS[debug_lift_gear_index]))
            if cur_Right and not last_Right:
                debug_lift_gear_index = (debug_lift_gear_index + 1) % len(DEBUG_LIFT_GEARS)
                print(">>> 升降臂档位: %s (%d°)" % (DEBUG_LIFT_NAMES[debug_lift_gear_index], DEBUG_LIFT_GEARS[debug_lift_gear_index]))
        else:
            max_idx = len(DEBUG_ALL_NAMES[debug_motor_type]) - 1
            if cur_Left and not last_Left:
                debug_motor_index = (debug_motor_index - 1) % (max_idx + 1)
                name = DEBUG_ALL_NAMES[debug_motor_type][debug_motor_index]
                print(">>> 电机编号: %s" % name)
            if cur_Right and not last_Right:
                debug_motor_index = (debug_motor_index + 1) % (max_idx + 1)
                name = DEBUG_ALL_NAMES[debug_motor_type][debug_motor_index]
                print(">>> 电机编号: %s" % name)

        # --- 空闲显示 ---
        if not cur_N1 and not cur_N4:
            if debug_show_ok:
                if novapi.timer() >= debug_show_speed_until:
                    debug_show_ok = False
            elif debug_show_speed_until > 0 and novapi.timer() < debug_show_speed_until:
                pass  # 速度/角度值在print中显示，LED统一显示BL速度
            else:
                pass  # 电机名在print中显示，LED统一显示BL速度

        # --- 标定完成后：N1/N2/N3 查看各电机增量（仅print，LED统一显示BL速度）---
        if _debug_cal_done:
            if cur_N1 and not last_N1:
                print(">>> M1 编码增量: %d" % _cal_delta_M1)
            if cur_N2 and not last_N2:
                print(">>> M2 编码增量: %d" % _cal_delta_M2)
            if cur_N3 and not last_N3:
                print(">>> M3 编码增量: %d" % _cal_delta_M3)

        # --- N1 / N4：正转/反转 或 升降臂去档位/回零 或 舵机移动/回零 ---
        if debug_motor_type == 0:
            if cur_N1:
                speed = DEBUG_TEST_SPEED
            elif cur_N4:
                speed = -DEBUG_TEST_SPEED
            else:
                speed = 0
            motor = [__motor_M1, __motor_M2, __motor_M3, __motor_M4][debug_motor_index]
            motor.set_power(speed)
        elif debug_motor_type == 1:
            if cur_N1:
                speed = DEBUG_TEST_SPEED
            elif cur_N4:
                speed = -DEBUG_TEST_SPEED
            else:
                speed = 0
            power_expand_board.set_power(DC_COLLECTOR_PORT, speed)
        elif debug_motor_type == 2:
            # 升降臂模式：N1→去档位, N4→回零（PID + 到位停电，调试仅控M5二号臂）
            if cur_N1 and not last_N1:
                target_deg = DEBUG_LIFT_GEARS[debug_lift_gear_index]
                lift_pid_start(1, target_deg)
                print(">>> 升降臂: M5 → %s (%d°)" % (DEBUG_LIFT_NAMES[debug_lift_gear_index], target_deg))
            if cur_N4 and not last_N4:
                lift_pid_start(1, 0)
                print(">>> 升降臂: M5 → 零点")
        elif debug_motor_type == 3:
            # 舵机模式：N1→测试角度, N4→回零
            if _HAS_SERVO:
                try:
                    _sv = [__servo_1, __servo_2][debug_motor_index]
                except Exception:
                    _sv = None
                if cur_N1 and not last_N1 and _sv:
                    _sv.move_to(servo_dir * DEBUG_SERVO_ANGLE, SERVO_MOVE_SPEED)
                    print(">>> 舵机 %s: → %d° (夹紧)" % (DEBUG_SERVO_NAMES[debug_motor_index], servo_dir * DEBUG_SERVO_ANGLE))
                if cur_N4 and not last_N4 and _sv:
                    _sv.move_to(0, SERVO_MOVE_SPEED)
                    print(">>> 舵机 %s: → 0° (全开)" % DEBUG_SERVO_NAMES[debug_motor_index])
        elif debug_motor_type == 4:
            # 无刷电机模式：N1→转动, N4→停止
            if cur_N1:
                speed = DEBUG_TEST_SPEED
            elif cur_N4:
                speed = 0
            else:
                speed = 0
            if debug_motor_index == 2:  # BL12：双电机同时
                power_expand_board.set_power("BL1", speed)
                power_expand_board.set_power("BL2", speed)
                # DC1 以 -100 反转辅助（配合无刷发射小球）
                power_expand_board.set_power(DC_COLLECTOR_PORT, -100 if speed != 0 else 0)
            else:
                port = ["BL1", "BL2"][debug_motor_index]
                power_expand_board.set_power(port, speed)

        # --- N2 / N3：调整测试速度/升降臂角度/舵机角度 ± ---
        if cur_N2 and not last_N2:
            if debug_motor_type == 3:
                DEBUG_SERVO_ANGLE = min(SERVO_MAX_ANGLE, DEBUG_SERVO_ANGLE + 5)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 舵机夹紧幅度: %d° (N1发出 %d°)" % (DEBUG_SERVO_ANGLE, servo_dir * DEBUG_SERVO_ANGLE))
            elif debug_motor_type == 2:
                DEBUG_LIFT_GEARS[debug_lift_gear_index] = min(3600, DEBUG_LIFT_GEARS[debug_lift_gear_index] + 30)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 升降臂档位 %s: %d°" % (DEBUG_LIFT_NAMES[debug_lift_gear_index], DEBUG_LIFT_GEARS[debug_lift_gear_index]))
            else:
                DEBUG_TEST_SPEED = min(100, DEBUG_TEST_SPEED + 10)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 测试速度: %d" % DEBUG_TEST_SPEED)
        if cur_N3 and not last_N3:
            if debug_motor_type == 3:
                DEBUG_SERVO_ANGLE = max(0, DEBUG_SERVO_ANGLE - 5)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 舵机夹紧幅度: %d° (N1发出 %d°)" % (DEBUG_SERVO_ANGLE, servo_dir * DEBUG_SERVO_ANGLE))
            elif debug_motor_type == 2:
                DEBUG_LIFT_GEARS[debug_lift_gear_index] = max(-3600, DEBUG_LIFT_GEARS[debug_lift_gear_index] - 30)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 升降臂档位 %s: %d°" % (DEBUG_LIFT_NAMES[debug_lift_gear_index], DEBUG_LIFT_GEARS[debug_lift_gear_index]))
            else:
                DEBUG_TEST_SPEED = max(10, DEBUG_TEST_SPEED - 10)
                debug_show_speed_until = novapi.timer() + 2.0
                print(">>> 测试速度: %d" % DEBUG_TEST_SPEED)

        # --- R1：舵机设置零点 ---
        if debug_motor_type == 3 and cur_R1 and not last_R1_debug and _HAS_SERVO:
            try:
                _sv = [__servo_1, __servo_2][debug_motor_index]
                _sv.set_zero()
                debug_show_speed_until = novapi.timer() + 1.0
                debug_show_ok = True
                print(">>> 舵机 %s: 零点已设置" % DEBUG_SERVO_NAMES[debug_motor_index])
            except Exception:
                pass

        # --- L1：升降臂退出 / 舵机方向切换 ---
        if debug_motor_type == 2 and cur_L1 and not last_L1_debug:
            debug_motor_type = 0
            debug_motor_index = 0
            print(">>> 升降臂: 退出 → 编码电机模式")
        if debug_motor_type == 3 and cur_L1 and not last_L1_debug:
            servo_dir = -servo_dir
            if servo_dir > 0:
                print(">>> 🚨🚨🚨 舵机方向=正向！N1将撑开夹爪→可能报废！🚨🚨🚨")
            else:
                print(">>> 舵机方向=负向(夹紧) 安全 ✅")

        # 更新边沿
        last_Up = cur_Up
        last_Down = cur_Down
        last_Left = cur_Left
        last_Right = cur_Right
        last_N1 = cur_N1
        last_N2 = cur_N2
        last_N3 = cur_N3
        last_N4 = cur_N4
        last_L1_debug = cur_L1
        last_R1_debug = cur_R1
        last_Menu = cur_Menu

        # 升降臂 PID 每帧更新（双臂同时，仅在活跃时运行）
        if _lift_pid_active[0] or _lift_pid_active[1]:
            lift_pid_tick_all()

        __led.show("S%d" % BL_ACTION_SPEED)
        time.sleep(LOOP_DELAY)
        continue  # 跳过正常模式

    # ================================================================
    #  正常模式（手动遥控）
    # ================================================================
    # --- 0. 提前读取 L1（旋转中也需要检测连按）---
    cur_L1 = gamepad.is_key_pressed("L1")

    # --- 1. 面切换旋转中：PID 闭环执行 + 连按叠加 ---
    if face_rotating:
        # 旋转中再次按 L1 → 叠加一个面
        if cur_L1 and not last_L1:
            face = (face + 1) % 3
            face_rot_target_deg += FACE_ROTATE_WHEEL_DEG
            face_rot_start_time = novapi.timer()   # 重置超时
            face_rot_settled = False               # 重新追赶
            print(">>> 连续换面！叠加至 %s，新目标=%.0f°/轮" %
                  (FACE_NAMES[face], face_rot_target_deg))

        if face_rotate_tick():
            # 旋转完成
            face_show_until = novapi.timer() + FACE_SHOW_MS / 1000.0
            print(">>> 面切换完成: %s" % FACE_NAMES[face])

        last_L1 = cur_L1
        __led.show("S%d" % BL_ACTION_SPEED)
        time.sleep(LOOP_DELAY)
        continue

    # --- 2. 读取摇杆原始值 ---
    Lx = gamepad.get_joystick("Lx")   # 左摇杆水平: -100(左) ~ +100(右)
    Ly = gamepad.get_joystick("Ly")   # 左摇杆垂直: -100(下) ~ +100(上)
    Rx = gamepad.get_joystick("Rx")   # 右摇杆水平: -100(左) ~ +100(右)

    # --- 2. 按键边沿触发 ---
    cur_R1 = gamepad.is_key_pressed("R1")
    cur_R2 = gamepad.is_key_pressed("R2")
    cur_L2 = gamepad.is_key_pressed("L2")
    # cur_L1 已在正常模式顶部读取，此处不重复
    cur_Plus = gamepad.is_key_pressed("+")
    cur_Menu = gamepad.is_key_pressed("≡")
    cur_N1 = gamepad.is_key_pressed("N1")
    cur_N2 = gamepad.is_key_pressed("N2")
    cur_N3 = gamepad.is_key_pressed("N3")
    cur_N4 = gamepad.is_key_pressed("N4")
    cur_Dpad_Up = gamepad.is_key_pressed("Up")
    cur_Dpad_Down = gamepad.is_key_pressed("Down")
    cur_Dpad_Left = gamepad.is_key_pressed("Left")
    cur_Dpad_Right = gamepad.is_key_pressed("Right")

    if cur_Menu and not last_Menu:      # ≡ 上升沿 → 调试模式开关
        if debug_mode:
            # 退出调试
            debug_stop_motor()
            debug_mode = False
            debug_cal_mode = False
            debug_cal_running = False
            debug_cal_done_time = 0.0
            stop_all_motors()
            print(">>> ≡ 退出调试模式，恢复正常操控")
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = last_N4 = False
            last_L1_debug = last_R1_debug = False
            last_Menu = cur_Menu
            time.sleep(0.3)
        else:
            # 进入调试
            debug_mode = True
            debug_motor_type = 0
            debug_motor_index = 0
            debug_cal_mode = False
            debug_cal_running = False
            debug_cal_done_time = 0.0
            # 重置调试按键边沿防止残留触发
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = last_N4 = False
            last_L1_debug = last_R1_debug = False
            last_Menu = cur_Menu
            print("=" * 40)
            print("  >>> 进入调试模式！")
            print("  ↑↓ 切换类型 | ← → 切换子项/升降档位")
            print("  N1=正转/去档位  N4=反转/回零")
            print("  N2/N3=调速/调角度  |  L1+R1=标定模式")
            print("  编码电机 M1-M4 | 直流 DC1 | 升降臂 L0-L2 | 舵机 SV1-SV2 | 无刷 BL1-BL2")
            print("  舵机: R1=设零点 L1=方向 | 升降臂: L1=退出")
            print("  无刷: N1=转  N4=停  N2/N3=调速")
            print("=" * 40)
            # 屏幕默认显示BL速度
        continue

    # ================================================================
    #  按键处理 — 面映射分发
    #  键A(N2): Face0=DC1收球  Face1/2=升降档位
    #  键B(R2): Face0=BL发射    Face1/2=自适应夹爪
    #  D-pad↑↓: Face0=BL调速   Face1/2=升降档位
    #  N3:      Face1/2=回零   Face0=无
    #  R1/N1:   已由键A/键B/面映射接管，原功能迁移
    # ================================================================
    cur_motor_idx = FACE_LIFT_MOTOR[face]    # 当前面升降臂索引 (None/0/1)
    cur_servo_idx = FACE_GRIP_SERVO[face]    # 当前面夹爪索引   (None/0/1)

    # --- R1：闲置（BL发射已由键B在Face0接管）---
    # 保留边沿更新，不做任何操作

    # --- L1 上升沿 → 顺时针换面 + BL强制停 + DC1保持 ---
    if cur_L1 and not last_L1:
        # 🚨 离开当前面时，强制停止无刷电机（安全）
        if bl1_on or bl2_on:
            power_expand_board.stop("BL1")
            power_expand_board.stop("BL2")
            bl1_on = False
            bl2_on = False
            print(">>> 换面: BL1/BL2 强制停止")
        # ✅ DC1 收球状态保持不动（全局状态）

        face = (face + 1) % 3           # 顺时针切换面: 0→1→2→0
        print(">>> 面切换触发: %s（顺时针旋转120°）" % FACE_NAMES[face])
        face_rotate_start()             # 启动 PID 闭环旋转
        last_L1 = cur_L1
        last_R1 = cur_R1
        time.sleep(LOOP_DELAY)
        continue

    # --- + 键 → 自动程序 ---
    if cur_Plus and not last_Plus:
        auto_mode = True
        auto_step = 0
        auto_step_start = novapi.timer()
        auto_prev_Vx = 0.0
        auto_prev_Vy = 0.0
        auto_prev_omega = 0.0
        _debug_cal_done = False
        _enc_cal_auto_started = False
        _enc_step_initialized = -1
        _cal_delta_M1 = _cal_delta_M2 = _cal_delta_M3 = None
        step0 = AUTO_SEQUENCE[0]
        if isinstance(step0[0], str):
            print(">>> 自动程序启动！步骤 1/%d: %s" % (len(AUTO_SEQUENCE), step0[0]))
        else:
            dur0, Vx0, Vy0, w0 = step0
            print(">>> 自动程序启动！步骤 1/%d: Vx=%d Vy=%d ω=%d (%.1fs)" %
                  (len(AUTO_SEQUENCE), Vx0, Vy0, w0, dur0))

    # --- 编码器标定后 N1/N2/N3 查看增量（仅print）---
    if _debug_cal_done:
        if cur_N1 and not last_N1:
            print(">>> M1 编码增量: %d" % _cal_delta_M1)
        if cur_N2 and not last_N2:
            print(">>> M2 编码增量: %d" % _cal_delta_M2)
        if cur_N3 and not last_N3:
            print(">>> M3 编码增量: %d" % _cal_delta_M3)

    # --- 键A (N2)：面映射分发 ---
    if cur_N2 and not last_N2 and not _debug_cal_done:
        if face == 0:
            # Face 0：DC1 收球开关
            collector_on = not collector_on
            if collector_on:
                power_expand_board.set_power(DC_COLLECTOR_PORT, DC_COLLECTOR_SPEED)
                print(">>> 键A(N2): DC1收球 开")
            else:
                power_expand_board.set_power(DC_COLLECTOR_PORT, 0)
                print(">>> 键A(N2): DC1收球 关")
        elif cur_motor_idx is not None:
            # Face 1/2：升降档位切一圈 L0→L1→L2→L0
            lift_gear[cur_motor_idx] = (lift_gear[cur_motor_idx] + 1) % len(DEBUG_LIFT_GEARS)
            target_deg = DEBUG_LIFT_GEARS[lift_gear[cur_motor_idx]]
            lift_pid_start(cur_motor_idx, target_deg)
            arm = "M5" if cur_motor_idx == 1 else "M4"
            print(">>> 键A(N2): %s → %s (%d°)" % (arm, DEBUG_LIFT_NAMES[lift_gear[cur_motor_idx]], target_deg))

    # --- 键B (R2)：面映射分发 ---
    if cur_R2 and not last_R2 and not _debug_cal_done:
        if face == 0:
            # Face 0：BL1+BL2 发射开关
            bl1_on = not bl1_on
            bl2_on = not bl2_on
            if bl1_on:
                power_expand_board.set_power("BL1", BL_ACTION_SPEED)
                power_expand_board.set_power("BL2", BL_ACTION_SPEED)
                print(">>> 键B(R2): 双无刷启动 @%d" % BL_ACTION_SPEED)
            else:
                power_expand_board.stop("BL1")
                power_expand_board.stop("BL2")
                print(">>> 键B(R2): 双无刷停止")
        elif cur_servo_idx is not None:
            # Face 1/2：自适应夹爪
            _grip_active[cur_servo_idx] = not _grip_active[cur_servo_idx]
            sv = [__servo_1, __servo_2][cur_servo_idx]
            sv_name = "SV1" if cur_servo_idx == 0 else "SV2"
            if _grip_active[cur_servo_idx]:
                _grip_closing[cur_servo_idx] = True
                _grip_hold[cur_servo_idx] = False
                _grip_start_time[cur_servo_idx] = novapi.timer()
                _grip_settled[cur_servo_idx] = False
                _grip_last_print[cur_servo_idx] = 0.0
                if _HAS_SERVO:
                    try:
                        sv.set_power(GRIP_CLOSE_POWER)
                    except Exception:
                        pass
                print(">>> 键B(R2): %s 夹紧启动" % sv_name)
            else:
                _grip_closing[cur_servo_idx] = False
                _grip_hold[cur_servo_idx] = False
                if _HAS_SERVO:
                    try:
                        sv.set_power(0)
                        sv.move_to(0, 30)
                    except Exception:
                        pass
                print(">>> 键B(R2): %s 释放 → 0°" % sv_name)

    # --- D-pad ↑↓：Face0=BL调速, Face1/2=升降档位 ---
    if cur_Dpad_Up and not last_Dpad_Up_norm and not _debug_cal_done:
        if face == 0:
            BL_ACTION_SPEED = min(100, BL_ACTION_SPEED + 5)
            if bl1_on:
                power_expand_board.set_power("BL1", BL_ACTION_SPEED)
                power_expand_board.set_power("BL2", BL_ACTION_SPEED)
            print(">>> D-pad↑: 无刷速度 = %d" % BL_ACTION_SPEED)
        elif cur_motor_idx is not None:
            lift_gear[cur_motor_idx] = (lift_gear[cur_motor_idx] + 1) % len(DEBUG_LIFT_GEARS)
            target_deg = DEBUG_LIFT_GEARS[lift_gear[cur_motor_idx]]
            lift_pid_start(cur_motor_idx, target_deg)
            arm = "M5" if cur_motor_idx == 1 else "M4"
            print(">>> D-pad↑: %s → %s (%d°)" % (arm, DEBUG_LIFT_NAMES[lift_gear[cur_motor_idx]], target_deg))
    if cur_Dpad_Down and not last_Dpad_Down_norm and not _debug_cal_done:
        if face == 0:
            BL_ACTION_SPEED = max(10, BL_ACTION_SPEED - 5)
            if bl1_on:
                power_expand_board.set_power("BL1", BL_ACTION_SPEED)
                power_expand_board.set_power("BL2", BL_ACTION_SPEED)
            print(">>> D-pad↓: 无刷速度 = %d" % BL_ACTION_SPEED)
        elif cur_motor_idx is not None:
            lift_gear[cur_motor_idx] = (lift_gear[cur_motor_idx] - 1) % len(DEBUG_LIFT_GEARS)
            target_deg = DEBUG_LIFT_GEARS[lift_gear[cur_motor_idx]]
            lift_pid_start(cur_motor_idx, target_deg)
            arm = "M5" if cur_motor_idx == 1 else "M4"
            print(">>> D-pad↓: %s → %s (%d°)" % (arm, DEBUG_LIFT_NAMES[lift_gear[cur_motor_idx]], target_deg))

    # --- D-pad ←→：置空（原BL调速已移至↑↓在Face0）---
    # 保留边沿更新，无实际操作

    # --- N3：升降臂回零（Face 1/2）---
    if cur_N3 and not last_N3 and not _debug_cal_done:
        if cur_motor_idx is not None:
            lift_gear[cur_motor_idx] = 0
            lift_pid_start(cur_motor_idx, 0)
            arm = "M5" if cur_motor_idx == 1 else "M4"
            print(">>> N3: %s → 回零档 L0" % arm)

    # N1：闲置（原DC1收球已由键A接管）
    # N4：闲置

    last_R1 = cur_R1
    last_R2 = cur_R2
    last_L2 = cur_L2
    last_L1 = cur_L1
    last_Plus = cur_Plus
    last_Menu = cur_Menu
    last_N1 = cur_N1
    last_N2 = cur_N2
    last_N3 = cur_N3
    last_N4 = cur_N4
    last_Dpad_Up_norm = cur_Dpad_Up
    last_Dpad_Down_norm = cur_Dpad_Down
    last_Dpad_Left_norm = cur_Dpad_Left
    last_Dpad_Right_norm = cur_Dpad_Right

    # --- 升降臂 PID 每帧更新（双臂同时，各自独立到位停电，仅在活跃时运行）---
    if _lift_pid_active[0] or _lift_pid_active[1]:
        lift_pid_tick_all()

    # --- 自适应夹爪每帧监测（双舵机独立检测，仅在夹紧中运行）---
    if _grip_closing[0] or _grip_closing[1]:
        for sv_idx in range(2):
            if not _grip_closing[sv_idx] or not _HAS_SERVO:
                continue
            try:
                sv = [__servo_1, __servo_2][sv_idx]
                angle = sv.get_value("angle")
                speed = sv.get_value("speed")
                now = novapi.timer()
                elapsed = now - _grip_start_time[sv_idx]
                sv_name = "SV1" if sv_idx == 0 else "SV2"

                # 🚨 角度硬限位
                if abs(angle) >= SERVO_MAX_ANGLE:
                    sv.set_power(0)
                    _grip_closing[sv_idx] = False
                    print(">>> 🚨 %s: 到达机械极限 %d°！强制停电" % (sv_name, SERVO_MAX_ANGLE))
                elif not _grip_settled[sv_idx]:
                    if elapsed >= GRIP_SETTLE_TIME:
                        _grip_settled[sv_idx] = True
                        print(">>> %s: 开始监测 (速度=%.1f rpm, 角度=%.0f°)" % (sv_name, speed, abs(angle)))
                else:
                    if now - _grip_last_print[sv_idx] >= 0.5:
                        _grip_last_print[sv_idx] = now
                        print(">>> %s: 速度%.1f rpm 角度%.0f°" % (sv_name, abs(speed), abs(angle)))
                    if abs(speed) < GRIP_SPEED_THRESHOLD:
                        sv.set_power(0)
                        _grip_closing[sv_idx] = False
                        _grip_hold[sv_idx] = True
                        print(">>> %s: 夹到！速度=%.1f rpm (角度=%.0f°)" % (sv_name, abs(speed), abs(angle)))
                    elif elapsed > GRIP_TIMEOUT:
                        sv.set_power(0)
                        _grip_closing[sv_idx] = False
                        print(">>> %s: 超时停止（%.1fs）" % (sv_name, GRIP_TIMEOUT))
            except Exception:
                pass

    # --- 3. 死区过滤 ---
    Lx = apply_dead_zone(Lx)
    Ly = apply_dead_zone(Ly)
    Rx = apply_dead_zone(Rx)

    # --- 4. 指数响应曲线（低速细腻，高速爆发）---
    Lx = apply_response_curve(Lx, JOYSTICK_EXPO)
    Ly = apply_response_curve(Ly, JOYSTICK_EXPO)
    Rx = apply_response_curve(Rx, OMEGA_EXPO)

    # --- 5. 速度缩放 ---
    Vx = Lx * SPEED_SCALE
    Vy = Ly * SPEED_SCALE
    omega = Rx * SPEED_SCALE

    # --- 6. 方向校准 ---
    if INVERT_VX: Vx = -Vx
    if INVERT_VY: Vy = -Vy
    if INVERT_OMEGA: omega = -omega

    # --- 7. 根据当前正面旋转速度向量 ---
    Vx, Vy = rotate_velocity(Vx, Vy, face)

    # --- 8. 运动学解算 ---
    M1_power, M2_power, M3_power = omni_kinematics(Vx, Vy, omega)

    # --- 9. 输出到电机 ---
    __motor_M1.set_power(M1_power)
    __motor_M2.set_power(M2_power)
    __motor_M3.set_power(M3_power)

    # --- 10. LED 显示：始终显示当前无刷电机速度 ---
    __led.show("S%d" % BL_ACTION_SPEED)

    # --- 11. 循环延时 ---
    time.sleep(LOOP_DELAY)
