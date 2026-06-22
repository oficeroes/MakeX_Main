"""
三轮全向机器人 — 遥控操控程序
================================
描述：基于 Novapi 平台的三轮全向底盘遥控程序。
      左摇杆控制全向移动（前进/后退/左右横移），
      右摇杆左右控制原地自旋，
      两个摇杆可同时操作实现复合运动（如边前进边转圈）。

硬件需求：编码电机 ×3
  - M1（前左轮）
  - M2（前右轮）
  - M3（尾部轮）

操控速查：
  ┌────────────┬──────────────────────┐
  │  你想做的   │      手柄操作         │
  ├────────────┼──────────────────────┤
  │  前进后退   │  左摇杆 ↑↓           │
  │  左右横移   │  左摇杆 ←→           │
  │  原地左转   │  右摇杆 ←            │
  │  原地右转   │  右摇杆 →            │
  │  斜向移动   │  左摇杆 ↖↗↙↘        │
  │  复合运动   │  两摇杆同时推         │
  │  正面右切   │  按 R1（右转120°）   │
  │  正面左切   │  按 L1（左转120°）   │
  │  自动程序   │  按 +（加号键）      │
  │  收球开关   │  按 N1（开/关切换）  │
  └────────────┴──────────────────────┘

正面切换说明：
  机器人有三个面（三角形三条边），默认 M1-M2 边为正面。
  按 R1 → 正面顺时针切换（M1-M2 → M2-M3 → M3-M1 → M1-M2...）
  按 L1 → 正面逆时针切换（M1-M2 → M3-M1 → M2-M3 → M1-M2...）
  切换后摇杆方向会自动重新映射，保持操作直觉一致。

自动程序说明：
  按 + 键触发预设自动程序（运行期间遥操控被禁用，程序结束后自动恢复）。
  自动程序通过步骤列表定义，每步指定：持续时间(秒) + 速度向量(Vx, Vy, ω)。
  当前预设：前进 4 秒后停止。

运动学模型：三轮全向 120° 对称布局
===================================
         前进方向 (+Vy)
              ↑
       M1 ←──┼──→ M2      (前左 M1, 前右 M2)
              │
              │
             M3            (尾部 M3)

  每个全向轮推力方向（正转时）：
  - M1（θ=150°）：左前上方向
  - M2（θ=30°）： 右前上方向
  - M3（θ=270°）：正右方向

  逆运动学方程（顺时针旋转为正）：
    M1_power = -0.5·Vx - 0.866·Vy - ω
    M2_power = -0.5·Vx + 0.866·Vy - ω
    M3_power =  1.0·Vx            - ω

  其中 Vx=横向速度(+右), Vy=纵向速度(+前), ω=旋转速度(+顺时针)

  ⚠️ 实测修正：摇杆上推(Ly>0)时电机正转为后退，
     故代码中 Vy = -Ly * SPEED_SCALE（取反修正）
"""
import novapi
import time
from mbuild import gamepad
from mbuild import power_expand_board
from mbuild.encoder_motor import encoder_motor_class
from mbuild.led_matrix import led_matrix_class

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
DC_COLLECTOR_SPEED = 100   # 收球最大速度（正转收球）

# 8x16 LED 点阵屏配置（横屏：16列 × 8行）
LED_PORT = "PORT2"          # 点阵屏连接的 PORT 口（PORT1~PORT4）
LED_INDEX = "INDEX1"        # 端口链上的序号

# 调试模式配置
DEBUG_SEQUENCE = ["N1", "N3", "N2", "N4", "N4"]  # 密码序列: 13244
DEBUG_TIMEOUT = 6.0          # 开机后多少秒内输入有效
DEBUG_TEST_SPEED = 50        # 调试模式中电机测试速度
DEBUG_MOTOR_TYPES = ["编码电机", "直流电机"]       # 电机类型名称
DEBUG_MOTOR_NAMES = ["M1", "M2", "M3"]            # 电机编号名称

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
__led = led_matrix_class(LED_PORT, LED_INDEX)     # 8x16 点阵屏

# ==================== 正面状态 ====================
face = 0  # 当前正面: 0=M1-M2, 1=M2-M3, 2=M3-M1

# ==================== 自动程序 ====================
# 自动程序步骤列表：每步 = (持续时间_秒, Vx, Vy, omega)
#   Vx: 横向速度 (+右)，Vy: 纵向速度 (+前)，omega: 旋转速度 (+顺时针)
#   最后一个步骤建议设为 (0.1, 0, 0, 0) 作为停止缓冲
AUTO_SEQUENCE = [
    (4.0,  0, AUTO_SPEED,  0),   # 步骤 0: 前进 4 秒
    (0.1,  0,           0,  0),   # 步骤 1: 停止 (缓冲结束)
]

# 自动程序运行时状态
auto_mode = False        # 当前是否在自动模式
auto_step = 0            # 当前步骤索引
auto_step_start = 0.0    # 当前步骤开始时间

# ==================== 收球开关状态 ====================
collector_on = False     # 收球电机当前状态: False=关, True=开

# ==================== 调试模式状态 ====================
debug_mode = False            # 当前是否在调试模式
debug_entered_once = False    # 是否已进入过调试模式（仅一次）
debug_motor_type = 0          # 0=编码电机, 1=直流电机
debug_motor_index = 0         # 0=M1, 1=M2, 2=M3
debug_seq_pos = 0             # 密码序列当前匹配位置
debug_seq_last = {}           # 序列各键上一次状态（边沿检测用）

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
    """紧急停止所有电机"""
    __motor_M1.set_power(0)
    __motor_M2.set_power(0)
    __motor_M3.set_power(0)


def debug_stop_motor():
    """停止调试模式中当前选中的测试电机"""
    if debug_motor_type == 0:  # 编码电机
        motor = [__motor_M1, __motor_M2, __motor_M3][debug_motor_index]
        motor.set_power(0)
    else:  # 直流电机
        power_expand_board.set_power(DC_COLLECTOR_PORT, 0)


# ==================== 边沿触发辅助变量 ====================
last_R1 = False
last_L1 = False
last_Plus = False
last_N1 = False

# 调试模式按键边沿变量
last_Up = False
last_Down = False
last_Left = False
last_Right = False
last_N2 = False
last_N3 = False
last_N4 = False

# ==================== 启动确认 ====================
print("=" * 40)
print("  三轮全向机器人已启动！")
print("  左摇杆 → 全向移动")
print("  右摇杆 ←→ 自旋")
print("  R1 / L1 → 切换正面")
print("  + 键 → 自动程序")
print("  前6秒输入13244 → 调试模式")
print("=" * 40)

# 点阵屏开机显示
__led.show("Main")
novapi.reset_timer()

# 初始化序列检测的按键历史状态
for key in DEBUG_SEQUENCE:
    debug_seq_last[key] = False

# ==================== 主循环 ====================
while True:
    # ================================================================
    #  密码序列检测（仅开机前 DEBUG_TIMEOUT 秒、未进入过调试模式）
    #  密码: 13244 → [N1, N3, N2, N4, N4]
    # ================================================================
    if not debug_entered_once and not debug_mode and novapi.timer() < DEBUG_TIMEOUT:
        expected_key = DEBUG_SEQUENCE[debug_seq_pos]
        cur_key = gamepad.is_key_pressed(expected_key)
        prev_key = debug_seq_last.get(expected_key, False)

        if cur_key and not prev_key:      # 上升沿 → 按对了当前位
            debug_seq_pos += 1
            print(">>> 密码 %d/%d: %s ✓" % (debug_seq_pos, len(DEBUG_SEQUENCE), expected_key))
            if debug_seq_pos >= len(DEBUG_SEQUENCE):
                # 序列完整匹配 → 进入调试模式
                debug_mode = True
                debug_entered_once = True
                debug_motor_type = 0       # 默认编码电机
                debug_motor_index = 0      # 默认 M1
                debug_seq_pos = 0
                __led.show("Test")
                print("=" * 40)
                print("  >>> 进入调试模式！")
                print("  ↑↓ 切换电机类型 | ← → 切换电机编号")
                print("  N1=+50  N2=-50  |  N2+N3=退出")
                print("=" * 40)
        elif cur_key and prev_key:
            pass  # 按住中，不触发
        else:
            # 如果按了错误的键（序列外的键），重置进度
            # 检查是否有任何数字键被按下
            any_seq_key = False
            for k in DEBUG_SEQUENCE:
                if gamepad.is_key_pressed(k):
                    any_seq_key = True
                    break
            # 简单策略：只有按下序列中不匹配的键才重置
            pass  # 暂不重置，只检测匹配

        # 更新所有序列键的状态
        for key in DEBUG_SEQUENCE:
            debug_seq_last[key] = gamepad.is_key_pressed(key)

    # ================================================================
    #  自动模式
    # ================================================================
    if auto_mode:
        # 检查当前步骤是否超时
        elapsed = novapi.timer() - auto_step_start
        step_duration, step_Vx, step_Vy, step_omega = AUTO_SEQUENCE[auto_step]

        if elapsed >= step_duration:
            # 进入下一步
            auto_step += 1
            if auto_step >= len(AUTO_SEQUENCE):
                # 所有步骤完成，退出自动模式
                stop_all_motors()
                auto_mode = False
                auto_step = 0
                print(">>> 自动程序完成，恢复遥控")
            else:
                # 切换到下一步
                auto_step_start = novapi.timer()
                next_dur, next_Vx, next_Vy, next_omega = AUTO_SEQUENCE[auto_step]
                print(">>> 自动步骤 %d/%d: Vx=%d Vy=%d ω=%d (%.1fs)" %
                      (auto_step + 1, len(AUTO_SEQUENCE), next_Vx, next_Vy, next_omega, next_dur))
        else:
            # 执行当前步骤：根据当前正面旋转速度，再经运动学输出
            Vx_rot, Vy_rot = rotate_velocity(step_Vx, step_Vy, face)
            M1_power, M2_power, M3_power = omni_kinematics(Vx_rot, Vy_rot, step_omega)
            __motor_M1.set_power(M1_power)
            __motor_M2.set_power(M2_power)
            __motor_M3.set_power(M3_power)

        time.sleep(AUTO_STEP_DELAY)
        continue  # 跳过手动/调试逻辑

    # ================================================================
    #  调试模式
    # ================================================================
    if debug_mode:
        # --- 读取十字键和功能键 ---
        cur_Up = gamepad.is_key_pressed("Up")
        cur_Down = gamepad.is_key_pressed("Down")
        cur_Left = gamepad.is_key_pressed("Left")
        cur_Right = gamepad.is_key_pressed("Right")
        cur_N1 = gamepad.is_key_pressed("N1")
        cur_N2 = gamepad.is_key_pressed("N2")
        cur_N3 = gamepad.is_key_pressed("N3")

        # --- 十字键 ↑/↓：切换电机类型 ---
        if cur_Up and not last_Up:
            debug_motor_type = (debug_motor_type + 1) % len(DEBUG_MOTOR_TYPES)
            print(">>> 电机类型: %s" % DEBUG_MOTOR_TYPES[debug_motor_type])
        if cur_Down and not last_Down:
            debug_motor_type = (debug_motor_type - 1) % len(DEBUG_MOTOR_TYPES)
            print(">>> 电机类型: %s" % DEBUG_MOTOR_TYPES[debug_motor_type])

        # --- 十字键 ←/→：切换电机编号 ---
        if cur_Left and not last_Left:
            debug_motor_index = (debug_motor_index - 1) % len(DEBUG_MOTOR_NAMES)
            print(">>> 电机编号: %s" % DEBUG_MOTOR_NAMES[debug_motor_index])
        if cur_Right and not last_Right:
            debug_motor_index = (debug_motor_index + 1) % len(DEBUG_MOTOR_NAMES)
            print(">>> 电机编号: %s" % DEBUG_MOTOR_NAMES[debug_motor_index])

        # --- N2 + N3 同时按下 → 退出调试模式 ---
        if cur_N2 and cur_N3:
            debug_stop_motor()
            debug_mode = False
            __led.show("Main")
            print(">>> 退出调试模式，恢复正常操控")
            # 重置边沿变量防止残留触发
            last_Up = last_Down = last_Left = last_Right = False
            last_N1 = last_N2 = last_N3 = last_N4 = False
            time.sleep(0.3)   # 防抖
            continue

        # --- N1 / N2：正转 / 反转测试 ---
        if cur_N1:
            speed = DEBUG_TEST_SPEED
        elif cur_N2:
            speed = -DEBUG_TEST_SPEED
        else:
            speed = 0

        # 输出到对应电机
        if debug_motor_type == 0:  # 编码电机
            motor = [__motor_M1, __motor_M2, __motor_M3][debug_motor_index]
            motor.set_power(speed)
        else:  # 直流电机
            power_expand_board.set_power(DC_COLLECTOR_PORT, speed)

        # 更新边沿变量
        last_Up = cur_Up
        last_Down = cur_Down
        last_Left = cur_Left
        last_Right = cur_Right
        last_N1 = cur_N1
        last_N2 = cur_N2
        last_N3 = cur_N3

        time.sleep(LOOP_DELAY)
        continue  # 跳过正常模式

    # ================================================================
    #  正常模式（手动遥控）
    # ================================================================
    # --- 1. 读取摇杆原始值 ---
    Lx = gamepad.get_joystick("Lx")   # 左摇杆水平: -100(左) ~ +100(右)
    Ly = gamepad.get_joystick("Ly")   # 左摇杆垂直: -100(下) ~ +100(上)
    Rx = gamepad.get_joystick("Rx")   # 右摇杆水平: -100(左) ~ +100(右)

    # --- 2. 按键边沿触发 ---
    cur_R1 = gamepad.is_key_pressed("R1")
    cur_L1 = gamepad.is_key_pressed("L1")
    cur_Plus = gamepad.is_key_pressed("+")
    cur_N1 = gamepad.is_key_pressed("N1")

    if cur_R1 and not last_R1:          # R1 上升沿 → 右转切面
        face = (face - 1) % 3
        print(">>> 正面切换: %s" % FACE_NAMES[face])
    if cur_L1 and not last_L1:          # L1 上升沿 → 左转切面
        face = (face + 1) % 3
        print(">>> 正面切换: %s" % FACE_NAMES[face])
    if cur_Plus and not last_Plus:      # + 上升沿 → 启动自动程序
        auto_mode = True
        auto_step = 0
        auto_step_start = novapi.timer()
        dur0, Vx0, Vy0, w0 = AUTO_SEQUENCE[0]
        print(">>> 自动程序启动！步骤 1/%d: Vx=%d Vy=%d ω=%d (%.1fs)" %
              (len(AUTO_SEQUENCE), Vx0, Vy0, w0, dur0))
    if cur_N1 and not last_N1:          # N1 上升沿 → 收球开关翻转
        collector_on = not collector_on
        if collector_on:
            power_expand_board.set_power(DC_COLLECTOR_PORT, DC_COLLECTOR_SPEED)
            print(">>> 收球电机: 开 (正转 %d)" % DC_COLLECTOR_SPEED)
        else:
            power_expand_board.set_power(DC_COLLECTOR_PORT, 0)
            print(">>> 收球电机: 关")

    last_R1 = cur_R1
    last_L1 = cur_L1
    last_Plus = cur_Plus
    last_N1 = cur_N1

    # --- 3. 死区过滤 ---
    Lx = apply_dead_zone(Lx)
    Ly = apply_dead_zone(Ly)
    Rx = apply_dead_zone(Rx)

    # --- 4. 速度缩放与映射 ---
    Vx_raw =  Lx * SPEED_SCALE       # 左摇杆水平 → 原始横移速度（+右）
    Vy_raw = -Ly * SPEED_SCALE       # 左摇杆垂直 → 原始前进速度（摇杆上推时 Ly>0，电机正转为后退，故取反）
    omega  =  Rx * SPEED_SCALE       # 右摇杆水平 → 旋转速度（+顺时针）

    # --- 5. 根据当前正面旋转速度向量 ---
    Vx, Vy = rotate_velocity(Vx_raw, Vy_raw, face)

    # --- 6. 运动学解算 ---
    M1_power, M2_power, M3_power = omni_kinematics(Vx, Vy, omega)

    # --- 7. 输出到电机 ---
    __motor_M1.set_power(M1_power)
    __motor_M2.set_power(M2_power)
    __motor_M3.set_power(M3_power)

    # --- 8. 循环延时 ---
    time.sleep(LOOP_DELAY)
