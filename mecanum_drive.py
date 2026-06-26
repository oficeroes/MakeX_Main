"""
麦克纳姆轮机器人 — 遥控操控程序
================================
描述：基于 Novapi 平台的麦克纳姆轮底盘遥控程序。
      左摇杆控制全向移动（前进/后退/左右横移），
      右摇杆左右控制原地自旋，
      两个摇杆可同时操作实现复合运动（如边前进边转圈）。

硬件需求：
  底盘 — 编码电机 ×4（X 型麦克纳姆布局）
    - M1（左前轮）— 正常接线
    - M2（右前轮）— 反接 ⚠️
    - M3（左后轮）— 正常接线
    - M4（右后轮）— 反接 ⚠️
  执行机构 —
    - DC1 / DC2 — 直流电机（收球），接动力扩展板
    - M5 — 编码电机（滚球）

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
  │  收球正转   │  按 N2（开/关切换）  │
  │  收球反转   │  按 N3（开/关切换）  │
  │  滚球正转   │  按 N1（开/关切换）  │
  │  滚球反转   │  按 N4（开/关切换）  │
  └────────────┴──────────────────────┘

  互斥规则：
  - 收球正转(100)中按 N3 → 取消(0)，反之亦然
  - 滚球正转中按 N4 → 取消(0)，反之亦然

麦克纳姆轮运动学模型（X 型布局）
==============================
         前进方向 (+Vy)
              ↑
       M1 ←──┼──→ M2      (左前 M1, 右前 M2)
              │
              │
       M3 ←──┼──→ M4      (左后 M3, 右后 M4)

  X 型布局特征：
  - M1、M4 辊子呈 \ 走向（左上→右下）
  - M2、M3 辊子呈 / 走向（右上→左下）
  - 轮毂正转时，M1/M4 产生"右前"推力，M2/M3 产生"左前"推力

  标准逆运动学方程（全轮正转=前进）:
    M1_raw =  Vx + Vy + ω
    M2_raw = -Vx + Vy - ω
    M3_raw = -Vx + Vy + ω
    M4_raw =  Vx + Vy - ω

  其中 Vx=横向速度(+右), Vy=纵向速度(+前), ω=旋转速度(+顺时针)

  ⚠️ M2、M4 电机反接：
     电机正转时物理反转，故在输出端对 M2/M4 取反：
       M1_out =  Vx + Vy + ω
       M2_out = -(-Vx + Vy - ω) =  Vx - Vy + ω
       M3_out = -Vx + Vy + ω
       M4_out = -( Vx + Vy - ω) = -Vx - Vy + ω

  向量缩放：
  当任一电机速度的绝对值超过 100 时，
  等比例缩放全部四个电机速度，保持运动方向不变。

  方向校准说明：
  若实际运行时出现以下情况，请调整对应的符号常量：
  - 推前进却后退 → 将 INVERT_VY 取反（True↔False）
  - 推右横移却左移 → 将 INVERT_VX 取反（True↔False）
  - 推右转却左转 → 将 INVERT_OMEGA 取反（True↔False）
"""
import novapi
import time
from mbuild import gamepad
from mbuild import power_expand_board
from mbuild.encoder_motor import encoder_motor_class

# ==================== 配置常量 ====================
SPEED_SCALE = 0.6       # 全局速度倍率 (0~1)，安全起见默认 60%
DEAD_ZONE = 8           # 摇杆死区阈值，小于此值的输入视为 0（防误触）
LOOP_DELAY = 0.02       # 主循环周期（秒），20ms = 50Hz

# 方向校准开关（若方向反了，将对应常量改为相反的布尔值即可）
INVERT_VX = False       # True = 横向取反
INVERT_VY = True        # True = 纵向取反（参考三轮车：电机正转=后退）
INVERT_OMEGA = False    # True = 旋转取反

# 收球直流电机配置
DC_COLLECTOR_PORT1 = "DC1"   # 收球电机 1（动力扩展板通道 1）
DC_COLLECTOR_PORT2 = "DC2"   # 收球电机 2（动力扩展板通道 2）
DC_COLLECTOR_SPEED = 100     # 收球最大速度（正转收球）

# 滚球编码电机配置
ROLLER_SPEED = 80            # 滚球电机默认速度（0~100）

# ==================== 硬件初始化 ====================
# M1: 左前轮（正常接线）
# M2: 右前轮（反接 ⚠️）
# M3: 左后轮（正常接线）
# M4: 右后轮（反接 ⚠️）
__motor_M1 = encoder_motor_class("M1", "INDEX1")
__motor_M2 = encoder_motor_class("M2", "INDEX1")
__motor_M3 = encoder_motor_class("M3", "INDEX1")
__motor_M4 = encoder_motor_class("M4", "INDEX1")
__motor_M5 = encoder_motor_class("M5", "INDEX1")  # 滚球电机

# ==================== 运动学函数 ====================
def mecanum_kinematics(Vx, Vy, omega):
    """
    麦克纳姆轮逆运动学计算（X 型布局）

    参数:
        Vx  (float): 横向目标速度（+右, -左），范围 -100~100
        Vy  (float): 纵向目标速度（+前, -后），范围 -100~100
        omega (float): 旋转目标速度（+顺时针, -逆时针），范围 -100~100

    返回:
        tuple: (M1_power, M2_power, M3_power, M4_power)，范围 -100~100

    实现细节:
        1. 标准逆运动学解算四轮原始速度
        2. M2、M4 反接修正：对其输出取反
        3. 向量等比缩放：若任一 |power| > 100，等比例压缩
    """
    # --- 第 1 步：标准逆运动学 ---
    M1 =  Vx + Vy + omega
    M2 = -Vx + Vy - omega
    M3 = -Vx + Vy + omega
    M4 =  Vx + Vy - omega

    # --- 第 2 步：M2、M4 反接修正 ---
    M2 = -M2   # → Vx - Vy + omega
    M4 = -M4   # → -Vx - Vy + omega

    # --- 第 3 步：向量等比缩放（避免硬截断导致方向畸变）---
    max_abs = max(abs(M1), abs(M2), abs(M3), abs(M4))
    if max_abs > 100:
        scale = 100.0 / max_abs
        M1 *= scale
        M2 *= scale
        M3 *= scale
        M4 *= scale

    return M1, M2, M3, M4


def apply_dead_zone(value, threshold=DEAD_ZONE):
    """摇杆死区过滤：绝对值小于阈值的值置零，防止手柄漂移误触"""
    if abs(value) < threshold:
        return 0
    return value


def stop_all_motors():
    """紧急停止所有电机（含执行机构）"""
    __motor_M1.set_power(0)
    __motor_M2.set_power(0)
    __motor_M3.set_power(0)
    __motor_M4.set_power(0)
    __motor_M5.set_power(0)
    power_expand_board.set_power(DC_COLLECTOR_PORT1, 0)
    power_expand_board.set_power(DC_COLLECTOR_PORT2, 0)

# ==================== 启动确认 ====================
# 收球 / 滚球状态（三态: 1=正转, -1=反转, 0=停止）
dc_state = 0          # DC1+DC2 收球电机状态
roller_state = 0      # M5 滚球电机状态

# 按键边沿检测变量
last_N1 = False
last_N2 = False
last_N3 = False
last_N4 = False

print("=" * 40)
print("  麦克纳姆轮机器人已启动！")
print("  左摇杆 → 全向移动")
print("  右摇杆 ←→ 自旋")
print("  N1 → M5 滚球正转  |  N4 → M5 滚球反转")
print("  N2 → DC 收球正转  |  N3 → DC 收球反转")
print("  (同组按键互斥：按另一方向=取消)")
print("  方向校准: VX=%s VY=%s ω=%s" %
      ("反" if INVERT_VX else "正",
       "反" if INVERT_VY else "正",
       "反" if INVERT_OMEGA else "正"))
print("=" * 40)

# ==================== 主循环 ====================
while True:
    # --- 1. 读取摇杆原始值 ---
    Lx = gamepad.get_joystick("Lx")   # 左摇杆水平: -100(左) ~ +100(右)
    Ly = gamepad.get_joystick("Ly")   # 左摇杆垂直: -100(下) ~ +100(上)
    Rx = gamepad.get_joystick("Rx")   # 右摇杆水平: -100(左) ~ +100(右)

    # --- 2. 死区过滤 ---
    Lx = apply_dead_zone(Lx)
    Ly = apply_dead_zone(Ly)
    Rx = apply_dead_zone(Rx)

    # --- 3. 速度缩放与方向映射 ---
    Vx = Lx * SPEED_SCALE       # 左摇杆水平 → 横移速度（+右）
    Vy = Ly * SPEED_SCALE       # 左摇杆垂直 → 前进速度（+前）
    omega = Rx * SPEED_SCALE    # 右摇杆水平 → 旋转速度（+顺时针）

    # 方向校准（参考三轮车经验：电机正转=后退，故 Vy 默认取反）
    if INVERT_VX:
        Vx = -Vx
    if INVERT_VY:
        Vy = -Vy
    if INVERT_OMEGA:
        omega = -omega

    # --- 4. 运动学解算 ---
    M1_power, M2_power, M3_power, M4_power = mecanum_kinematics(Vx, Vy, omega)

    # --- 5. 按键边沿触发（收球 / 滚球）---
    cur_N1 = gamepad.is_key_pressed("N1")
    cur_N2 = gamepad.is_key_pressed("N2")
    cur_N3 = gamepad.is_key_pressed("N3")
    cur_N4 = gamepad.is_key_pressed("N4")

    # N1: M5 滚球正转（与 N4 反转互斥）
    if cur_N1 and not last_N1:
        if roller_state == 1:
            roller_state = 0
            __motor_M5.set_power(0)
            print(">>> 滚球电机: 关")
        else:
            roller_state = 1
            __motor_M5.set_power(ROLLER_SPEED)
            print(">>> 滚球电机: 正转 (M5, +%d)" % ROLLER_SPEED)

    # N4: M5 滚球反转（与 N1 正转互斥）
    if cur_N4 and not last_N4:
        if roller_state == -1:
            roller_state = 0
            __motor_M5.set_power(0)
            print(">>> 滚球电机: 关")
        else:
            roller_state = -1
            __motor_M5.set_power(-ROLLER_SPEED)
            print(">>> 滚球电机: 反转 (M5, -%d)" % ROLLER_SPEED)

    # N2: DC1+DC2 收球正转（与 N3 反转互斥）
    if cur_N2 and not last_N2:
        if dc_state == 1:
            dc_state = 0
            power_expand_board.set_power(DC_COLLECTOR_PORT1, 0)
            power_expand_board.set_power(DC_COLLECTOR_PORT2, 0)
            print(">>> 收球电机: 关")
        else:
            dc_state = 1
            power_expand_board.set_power(DC_COLLECTOR_PORT1, DC_COLLECTOR_SPEED)
            power_expand_board.set_power(DC_COLLECTOR_PORT2, DC_COLLECTOR_SPEED)
            print(">>> 收球电机: 正转 (DC1+DC2, +%d)" % DC_COLLECTOR_SPEED)

    # N3: DC1+DC2 收球反转（与 N2 正转互斥）
    if cur_N3 and not last_N3:
        if dc_state == -1:
            dc_state = 0
            power_expand_board.set_power(DC_COLLECTOR_PORT1, 0)
            power_expand_board.set_power(DC_COLLECTOR_PORT2, 0)
            print(">>> 收球电机: 关")
        else:
            dc_state = -1
            power_expand_board.set_power(DC_COLLECTOR_PORT1, -DC_COLLECTOR_SPEED)
            power_expand_board.set_power(DC_COLLECTOR_PORT2, -DC_COLLECTOR_SPEED)
            print(">>> 收球电机: 反转 (DC1+DC2, -%d)" % DC_COLLECTOR_SPEED)

    last_N1 = cur_N1
    last_N2 = cur_N2
    last_N3 = cur_N3
    last_N4 = cur_N4

    # --- 6. 输出到底盘电机 ---
    __motor_M1.set_power(M1_power)
    __motor_M2.set_power(M2_power)
    __motor_M3.set_power(M3_power)
    __motor_M4.set_power(M4_power)

    # --- 7. 循环延时 ---
    time.sleep(LOOP_DELAY)
