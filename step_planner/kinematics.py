"""step_planner.kinematics — Step → AUTO_SEQUENCE 元组转换 + 死算轨迹

全部纯函数，无 Qt 依赖，无路径几何，无近似。

核心设计原则
============
用户填 "前进 50 cm 功率 70%"
程序做：ticks = round(50 * encoder_ticks_per_cm) = 474
输出：('enc_move', 474, 70, 0)
中间没有任何其他步骤。

坐标轴约定（match firmware）
============================
  Vy > 0  = 前进（+Y 方向）
  Vx > 0  = 右移（+X 方向）
  omega > 0 = 逆时针（firmware 约定 CCW = 正）
             → ROTATE_CW 时 omega_power 取负值

死算轨迹坐标系
==============
  +X = 场地右方
  +Y = 场地上方
  heading 0° = 机器人朝上(+Y)，顺时针增加（对应 ROTATE_CW）

  FORWARD:  dx = sin(h)*d,  dy = cos(h)*d
  BACKWARD: dx = -sin(h)*d, dy = -cos(h)*d
  STRAFE_LEFT:  -90° relative to heading
  STRAFE_RIGHT: +90° relative to heading
  ROTATE_CW:  heading += degrees
  ROTATE_CCW: heading -= degrees

API
===
  step_to_tuples(step, cal)          -> list[tuple]
  build_sequence(steps, cal)         -> list[tuple]   # 末尾追加 enc_stop
  dead_reckoning_poses(steps, start) -> list[Pose]    # len = len(steps)+1
  estimate_total_distance(steps)     -> float         # cm, 仅线性步骤
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .models import CalibrationState, Pose, Step, StepType

from .models import (
    LINEAR_TYPES,
    MOTION_TYPES,
    CalibrationState,
    Pose,
    Step,
    StepType,
)


# =========================================================
#  step_to_tuples — 单步转换
# =========================================================
def _enc_move_with_ramp(
    total_ticks: int, vy: int, vx: int,
    accel_cm: float, decel_cm: float, min_pwr: int, tpc: float,
) -> list[tuple]:
    """生成一条 enc_move_s（S 曲线加减速）或普通 enc_move。

    输出格式：
      ('enc_move_s', total_ticks, vy, vx, accel_ticks, decel_ticks, min_pwr)
    固件端用 smoothstep（3t²-2t³）从 min_pwr→max 加速、max→0 减速，
    中间段保持满功率。无 accel/decel 时退化为普通 enc_move。
    """
    if accel_cm <= 0 and decel_cm <= 0:
        return [("enc_move", total_ticks, vy, vx)]

    max_pwr = max(abs(vy), abs(vx))
    if max_pwr < 1:
        return [("enc_move", total_ticks, vy, vx)]

    accel_ticks = min(int(round(accel_cm * tpc)), total_ticks)
    decel_ticks = min(int(round(decel_cm * tpc)), total_ticks - accel_ticks)
    clamped_min = max(0, min(min_pwr, 100))

    return [("enc_move_s", total_ticks, vy, vx,
             accel_ticks, decel_ticks, clamped_min)]


# 三轮全向三个面的旋转矩阵 (c, ns, s, nc)，与固件 FACE_ROTATIONS 一致
# face0=(-1,0,0,-1) 180°（M3顶点为前），face1=120°CW，face2=240°CW
_OMNI3_FACE_ROTATIONS = [
    (-1.0,  0.0,  0.0, -1.0),
    (-0.5, -0.866, 0.866, -0.5),
    (-0.5,  0.866, -0.866, -0.5),
]

def _apply_face_rot(vy: float, vx: float, face_rot) -> tuple:
    """将 (vy, vx) 向量按面旋转矩阵变换，返回新的 (vy, vx)。"""
    if face_rot is None:
        return vy, vx
    c, ns, s, nc = face_rot
    new_vy = c * vy + ns * vx
    new_vx = s * vy + nc * vx
    return new_vy, new_vx


def step_to_tuples(step: Step, cal: CalibrationState,
                   face_rot=None) -> list[tuple]:
    """将一条 Step 转换为零个或多个 AUTO_SEQUENCE 元组。

    不修改 step，不产生副作用。

    轴反转语义（invert_vx / invert_vy / invert_omega）：
      GUI 画面坐标系与固件实车坐标系可能镜像。若勾选 invert_vy，
      则"GUI 上的前进"对应固件的 Vy 取反（vy_power 符号翻转）。
      Vx/omega 同理。反转只影响符号，不改变幅值和 ticks 数量。

    Vy/Vx 符号约定（验证自 mecanum_drive.py mecanum_kinematics）：
      Vy > 0  → M1/M2/M3/M4 均正 → 前进
      Vx > 0  → M1−, M2+, M3+, M4− → 右横移（X 型麦轮）

    enc_rot omega 符号（验证自两个 firmware 的 _enc_rot_start）：
      omega_power > 0  → 逆时针（CCW）
      omega_power < 0  → 顺时针（CW）
      → ROTATE_CW 用负号，ROTATE_CCW 用正号
    """
    tpc = cal.encoder_ticks_per_cm   # °/cm，线性
    ttd = cal.turn_ticks_per_deg     # °/体转°，旋转
    t   = step.step_type

    # 反转符号（1 或 -1）
    sv = -1 if cal.invert_vy    else 1
    sx = -1 if cal.invert_vx    else 1
    so = -1 if cal.invert_omega else 1

    # 加减速参数（仅线性步骤使用）
    ac  = max(0.0, step.accel_cm)
    dc  = max(0.0, step.decel_cm)
    mp  = max(0, min(100, step.min_power_pct))

    # ── 线性运动 ──────────────────────────────────────────────────
    if t == StepType.FORWARD:
        ticks = max(1, int(round(step.distance_cm * tpc)))
        vy, vx = _apply_face_rot(sv * step.power_pct, 0, face_rot)
        return _enc_move_with_ramp(ticks, int(round(vy)), int(round(vx)), ac, dc, mp, tpc)

    elif t == StepType.BACKWARD:
        ticks = max(1, int(round(step.distance_cm * tpc)))
        vy, vx = _apply_face_rot(sv * (-step.power_pct), 0, face_rot)
        return _enc_move_with_ramp(ticks, int(round(vy)), int(round(vx)), ac, dc, mp, tpc)

    elif t == StepType.STRAFE_LEFT:
        ticks = max(1, int(round(step.distance_cm * tpc)))
        vy, vx = _apply_face_rot(0, sx * (-step.power_pct), face_rot)
        return _enc_move_with_ramp(ticks, int(round(vy)), int(round(vx)), ac, dc, mp, tpc)

    elif t == StepType.STRAFE_RIGHT:
        ticks = max(1, int(round(step.distance_cm * tpc)))
        vy, vx = _apply_face_rot(0, sx * step.power_pct, face_rot)
        return _enc_move_with_ramp(ticks, int(round(vy)), int(round(vx)), ac, dc, mp, tpc)

    elif t == StepType.MOVE:
        ticks = max(1, int(round(step.distance_cm * tpc)))
        rad = math.radians(step.direction_deg)
        vy  = sv * int(round(step.power_pct * math.cos(rad)))
        vx  = sx * int(round(step.power_pct * math.sin(rad)))
        if vy == 0 and vx == 0:
            vy = sv * step.power_pct
        vy, vx = _apply_face_rot(vy, vx, face_rot)
        return _enc_move_with_ramp(ticks, int(round(vy)), int(round(vx)), ac, dc, mp, tpc)

    # ── 旋转 ──────────────────────────────────────────────────────
    elif t == StepType.ROTATE_CW:
        rot_ticks = max(1, int(round(step.degrees * ttd)))
        at = max(0, int(round(step.rot_accel_deg * ttd)))
        dt = max(0, int(round(step.rot_decel_deg * ttd)))
        mp = max(0, min(100, step.rot_min_power_pct))
        if at > 0 or dt > 0:
            return [("enc_rot", rot_ticks, 0, so * (-abs(step.omega_pct)), at, dt, mp)]
        return [("enc_rot", rot_ticks, 0, so * (-abs(step.omega_pct)))]

    elif t == StepType.ROTATE_CCW:
        rot_ticks = max(1, int(round(step.degrees * ttd)))
        at = max(0, int(round(step.rot_accel_deg * ttd)))
        dt = max(0, int(round(step.rot_decel_deg * ttd)))
        mp = max(0, min(100, step.rot_min_power_pct))
        if at > 0 or dt > 0:
            return [("enc_rot", rot_ticks, 0, so * (+abs(step.omega_pct)), at, dt, mp)]
        return [("enc_rot", rot_ticks, 0, so * (+abs(step.omega_pct)))]

    # ── 执行机构 ──────────────────────────────────────────────────
    elif t == StepType.SERVO:
        action = 1 if step.grip_action == "close" else 0
        return [("grip", step.servo_id, action)]

    elif t == StepType.DELAY:
        return [("delay", step.duration_s)]

    elif t == StepType.MOTOR:
        return [("motor", step.motor_id, step.motor_power)]

    elif t == StepType.DC_MOTOR:
        return [("dc_motor", step.dc_port, step.dc_power)]

    # ── 平移自旋 (MOVE_SPIN → enc_moverot) ────────────────────────
    elif t == StepType.MOVE_SPIN:
        move_ticks = max(1, int(round(step.distance_cm * tpc)))
        rad = math.radians(step.direction_deg)
        vy  = sv * int(round(step.power_pct * math.cos(rad)))
        vx  = sx * int(round(step.power_pct * math.sin(rad)))
        if vy == 0 and vx == 0:
            vy = sv * step.power_pct
        vy, vx = _apply_face_rot(vy, vx, face_rot)
        rot_ticks = max(1, int(round(abs(step.spin_degrees) * ttd)))
        omega = so * (-abs(step.omega_pct) if step.spin_degrees >= 0
                      else +abs(step.omega_pct))
        return [("enc_moverot", move_ticks, int(round(vy)), int(round(vx)), omega, rot_ticks)]

    # ── 开环移动 (OPEN_MOVE) ────────────────────────────────────────
    elif t == StepType.OPEN_MOVE:
        rad = math.radians(step.direction_deg)
        vy  = sv * int(round(step.power_pct * math.cos(rad)))
        vx  = sx * int(round(step.power_pct * math.sin(rad)))
        if vy == 0 and vx == 0:
            vy = sv * step.power_pct
        vy, vx = _apply_face_rot(vy, vx, face_rot)
        omega = so * step.open_omega_pct
        return [("open_move", step.open_duration_ms, int(round(vy)), int(round(vx)), omega)]

    # ── 开环自旋 (OPEN_ROT) ─────────────────────────────────────────
    elif t == StepType.OPEN_ROT:
        omega = so * step.omega_pct
        return [("open_move", step.open_duration_ms, 0, 0, omega)]

    # ── 开环直行 / 平移 (OPEN_FORWARD / BACKWARD / LEFT / RIGHT) ─────
    elif t == StepType.OPEN_FORWARD:
        vy, vx = _apply_face_rot(sv * step.power_pct, 0, face_rot)
        return [("open_move", step.open_duration_ms, int(round(vy)), int(round(vx)), 0)]

    elif t == StepType.OPEN_BACKWARD:
        vy, vx = _apply_face_rot(sv * (-step.power_pct), 0, face_rot)
        return [("open_move", step.open_duration_ms, int(round(vy)), int(round(vx)), 0)]

    elif t == StepType.OPEN_LEFT:
        vy, vx = _apply_face_rot(0, sx * (-step.power_pct), face_rot)
        return [("open_move", step.open_duration_ms, int(round(vy)), int(round(vx)), 0)]

    elif t == StepType.OPEN_RIGHT:
        vy, vx = _apply_face_rot(0, sx * step.power_pct, face_rot)
        return [("open_move", step.open_duration_ms, int(round(vy)), int(round(vx)), 0)]

    elif t == StepType.LIFT:
        # arm_code: 0=二号臂M4, 1=一号臂M5, 2=双臂
        arm_map = {"arm2": 0, "arm1": 1, "both": 2}
        arm_code = arm_map.get(step.lift_arm, 2)
        tag = "lift_async" if step.lift_async else "lift"
        return [(tag, arm_code, int(round(step.lift_angle_deg)))]

    elif t == StepType.FACE_CHANGE:
        # 换面在 build_sequence 里处理（更新 face_index），这里不输出元组
        return []

    return []


# =========================================================
#  build_sequence — 完整序列
# =========================================================
def build_sequence(steps: list[Step], cal: CalibrationState,
                   use_face_rotations: bool = False) -> list[tuple]:
    """把所有步骤转换为 AUTO_SEQUENCE 元组列表，末尾追加 enc_stop。

    use_face_rotations=True 时（三轮全向），遇到 FACE_CHANGE 步骤只递增
    面索引，不输出元组；后续平移步骤的 vy/vx 已在导出时按该面旋转矩阵
    变换完毕，固件无需处理换面逻辑。
    """
    seq: list[tuple] = []
    face_index = 0
    for step in steps:
        if use_face_rotations and step.step_type == StepType.FACE_CHANGE:
            face_index = (face_index + 1) % 3
            continue
        face_rot = _OMNI3_FACE_ROTATIONS[face_index] if use_face_rotations else None
        seq.extend(step_to_tuples(step, cal, face_rot=face_rot))
    seq.append(("enc_stop", 0, 0, 0))
    return seq


# =========================================================
#  dead_reckoning_poses — 死算轨迹
# =========================================================
def dead_reckoning_poses(steps: list[Step], start: Pose) -> list[Pose]:
    """从起始位姿推算每步结束后的位姿，返回 len(steps)+1 个 Pose。

    poses[0] = start（步骤开始前）
    poses[i+1] = 执行 steps[i] 后的位姿

    非运动步骤（SERVO / DELAY / MOTOR / DC_MOTOR）：位姿不变（复制上一个）。

    这是纯几何预测，不涉及实际电机模型，仅用于 GUI 预览。
    """
    poses: list[Pose] = [Pose(start.x, start.y, start.heading)]
    x, y, h = start.x, start.y, start.heading  # h in degrees

    for step in steps:
        t = step.step_type
        d = step.distance_cm

        if t == StepType.FORWARD:
            rad = math.radians(h)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.BACKWARD:
            rad = math.radians(h)
            x -= d * math.sin(rad)
            y -= d * math.cos(rad)

        elif t == StepType.STRAFE_LEFT:
            # 左移 = heading − 90°
            rad = math.radians(h - 90)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.STRAFE_RIGHT:
            # 右移 = heading + 90°
            rad = math.radians(h + 90)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.MOVE:
            # direction_deg: 0°=前进方向, 90°=右（机器人坐标系）
            rad = math.radians(h + step.direction_deg)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.ROTATE_CW:
            h = (h + step.degrees) % 360

        elif t == StepType.ROTATE_CCW:
            h = (h - step.degrees) % 360

        elif t == StepType.MOVE_SPIN:
            # 平移按 direction_deg 移动，朝向按 spin_degrees 旋转（同时进行）
            rad = math.radians(h + step.direction_deg)
            x += d * math.sin(rad)
            y += d * math.cos(rad)
            if step.spin_degrees >= 0:
                h = (h + step.spin_degrees) % 360
            else:
                h = (h + step.spin_degrees) % 360

        elif t == StepType.OPEN_MOVE:
            # 开环：只知道方向角和功率，无 ticks，按方向预览位移（距离用 distance_cm 字段）
            rad = math.radians(h + step.direction_deg)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.OPEN_ROT:
            # 开环自旋：只旋转，无位移。正 omega_pct → CCW → heading 减小（与 ROTATE_CCW 一致）
            # 无编码器，无法知道实际角度，这里用 omega_pct*duration/1000 做粗略预览
            # 预览仅供方向参考，不精确
            rough_deg = abs(step.omega_pct) * step.open_duration_ms / 1000.0 * 1.5
            if step.omega_pct > 0:
                h = (h - rough_deg) % 360   # CCW
            elif step.omega_pct < 0:
                h = (h + rough_deg) % 360   # CW

        elif t == StepType.OPEN_FORWARD:
            rad = math.radians(h)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.OPEN_BACKWARD:
            rad = math.radians(h)
            x -= d * math.sin(rad)
            y -= d * math.cos(rad)

        elif t == StepType.OPEN_LEFT:
            rad = math.radians(h - 90)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        elif t == StepType.OPEN_RIGHT:
            rad = math.radians(h + 90)
            x += d * math.sin(rad)
            y += d * math.cos(rad)

        # SERVO / DELAY / MOTOR / DC_MOTOR: 不改变位姿

        elif t == StepType.FACE_CHANGE:
            # 换面 = 底盘顺时针旋转120°
            h = (h + 120) % 360

        poses.append(Pose(x, y, h))

    return poses


# =========================================================
#  estimate_total_distance — 总行程估算
# =========================================================
def estimate_total_distance(steps: list[Step]) -> float:
    """仅统计线性运动步骤的距离之和（cm），不包括旋转、延时等。"""
    return sum(s.distance_cm for s in steps if s.step_type in LINEAR_TYPES)


def estimate_total_rotation(steps: list[Step]) -> float:
    """统计所有旋转步骤的角度之和（deg）。"""
    return sum(
        s.degrees for s in steps
        if s.step_type in (StepType.ROTATE_CW, StepType.ROTATE_CCW)
    )


# =========================================================
#  smoke test（直接运行此文件时执行）
# =========================================================
if __name__ == "__main__":
    from .models import CalibrationState, Pose, Step, StepType

    # 无反转标定（全 False），验证纯符号逻辑
    cal = CalibrationState(encoder_ticks_per_cm=9.476, turn_ticks_per_deg=4.738,
                           invert_vx=False, invert_vy=False, invert_omega=False)

    # 1. 前进 50 cm 功率 70%（无反转 → vy=+70）
    s1 = Step(step_type=StepType.FORWARD, distance_cm=50.0, power_pct=70)
    r1 = step_to_tuples(s1, cal)
    assert r1 == [("enc_move", 474, 70, 0)], f"FAIL FORWARD: {r1}"

    # 2. 后退 30 cm 功率 60%（无反转 → vy=-60）
    s2 = Step(step_type=StepType.BACKWARD, distance_cm=30.0, power_pct=60)
    r2 = step_to_tuples(s2, cal)
    assert r2 == [("enc_move", 284, -60, 0)], f"FAIL BACKWARD: {r2}"

    # 3. 顺时针 90° 功率 50%（无反转 → omega=-50）
    s3 = Step(step_type=StepType.ROTATE_CW, degrees=90.0, omega_pct=50)
    r3 = step_to_tuples(s3, cal)
    assert r3 == [("enc_rot", 426, 0, -50)], f"FAIL ROTATE_CW: {r3}"

    # 4. 逆时针 45°（无反转 → omega=+40）
    s4 = Step(step_type=StepType.ROTATE_CCW, degrees=45.0, omega_pct=40)
    r4 = step_to_tuples(s4, cal)
    assert r4 == [("enc_rot", 213, 0, 40)], f"FAIL ROTATE_CCW: {r4}"

    # 4b. 反转测试（invert_vy=True → 前进 vy 变负，invert_omega=True → 顺时针 omega 变正）
    cal_inv = CalibrationState(encoder_ticks_per_cm=9.476, turn_ticks_per_deg=4.738,
                               invert_vx=False, invert_vy=True, invert_omega=True)
    ri_fwd = step_to_tuples(Step(step_type=StepType.FORWARD, distance_cm=50.0, power_pct=70), cal_inv)
    assert ri_fwd == [("enc_move", 474, -70, 0)], f"FAIL FORWARD invert_vy: {ri_fwd}"
    ri_cw  = step_to_tuples(Step(step_type=StepType.ROTATE_CW, degrees=90.0, omega_pct=50), cal_inv)
    assert ri_cw  == [("enc_rot", 426, 0, 50)], f"FAIL ROTATE_CW invert_omega: {ri_cw}"

    # 5. 延时 0.5 s
    s5 = Step(step_type=StepType.DELAY, duration_s=0.5)
    r5 = step_to_tuples(s5, cal)
    assert r5 == [("delay", 0.5)], f"FAIL DELAY: {r5}"

    # 6. 夹爪（close → grip action=1，open → grip action=0）
    s6_close = Step(step_type=StepType.SERVO, servo_id="SV1", grip_action="close")
    r6_close = step_to_tuples(s6_close, cal)
    assert r6_close == [("grip", "SV1", 1)], f"FAIL SERVO close: {r6_close}"
    s6_open = Step(step_type=StepType.SERVO, servo_id="SV2", grip_action="open")
    r6_open = step_to_tuples(s6_open, cal)
    assert r6_open == [("grip", "SV2", 0)], f"FAIL SERVO open: {r6_open}"

    # 7. build_sequence 末尾有 enc_stop
    seq = build_sequence([s1, s3], cal)
    assert seq[-1] == ("enc_stop", 0, 0, 0), f"FAIL enc_stop: {seq[-1]}"
    assert len(seq) == 3, f"FAIL len: {seq}"

    # 8. 死算：前进 100cm 四次 + 顺时针 90° 三次 = 回到原点
    turn = Step(step_type=StepType.ROTATE_CW, degrees=90.0, omega_pct=50)
    fwd  = Step(step_type=StepType.FORWARD, distance_cm=100.0, power_pct=70)
    sq_steps = [fwd, turn, fwd, turn, fwd, turn, fwd]
    poses = dead_reckoning_poses(sq_steps, Pose(0, 0, 0))
    last = poses[-1]
    assert abs(last.x) < 0.001 and abs(last.y) < 0.001, \
        f"FAIL square dead-reckoning: end=({last.x:.4f}, {last.y:.4f})"

    # 9. estimate_total_distance
    dist = estimate_total_distance([s1, s2, s3])
    assert abs(dist - 80.0) < 0.001, f"FAIL distance: {dist}"

    print("All kinematics smoke tests passed.")
