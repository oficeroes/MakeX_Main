"""
轨迹 → AUTO_SEQUENCE 转换  (v5: 闭环编码器模式)

目的
====
把平滑后的点列（cm 坐标，+Y 朝前 / +X 朝右）变成机器人可执行的
AUTO_SEQUENCE 元组列表。

v5 新增：编码器闭环模式（encoder_based=True）
  每段输出 ('enc_move', target_ticks, Vy_power, 0) 而非时间步。
  机器人用 PID 追踪编码器目标，从根源消除"一卡一卡"问题。

v5.1 修复："一格一格走"问题
  - 添加 merge_enc_sequence() 合并连续同向小段
  - 添加 MIN_ENC_STEP_TICKS 最小步长阈值
  - 合并策略：同向（Vy 同号）且功率偏差 ≤30% 的相邻步合并为一步

曲率自适应速度
==============
  转弯处速度降至 90%，直线处提升至 105%。
  速度由 BASE_SPEED_CM_PER_SEC 控制，用户在 GUI 调节。

两种模式
========
  MODE_TRANSLATION：纯平移，编码器追踪直走距离
  MODE_HEADING：车头跟随，每段先自旋再直走

AUTO_SEQUENCE 格式（v5，encoder_based=True）
===========================================
  ('enc_move', ticks, Vy_power, 0)     — 直走 ticks 个编码单位
  ('enc_rot',  ticks, 0, omega_power)  — 自旋 ticks 个编码单位
  ('enc_stop', 0, 0, 0)                — 停止缓冲
  其中 ticks = 距离(cm) × ENCODER_TICKS_PER_CM

AUTO_SEQUENCE 格式（v4 兼容，encoder_based=False）
=================================================
  (duration, Vx, Vy, omega)             — 时间驱动步
"""
import math
from .config import (
    MODE_TRANSLATION,
    MODE_HEADING,
    ROTATION_DEADBAND_DEG,
    MERGE_ANGLE_DEG,
    MERGE_MAX_DURATION,
    STOP_BUFFER,
    POWER_MIN,
    POWER_MAX,
    ENCODER_TICKS_PER_CM,
    CURVE_SLOWDOWN_FACTOR,
    STRAIGHT_BOOST_FACTOR,
    CURVATURE_THRESHOLD,
    BASE_SPEED_CM_PER_SEC,
    BASE_ROT_DEG_PER_SEC,
    MIN_ENC_STEP_TICKS,
    ENC_MERGE_POWER_RATIO,
    DEFAULT_TRAJ_SAMPLE_DT,
)


def _clamp_power(p):
    return max(POWER_MIN, min(POWER_MAX, p))


def _wrap_deg(angle_deg):
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def _scale_vector_to_power(vx, vy, max_power):
    mag = math.hypot(vx, vy)
    if mag > max_power and mag > 1e-9:
        s = max_power / mag
        return vx * s, vy * s, max_power
    return vx, vy, mag


# ================================================================
#  曲率计算 & 自适应速度
# ================================================================

def compute_curvature(points):
    """Menger 曲率：用三点外接圆半径的倒数估算每点曲率 (1/cm)。

    首尾点用相邻点外推。曲率越大 = 转弯越急。
    """
    n = len(points)
    if n < 3:
        return [0.0] * n

    curv = []
    for i in range(n):
        if i == 0:
            p0, p1, p2 = points[0], points[0], points[1]
        elif i == n - 1:
            p0, p1, p2 = points[-2], points[-1], points[-1]
        else:
            p0, p1, p2 = points[i - 1], points[i], points[i + 1]

        # 三角形边长
        a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])

        # 海伦公式求面积
        s = (a + b + c) / 2.0
        area_sq = max(0, s * (s - a) * (s - b) * (s - c))
        area = math.sqrt(area_sq)

        # 曲率 = 4*面积 / (a*b*c)，即 1/R
        denom = a * b * c
        if denom < 1e-12 or area < 1e-12:
            curv.append(0.0)
        else:
            curv.append(4.0 * area / denom)

    return curv


def assign_adaptive_speed(points, base_power, base_speed_cm_s,
                          slow_factor=CURVE_SLOWDOWN_FACTOR,
                          boost_factor=STRAIGHT_BOOST_FACTOR,
                          curve_thresh=CURVATURE_THRESHOLD):
    """根据曲率为每个线段分配功率和速度。

    返回: list of (vx_power, vy_power, speed_cm_s, factor)
      factor = 慢/快倍率（0.9 转弯, 1.05 直线）
    """
    if len(points) < 2:
        return []

    curv = compute_curvature(points)
    power = _clamp_power(base_power)
    result = []

    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue

        # 线段中点曲率
        mid_curv = (curv[i] + curv[i + 1]) / 2.0

        if mid_curv > curve_thresh:
            factor = slow_factor       # 转弯 → 减速
        else:
            factor = boost_factor      # 直线 → 加速

        speed = base_speed_cm_s * factor
        ux, uy = dx / length, dy / length
        seg_power = power * factor     # 功率也等比缩放
        vx = seg_power * ux
        vy = seg_power * uy
        vx, vy, _ = _scale_vector_to_power(vx, vy, max(seg_power, power))

        result.append((vx, vy, speed, length, factor))

    return result


# ================================================================
#  编码器闭环序列生成（v5 — 基于 cm 距离的速度曲线）
# ================================================================

def _path_total_length(points):
    """计算路径总长度（cm）"""
    total = 0.0
    for i in range(len(points) - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        total += math.hypot(dx, dy)
    return total


def _speed_at_position(dist_from_start, total_len,
                       accel_cm, decel_cm, min_spd, max_spd,
                       curve_factor=1.0):
    """根据沿路径的距离位置，返回该点的目标速度（cm/s）。

    加速段（0 → accel_cm）：   min_spd → max_spd  线性递增
    匀速段（accel_cm → total_len - decel_cm）：max_spd × curve_factor
    减速段（total_len - decel_cm → total_len）：max_spd → min_spd  线性递减
    accel_cm/decel_cm 可为 0。0 表示关闭对应阶段，直接匀速。
    路径太短时自动缩小加减速距离，避免除零。
    """
    total_len = max(float(total_len), 0.0)
    accel_cm = max(float(accel_cm or 0.0), 0.0)
    decel_cm = max(float(decel_cm or 0.0), 0.0)
    dist_from_start = max(0.0, min(float(dist_from_start), total_len))

    if total_len <= 1e-9:
        return max_spd * curve_factor

    # If both ramp phases are disabled, this is the basic constant-speed mode.
    if accel_cm <= 1e-9 and decel_cm <= 1e-9:
        return max_spd * curve_factor

    # Safety clamp for direct callers: never let accel+decel exceed the path.
    ramp_total = accel_cm + decel_cm
    if ramp_total > total_len and ramp_total > 1e-9:
        scale = total_len / ramp_total
        accel_cm *= scale
        decel_cm *= scale

    if accel_cm > 1e-9 and dist_from_start < accel_cm:
        return min_spd + (max_spd - min_spd) * (dist_from_start / accel_cm)

    decel_start = total_len - decel_cm
    if decel_cm > 1e-9 and dist_from_start >= decel_start:
        remaining = max(0.0, total_len - dist_from_start)
        return min_spd + (max_spd - min_spd) * (remaining / decel_cm)

    return max_spd * curve_factor


def build_encoder_sequence(points, base_power, base_speed_cm_s=BASE_SPEED_CM_PER_SEC,
                           ticks_per_cm=ENCODER_TICKS_PER_CM,
                           invert_x=False, invert_y=False,
                           accel_cm=None, decel_cm=None,
                           min_speed=None, max_speed=None,
                           curve_adaptive=True):
    """纯平移 + 编码器闭环 + 基于距离的速度曲线。

    加速段→匀速段→减速段。加速/减速距离按路径总长自适应：
      - 长路径：使用用户设定的 accel_cm / decel_cm
      - 短路径：自动缩小，确保至少 30% 路径为匀速段

    前后重量补偿在机器人端 mecánum_kinematics() 中处理（FRONT_BACK_COMPENSATION），
    GUI 只负责把补偿值写入机器人源文件，不参与序列生成。
    """
    if len(points) < 2:
        return []

    # 参数默认值（用户设定）
    if accel_cm is None:
        from .config import PROFILE_ACCEL_CM as accel_cm
    if decel_cm is None:
        from .config import PROFILE_DECEL_CM as decel_cm
    if min_speed is None:
        from .config import PROFILE_MIN_SPEED as min_speed
    if max_speed is None:
        from .config import PROFILE_MAX_SPEED as max_speed

    total = _path_total_length(points)
    if total < 0.1:
        return [('enc_stop', 0, 0, 0)]

    # ★ 自适应：确保加速+减速不超过路径的 70%，预留 ≥30% 匀速段
    _MAX_ACCEL_RATIO = 0.35  # 加速最多占 35%
    _MAX_DECEL_RATIO = 0.35  # 减速最多占 35%
    accel_cm = min(accel_cm, total * _MAX_ACCEL_RATIO)
    decel_cm = min(decel_cm, total * _MAX_DECEL_RATIO)

    power = _clamp_power(base_power)
    curv = compute_curvature(points) if curve_adaptive else [0.0] * len(points)

    seq = []
    cum_dist = 0.0  # 沿路径累计距离

    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            continue

        # 线段中点位置 + 曲率因子
        mid_dist = cum_dist + seg_len / 2.0
        mid_curv = (curv[i] + curv[i + 1]) / 2.0
        if curve_adaptive:
            cf = CURVE_SLOWDOWN_FACTOR if mid_curv > CURVATURE_THRESHOLD else STRAIGHT_BOOST_FACTOR
        else:
            cf = 1.0

        # 该段平均速度（取中点速度作为代表）
        avg_speed = _speed_at_position(mid_dist, total, accel_cm, decel_cm,
                                       min_speed, max_speed, cf)
        avg_speed = max(min_speed, avg_speed)

        # 方向 + 功率
        ux, uy = dx / seg_len, dy / seg_len
        seg_power = power * (avg_speed / max_speed)  # 功率按速度比例缩放
        seg_power = _clamp_power(seg_power)
        vx = seg_power * ux
        vy = seg_power * uy
        vx, vy, _ = _scale_vector_to_power(vx, vy, seg_power)

        if invert_x: vx = -vx
        if invert_y: vy = -vy

        ticks = int(round(seg_len * ticks_per_cm))
        vy_power = int(round(vy))
        vx_power = int(round(vx))
        if ticks > 0:
            seq.append(('enc_move', ticks, vy_power, vx_power))

        cum_dist += seg_len

    seq.append(('enc_stop', 0, 0, 0))
    return seq


def build_travel_spin_sequence(points, base_power, omega_power,
                               total_spin_deg,
                               steps=None,
                               base_speed_cm_s=None,
                               ticks_per_cm=ENCODER_TICKS_PER_CM,
                               invert_x=False, invert_y=False,
                               spin_precision_cm=8.0):
    """方案 C：行进中自旋 — 边走边转的微步闭环序列。

    把整段路径按弧长切成 N 个微步，每步走一点 + 转总角的 1/N。
    每步输出 ('enc_moverot', ticks, vy_power, vx_power, omega_power, rot_ticks)，
    由机器人端 _enc_moverot_start 把平移与旋转的各轮目标角度叠加、一次闭环到位。

    total_spin_deg:    从起点到终点全程要旋转的总角度（+顺时针）
    steps:             微步数（None = 按 spin_precision_cm 自适应）
    spin_precision_cm: 自旋精度—每个微步的目标弧长（cm），越小越细越准、步数越多

    转角编码与 enc_rot 一致：rot_ticks = deg * ticks_per_cm * 0.5，符号由 omega 携带。
    平移方向做车身坐标补偿：随着车头累计转过的角度，把场地系的路径切向旋转回车身系。
    """
    if len(points) < 2:
        return []
    if base_speed_cm_s is None:
        from .config import PROFILE_MAX_SPEED as base_speed_cm_s

    total = _path_total_length(points)
    if total < 0.1:
        return [('enc_stop', 0, 0, 0)]

    # 微步数：按精度自适应，至少 1 步
    try:
        spin_precision_cm = float(spin_precision_cm)
    except (TypeError, ValueError):
        spin_precision_cm = 8.0
    if spin_precision_cm < 0.5:
        spin_precision_cm = 0.5
    if steps is None:
        steps = max(1, int(math.ceil(total / spin_precision_cm)))
    else:
        steps = max(1, int(steps))

    cum = _arc_length_table(points)
    power = _clamp_power(base_power)
    ow = int(round(abs(omega_power)))
    seg_arc = total / steps
    deg_per_step = float(total_spin_deg) / steps

    # 转角符号：+顺时针 → omega 为负（与固件 mecanum/omni 旋转约定一致，与 enc_rot 同）
    rot_ticks_per_step = int(round(abs(deg_per_step) * ticks_per_cm * 0.5))
    spin_sign = 1 if deg_per_step >= 0 else -1

    seq = []
    heading_off = 0.0   # 车身累计转过的角度（度）
    for k in range(steps):
        s0 = seg_arc * k
        s1 = seg_arc * (k + 1)
        x0, y0 = _point_at_s(points, cum, s0)
        x1, y1 = _point_at_s(points, cum, s1)
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)

        # 车身坐标补偿：把场地系方向旋转 -heading_off（车头已转过的角度）到车身系
        mid_heading = heading_off + deg_per_step * 0.5
        ang = math.radians(mid_heading)
        cos_a = math.cos(ang)
        sin_a = math.sin(ang)
        if seg_len > 1e-6:
            ux, uy = dx / seg_len, dy / seg_len
        else:
            ux, uy = 0.0, 0.0
        # 旋转到车身系（逆旋转 mid_heading）
        bx = ux * cos_a + uy * sin_a
        by = -ux * sin_a + uy * cos_a

        vx = power * bx
        vy = power * by
        vx, vy, _ = _scale_vector_to_power(vx, vy, power)
        if invert_x:
            vx = -vx
        if invert_y:
            vy = -vy

        ticks = int(round(seg_len * ticks_per_cm))
        vy_power = int(round(vy))
        vx_power = int(round(vx))

        step_omega = spin_sign * ow if rot_ticks_per_step > 0 else 0
        if ticks > 0 or rot_ticks_per_step > 0:
            seq.append(('enc_moverot', ticks, vy_power, vx_power,
                        int(step_omega), rot_ticks_per_step))
        heading_off += deg_per_step

    seq.append(('enc_stop', 0, 0, 0))
    return seq


# ================================================================
#  连续速度轨迹（v6 — 曲线不再拆成位置步）
# ================================================================

def _arc_length_table(points):
    cum = [0.0]
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
    return cum


def _point_at_s(points, cum, s):
    if not points:
        return (0.0, 0.0)
    if s <= 0:
        return points[0]
    if s >= cum[-1]:
        return points[-1]

    # 线性扫描足够快：GUI 轨迹通常只有几十到几百点。
    for i in range(len(cum) - 1):
        s0 = cum[i]
        s1 = cum[i + 1]
        if s <= s1:
            span = max(s1 - s0, 1e-9)
            t = (s - s0) / span
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
    return points[-1]


def _power_for_speed(speed_cm_s, cm_per_s_at_p50, power_cap):
    power = 50.0 * speed_cm_s / max(cm_per_s_at_p50, 0.1)
    power = min(power, float(power_cap), float(POWER_MAX))
    if 0 < abs(power) < POWER_MIN:
        power = POWER_MIN
    return power


def _axis_powers_for_path_speed(ux, uy, target_speed_cm_s,
                                forward_cm_s_at_p50,
                                strafe_cm_s_at_p50,
                                power_cap):
    """Convert desired path velocity to Vx/Vy powers with anisotropic calibration.

    Mecanum sideways speed is usually lower than forward speed.  This keeps the
    physical velocity vector aligned with the path by using separate P50 speed
    constants for X and Y, then scales both axes down if the requested power is
    above the allowed cap.
    """
    forward_cm_s_at_p50 = max(float(forward_cm_s_at_p50), 0.1)
    strafe_cm_s_at_p50 = max(float(strafe_cm_s_at_p50), 0.1)
    target_speed_cm_s = max(float(target_speed_cm_s), 0.1)

    vx_power = 50.0 * target_speed_cm_s * ux / strafe_cm_s_at_p50
    vy_power = 50.0 * target_speed_cm_s * uy / forward_cm_s_at_p50

    mag = math.hypot(vx_power, vy_power)
    cap = max(1.0, min(float(power_cap), float(POWER_MAX)))
    if mag > cap:
        scale = cap / mag
        vx_power *= scale
        vy_power *= scale

    actual_vx = vx_power * strafe_cm_s_at_p50 / 50.0
    actual_vy = vy_power * forward_cm_s_at_p50 / 50.0
    path_speed = max(0.1, actual_vx * ux + actual_vy * uy)
    return vx_power, vy_power, path_speed


def _merge_traj_v_sequence(seq, angle_tol_deg=2.0, max_dur=0.10):
    """合并方向几乎一致的 traj_v，减少 AUTO_SEQUENCE 长度但不制造停顿。"""
    if len(seq) < 2:
        return list(seq)

    merged = [list(seq[0])]
    for step in seq[1:]:
        if not (isinstance(step[0], str) and step[0] == 'traj_v'):
            merged.append(list(step))
            continue

        prev = merged[-1]
        if not (isinstance(prev[0], str) and prev[0] == 'traj_v'):
            merged.append(list(step))
            continue

        _, pd, pvx, pvy, pw = prev
        _, cd, cvx, cvy, cw = step
        pmag = math.hypot(pvx, pvy)
        cmag = math.hypot(cvx, cvy)
        can_merge = False
        if pw == cw and pd + cd <= max_dur and pmag > 1 and cmag > 1:
            dot = (pvx * cvx + pvy * cvy) / (pmag * cmag)
            dot = max(-1.0, min(1.0, dot))
            angle = math.degrees(math.acos(dot))
            can_merge = angle <= angle_tol_deg

        if can_merge:
            total = pd + cd
            prev[1] = round(total, 3)
            prev[2] = int(round((pvx * pd + cvx * cd) / total))
            prev[3] = int(round((pvy * pd + cvy * cd) / total))
        else:
            merged.append(list(step))

    return [tuple(s) for s in merged]


def build_velocity_sequence(points, cm_per_s_at_p50, auto_power,
                            target_speed_cm_s=BASE_SPEED_CM_PER_SEC,
                            strafe_cm_s_at_p50=None,
                            sample_dt=DEFAULT_TRAJ_SAMPLE_DT,
                            invert_x=False, invert_y=False,
                            accel_cm=0.0, decel_cm=0.0,
                            min_speed=30.0, curve_adaptive=False,
                            add_stop=True):
    """生成连续速度播放序列。

    输出 ('traj_v', duration_s, Vx_power, Vy_power, omega_power)。
    它不调用编码电机位置模式，因此曲线不会在每个短线段末尾减速停顿。
    距离精度暂时依赖 P50 速度标定和功率-速度近似线性关系。
    """
    if len(points) < 2:
        return []

    total = _path_total_length(points)
    if total < 0.1:
        return [('traj_stop', 0, 0, 0)]

    power_cap = _clamp_power(auto_power)
    if strafe_cm_s_at_p50 is None:
        strafe_cm_s_at_p50 = cm_per_s_at_p50
    physical_cap = max(cm_per_s_at_p50, strafe_cm_s_at_p50) * power_cap / 50.0
    max_speed = max(1.0, min(float(target_speed_cm_s), physical_cap))
    min_speed = min(float(min_speed), max_speed)

    accel_cm = max(0.0, min(float(accel_cm or 0.0), total * 0.35))
    decel_cm = max(0.0, min(float(decel_cm or 0.0), total * 0.35))
    sample_dt = max(0.02, float(sample_dt or DEFAULT_TRAJ_SAMPLE_DT))

    cum = _arc_length_table(points)
    curv = compute_curvature(points) if curve_adaptive else [0.0] * len(points)

    seq = []
    s = 0.0
    while s < total - 1e-6:
        mid_s = min(total, s + max_speed * sample_dt * 0.5)
        # 找中点附近曲率，简单取最近路径点。
        mid_idx = 0
        for i in range(len(cum)):
            if cum[i] >= mid_s:
                mid_idx = i
                break
        curve_factor = 1.0
        if curve_adaptive and curv[mid_idx] > CURVATURE_THRESHOLD:
            curve_factor = CURVE_SLOWDOWN_FACTOR

        speed = _speed_at_position(mid_s, total, accel_cm, decel_cm,
                                   min_speed, max_speed, curve_factor)
        speed = max(1.0, min(speed, physical_cap))
        chunk_len = min(speed * sample_dt, total - s)
        if chunk_len < 0.2 and seq:
            # 把极短尾段并入上一段，避免 0.00/0.01 秒采样。
            last = list(seq[-1])
            last[1] = round(float(last[1]) + chunk_len / max(speed, 1.0), 3)
            seq[-1] = tuple(last)
            break

        p0 = _point_at_s(points, cum, s)
        p1 = _point_at_s(points, cum, s + chunk_len)
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            s += chunk_len
            continue

        ux = dx / seg_len
        uy = dy / seg_len
        vx, vy, path_speed = _axis_powers_for_path_speed(
            ux, uy, speed, cm_per_s_at_p50, strafe_cm_s_at_p50, power_cap)
        vx, vy, _ = _scale_vector_to_power(vx, vy, power_cap)
        if invert_x:
            vx = -vx
        if invert_y:
            vy = -vy

        duration = max(0.02, chunk_len / max(path_speed, 1.0))
        seq.append(('traj_v', round(duration, 3),
                    int(round(vx)), int(round(vy)), 0))
        s += chunk_len

    seq = _merge_traj_v_sequence(seq)
    if add_stop:
        seq.append(('traj_stop', 0, 0, 0))
    return seq


def _enc_step_tag(step):
    return step[0] if step and isinstance(step[0], str) else None


def _merge_weighted_enc_step(a, b):
    tag = _enc_step_tag(a)
    total_ticks = int(a[1] + b[1])
    if total_ticks <= 0:
        return tuple(a)

    if tag == 'enc_move':
        vy = int(round((a[2] * a[1] + b[2] * b[1]) / total_ticks))
        vx = int(round((a[3] * a[1] + b[3] * b[1]) / total_ticks))
        return ('enc_move', total_ticks, vy, vx)

    power = int(round((a[3] * a[1] + b[3] * b[1]) / total_ticks))
    return ('enc_rot', total_ticks, 0, power)


def _can_merge_enc_steps(a, b, power_ratio):
    tag = _enc_step_tag(a)
    if tag != _enc_step_tag(b) or tag not in ('enc_move', 'enc_rot'):
        return False

    if tag == 'enc_rot':
        ap = a[3]
        bp = b[3]
        if abs(ap) < 1 or abs(bp) < 1:
            return False
        same_sign = (ap >= 0 and bp >= 0) or (ap <= 0 and bp <= 0)
        if not same_sign:
            return False
        ratio = abs(bp - ap) / max(abs(ap), abs(bp), 1)
        return ratio <= power_ratio

    # enc_move is a 2D translation vector: (Vy, Vx).  The old merge logic
    # only kept Vy, so pure strafe became ('enc_move', ticks, 0, 0).
    a_vy, a_vx = float(a[2]), float(a[3])
    b_vy, b_vx = float(b[2]), float(b[3])
    a_mag = math.hypot(a_vx, a_vy)
    b_mag = math.hypot(b_vx, b_vy)
    if a_mag < 1 or b_mag < 1:
        return False

    ratio = abs(b_mag - a_mag) / max(a_mag, b_mag, 1.0)
    if ratio > power_ratio:
        return False

    dot = (a_vx * b_vx + a_vy * b_vy) / (a_mag * b_mag)
    dot = max(-1.0, min(1.0, dot))
    angle = math.degrees(math.acos(dot))
    return angle <= MERGE_ANGLE_DEG


def _merge_enc_sequence_v2(seq, min_ticks, power_ratio):
    if len(seq) < 2:
        return list(seq)

    filtered = []
    pending_small = None

    for raw_step in seq:
        step = tuple(raw_step)
        tag = _enc_step_tag(step)

        if tag in ('enc_move', 'enc_rot'):
            if pending_small is not None:
                if _can_merge_enc_steps(pending_small, step, power_ratio):
                    step = _merge_weighted_enc_step(pending_small, step)
                    pending_small = None
                else:
                    filtered.append(pending_small)
                    pending_small = None

            if step[1] < min_ticks:
                pending_small = step
            else:
                filtered.append(step)
            continue

        if pending_small is not None:
            filtered.append(pending_small)
            pending_small = None
        filtered.append(step)

    if pending_small is not None:
        filtered.append(pending_small)

    if len(filtered) < 2:
        return filtered

    merged = [filtered[0]]
    for step in filtered[1:]:
        prev = merged[-1]
        if _can_merge_enc_steps(prev, step, power_ratio):
            merged[-1] = _merge_weighted_enc_step(prev, step)
        else:
            merged.append(step)

    return merged


def build_encoder_sequence_heading(points, base_power, omega_power,
                                   base_speed_cm_s=BASE_SPEED_CM_PER_SEC,
                                   base_rot_deg_s=BASE_ROT_DEG_PER_SEC,
                                   ticks_per_cm=ENCODER_TICKS_PER_CM,
                                   invert_x=False, invert_y=False,
                                   accel_cm=None, decel_cm=None,
                                   min_speed=None, max_speed=None):
    """车头跟随 + 编码器闭环 + 距离速度曲线。"""
    if len(points) < 2:
        return []

    if accel_cm is None:
        from .config import PROFILE_ACCEL_CM as accel_cm
    if decel_cm is None:
        from .config import PROFILE_DECEL_CM as decel_cm
    if min_speed is None:
        from .config import PROFILE_MIN_SPEED as min_speed
    if max_speed is None:
        from .config import PROFILE_MAX_SPEED as max_speed

    total = _path_total_length(points)
    if total < 0.1:
        return [('enc_stop', 0, 0, 0)]

    # ★ 自适应：确保加速+减速不超过路径的 70%，预留 ≥30% 匀速段
    _MAX_ACCEL_RATIO = 0.35
    _MAX_DECEL_RATIO = 0.35
    accel_cm = min(accel_cm, total * _MAX_ACCEL_RATIO)
    decel_cm = min(decel_cm, total * _MAX_DECEL_RATIO)

    power = _clamp_power(base_power)
    seq = []
    heading_deg = 90.0
    cum_dist = 0.0

    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            continue

        mid_dist = cum_dist + seg_len / 2.0
        avg_speed = _speed_at_position(mid_dist, total, accel_cm, decel_cm,
                                       min_speed, max_speed)

        target_deg = math.degrees(math.atan2(dy, dx))
        delta_deg = _wrap_deg(target_deg - heading_deg)

        # 旋转步
        if abs(delta_deg) >= ROTATION_DEADBAND_DEG:
            rot_ticks = int(round(abs(delta_deg) * ticks_per_cm * 0.5))
            rot_sign = -1 if delta_deg > 0 else 1
            rot_w = int(round(rot_sign * omega_power))
            if rot_ticks > 0:
                seq.append(('enc_rot', rot_ticks, 0, rot_w))
            heading_deg = target_deg

        # 直走步
        fwd_ticks = int(round(seg_len * ticks_per_cm))
        seg_power = power * (avg_speed / max_speed) if max_speed > 0 else power
        seg_power = _clamp_power(seg_power)
        vy_power = int(round(seg_power))
        if invert_y:
            vy_power = -vy_power
        if fwd_ticks > 0:
            seq.append(('enc_move', fwd_ticks, vy_power, 0))

        cum_dist += seg_len

    seq.append(('enc_stop', 0, 0, 0))
    return seq


# ================================================================
#  编码器序列合并（v5.1 — 解决"一格一格走"问题）
# ================================================================

def merge_enc_sequence(seq, min_ticks=MIN_ENC_STEP_TICKS,
                       power_ratio=ENC_MERGE_POWER_RATIO):
    """合并连续同向的 enc_move / enc_rot 步，消除步间停顿。

    策略
    ====
    1. 过滤掉 ticks < min_ticks 的过小步（累积到下一步）。
    2. 合并连续的同类型同向步：
       - enc_move 之间：Vy 同号 且 功率偏差 ≤ power_ratio
       - enc_rot 之间：omega 同号 且 功率偏差 ≤ power_ratio
    3. 合并时 ticks 累加，功率取 ticks 加权平均。
    4. enc_stop / 非编码器步 作为分隔符，不跨类型合并。

    参数
    ====
    seq         : [(tag, ticks, p1, p2), ...]
    min_ticks   : 小于此值的步被合并/丢弃
    power_ratio : 功率偏差容许比例（0.3 = ±30%）

    返回合并后的序列
    """
    return _merge_enc_sequence_v2(seq, min_ticks, power_ratio)

    if len(seq) < 2:
        return list(seq)

    # 第一步：过滤过小的步，累积 ticks 到后续同向步
    filtered = []
    carry_ticks = 0
    carry_power = 0
    carry_tag = None

    for step in seq:
        tag = step[0] if isinstance(step[0], str) else None

        if tag in ('enc_move', 'enc_rot'):
            ticks = step[1]
            power_val = step[2] if tag == 'enc_move' else step[3]

            if carry_tag == tag and (carry_ticks > 0):
                # 有累积的 ticks，加到当前步
                total_ticks = carry_ticks + ticks
                # 功率加权平均
                merged_power = int(round(
                    (carry_power * carry_ticks + power_val * ticks) / max(total_ticks, 1)
                ))
                if tag == 'enc_move':
                    filtered.append(('enc_move', total_ticks, merged_power, 0))
                else:
                    filtered.append(('enc_rot', total_ticks, 0, merged_power))
                carry_ticks = 0
                carry_power = 0
                carry_tag = None
            elif ticks < min_ticks:
                # 太小，累积起来
                carry_ticks += ticks
                carry_power = power_val
                carry_tag = tag
            else:
                filtered.append(step)
        else:
            # 非编码器步：先把累积的 tick 追加（如果有的话）
            if carry_tag and carry_ticks > 0:
                if carry_tag == 'enc_move':
                    filtered.append(('enc_move', carry_ticks, int(round(carry_power)), 0))
                else:
                    filtered.append(('enc_rot', carry_ticks, 0, int(round(carry_power))))
                carry_ticks = 0
                carry_power = 0
                carry_tag = None
            filtered.append(step)

    # 末尾残余
    if carry_tag and carry_ticks > 0:
        if carry_tag == 'enc_move':
            filtered.append(('enc_move', carry_ticks, int(round(carry_power)), 0))
        else:
            filtered.append(('enc_rot', carry_ticks, 0, int(round(carry_power))))

    if len(filtered) < 2:
        return filtered

    # 第二步：合并连续同向同类型步
    merged = [list(filtered[0])]
    for step in filtered[1:]:
        prev = merged[-1]
        prev_tag = prev[0] if isinstance(prev[0], str) else None
        cur_tag = step[0] if isinstance(step[0], str) else None

        can_merge = False
        if prev_tag == cur_tag and prev_tag in ('enc_move', 'enc_rot'):
            if prev_tag == 'enc_move':
                prev_power = prev[2]
                cur_power = step[2]
                # 同号（同方向）
                same_sign = (prev_power >= 0 and cur_power >= 0) or (prev_power <= 0 and cur_power <= 0)
                if same_sign and max(abs(prev_power), 1) > 0:
                    ratio = abs(cur_power - prev_power) / max(abs(prev_power), abs(cur_power), 1)
                    if ratio <= power_ratio:
                        can_merge = True
            else:  # enc_rot
                prev_power = prev[3]
                cur_power = step[3]
                same_sign = (prev_power >= 0 and cur_power >= 0) or (prev_power <= 0 and cur_power <= 0)
                if same_sign and max(abs(prev_power), 1) > 0:
                    ratio = abs(cur_power - prev_power) / max(abs(prev_power), abs(cur_power), 1)
                    if ratio <= power_ratio:
                        can_merge = True

        if can_merge:
            # 合并：ticks 累加，功率加权平均
            total_ticks = prev[1] + step[1]
            if prev_tag == 'enc_move':
                w_avg = int(round((prev[2] * prev[1] + step[2] * step[1]) / max(total_ticks, 1)))
                merged[-1] = ['enc_move', total_ticks, w_avg, 0]
            else:
                w_avg = int(round((prev[3] * prev[1] + step[3] * step[1]) / max(total_ticks, 1)))
                merged[-1] = ['enc_rot', total_ticks, 0, w_avg]
        else:
            merged.append(list(step))

    return [tuple(s) for s in merged]


# ================================================================
#  旧版时间驱动管线（v4 兼容，保留）
# ================================================================


def _clamp_power(p):
    return max(POWER_MIN, min(POWER_MAX, p))


def _wrap_deg(angle_deg):
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def _scale_vector_to_power(vx, vy, max_power):
    """限制 √(vx²+vy²) ≤ max_power，避免 omni_kinematics 内部再缩放"""
    mag = math.hypot(vx, vy)
    if mag > max_power and mag > 1e-9:
        s = max_power / mag
        return vx * s, vy * s, max_power
    return vx, vy, mag


def path_to_sequence_translation(points, cm_per_s_at_p50, auto_power):
    """模式 A：纯平移。每段独立输出 (dt, Vx, Vy, 0)"""
    if len(points) < 2:
        return []

    power = _clamp_power(auto_power)
    speed_at_full = cm_per_s_at_p50 * power / 50.0

    seq = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        ux, uy = dx / length, dy / length
        vx = power * ux
        vy = power * uy
        vx, vy, _ = _scale_vector_to_power(vx, vy, power)
        duration = length / speed_at_full
        seq.append((duration, vx, vy, 0))
    return seq


def path_to_sequence_heading(points, cm_per_s_at_p50, deg_per_s_at_omega50,
                              auto_power, omega_power):
    """模式 B：车头跟随路径。每段先转再走"""
    if len(points) < 2:
        return []

    auto_power = _clamp_power(auto_power)
    omega_power = _clamp_power(omega_power)
    fwd_speed = cm_per_s_at_p50 * auto_power / 50.0
    rot_speed = deg_per_s_at_omega50 * omega_power / 50.0

    seq = []
    heading_deg = 90.0  # 初始车头：+Y 方向 = 数学坐标 90°

    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue

        target_deg = math.degrees(math.atan2(dy, dx))
        delta_deg = _wrap_deg(target_deg - heading_deg)

        if abs(delta_deg) >= ROTATION_DEADBAND_DEG:
            rot_duration = abs(delta_deg) / rot_speed
            # 机器人 +omega = 顺时针；数学角度增 = 逆时针，故符号取反
            rot_sign = -1 if delta_deg > 0 else 1
            seq.append((rot_duration, 0, 0, rot_sign * omega_power))
            heading_deg = target_deg

        seq.append((length / fwd_speed, 0, auto_power, 0))

    return seq


def merge_collinear(seq, angle_tol_deg=MERGE_ANGLE_DEG, max_dur=MERGE_MAX_DURATION):
    """合并方向接近且 omega 相同的连续元组"""
    if len(seq) < 2:
        return list(seq)

    out = [list(seq[0])]
    for dur, vx, vy, w in seq[1:]:
        prev_dur, pvx, pvy, pw = out[-1]
        same_omega = (w == pw)
        prev_mag = math.hypot(pvx, pvy)
        cur_mag = math.hypot(vx, vy)
        can_merge = False
        if same_omega and prev_dur + dur <= max_dur:
            if prev_mag < 1e-6 and cur_mag < 1e-6:
                can_merge = True
            elif prev_mag > 1e-6 and cur_mag > 1e-6:
                dot = (pvx * vx + pvy * vy) / (prev_mag * cur_mag)
                dot = max(-1.0, min(1.0, dot))
                angle = math.degrees(math.acos(dot))
                if angle <= angle_tol_deg:
                    can_merge = True
        if can_merge:
            total = prev_dur + dur
            avg_vx = (pvx * prev_dur + vx * dur) / total
            avg_vy = (pvy * prev_dur + vy * dur) / total
            out[-1] = [total, avg_vx, avg_vy, w]
        else:
            out.append([dur, vx, vy, w])
    return [tuple(s) for s in out]


def round_sequence(seq):
    """格式化：时间保留 2 位小数，速度取整"""
    out = []
    for dur, vx, vy, w in seq:
        out.append((
            round(dur, 2),
            int(round(vx)),
            int(round(vy)),
            int(round(w)),
        ))
    return out


def apply_velocity_ramp(seq, ramp_time=0.25, ramp_steps=5, min_ratio=0.15):
    """给每个路段插入缓升缓降子步骤，防止高功率起步打滑。

    原理
    ====
    把每个 (dur, Vx, Vy, omega) 拆成三个阶段：
      - 加速段（ramp_time 秒，功率从 min_ratio → 1.0 线性递增）
      - 匀速段（剩余时间，满功率）
      - 减速段（ramp_time 秒，功率从 1.0 → min_ratio 线性递减）

    短路段（dur ≤ 2×ramp_time）：走三角形斜坡，先升后降。

    参数
    ====
    ramp_time : 加速/减速各占多少秒
    ramp_steps: 加速/减速各切成几个子步骤（越大越平滑，导出步骤越多）
    min_ratio : 起始/结束功率比例（0.15=15%，克服静摩擦的最低值）

    返回
    ====
    list[tuple]  新的步骤列表，每个子步骤时长均匀、功率线性渐变。
    """
    if not seq or ramp_time <= 0 or ramp_steps <= 0:
        return list(seq)

    result = []
    for dur, vx, vy, w in seq:
        # 跳过停止/空步骤
        if dur <= 0.02 or (vx == 0 and vy == 0 and w == 0):
            result.append((dur, vx, vy, w))
            continue

        total_ramp = ramp_time * 2  # 加速 + 减速总时长

        if dur <= total_ramp:
            # ── 短路段：三角形斜坡 ──
            half = dur / 2.0
            for i in range(ramp_steps * 2):
                t = (i + 0.5) / (ramp_steps * 2)  # 子步骤中点位置 0→1
                if t < 0.5:
                    ratio = min_ratio + (1.0 - min_ratio) * (t / 0.5)
                else:
                    ratio = 1.0 - (1.0 - min_ratio) * ((t - 0.5) / 0.5)
                sub_dur = dur / (ramp_steps * 2)
                result.append((
                    max(0.01, sub_dur),
                    vx * ratio, vy * ratio, w * ratio
                ))
        else:
            # ── 长路段：加速 → 匀速 → 减速 ──
            sub_dur = ramp_time / ramp_steps
            # 加速段
            for i in range(ramp_steps):
                ratio = min_ratio + (1.0 - min_ratio) * (i + 0.5) / ramp_steps
                result.append((max(0.01, sub_dur), vx * ratio, vy * ratio, w * ratio))
            # 匀速段
            cruise = dur - total_ramp
            if cruise > 0.01:
                result.append((cruise, vx, vy, w))
            # 减速段
            for i in range(ramp_steps):
                ratio = min_ratio + (1.0 - min_ratio) * (ramp_steps - i - 0.5) / ramp_steps
                result.append((max(0.01, sub_dur), vx * ratio, vy * ratio, w * ratio))

    return result


def build_sequence(points, mode, cm_per_s_at_p50, deg_per_s_at_omega50,
                   auto_power, omega_power, invert_x=False, invert_y=False,
                   add_stop=True,
                   ramp_enabled=True, ramp_time=0.25, ramp_steps=5,
                   ramp_min_ratio=0.15):
    """完整管线：模式分发 → 合并 → 反转 → 缓升缓降 → 格式化 → 停止缓冲

    add_stop=False 时不追加 STOP_BUFFER，用于多段合并时每段单独生成。

    反转一个轴等于镜像，所以 invert_x 或 invert_y 任一为真都要把 omega 同时取反，
    否则模式 B 车头会反转。两者同时为真则等于绕原点 180° 旋转，omega 不变。

    缓升缓降在每个路段的首尾插入功率渐变子步骤，防止高功率起步打滑。
    ramp_enabled=False 或 ramp_time=0 时跳过。

    前后重量补偿在机器人端 mecánum_kinematics() 中处理，不参与序列生成。
    """
    if mode == MODE_TRANSLATION:
        raw = path_to_sequence_translation(points, cm_per_s_at_p50, auto_power)
    elif mode == MODE_HEADING:
        raw = path_to_sequence_heading(
            points, cm_per_s_at_p50, deg_per_s_at_omega50, auto_power, omega_power
        )
    else:
        raise ValueError("unknown mode: %s" % mode)

    merged = merge_collinear(raw)

    # 1) 反转 X / Y（机器人装反时使用）。镜像一次 → omega 取反
    omega_flip = -1 if (invert_x ^ invert_y) else 1
    if invert_x or invert_y or omega_flip == -1:
        flipped = []
        for dur, vx, vy, w in merged:
            if invert_x:
                vx = -vx
            if invert_y:
                vy = -vy
            w = w * omega_flip
            flipped.append((dur, vx, vy, w))
        merged = flipped

    # 2) 缓升缓降：每段首尾插入功率渐变子步骤
    if ramp_enabled and ramp_time > 0:
        merged = apply_velocity_ramp(merged, ramp_time, ramp_steps, ramp_min_ratio)

    rounded = round_sequence(merged)
    if add_stop:
        rounded.append(STOP_BUFFER)
    return rounded
