"""轨迹 → AUTO_SEQUENCE 转换

输入：平滑后的点列（cm 坐标，+Y 前 / +X 右），标定参数
输出：[(duration_sec, Vx, Vy, omega), ...]
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


def build_sequence(points, mode, cm_per_s_at_p50, deg_per_s_at_omega50,
                   auto_power, omega_power, invert_x=False, invert_y=False,
                   drift_left_omega=0, drift_right_omega=0):
    """完整管线：模式分发 → 合并 → 漂移补偿（GUI 视角）→ 反转 → 加停止缓冲 → 格式化

    补偿在反转之前做。这样无论 invert_x/y 怎么设，drift_left_omega 始终对应
    "你画图时往左的那个方向"的补偿，所见即所得。

    反转一个轴等于镜像，所以 invert_x 或 invert_y 任一为真都要把 omega 同时取反，
    否则模式 B 车头会反转、补偿 omega 也会颠倒。两者同时为真则等于绕原点 180°
    旋转，omega 不变。
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

    # 1) 左右平移漂移补偿：在 GUI 视角下，Vx<0 加左补偿，Vx>0 加右补偿
    #    强度按 |Vx|/auto_power 线性缩放（斜走时按比例补偿）
    if drift_left_omega or drift_right_omega:
        compensated = []
        for dur, vx, vy, w in merged:
            if abs(vx) > 1e-6:
                strength = min(1.0, abs(vx) / max(1, auto_power))
                if vx < 0:
                    w = w + drift_left_omega * strength
                else:
                    w = w + drift_right_omega * strength
            compensated.append((dur, vx, vy, w))
        merged = compensated

    # 2) 反转 X / Y（机器人装反时使用）。镜像一次 → omega 取反
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

    rounded = round_sequence(merged)
    rounded.append(STOP_BUFFER)
    return rounded
