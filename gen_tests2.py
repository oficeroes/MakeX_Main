import os, ast

base = open('D:/UserData/Desktop/MakeX_Main/mecanum_forward.py', encoding='utf-8').read()
out_dir = 'D:/UserData/Desktop/MakeX_Main/tests'
os.makedirs(out_dir, exist_ok=True)

def patch_led(content, tag):
    return content.replace("__led.show('Main')", "__led.show('" + tag + "')", 1)

def check(name, content):
    path = out_dir + '/' + name + '.py'
    open(path, 'w', encoding='utf-8').write(content)
    try:
        ast.parse(content)
        print("  " + name + ": AST OK  " + str(len(content.encode('utf-8'))) + " bytes")
    except Exception as e:
        print("  " + name + ": AST FAIL " + str(e))


# ── T6: 函数体内用分号 ──────────────────────────────────────────
inject_t6 = """

def _test_semi_func():
    a = 1; b = 2; c = 3
    x = a + b; y = x * c; z = y - 1
    return z

"""
check('t6_semicolons', patch_led(base + inject_t6, 'T6'))


# ── T7: if_ 作变量名 ────────────────────────────────────────────
inject_t7 = """

def _test_ifvar_func(x, mn):
    if_ = (mn / 100.0) if x > 0 else 1.0
    factor = if_ * x
    return factor

"""
check('t7_if_varname', patch_led(base + inject_t7, 'T7'))


# ── T8: 分号 + if_ 同时存在（精确重现之前崩溃的压缩写法） ─────
inject_t8 = """

def _test_both(x, mn):
    if_ = (mn / 100.0) if x > 0 else 1.0
    a = 1; b = 2; c = 3
    factor = if_ * (a + b + c)
    return int(factor * x)

"""
check('t8_semi_plus_if', patch_led(base + inject_t8, 'T8'))


# ── T9: 干净版 per-frame enc_moverot（无分号/无if_，新算法） ───
globals_t9 = """
_mr_start_angles   = [0, 0, 0]
_mr_total_deg_s    = 1.0
_mr_wheel_pwr_s    = [0.0, 0.0, 0.0]
_mr_accel_ticks_s  = 0
_mr_decel_ticks_s  = 0
_mr_total_ticks_s  = 1
_mr_min_pwr_s      = 18
"""

new_start = """def _enc_moverot_start(total_ticks, vy_power, vx_power, omega_power, rot_ticks,
                       accel_ticks=0, decel_ticks=0, min_pwr=18):
    global _mr_move_targets, _mr_rot_targets, _mr_move_done, _mr_rot_done
    global _enc_step_start_time
    global _mr_start_angles, _mr_total_deg_s, _mr_wheel_pwr_s
    global _mr_accel_ticks_s, _mr_decel_ticks_s, _mr_total_ticks_s, _mr_min_pwr_s
    motors = _get_motors()
    starts = _read_motor_angles(motors)
    move_deg = [0.0, 0.0, 0.0]
    wheel_powers = [0.0, 0.0, 0.0]
    if abs(vy_power) >= 1 or abs(vx_power) >= 1:
        dist_cm = abs(total_ticks) / ENCODER_TICKS_PER_CM
        total_d = dist_cm * _DEG_PER_CM
        direction = 1 if total_ticks >= 0 else -1
        M1p, M2p, M3p = omni_kinematics(vx_power * direction, vy_power * direction, 0)
        wheel_powers = [M1p, M2p, M3p]
        max_wp = max(abs(M1p), abs(M2p), abs(M3p))
        if max_wp >= 1:
            scales = [abs(p) / max_wp for p in wheel_powers]
            signs = [1 if p >= 0 else -1 for p in wheel_powers]
            move_deg = [signs[i] * total_d * scales[i] for i in range(3)]
    rot_deg = [0.0, 0.0, 0.0]
    if abs(omega_power) >= 1:
        rot_dist_cm = abs(rot_ticks) / TURN_TICKS_PER_DEG
        rot_dir = 1 if rot_ticks >= 0 else -1
        R1, R2, R3 = omni_kinematics(0, 0, abs(omega_power) * rot_dir)
        rsigns = [1 if p >= 0 else -1 for p in (R1, R2, R3)]
        rot_deg = [rsigns[i] * rot_dist_cm for i in range(3)]
    _mr_move_targets = [int(starts[i] + move_deg[i]) for i in range(3)]
    _mr_rot_targets = [int(starts[i] + rot_deg[i]) for i in range(3)]
    _mr_move_done = (max(abs(move_deg[0]), abs(move_deg[1]), abs(move_deg[2])) < 0.5)
    _mr_rot_done = (max(abs(rot_deg[0]), abs(rot_deg[1]), abs(rot_deg[2])) < 0.5)
    _mr_start_angles = [int(v) for v in starts]
    _mr_total_deg_s = max(abs(move_deg[0]), abs(move_deg[1]), abs(move_deg[2]), 1.0)
    _mr_wheel_pwr_s = list(wheel_powers)
    _mr_accel_ticks_s = max(0, accel_ticks)
    _mr_decel_ticks_s = max(0, decel_ticks)
    _mr_total_ticks_s = max(abs(total_ticks), 1)
    _mr_min_pwr_s = max(1, min_pwr)
    init_f = _mr_min_pwr_s / 100.0
    for i in range(3):
        combined = move_deg[i] + rot_deg[i]
        if abs(combined) < 0.5:
            continue
        try:
            p = int(wheel_powers[i] * init_f)
            if p == 0 and abs(wheel_powers[i]) >= 1:
                p = 1 if wheel_powers[i] > 0 else -1
            motors[i].set_power(p)
        except Exception:
            pass
    _enc_step_start_time = novapi.timer()
"""

new_tick = """def _enc_moverot_tick(stop_motors=True):
    motors = _get_motors()
    if novapi.timer() - _enc_step_start_time > _ENC_STEP_TIMEOUT:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True
    max_moved = 0.0
    for i in range(3):
        try:
            cur = motors[i].get_value('angle')
            if cur is None:
                cur = _mr_start_angles[i]
            moved = abs(cur - _mr_start_angles[i])
            if moved > max_moved:
                max_moved = moved
        except Exception:
            pass
    progress = max_moved / _mr_total_deg_s
    if progress >= 1.0:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True
    ticks_done = int(progress * _mr_total_ticks_s)
    total = _mr_total_ticks_s
    at = _mr_accel_ticks_s
    dt = _mr_decel_ticks_s
    min_f = _mr_min_pwr_s / 100.0
    if at > 0 and ticks_done < at:
        factor = min_f + (1.0 - min_f) * _smoothstep(ticks_done / at)
    elif dt > 0 and ticks_done > (total - dt):
        tt = (total - ticks_done) / dt
        factor = max((1.0 - min_f) * _smoothstep(tt), 0.05)
    else:
        factor = 1.0
    all_done = True
    for i in range(3):
        try:
            cur = motors[i].get_value('angle')
            if cur is None:
                cur = 0
            move_chk = _mr_move_done or abs(_mr_move_targets[i] - cur) <= PROFILE_DEADBAND_DEG
            rot_chk = _mr_rot_done or abs(_mr_rot_targets[i] - cur) <= PROFILE_DEADBAND_DEG
            if not (move_chk and rot_chk):
                all_done = False
                break
        except Exception:
            pass
    if all_done:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True
    for i in range(3):
        try:
            cur = motors[i].get_value('angle')
            if cur is None:
                cur = 0
            move_ok = _mr_move_done or abs(_mr_move_targets[i] - cur) <= PROFILE_DEADBAND_DEG
            rot_ok = _mr_rot_done or abs(_mr_rot_targets[i] - cur) <= PROFILE_DEADBAND_DEG
            if move_ok and rot_ok:
                motors[i].set_power(0)
            else:
                p = int(_mr_wheel_pwr_s[i] * factor)
                if p == 0 and abs(_mr_wheel_pwr_s[i]) >= 1:
                    p = 1 if _mr_wheel_pwr_s[i] > 0 else -1
                motors[i].set_power(p)
        except Exception:
            pass
    return False
"""

old_start = """def _enc_moverot_start(total_ticks, vy_power, vx_power, omega_power, rot_ticks,
                       accel_ticks=0, decel_ticks=0, min_pwr=18):
    global _mr_move_targets, _mr_rot_targets, _mr_move_done, _mr_rot_done
    global _enc_step_start_time
    motors = _get_motors()
    starts = _read_motor_angles(motors)
    move_deg = [0.0, 0.0, 0.0]
    if abs(vy_power) >= 1 or abs(vx_power) >= 1:
        dist_cm = abs(total_ticks) / ENCODER_TICKS_PER_CM
        total_d = dist_cm * _DEG_PER_CM
        direction = 1 if total_ticks >= 0 else -1
        M1p, M2p, M3p = omni_kinematics(vx_power * direction, vy_power * direction, 0)
        wheel_powers = [M1p, M2p, M3p]
        max_wp = max(abs(M1p), abs(M2p), abs(M3p))
        if max_wp >= 1:
            scales = [abs(p) / max_wp for p in wheel_powers]
            signs  = [1 if p >= 0 else -1 for p in wheel_powers]
            move_deg = [signs[i] * total_d * scales[i] for i in range(3)]
    rot_deg = [0.0, 0.0, 0.0]
    if abs(omega_power) >= 1:
        rot_dist_cm = abs(rot_ticks) / TURN_TICKS_PER_DEG
        rot_dir = 1 if rot_ticks >= 0 else -1
        R1, R2, R3 = omni_kinematics(0, 0, abs(omega_power) * rot_dir)
        rsigns = [1 if p >= 0 else -1 for p in (R1, R2, R3)]
        rot_deg = [rsigns[i] * rot_dist_cm for i in range(3)]
    _mr_move_targets = [int(starts[i] + move_deg[i]) for i in range(3)]
    _mr_rot_targets  = [int(starts[i] + rot_deg[i])  for i in range(3)]
    _mr_move_done    = (max(abs(move_deg[0]), abs(move_deg[1]), abs(move_deg[2])) < 0.5)
    _mr_rot_done     = (max(abs(rot_deg[0]),  abs(rot_deg[1]),  abs(rot_deg[2]))  < 0.5)
    max_pwr = max(abs(vy_power), abs(vx_power), abs(omega_power))
    motor_rpm = max(30, int(max_pwr * _RPM_PER_POWER))
    for i in range(3):
        combined = move_deg[i] + rot_deg[i]
        try:
            if abs(combined) > 0.5:
                try:
                    motors[i].move(int(combined), motor_rpm)
                except Exception:
                    motors[i].move(int(combined), 50)
        except Exception:
            pass
    _enc_step_start_time = novapi.timer()"""

old_tick = """def _enc_moverot_tick(stop_motors=True):
    motors = _get_motors()
    if novapi.timer() - _enc_step_start_time > _ENC_STEP_TIMEOUT:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True
    all_done = True
    for i in range(3):
        try:
            cur = motors[i].get_value('angle')
            if cur is None:
                cur = 0
            if not _mr_move_done and abs(_mr_move_targets[i] - cur) > PROFILE_DEADBAND_DEG:
                all_done = False
                break
            if not _mr_rot_done and abs(_mr_rot_targets[i] - cur) > PROFILE_DEADBAND_DEG:
                all_done = False
                break
        except Exception:
            pass
    if all_done:
        if stop_motors:
            for m in motors:
                m.set_power(0)
        return True
    return False"""

base_t9 = base.replace(old_start, new_start.strip())
base_t9 = base_t9.replace(old_tick, new_tick.strip())
base_t9 = base_t9.replace(
    '_mr_move_done    = False\n_mr_rot_done     = False',
    '_mr_move_done    = False\n_mr_rot_done     = False\n' + globals_t9
)
check('t9_clean_per_frame', patch_led(base_t9, 'T9'))


# ── T10: 每帧多跑50次 smoothstep 运算 ─────────────────────────
inject_t10 = """

def _stress_calc():
    total = 0.0
    for idx in range(50):
        tt = idx / 50.0
        v = tt * tt * (3.0 - 2.0 * tt)
        total = total + v
    return total

"""
# 在 remote 主循环最后一个 time.sleep 前插入
import re
# 只替换最后一个 time.sleep(LOOP_DELAY)
base_t10 = base + inject_t10
idx = base_t10.rfind('    time.sleep(LOOP_DELAY)')
if idx >= 0:
    base_t10 = base_t10[:idx] + '    _stress_calc()\n    time.sleep(LOOP_DELAY)' + base_t10[idx+len('    time.sleep(LOOP_DELAY)'):]
check('t10_compute_stress', patch_led(base_t10, 'T10'))


print("\nDone. Files in:", out_dir)
