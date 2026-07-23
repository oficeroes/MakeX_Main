"""step_planner.models — 所有纯数据类型，无 Qt 依赖

StepType  — 步骤类型枚举
Step      — 单条步骤数据（dataclass）
CalibrationState — 标定常数（dataclass）
Pose      — 死算中机器人位姿
"""

from __future__ import annotations

import copy
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Optional


# =========================================================
#  StepType — 步骤类型
# =========================================================
class StepType(str, Enum):
    """每个值和 JSON 里存的字符串一一对应（str 子类方便比较）"""
    FORWARD      = "forward"       # ➡ 前进
    BACKWARD     = "backward"      # ⬅ 后退
    STRAFE_LEFT  = "strafe_left"   # ← 左移
    STRAFE_RIGHT = "strafe_right"  # → 右移
    MOVE         = "move"          # ↗ 任向平移（机器人坐标系，方向角 + 距离）
    ROTATE_CW    = "rotate_cw"     # ↻ 顺时针自转
    ROTATE_CCW   = "rotate_ccw"    # ↺ 逆时针自转
    SERVO        = "servo"         # ⚙ 舵机
    DELAY        = "delay"         # ⏱ 延时
    MOTOR        = "motor"         # M 编码电机持续功率
    DC_MOTOR     = "dc_motor"      # D 直流电机功率
    MOVE_SPIN    = "move_spin"     # ↗↻ 平移自旋（闭环同步，不停车）
    OPEN_MOVE    = "open_move"     # ⚡ 开环移动（纯时间，无编码器）
    OPEN_ROT     = "open_rot"      # ⚡↻ 开环自旋（纯时间，无编码器，最快旋转）
    OPEN_FORWARD = "open_forward"  # ⚡➡ 开环前进
    OPEN_BACKWARD= "open_backward" # ⚡⬅ 开环后退
    OPEN_LEFT    = "open_left"     # ⚡← 开环左移
    OPEN_RIGHT   = "open_right"    # ⚡→ 开环右移
    LIFT         = "lift"          # ↕ 升降臂档位
    FACE_CHANGE  = "face_change"   # ⟳ 换面（底盘顺时针旋转120°）


# 中文显示名（按 StepType 顺序）
STEP_TYPE_LABELS: dict[StepType, str] = {
    StepType.FORWARD:      "➡ 前进",
    StepType.BACKWARD:     "⬅ 后退",
    StepType.STRAFE_LEFT:  "← 左移",
    StepType.STRAFE_RIGHT: "→ 右移",
    StepType.MOVE:         "↗ 任向平移",
    StepType.ROTATE_CW:    "↻ 顺时针",
    StepType.ROTATE_CCW:   "↺ 逆时针",
    StepType.SERVO:        "✊ 夹爪",
    StepType.DELAY:        "⏱ 延时",
    StepType.MOTOR:        "M 电机",
    StepType.DC_MOTOR:     "D 直流",
    StepType.MOVE_SPIN:    "↗↻ 平移自旋",
    StepType.OPEN_MOVE:    "⚡ 开环移动",
    StepType.OPEN_ROT:     "⚡↻ 开环自旋",
    StepType.OPEN_FORWARD: "⚡➡ 开环前进",
    StepType.OPEN_BACKWARD:"⚡⬅ 开环后退",
    StepType.OPEN_LEFT:    "⚡← 开环左移",
    StepType.OPEN_RIGHT:   "⚡→ 开环右移",
    StepType.LIFT:         "↕ 升降臂",
    StepType.FACE_CHANGE:  "⟳ 换面",
}

# 会改变机器人位置/朝向的步骤（用于死算轨迹 + 总行程统计）
MOTION_TYPES = frozenset({
    StepType.FORWARD, StepType.BACKWARD,
    StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT,
    StepType.MOVE, StepType.ROTATE_CW, StepType.ROTATE_CCW,
    StepType.MOVE_SPIN, StepType.OPEN_MOVE, StepType.OPEN_ROT,
    StepType.OPEN_FORWARD, StepType.OPEN_BACKWARD,
    StepType.OPEN_LEFT, StepType.OPEN_RIGHT,
})

# 用于直线距离统计（不含旋转）
LINEAR_TYPES = frozenset({
    StepType.FORWARD, StepType.BACKWARD,
    StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT,
    StepType.MOVE, StepType.MOVE_SPIN, StepType.OPEN_MOVE,
})


# =========================================================
#  Step — 单条步骤
# =========================================================
# 每个字段对每种类型的意义：
#
#  FORWARD / BACKWARD:    distance_cm, power_pct
#  STRAFE_LEFT/RIGHT:     distance_cm, power_pct
#  MOVE:                  direction_deg (机器人坐标系, 0°=前), distance_cm, power_pct
#  ROTATE_CW / ROTATE_CCW: degrees (>0), omega_pct
#  SERVO:                 servo_id, servo_angle, servo_speed, servo_wait_ms
#  DELAY:                 duration_s
#  MOTOR:                 motor_id, motor_power (−100~100)
#  DC_MOTOR:              dc_port, dc_power (−100~100)
#
# 不相关的字段被保留（不会丢失），但不影响导出。

@dataclass
class Step:
    # ── 总是存在 ────────────────────────────────────────────────────
    step_type: StepType = StepType.FORWARD

    # ── 线性运动 (FORWARD / BACKWARD / STRAFE_* / MOVE) ────────────
    distance_cm: float   = 50.0      # 距离，cm，始终 ≥ 0
    direction_deg: float = 0.0       # MOVE 专用：0°=前进方向，90°=右（机器人坐标系）
    power_pct: int       = 70        # 功率 0–100

    # ── 自转 (ROTATE_CW / ROTATE_CCW) ──────────────────────────────
    degrees: float = 90.0            # 角度，始终 > 0；方向由 step_type 决定
    omega_pct: int = 50              # 转速功率 0–100

    # ── 舵机/夹爪 (SERVO) ───────────────────────────────────────────
    servo_id: str        = "SV1"     # "SV1" | "SV2" | "SV3"
    servo_angle: float   = 0.0       # 目标角度（deg，仅旧 servo tag 使用）
    servo_speed: int     = 50        # 速度 0–100（仅旧 servo tag 使用）
    servo_wait_ms: int   = 500       # 发送命令后等待时长（ms，仅旧 servo tag 使用）
    grip_action: str     = "close"   # "close"=自适应夹紧，"open"=松开归零
    grip_target_angle: int = 0       # >0 时用 move_to 到指定角度，0=自适应

    # ── 延时 (DELAY) ────────────────────────────────────────────────
    duration_s: float = 0.5          # 秒

    # ── 编码电机直接功率 (MOTOR) ────────────────────────────────────
    motor_id: str    = "M1"          # "M1" .. "M5"
    motor_power: int = 0             # −100 ~ 100

    # ── 直流电机 (DC_MOTOR) ─────────────────────────────────────────
    dc_port: str    = "DC1"          # "DC1" | "DC2"
    dc_power: int   = 0              # −100 ~ 100

    # ── 加减速（线性运动专用）──────────────────────────────────────
    accel_cm: float    = 0.0    # 加速段距离 cm，0 = 不拆段
    decel_cm: float    = 0.0    # 减速段距离 cm
    min_power_pct: int = 30     # 加减速段最低功率 0-100

    # ── 平移自旋 (MOVE_SPIN) ─────────────────────────────────────────
    spin_degrees: float = 90.0
    # omega_pct 复用上面已有字段

    # ── 旋转加减速 (ROTATE_CW / ROTATE_CCW) ─────────────────────────
    rot_accel_deg: float = 0.0      # 加速段角度 °，0=不加速
    rot_decel_deg: float = 0.0      # 减速段角度 °，0=不减速
    rot_min_power_pct: int = 18     # 加减速最低功率 0-100

    # ── 开环移动 (OPEN_MOVE) ─────────────────────────────────────────
    # 直接 set_power，靠 open_duration_ms 计时停止，无编码器
    open_duration_ms: int   = 500   # 持续时长（ms）
    open_omega_pct:   int   = 0     # 同时自旋功率（−100~100，0=不转）
    # direction_deg / power_pct 复用上面已有字段

    # ── 开环自旋 (OPEN_ROT) ──────────────────────────────────────────
    # 只旋转，靠 open_duration_ms 计时停止，无编码器，omega_pct 控制方向与速度
    # 正 omega_pct = CCW；负 omega_pct = CW（与闭环 enc_rot 约定一致）
    # open_duration_ms 复用上面已有字段
    # omega_pct 复用上面已有字段

    # ── 升降臂 (LIFT) ────────────────────────────────────────────────
    # lift_arm: 'arm1'=一号臂(M5), 'arm2'=二号臂(M4), 'both'=双臂
    lift_arm: str       = "both"
    # lift_mode: 'gear'=按档位, 'angle'=自定义角度
    lift_mode: str      = "gear"
    lift_gear_index: int = 1       # gear模式：档位索引（对应 DEBUG_LIFT_GEARS）
    lift_angle_deg: float = 315.0  # angle模式：直接指定目标角度（°）
    lift_async: bool    = False    # True=后台运行，启动后立即执行下一步

    # ── 元数据 ──────────────────────────────────────────────────────
    comment: str = ""

    # ──────────────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        d = asdict(self)
        d["step_type"] = self.step_type.value   # 存为字符串
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Step":
        d = dict(d)
        d["step_type"] = StepType(d["step_type"])
        known = {f for f in cls.__dataclass_fields__}  # type: ignore[attr-defined]
        return cls(**{k: v for k, v in d.items() if k in known})

    def copy(self) -> "Step":
        return copy.deepcopy(self)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Step):
            return NotImplemented
        return self.to_dict() == other.to_dict()


# =========================================================
#  CalibrationState — 标定常数
# =========================================================
@dataclass
class CalibrationState:
    """两套独立标定值，每个底盘各自保存一份

    encoder_ticks_per_cm:
        线性移动：电机角度（°/cm）
        通过「直走1秒→量距离→录增量」获得
        即：sum(M_delta) / 4 / measured_cm（四轮）
        或：sum(M_delta) / 3 / measured_cm（三轮）

    turn_ticks_per_deg:
        原地旋转：电机角度（°/体转°）
        通过「发 enc_rot 1000 ticks → 量实转角 192° → 1000/192 = 5.208」获得
        完全独立于 encoder_ticks_per_cm，消除 0.5 魔法常数
    """
    encoder_ticks_per_cm: float = 9.476    # 默认值：三轮全向实测
    turn_ticks_per_deg:   float = 4.738    # 默认值：encoder_ticks_per_cm * 0.5 的初始估计

    # 轴反转（True = 对应轴取反，导出时写入固件 INVERT_* 常量）
    invert_vx:    bool = False
    invert_vy:    bool = True    # 四轮默认 True（车尾为前进方向）
    invert_omega: bool = True    # 四轮默认 True

    # 斜线旋转偏量修正（写入固件 FRONT_BACK_COMPENSATION）
    # 正值 = 压制后轮功率，负值 = 压制前轮功率
    front_back_compensation: float = 0.0

    def to_dict(self) -> dict:
        return {
            "encoder_ticks_per_cm":    self.encoder_ticks_per_cm,
            "turn_ticks_per_deg":      self.turn_ticks_per_deg,
            "invert_vx":               self.invert_vx,
            "invert_vy":               self.invert_vy,
            "invert_omega":            self.invert_omega,
            "front_back_compensation": self.front_back_compensation,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CalibrationState":
        return cls(
            encoder_ticks_per_cm=float(d.get("encoder_ticks_per_cm", 9.476)),
            turn_ticks_per_deg=float(d.get("turn_ticks_per_deg", 4.738)),
            invert_vx=bool(d.get("invert_vx", False)),
            invert_vy=bool(d.get("invert_vy", True)),
            invert_omega=bool(d.get("invert_omega", True)),
            front_back_compensation=float(d.get("front_back_compensation", 0.0)),
        )

    def __eq__(self, other) -> bool:
        if not isinstance(other, CalibrationState):
            return NotImplemented
        return self.to_dict() == other.to_dict()


# =========================================================
#  Pose — 机器人位姿（用于死算）
# =========================================================
@dataclass
class Pose:
    """场地坐标系中的机器人位姿

    坐标系：
      +X = 场地右方
      +Y = 场地上方（前方）
      heading = 0° 时机器人朝向 +Y（上）
      heading 顺时针增加（与 ROTATE_CW 一致）

    场地原点在左下角。
    """
    x: float       = 50.0    # cm
    y: float       = 50.0    # cm
    heading: float = 0.0     # degrees, 0 = 朝上(+Y), CW = 正

    def to_dict(self) -> dict:
        return {"x_cm": self.x, "y_cm": self.y, "heading_deg": self.heading}

    @classmethod
    def from_dict(cls, d: dict) -> "Pose":
        return cls(
            x=float(d.get("x_cm", 50.0)),
            y=float(d.get("y_cm", 50.0)),
            heading=float(d.get("heading_deg", 0.0)),
        )
