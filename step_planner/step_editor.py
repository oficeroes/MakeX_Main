"""step_planner.step_editor — 右侧参数编辑面板（QStackedWidget）

当用户在步骤表格中选中某行时，此面板显示该步骤的完整参数（包括表格里列不下的字段）。
每种 StepType 对应一个 QWidget 页面，切换时用 QStackedWidget.setCurrentIndex()。

用法
====
  editor = StepEditor(parent)
  editor.load_step(step)     # 切换到对应类型的页面并填入值
  editor.step_changed        # 信号：用户修改了参数 → 返回新的 Step
  editor.get_step()          # 获取当前显示的步骤

注意
====
  这里的参数编辑与 step_table.py 里的行内控件是独立的，两者通过 main_window.py
  协调同步：
    - 表格行选中 → main_window 调用 editor.load_step(step)
    - editor.step_changed → main_window 调用 table.update_step_at(row, new_step)
  不直接互相引用，保持解耦。
"""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .models import Step, StepType, STEP_TYPE_LABELS


def _load_lift_gears() -> list[int]:
    """返回 DEBUG_LIFT_GEARS 的整数列表，解析失败返回默认值。"""
    try:
        fw = Path(__file__).parent.parent / "mecanum_forward.py"
        text = fw.read_text(encoding="utf-8")
        m = re.search(r"DEBUG_LIFT_GEARS\s*=\s*\[([^\]]+)\]", text)
        if m:
            return [int(x.strip()) for x in m.group(1).split(",") if x.strip()]
    except Exception:
        pass
    return [0, 315, 420, 600]


def _load_lift_gear_labels() -> list[str]:
    """从 mecanum_forward.py 解析 DEBUG_LIFT_GEARS / DEBUG_LIFT_NAMES，
    返回每档的标签，例如 ['L0 (0°)', 'L1 (315°)', 'L2 (420°)', 'L3 (600°)']。
    解析失败时返回默认值。
    """
    try:
        fw = Path(__file__).parent.parent / "mecanum_forward.py"
        text = fw.read_text(encoding="utf-8")
        gears_m = re.search(r"DEBUG_LIFT_GEARS\s*=\s*\[([^\]]+)\]", text)
        names_m = re.search(r"DEBUG_LIFT_NAMES\s*=\s*\[([^\]]+)\]", text)
        if not gears_m:
            return ["L0 (0°)", "L1 (315°)", "L2 (420°)", "L3 (600°)"]
        gears = [int(x.strip()) for x in gears_m.group(1).split(",") if x.strip()]
        if names_m:
            raw = [x.strip().strip("'\"") for x in names_m.group(1).split(",") if x.strip()]
            names = raw
        else:
            names = [f"L{i}" for i in range(len(gears))]
        labels = []
        for i, g in enumerate(gears):
            n = names[i] if i < len(names) else f"L{i}"
            labels.append(f"{n} ({g}°)")
        return labels
    except Exception:
        return ["L0 (0°)", "L1 (315°)", "L2 (420°)", "L3 (600°)"]


class StepEditor(QWidget):
    """右侧步骤参数面板

    布局：
      ─ 标题 QLabel（当前步骤类型 + 编号）
      ─ QStackedWidget（每种类型一个页面）
      ─ （空白弹性区域）

    当无步骤被选中时显示「未选中」页面。
    """

    step_changed = pyqtSignal(Step)   # 用户在此面板修改参数后发出

    # ── 页面索引（与 _build_pages 顺序对应）──────────────────────
    _PAGE_NONE         = 0
    _PAGE_FORWARD      = 1
    _PAGE_BACKWARD     = 2
    _PAGE_STRAFE_LEFT  = 3
    _PAGE_STRAFE_RIGHT = 4
    _PAGE_MOVE         = 5
    _PAGE_ROTATE_CW    = 6
    _PAGE_ROTATE_CCW   = 7
    _PAGE_SERVO        = 8
    _PAGE_DELAY        = 9
    _PAGE_MOTOR        = 10
    _PAGE_DC_MOTOR     = 11
    _PAGE_MOVE_SPIN    = 12
    _PAGE_OPEN_MOVE    = 13
    _PAGE_OPEN_ROT     = 14
    _PAGE_OPEN_FORWARD = 15
    _PAGE_OPEN_BACKWARD= 16
    _PAGE_OPEN_LEFT    = 17
    _PAGE_OPEN_RIGHT   = 18
    _PAGE_LIFT         = 19
    _PAGE_FACE_CHANGE  = 20

    _TYPE_TO_PAGE = {
        StepType.FORWARD:      _PAGE_FORWARD,
        StepType.BACKWARD:     _PAGE_BACKWARD,
        StepType.STRAFE_LEFT:  _PAGE_STRAFE_LEFT,
        StepType.STRAFE_RIGHT: _PAGE_STRAFE_RIGHT,
        StepType.MOVE:         _PAGE_MOVE,
        StepType.ROTATE_CW:    _PAGE_ROTATE_CW,
        StepType.ROTATE_CCW:   _PAGE_ROTATE_CCW,
        StepType.SERVO:        _PAGE_SERVO,
        StepType.DELAY:        _PAGE_DELAY,
        StepType.MOTOR:        _PAGE_MOTOR,
        StepType.DC_MOTOR:     _PAGE_DC_MOTOR,
        StepType.MOVE_SPIN:    _PAGE_MOVE_SPIN,
        StepType.OPEN_MOVE:    _PAGE_OPEN_MOVE,
        StepType.OPEN_ROT:     _PAGE_OPEN_ROT,
        StepType.OPEN_FORWARD: _PAGE_OPEN_FORWARD,
        StepType.OPEN_BACKWARD:_PAGE_OPEN_BACKWARD,
        StepType.OPEN_LEFT:    _PAGE_OPEN_LEFT,
        StepType.OPEN_RIGHT:   _PAGE_OPEN_RIGHT,
        StepType.LIFT:         _PAGE_LIFT,
        StepType.FACE_CHANGE:  _PAGE_FACE_CHANGE,
    }

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)

        self._current_step: Optional[Step] = None
        self._loading = False   # 防止 load_step 时触发 step_changed

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 标题
        self._lbl_title = QLabel("未选中步骤")
        self._lbl_title.setAlignment(Qt.AlignCenter)
        self._lbl_title.setStyleSheet(
            "font-weight: bold; font-size: 13px; color: #1B4E8A;")
        layout.addWidget(self._lbl_title)

        # 堆叠页面
        self._stack = QStackedWidget()
        layout.addWidget(self._stack)
        layout.addStretch(1)

        self._build_pages()

    # =========================================================
    #  公开 API
    # =========================================================
    def load_step(self, step: Optional[Step], row: int = -1) -> None:
        """切换到对应类型的页面并填入值。step=None 显示「未选中」页面。"""
        self._loading = True
        self._current_step = copy.deepcopy(step) if step else None

        if step is None:
            self._lbl_title.setText("未选中步骤")
            self._stack.setCurrentIndex(self._PAGE_NONE)
        else:
            row_text = f"步骤 {row + 1}：" if row >= 0 else ""
            self._lbl_title.setText(row_text + STEP_TYPE_LABELS[step.step_type])
            page_idx = self._TYPE_TO_PAGE.get(step.step_type, self._PAGE_NONE)
            self._fill_page(page_idx, step)
            self._stack.setCurrentIndex(page_idx)

        self._loading = False

    def get_step(self) -> Optional[Step]:
        return copy.deepcopy(self._current_step) if self._current_step else None

    # =========================================================
    #  内部：构建页面
    # =========================================================
    def _build_pages(self) -> None:
        """按 _PAGE_* 顺序创建各类型页面，存入 self._pages dict。"""
        self._pages: dict[int, dict] = {}

        # PAGE 0: 未选中
        none_page = QWidget()
        lbl = QLabel("选中步骤表格中的一行\n即可在此编辑参数")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("color: #888888;")
        v = QVBoxLayout(none_page)
        v.addStretch(1)
        v.addWidget(lbl)
        v.addStretch(1)
        self._stack.addWidget(none_page)

        # 线性运动类型（共用同一种页面布局）
        for page_idx, step_type in [
            (self._PAGE_FORWARD,      StepType.FORWARD),
            (self._PAGE_BACKWARD,     StepType.BACKWARD),
            (self._PAGE_STRAFE_LEFT,  StepType.STRAFE_LEFT),
            (self._PAGE_STRAFE_RIGHT, StepType.STRAFE_RIGHT),
        ]:
            page, widgets = self._make_linear_page()
            self._pages[page_idx] = widgets
            self._stack.addWidget(page)

        # MOVE
        page, widgets = self._make_move_page()
        self._pages[self._PAGE_MOVE] = widgets
        self._stack.addWidget(page)

        # ROTATE_CW / ROTATE_CCW
        for page_idx in (self._PAGE_ROTATE_CW, self._PAGE_ROTATE_CCW):
            page, widgets = self._make_rotate_page()
            self._pages[page_idx] = widgets
            self._stack.addWidget(page)

        # SERVO
        page, widgets = self._make_servo_page()
        self._pages[self._PAGE_SERVO] = widgets
        self._stack.addWidget(page)

        # DELAY
        page, widgets = self._make_delay_page()
        self._pages[self._PAGE_DELAY] = widgets
        self._stack.addWidget(page)

        # MOTOR
        page, widgets = self._make_motor_page()
        self._pages[self._PAGE_MOTOR] = widgets
        self._stack.addWidget(page)

        # DC_MOTOR
        page, widgets = self._make_dc_motor_page()
        self._pages[self._PAGE_DC_MOTOR] = widgets
        self._stack.addWidget(page)

        # MOVE_SPIN
        page, widgets = self._make_move_spin_page()
        self._pages[self._PAGE_MOVE_SPIN] = widgets
        self._stack.addWidget(page)

        # OPEN_MOVE
        page, widgets = self._make_open_move_page()
        self._pages[self._PAGE_OPEN_MOVE] = widgets
        self._stack.addWidget(page)

        # OPEN_ROT
        page, widgets = self._make_open_rot_page()
        self._pages[self._PAGE_OPEN_ROT] = widgets
        self._stack.addWidget(page)

        # OPEN_FORWARD / BACKWARD / LEFT / RIGHT — 共用同一个布局构建器
        for page_id, title, hint in [
            (self._PAGE_OPEN_FORWARD,  "开环前进", "无编码器，靠时间停止；向前直行"),
            (self._PAGE_OPEN_BACKWARD, "开环后退", "无编码器，靠时间停止；向后直行"),
            (self._PAGE_OPEN_LEFT,     "开环左移", "无编码器，靠时间停止；向左平移"),
            (self._PAGE_OPEN_RIGHT,    "开环右移", "无编码器，靠时间停止；向右平移"),
        ]:
            page, widgets = self._make_open_linear_page(title, hint)
            self._pages[page_id] = widgets
            self._stack.addWidget(page)

        # LIFT
        page, widgets = self._make_lift_page()
        self._pages[self._PAGE_LIFT] = widgets
        self._stack.addWidget(page)

        # FACE_CHANGE
        page, widgets = self._make_face_change_page()
        self._pages[self._PAGE_FACE_CHANGE] = widgets
        self._stack.addWidget(page)

    # ── 页面构建器 ────────────────────────────────────────────────
    def _make_group(self, title: str) -> tuple[QWidget, QFormLayout]:
        page = QWidget()
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(0, 0, 0, 0)
        group = QGroupBox(title)
        form = QFormLayout(group)
        form.setLabelAlignment(Qt.AlignRight)
        vbox.addWidget(group)
        vbox.addStretch(1)
        return page, form

    def _dspin(self, lo: float, hi: float, suffix: str = "",
               dec: int = 1, step: float = 1.0) -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSuffix(suffix)
        w.setSingleStep(step)
        w.setDecimals(dec)
        w.setButtonSymbols(QAbstractSpinBox.NoButtons)
        w.valueChanged.connect(self._on_any_value_changed)
        return w

    def _ispin(self, lo: int, hi: int, suffix: str = "") -> QSpinBox:
        w = QSpinBox()
        w.setRange(lo, hi)
        w.setSuffix(suffix)
        w.setButtonSymbols(QAbstractSpinBox.NoButtons)
        w.valueChanged.connect(self._on_any_value_changed)
        return w

    def _cbox(self, items: list[str]) -> QComboBox:
        w = QComboBox()
        w.addItems(items)
        w.currentTextChanged.connect(self._on_any_value_changed)
        return w

    def _make_linear_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("线性运动")
        d_spin  = self._dspin(0, 2000, " cm")
        p_spin  = self._ispin(0, 100, " %")
        ac_spin = self._dspin(0, 500, " cm", dec=1, step=1.0)
        dc_spin = self._dspin(0, 500, " cm", dec=1, step=1.0)
        mp_spin = self._ispin(0, 100, " %")
        ac_spin.setToolTip("加速段距离，0 = 不拆段")
        dc_spin.setToolTip("减速段距离，0 = 不拆段")
        mp_spin.setToolTip("加减速段功率（需小于主功率才有效）")
        form.addRow("距离:", d_spin)
        form.addRow("功率:", p_spin)
        form.addRow("加速段:", ac_spin)
        form.addRow("减速段:", dc_spin)
        form.addRow("最低功率:", mp_spin)
        return page, {
            "distance_cm":   d_spin,
            "power_pct":     p_spin,
            "accel_cm":      ac_spin,
            "decel_cm":      dc_spin,
            "min_power_pct": mp_spin,
        }

    def _make_move_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("任向平移（机器人坐标系）")
        dir_spin = self._dspin(-360, 360, "°", dec=1)
        dir_spin.setToolTip("0° = 前进方向，90° = 右，−90° = 左")
        d_spin   = self._dspin(0, 2000, " cm")
        p_spin   = self._ispin(0, 100, " %")
        ac_spin  = self._dspin(0, 500, " cm", dec=1, step=1.0)
        dc_spin  = self._dspin(0, 500, " cm", dec=1, step=1.0)
        mp_spin  = self._ispin(0, 100, " %")
        form.addRow("方向角:", dir_spin)
        form.addRow("距离:", d_spin)
        form.addRow("功率:", p_spin)
        hint = QLabel("0°=前进  90°=右  -90°=左  180°=后退")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("", hint)
        form.addRow("加速段:", ac_spin)
        form.addRow("减速段:", dc_spin)
        form.addRow("最低功率:", mp_spin)
        return page, {
            "direction_deg": dir_spin,
            "distance_cm":   d_spin,
            "power_pct":     p_spin,
            "accel_cm":      ac_spin,
            "decel_cm":      dc_spin,
            "min_power_pct": mp_spin,
        }

    def _make_rotate_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("原地旋转")
        deg_spin  = self._dspin(0, 3600, "°")
        omg_spin  = self._ispin(0, 100, " %")
        ac_spin   = self._dspin(0, 3600, "°", dec=1, step=5.0)
        dc_spin   = self._dspin(0, 3600, "°", dec=1, step=5.0)
        mp_spin   = self._ispin(0, 100, " %")
        ac_spin.setToolTip("加速段角度，0 = 不加速")
        dc_spin.setToolTip("减速段角度，0 = 不减速")
        mp_spin.setToolTip("加减速段最低功率（需小于转速功率才有效）")
        form.addRow("角度:", deg_spin)
        form.addRow("转速:", omg_spin)
        form.addRow("加速段:", ac_spin)
        form.addRow("减速段:", dc_spin)
        form.addRow("最低功率:", mp_spin)
        return page, {
            "degrees":          deg_spin,
            "omega_pct":        omg_spin,
            "rot_accel_deg":    ac_spin,
            "rot_decel_deg":    dc_spin,
            "rot_min_power_pct": mp_spin,
        }

    def _make_servo_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("自适应夹爪")
        id_cbox = self._cbox(["SV1", "SV2", "SV3"])
        action_cbox = self._cbox(["夹紧（自适应停止）", "松开（归零位）"])
        angle_spin = self._ispin(0, 360, " °")
        hint = QLabel("夹紧角度：0=自适应失速停止，>0=移动到指定角度后停止")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        hint.setWordWrap(True)
        form.addRow("舵机:", id_cbox)
        form.addRow("动作:", action_cbox)
        form.addRow("夹紧角度:", angle_spin)
        form.addRow("", hint)
        return page, {"servo_id": id_cbox, "grip_action": action_cbox, "grip_target_angle": angle_spin}

    def _make_delay_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("延时")
        dur_spin = self._dspin(0, 60, " s", dec=2, step=0.1)
        form.addRow("时长:", dur_spin)
        return page, {"duration_s": dur_spin}

    def _make_motor_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("编码电机功率")
        id_cbox  = self._cbox(["M1", "M2", "M3", "M4", "M5"])
        pwr_spin = self._ispin(-100, 100, " %")
        form.addRow("电机:", id_cbox)
        form.addRow("功率:", pwr_spin)
        hint = QLabel("正 = 正转，负 = 反转，0 = 停止")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("", hint)
        return page, {"motor_id": id_cbox, "motor_power": pwr_spin}

    def _make_dc_motor_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("直流电机")
        port_cbox = self._cbox(["DC1", "DC2"])
        pwr_spin  = self._ispin(-100, 100, " %")
        form.addRow("端口:", port_cbox)
        form.addRow("功率:", pwr_spin)
        return page, {"dc_port": port_cbox, "dc_power": pwr_spin}

    def _make_move_spin_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("平移自旋（闭环同步）")
        dir_spin  = self._dspin(-360, 360, "°", dec=1)
        dir_spin.setToolTip("0°=前进  90°=右  -90°=左  180°=后退")
        d_spin    = self._dspin(0, 2000, " cm")
        p_spin    = self._ispin(0, 100, " %")
        spin_spin = self._dspin(-3600, 3600, "°", dec=1)
        spin_spin.setToolTip("正=顺时针，负=逆时针")
        omg_spin  = self._ispin(0, 100, " %")
        form.addRow("平移方向:", dir_spin)
        form.addRow("平移距离:", d_spin)
        form.addRow("平移功率:", p_spin)
        hint = QLabel("平移与自旋同时执行，各自闭环")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("", hint)
        form.addRow("自旋角度:", spin_spin)
        form.addRow("转速功率:", omg_spin)
        return page, {
            "direction_deg": dir_spin,
            "distance_cm":   d_spin,
            "power_pct":     p_spin,
            "spin_degrees":  spin_spin,
            "omega_pct":     omg_spin,
        }

    def _make_open_move_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("开环移动（时间控制）")
        dir_spin  = self._dspin(-360, 360, "°", dec=1)
        dir_spin.setToolTip("0°=前进  90°=右  -90°=左  180°=后退")
        p_spin    = self._dspin(0, 2000, " cm")
        p_spin.setToolTip("此处含义为预览距离（估算），不影响实车行程")
        pwr_spin  = self._ispin(0, 100, " %")
        dur_spin  = self._ispin(0, 30000, " ms")
        omg_spin  = self._ispin(-100, 100, " %")
        omg_spin.setToolTip("同时自旋：正=逆时针，负=顺时针；0=不转")
        form.addRow("移动方向:", dir_spin)
        form.addRow("预览距离:", p_spin)
        form.addRow("移动功率:", pwr_spin)
        form.addRow("持续时间:", dur_spin)
        form.addRow("自旋功率:", omg_spin)
        hint = QLabel("无编码器，靠时间停止；适合快速抢球等场景")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("", hint)
        return page, {
            "direction_deg":    dir_spin,
            "distance_cm":      p_spin,
            "power_pct":        pwr_spin,
            "open_duration_ms": dur_spin,
            "open_omega_pct":   omg_spin,
        }

    def _make_open_rot_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("开环自旋（时间控制）")
        omg_spin  = self._ispin(-100, 100, " %")
        omg_spin.setToolTip("正值=逆时针（CCW），负值=顺时针（CW）")
        dur_spin  = self._ispin(0, 30000, " ms")
        hint = QLabel("无编码器，靠时间停止；比闭环旋转更快，适合快速调头")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("自旋功率:", omg_spin)
        form.addRow("持续时间:", dur_spin)
        form.addRow("", hint)
        return page, {
            "omega_pct":        omg_spin,
            "open_duration_ms": dur_spin,
        }

    def _make_open_linear_page(self, title: str, hint_text: str) -> tuple[QWidget, dict]:
        """开环直行/平移通用页面（功率 + 时长）"""
        page, form = self._make_group(title)
        pwr_spin = self._ispin(0, 100, " %")
        dur_spin = self._ispin(0, 30000, " ms")
        hint = QLabel(hint_text)
        hint.setStyleSheet("color: #888; font-size: 11px;")
        form.addRow("功率:", pwr_spin)
        form.addRow("持续时间:", dur_spin)
        form.addRow("", hint)
        return page, {
            "power_pct":        pwr_spin,
            "open_duration_ms": dur_spin,
        }

    def _make_lift_page(self) -> tuple[QWidget, dict]:
        page, form = self._make_group("升降臂控制")
        # 选臂
        arm_cbox = QComboBox()
        arm_cbox.addItems(["双臂（同步）", "一号臂 (M5)", "二号臂 (M4)"])
        arm_cbox.currentIndexChanged.connect(self._on_any_value_changed)
        form.addRow("目标臂:", arm_cbox)
        # 模式选择
        mode_cbox = QComboBox()
        mode_cbox.addItems(["按档位", "自定义角度"])
        form.addRow("目标模式:", mode_cbox)
        # 档位选择（gear模式）
        labels = _load_lift_gear_labels()
        gear_cbox = QComboBox()
        gear_cbox.addItems(labels)
        gear_cbox.currentIndexChanged.connect(self._on_any_value_changed)
        from PyQt5.QtWidgets import QStackedWidget as _SW
        # 角度输入（angle模式）
        angle_spin = self._dspin(0, 3600, "°", dec=0, step=5.0)
        angle_spin.setToolTip("直接指定目标角度（°），0 = 归零位置")
        # 用 QStackedWidget 在两种模式间切换
        stack = _SW()
        gear_w = QWidget(); gear_l = QVBoxLayout(gear_w); gear_l.setContentsMargins(0,0,0,0); gear_l.addWidget(gear_cbox)
        angle_w = QWidget(); angle_l = QVBoxLayout(angle_w); angle_l.setContentsMargins(0,0,0,0); angle_l.addWidget(angle_spin)
        stack.addWidget(gear_w)    # index 0 = gear
        stack.addWidget(angle_w)   # index 1 = angle
        form.addRow("目标:", stack)
        hint = QLabel("档位列表来自 DEBUG_LIFT_GEARS，重启规划器后生效")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        hint.setWordWrap(True)
        form.addRow("", hint)
        async_chk = QCheckBox("后台运行（启动后立即执行下一步）")
        async_chk.stateChanged.connect(self._on_any_value_changed)
        form.addRow("", async_chk)
        # 模式切换连动
        def _on_mode(idx):
            stack.setCurrentIndex(idx)
            self._on_any_value_changed()
        mode_cbox.currentIndexChanged.connect(_on_mode)
        return page, {
            "lift_arm":        arm_cbox,
            "lift_mode":       mode_cbox,
            "lift_gear_index": gear_cbox,
            "lift_angle_deg":  angle_spin,
            "_lift_stack":     stack,
            "lift_async":      async_chk,
        }

    def _make_face_change_page(self) -> tuple[QWidget, dict]:
        """换面操作页面：无参数，仅显示说明文字。"""
        page, form = self._make_group("换面")
        hint = QLabel("底盘顺时针旋转 120°，切换到下一个面。\n"
                       "死算预览：朝向 +120°（顺时针）。\n"
                       "固件：调用 face_rotate_start() 并等待完成。")
        hint.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #555; font-size: 11px;")
        form.addRow(hint)
        return page, {}

    # =========================================================
    #  内部：填入值 / 收集值
    # =========================================================
    def _fill_page(self, page_idx: int, step: Step) -> None:
        """把 step 的字段值写入对应页面的控件（不触发信号）。"""
        widgets = self._pages.get(page_idx, {})

        def set_w(key: str, val) -> None:
            w = widgets.get(key)
            if w is None:
                return
            w.blockSignals(True)
            if isinstance(w, (QDoubleSpinBox, QSpinBox)):
                w.setValue(val)
            elif isinstance(w, QComboBox):
                w.setCurrentText(str(val))
            w.blockSignals(False)

        t = step.step_type
        if t in (StepType.FORWARD, StepType.BACKWARD,
                 StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT):
            set_w("distance_cm",   step.distance_cm)
            set_w("power_pct",     step.power_pct)
            set_w("accel_cm",      step.accel_cm)
            set_w("decel_cm",      step.decel_cm)
            set_w("min_power_pct", step.min_power_pct)

        elif t == StepType.MOVE:
            set_w("direction_deg", step.direction_deg)
            set_w("distance_cm",   step.distance_cm)
            set_w("power_pct",     step.power_pct)
            set_w("accel_cm",      step.accel_cm)
            set_w("decel_cm",      step.decel_cm)
            set_w("min_power_pct", step.min_power_pct)

        elif t in (StepType.ROTATE_CW, StepType.ROTATE_CCW):
            set_w("degrees",           step.degrees)
            set_w("omega_pct",         step.omega_pct)
            set_w("rot_accel_deg",     step.rot_accel_deg)
            set_w("rot_decel_deg",     step.rot_decel_deg)
            set_w("rot_min_power_pct", step.rot_min_power_pct)

        elif t == StepType.SERVO:
            set_w("servo_id", step.servo_id)
            action_w = self._pages.get(self._PAGE_SERVO, {}).get("grip_action")
            if action_w is not None:
                action_w.blockSignals(True)
                action_w.setCurrentIndex(0 if step.grip_action == "close" else 1)
                action_w.blockSignals(False)
            set_w("grip_target_angle", step.grip_target_angle)

        elif t == StepType.DELAY:
            set_w("duration_s", step.duration_s)

        elif t == StepType.MOTOR:
            set_w("motor_id",    step.motor_id)
            set_w("motor_power", step.motor_power)

        elif t == StepType.DC_MOTOR:
            set_w("dc_port",  step.dc_port)
            set_w("dc_power", step.dc_power)

        elif t == StepType.MOVE_SPIN:
            set_w("direction_deg", step.direction_deg)
            set_w("distance_cm",   step.distance_cm)
            set_w("power_pct",     step.power_pct)
            set_w("spin_degrees",  step.spin_degrees)
            set_w("omega_pct",     step.omega_pct)

        elif t == StepType.OPEN_MOVE:
            set_w("direction_deg",    step.direction_deg)
            set_w("distance_cm",      step.distance_cm)
            set_w("power_pct",        step.power_pct)
            set_w("open_duration_ms", step.open_duration_ms)
            set_w("open_omega_pct",   step.open_omega_pct)

        elif t == StepType.OPEN_ROT:
            set_w("omega_pct",        step.omega_pct)
            set_w("open_duration_ms", step.open_duration_ms)

        elif t in (StepType.OPEN_FORWARD, StepType.OPEN_BACKWARD,
                   StepType.OPEN_LEFT,    StepType.OPEN_RIGHT):
            set_w("power_pct",        step.power_pct)
            set_w("open_duration_ms", step.open_duration_ms)

        elif t == StepType.LIFT:
            ws = self._pages.get(self._PAGE_LIFT, {})
            # 臂选
            arm_map = {"both": 0, "arm1": 1, "arm2": 2}
            arm_w = ws.get("lift_arm")
            if arm_w is not None:
                arm_w.blockSignals(True)
                arm_w.setCurrentIndex(arm_map.get(step.lift_arm, 0))
                arm_w.blockSignals(False)
            # 模式
            mode_w = ws.get("lift_mode")
            stack_w = ws.get("_lift_stack")
            mode_idx = 0 if step.lift_mode == "gear" else 1
            if mode_w is not None:
                mode_w.blockSignals(True)
                mode_w.setCurrentIndex(mode_idx)
                mode_w.blockSignals(False)
            if stack_w is not None:
                stack_w.setCurrentIndex(mode_idx)
            # 档位
            gear_w = ws.get("lift_gear_index")
            if gear_w is not None:
                gear_w.blockSignals(True)
                gear_w.setCurrentIndex(max(0, min(step.lift_gear_index, gear_w.count() - 1)))
                gear_w.blockSignals(False)
            # 角度
            angle_w = ws.get("lift_angle_deg")
            if angle_w is not None:
                angle_w.blockSignals(True)
                angle_w.setValue(step.lift_angle_deg)
                angle_w.blockSignals(False)
            # 后台运行
            async_w = ws.get("lift_async")
            if async_w is not None:
                async_w.blockSignals(True)
                async_w.setChecked(step.lift_async)
                async_w.blockSignals(False)

    def _collect_from_page(self, page_idx: int, base_step: Step) -> Step:
        """从当前页面控件读取参数，返回更新后的 Step。"""
        step = copy.deepcopy(base_step)
        widgets = self._pages.get(page_idx, {})

        def rv(key: str, default=None):
            w = widgets.get(key)
            if w is None:
                return default
            if isinstance(w, (QDoubleSpinBox, QSpinBox)):
                return w.value()
            if isinstance(w, QComboBox):
                return w.currentText()
            return default

        t = step.step_type
        if t in (StepType.FORWARD, StepType.BACKWARD,
                 StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT):
            step.distance_cm   = float(rv("distance_cm",   step.distance_cm))
            step.power_pct     = int(rv("power_pct",       step.power_pct))
            step.accel_cm      = float(rv("accel_cm",      step.accel_cm))
            step.decel_cm      = float(rv("decel_cm",      step.decel_cm))
            step.min_power_pct = int(rv("min_power_pct",   step.min_power_pct))

        elif t == StepType.MOVE:
            step.direction_deg = float(rv("direction_deg", step.direction_deg))
            step.distance_cm   = float(rv("distance_cm",   step.distance_cm))
            step.power_pct     = int(rv("power_pct",       step.power_pct))
            step.accel_cm      = float(rv("accel_cm",      step.accel_cm))
            step.decel_cm      = float(rv("decel_cm",      step.decel_cm))
            step.min_power_pct = int(rv("min_power_pct",   step.min_power_pct))

        elif t in (StepType.ROTATE_CW, StepType.ROTATE_CCW):
            step.degrees           = float(rv("degrees",           step.degrees))
            step.omega_pct         = int(rv("omega_pct",           step.omega_pct))
            step.rot_accel_deg     = float(rv("rot_accel_deg",     step.rot_accel_deg))
            step.rot_decel_deg     = float(rv("rot_decel_deg",     step.rot_decel_deg))
            step.rot_min_power_pct = int(rv("rot_min_power_pct",   step.rot_min_power_pct))

        elif t == StepType.SERVO:
            step.servo_id   = rv("servo_id", step.servo_id)
            action_w = self._pages.get(self._PAGE_SERVO, {}).get("grip_action")
            if action_w is not None:
                step.grip_action = "close" if action_w.currentIndex() == 0 else "open"
            step.grip_target_angle = int(rv("grip_target_angle", step.grip_target_angle))

        elif t == StepType.DELAY:
            step.duration_s = float(rv("duration_s", step.duration_s))

        elif t == StepType.MOTOR:
            step.motor_id    = rv("motor_id",    step.motor_id)
            step.motor_power = int(rv("motor_power", step.motor_power))

        elif t == StepType.DC_MOTOR:
            step.dc_port  = rv("dc_port",  step.dc_port)
            step.dc_power = int(rv("dc_power", step.dc_power))

        elif t == StepType.MOVE_SPIN:
            step.direction_deg = float(rv("direction_deg", step.direction_deg))
            step.distance_cm   = float(rv("distance_cm",   step.distance_cm))
            step.power_pct     = int(rv("power_pct",       step.power_pct))
            step.spin_degrees  = float(rv("spin_degrees",  step.spin_degrees))
            step.omega_pct     = int(rv("omega_pct",       step.omega_pct))

        elif t == StepType.OPEN_MOVE:
            step.direction_deg    = float(rv("direction_deg",    step.direction_deg))
            step.distance_cm      = float(rv("distance_cm",      step.distance_cm))
            step.power_pct        = int(rv("power_pct",          step.power_pct))
            step.open_duration_ms = int(rv("open_duration_ms",   step.open_duration_ms))
            step.open_omega_pct   = int(rv("open_omega_pct",     step.open_omega_pct))

        elif t == StepType.OPEN_ROT:
            step.omega_pct        = int(rv("omega_pct",          step.omega_pct))
            step.open_duration_ms = int(rv("open_duration_ms",   step.open_duration_ms))

        elif t in (StepType.OPEN_FORWARD, StepType.OPEN_BACKWARD,
                   StepType.OPEN_LEFT,    StepType.OPEN_RIGHT):
            step.power_pct        = int(rv("power_pct",          step.power_pct))
            step.open_duration_ms = int(rv("open_duration_ms",   step.open_duration_ms))

        elif t == StepType.LIFT:
            ws = self._pages.get(self._PAGE_LIFT, {})
            # 臂选
            arm_map_rev = {0: "both", 1: "arm1", 2: "arm2"}
            arm_map_fwd = {"both": 0, "arm1": 1, "arm2": 2}
            arm_w = ws.get("lift_arm")
            step.lift_arm = arm_map_rev.get(arm_w.currentIndex() if arm_w else 0, "both")
            # 模式
            mode_w = ws.get("lift_mode")
            step.lift_mode = "angle" if (mode_w and mode_w.currentIndex() == 1) else "gear"
            # 档位
            gear_w = ws.get("lift_gear_index")
            if gear_w is not None:
                step.lift_gear_index = gear_w.currentIndex()
            # 角度
            angle_w = ws.get("lift_angle_deg")
            if angle_w is not None:
                step.lift_angle_deg = float(angle_w.value())
            # 如果是档位模式，把档位对应角度同步到 lift_angle_deg，供 kinematics 统一读取
            if step.lift_mode == "gear":
                try:
                    gears_raw = _load_lift_gears()
                    if step.lift_gear_index < len(gears_raw):
                        step.lift_angle_deg = float(gears_raw[step.lift_gear_index])
                except Exception:
                    pass
            # 后台运行
            async_w = ws.get("lift_async")
            if async_w is not None:
                step.lift_async = async_w.isChecked()

        return step

    def _on_any_value_changed(self) -> None:
        if self._loading or self._current_step is None:
            return
        page_idx = self._stack.currentIndex()
        new_step = self._collect_from_page(page_idx, self._current_step)
        self._current_step = new_step
        self.step_changed.emit(copy.deepcopy(new_step))
