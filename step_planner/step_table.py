"""step_planner.step_table — 步骤表格组件（QTableWidget）

每行对应一条 Step，用 setCellWidget() 放置实际控件，无需 item delegate。
这种方式对 ≤200 行的场景比 delegate 更直接，所有编辑即时反映。

公开 API
========
  StepTableWidget.get_steps()          -> list[Step]
  StepTableWidget.set_steps(steps)     # 全量替换（用于文件加载、撤销/重做）
  StepTableWidget.add_step(step, row)  # 在 row 之后插入
  StepTableWidget.delete_step(row)
  StepTableWidget.move_step(row, delta)
  StepTableWidget.current_step()       -> Step | None
  StepTableWidget.current_row()        -> int

信号
====
  steps_changed    — 任何编辑（类型/参数/备注）
  step_selected(int) — 当前行变化（row index, −1=无选中）
"""

from __future__ import annotations

import copy
from typing import Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QAbstractSpinBox,
    QComboBox,
    QDoubleSpinBox,
    QHeaderView,
    QLineEdit,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QWidget,
)

from .models import Step, StepType, STEP_TYPE_LABELS


# =========================================================
#  列索引常量
# =========================================================
COL_NUM  = 0   # 行号（只读）
COL_TYPE = 1   # 步骤类型（QComboBox）
COL_P1   = 2   # 主参数（距离/角度/时长/端口等）
COL_P2   = 3   # 副参数（功率%/转速%/速度等）
COL_P3   = 4   # 第三参数（MOVE 功率 / SERVO 速度等）
COL_P4   = 5   # 第四参数（SERVO 等待 ms）
COL_NOTE = 6   # 备注

_HEADERS    = ["#", "类型", "参数 1", "参数 2", "参数 3", "参数 4", "备注"]
_COL_WIDTHS = [36,  155,    130,      110,       110,       100,      120]

ROW_HEIGHT = 32


class StepTableWidget(QTableWidget):
    """步骤表格

    内部用 self._steps: list[Step] 作为唯一数据源。
    所有对控件的读写都通过 _collect_step() / _populate_row() 进行。
    """

    steps_changed = pyqtSignal()        # 任何编辑
    step_selected = pyqtSignal(int)     # 当前行 index（-1 = 无）

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(0, len(_HEADERS), parent)
        self._steps: list[Step] = []
        self._rebuilding = False        # 防止 set_steps 时触发 steps_changed

        # ── 表格样式 ────────────────────────────────────────────
        self.setHorizontalHeaderLabels(_HEADERS)
        for col, w in enumerate(_COL_WIDTHS):
            self.setColumnWidth(col, w)
        self.horizontalHeader().setStretchLastSection(True)
        self.horizontalHeader().setSectionResizeMode(COL_TYPE, QHeaderView.Fixed)
        self.verticalHeader().setDefaultSectionSize(ROW_HEIGHT)
        self.verticalHeader().setVisible(False)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.selectionModel().currentRowChanged.connect(
            lambda cur, prev: self.step_selected.emit(cur.row()))

    # =========================================================
    #  公开 API
    # =========================================================
    def get_steps(self) -> list[Step]:
        return copy.deepcopy(self._steps)

    def set_steps(self, steps: list[Step]) -> None:
        """全量替换，用于文件加载和撤销/重做。"""
        self._rebuilding = True
        prev_row = self.currentRow()

        self.clearContents()
        self.setRowCount(0)
        self._steps = []

        for step in steps:
            self._steps.append(copy.deepcopy(step))
            row = self.rowCount()
            self.insertRow(row)
            self._populate_row(row)

        self._rebuilding = False

        # 尽量恢复选中行
        target = min(prev_row, self.rowCount() - 1)
        if target >= 0:
            self.setCurrentCell(target, COL_TYPE)

        self.steps_changed.emit()

    def add_step(self, step: Step, after_row: int = -1) -> None:
        """在 after_row 之后插入一行；after_row=-1 追加到末尾。"""
        insert_at = (after_row + 1) if after_row >= 0 else len(self._steps)
        insert_at = max(0, min(insert_at, len(self._steps)))

        self._steps.insert(insert_at, copy.deepcopy(step))
        self.insertRow(insert_at)
        self._populate_row(insert_at)
        self._renumber()

        self.setCurrentCell(insert_at, COL_TYPE)
        self.steps_changed.emit()

    def delete_step(self, row: int) -> None:
        if not (0 <= row < len(self._steps)):
            return
        self._steps.pop(row)
        self.removeRow(row)
        self._renumber()

        # 选中删除后的同一位置（或最后一行）
        new_row = min(row, self.rowCount() - 1)
        if new_row >= 0:
            self.setCurrentCell(new_row, COL_TYPE)

        self.steps_changed.emit()

    def move_step(self, row: int, delta: int) -> None:
        target = row + delta
        if not (0 <= row < len(self._steps)) or not (0 <= target < len(self._steps)):
            return
        self._steps[row], self._steps[target] = self._steps[target], self._steps[row]
        # 只刷新受影响的两行
        self._populate_row(row)
        self._populate_row(target)
        self._renumber()

        self.setCurrentCell(target, COL_TYPE)
        self.steps_changed.emit()

    def update_step_at(self, row: int, step: Step) -> None:
        """用 step 替换 row 的数据并刷新该行控件，不改变选中行，不触发全量重建。"""
        if not (0 <= row < len(self._steps)):
            return
        self._rebuilding = True
        self._steps[row] = copy.deepcopy(step)
        self._populate_row(row)
        self._rebuilding = False
        # 不 emit steps_changed，由调用方决定是否需要

    def current_step(self) -> Optional[Step]:
        row = self.currentRow()
        if 0 <= row < len(self._steps):
            return self._steps[row]
        return None

    def current_row(self) -> int:
        return self.currentRow()

    # =========================================================
    #  内部：行管理
    # =========================================================
    def _populate_row(self, row: int) -> None:
        """用 self._steps[row] 填充 row 的所有单元格控件。"""
        step = self._steps[row]

        # 行号（只读）
        num_item = QTableWidgetItem(str(row + 1))
        num_item.setFlags(Qt.ItemIsEnabled)
        num_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, COL_NUM, num_item)

        # 类型 combo
        type_combo = QComboBox()
        type_combo.setFrame(False)
        for st in StepType:
            type_combo.addItem(STEP_TYPE_LABELS[st], userData=st)
        idx = list(StepType).index(step.step_type)
        type_combo.setCurrentIndex(idx)
        type_combo.currentIndexChanged.connect(
            lambda _, r=row: self._on_type_changed(r))
        self.setCellWidget(row, COL_TYPE, type_combo)

        # 参数列（按类型各异）
        self._set_param_widgets(row, step)

        # 备注
        note_edit = QLineEdit(step.comment)
        note_edit.setFrame(False)
        note_edit.setPlaceholderText("备注…")
        note_edit.editingFinished.connect(
            lambda r=row: self._on_param_changed(r))
        self.setCellWidget(row, COL_NOTE, note_edit)

    def _set_param_widgets(self, row: int, step: Step) -> None:
        """为 row 设置 P1~P4 列的控件，因步骤类型而异。"""
        # 先清空旧控件和 item
        for col in (COL_P1, COL_P2, COL_P3, COL_P4):
            old = self.cellWidget(row, col)
            if old is not None:
                old.deleteLater()
                self.removeCellWidget(row, col)
            placeholder = QTableWidgetItem("")
            placeholder.setFlags(Qt.NoItemFlags)
            self.setItem(row, col, placeholder)

        t = step.step_type

        # ── 控件工厂 ────────────────────────────────────────────
        def dspin(val: float, lo: float = 0.0, hi: float = 9999.0,
                  suffix: str = "", step_: float = 1.0) -> QDoubleSpinBox:
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setValue(val)
            w.setSuffix(suffix)
            w.setSingleStep(step_)
            w.setDecimals(1)
            w.setButtonSymbols(QAbstractSpinBox.NoButtons)
            w.setFrame(False)
            w.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            w.valueChanged.connect(lambda _, r=row: self._on_param_changed(r))
            return w

        def ispin(val: int, lo: int = 0, hi: int = 100,
                  suffix: str = "") -> QSpinBox:
            w = QSpinBox()
            w.setRange(lo, hi)
            w.setValue(val)
            w.setSuffix(suffix)
            w.setButtonSymbols(QAbstractSpinBox.NoButtons)
            w.setFrame(False)
            w.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            w.valueChanged.connect(lambda _, r=row: self._on_param_changed(r))
            return w

        def cbox(items: list[str], current: str) -> QComboBox:
            w = QComboBox()
            w.setFrame(False)
            w.addItems(items)
            if current in items:
                w.setCurrentText(current)
            w.currentTextChanged.connect(lambda _, r=row: self._on_param_changed(r))
            return w

        # ── 按类型装控件 ────────────────────────────────────────
        if t in (StepType.FORWARD, StepType.BACKWARD,
                 StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT):
            self.setCellWidget(row, COL_P1, dspin(step.distance_cm, 0, 2000, " cm"))
            self.setCellWidget(row, COL_P2, ispin(step.power_pct, 0, 100, " %"))
            if step.accel_cm > 0 or step.decel_cm > 0:
                self.setCellWidget(row, COL_P3,
                                   dspin(step.accel_cm, 0, 500, " cm↑"))
                self.setCellWidget(row, COL_P4,
                                   dspin(step.decel_cm, 0, 500, " cm↓"))

        elif t == StepType.MOVE:
            self.setCellWidget(row, COL_P1, dspin(step.direction_deg, -360, 360, "°"))
            self.setCellWidget(row, COL_P2, dspin(step.distance_cm, 0, 2000, " cm"))
            self.setCellWidget(row, COL_P3, ispin(step.power_pct, 0, 100, " %"))

        elif t in (StepType.ROTATE_CW, StepType.ROTATE_CCW):
            self.setCellWidget(row, COL_P1, dspin(step.degrees, 0, 3600, "°"))
            self.setCellWidget(row, COL_P2, ispin(step.omega_pct, 0, 100, " %"))
            self.setCellWidget(row, COL_P3, dspin(step.rot_accel_deg, 0, 3600, "°↑"))
            self.setCellWidget(row, COL_P4, dspin(step.rot_decel_deg, 0, 3600, "°↓"))
        elif t == StepType.SERVO:
            action_label = "夹紧" if step.grip_action == "close" else "松开"
            self.setCellWidget(row, COL_P1, cbox(["SV1", "SV2", "SV3"], step.servo_id))
            self.setCellWidget(row, COL_P2, cbox(["夹紧", "松开"], action_label))

        elif t == StepType.DELAY:
            self.setCellWidget(row, COL_P1,
                               dspin(step.duration_s, 0, 60, " s", 0.1))

        elif t == StepType.MOTOR:
            self.setCellWidget(row, COL_P1,
                               cbox(["M1", "M2", "M3", "M4", "M5"], step.motor_id))
            self.setCellWidget(row, COL_P2, ispin(step.motor_power, -100, 100, " %"))

        elif t == StepType.DC_MOTOR:
            self.setCellWidget(row, COL_P1, cbox(["DC1", "DC2"], step.dc_port))
            self.setCellWidget(row, COL_P2, ispin(step.dc_power, -100, 100, " %"))

        elif t == StepType.MOVE_SPIN:
            self.setCellWidget(row, COL_P1, dspin(step.direction_deg, -360, 360, "°"))
            self.setCellWidget(row, COL_P2, dspin(step.distance_cm, 0, 2000, " cm"))
            self.setCellWidget(row, COL_P3, ispin(step.power_pct, 0, 100, " %"))
            self.setCellWidget(row, COL_P4, dspin(step.spin_degrees, -3600, 3600, "°↻"))

        elif t == StepType.OPEN_MOVE:
            self.setCellWidget(row, COL_P1, dspin(step.direction_deg, -360, 360, "°"))
            self.setCellWidget(row, COL_P2, ispin(step.power_pct, 0, 100, " %"))
            self.setCellWidget(row, COL_P3, ispin(step.open_duration_ms, 0, 30000, " ms"))

        elif t == StepType.OPEN_ROT:
            self.setCellWidget(row, COL_P1, ispin(step.omega_pct, -100, 100, " %"))
            self.setCellWidget(row, COL_P2, ispin(step.open_duration_ms, 0, 30000, " ms"))

        elif t in (StepType.OPEN_FORWARD, StepType.OPEN_BACKWARD,
                   StepType.OPEN_LEFT,    StepType.OPEN_RIGHT):
            self.setCellWidget(row, COL_P1, ispin(step.power_pct, 0, 100, " %"))
            self.setCellWidget(row, COL_P2, ispin(step.open_duration_ms, 0, 30000, " ms"))

        elif t == StepType.LIFT:
            arm_labels = {"both": "双臂", "arm1": "一号臂M5", "arm2": "二号臂M4"}
            arm_str = arm_labels.get(step.lift_arm, "双臂")
            self.setCellWidget(row, COL_P1, cbox(list(arm_labels.values()), arm_str))
            self.setCellWidget(row, COL_P2, dspin(step.lift_angle_deg, 0, 3600, "°", 5.0))

        elif t == StepType.FACE_CHANGE:
            pass  # 无参数，不放控件

    def _renumber(self) -> None:
        """更新所有行的行号 item。"""
        for r in range(self.rowCount()):
            item = self.item(r, COL_NUM)
            if item is not None:
                item.setText(str(r + 1))

    # =========================================================
    #  内部：信号处理
    # =========================================================
    def _on_type_changed(self, row: int) -> None:
        if self._rebuilding:
            return
        combo = self.cellWidget(row, COL_TYPE)
        if combo is None:
            return
        new_type = combo.currentData()
        old = self._steps[row]
        # 对同族类型切换时保留相关参数
        _LINEAR = (StepType.FORWARD, StepType.BACKWARD,
                   StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT)
        _ROTATE = (StepType.ROTATE_CW, StepType.ROTATE_CCW)
        new_step = Step(step_type=new_type, comment=old.comment)
        if old.step_type in _LINEAR and new_type in _LINEAR:
            new_step.distance_cm   = old.distance_cm
            new_step.power_pct     = old.power_pct
            new_step.accel_cm      = old.accel_cm
            new_step.decel_cm      = old.decel_cm
            new_step.min_power_pct = old.min_power_pct
        elif old.step_type in _ROTATE and new_type in _ROTATE:
            new_step.degrees           = old.degrees
            new_step.omega_pct         = old.omega_pct
            new_step.rot_accel_deg     = old.rot_accel_deg
            new_step.rot_decel_deg     = old.rot_decel_deg
            new_step.rot_min_power_pct = old.rot_min_power_pct
        self._steps[row] = new_step
        self._set_param_widgets(row, self._steps[row])
        self.steps_changed.emit()

    def _on_param_changed(self, row: int) -> None:
        if self._rebuilding:
            return
        if 0 <= row < len(self._steps):
            self._steps[row] = self._collect_step(row)
            self.steps_changed.emit()

    def _on_row_selected(self, row: int) -> None:
        self.step_selected.emit(row)

    def _collect_step(self, row: int) -> Step:
        """从 row 的所有单元格控件读取参数，构造 Step。"""
        type_w = self.cellWidget(row, COL_TYPE)
        st: StepType = type_w.currentData() if type_w else StepType.FORWARD
        note_w = self.cellWidget(row, COL_NOTE)
        comment = note_w.text() if isinstance(note_w, QLineEdit) else ""

        def dv(col: int, default: float = 0.0) -> float:
            w = self.cellWidget(row, col)
            if isinstance(w, (QDoubleSpinBox, QSpinBox)):
                return float(w.value())
            return default

        def iv(col: int, default: int = 0) -> int:
            return int(dv(col, default))

        def sv(col: int, default: str = "") -> str:
            w = self.cellWidget(row, col)
            return w.currentText() if isinstance(w, QComboBox) else default

        kw: dict = {"step_type": st, "comment": comment}

        if st in (StepType.FORWARD, StepType.BACKWARD,
                  StepType.STRAFE_LEFT, StepType.STRAFE_RIGHT):
            kw.update(distance_cm=dv(COL_P1), power_pct=iv(COL_P2),
                      accel_cm=dv(COL_P3), decel_cm=dv(COL_P4))

        elif st == StepType.MOVE:
            kw.update(direction_deg=dv(COL_P1),
                      distance_cm=dv(COL_P2),
                      power_pct=iv(COL_P3))
            # accel/decel 在 step_editor 侧设置，表格里不展示额外列

        elif st in (StepType.ROTATE_CW, StepType.ROTATE_CCW):
            kw.update(degrees=dv(COL_P1), omega_pct=iv(COL_P2),
                      rot_accel_deg=dv(COL_P3), rot_decel_deg=dv(COL_P4))

        elif st == StepType.SERVO:
            kw["servo_id"] = sv(COL_P1)
            action_text = sv(COL_P2)
            kw["grip_action"] = "close" if action_text == "夹紧" else "open"

        elif st == StepType.DELAY:
            kw.update(duration_s=dv(COL_P1))

        elif st == StepType.MOTOR:
            kw.update(motor_id=sv(COL_P1), motor_power=iv(COL_P2))

        elif st == StepType.DC_MOTOR:
            kw.update(dc_port=sv(COL_P1), dc_power=iv(COL_P2))

        elif st == StepType.MOVE_SPIN:
            kw.update(direction_deg=dv(COL_P1),
                      distance_cm=dv(COL_P2),
                      power_pct=iv(COL_P3),
                      spin_degrees=dv(COL_P4))

        elif st == StepType.OPEN_MOVE:
            kw.update(direction_deg=dv(COL_P1),
                      power_pct=iv(COL_P2),
                      open_duration_ms=iv(COL_P3))

        elif st == StepType.OPEN_ROT:
            kw.update(omega_pct=iv(COL_P1),
                      open_duration_ms=iv(COL_P2))

        elif st in (StepType.OPEN_FORWARD, StepType.OPEN_BACKWARD,
                    StepType.OPEN_LEFT,    StepType.OPEN_RIGHT):
            kw.update(power_pct=iv(COL_P1),
                      open_duration_ms=iv(COL_P2))

        elif st == StepType.LIFT:
            arm_map = {0: "both", 1: "arm1", 2: "arm2"}
            w_arm = self.cellWidget(row, COL_P1)
            arm_idx = w_arm.currentIndex() if isinstance(w_arm, QComboBox) else 0
            kw["lift_arm"] = arm_map.get(arm_idx, "both")
            kw["lift_angle_deg"] = dv(COL_P2, 315.0)
            # 保留来自 step_editor 的 lift_async，表格里没有这个控件
            if 0 <= row < len(self._steps):
                kw["lift_async"] = self._steps[row].lift_async

        elif st == StepType.FACE_CHANGE:
            pass  # 无参数

        return Step(**kw)
