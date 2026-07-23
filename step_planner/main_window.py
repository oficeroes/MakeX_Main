"""step_planner.main_window — 主窗口（StepMainWindow）

布局
====
  工具栏（顶）
  ┌── QSplitter(Horizontal) ──────────────────────────────────┐
  │  左区(60%): StepTableWidget + 标定状态栏                  │
  │  右区(40%): FieldView + 起始位姿行 + StepEditor           │
  └────────────────────────────────────────────────────────────┘
  状态栏（底）

撤销/重做
=========
  使用快照列表（list[list[Step]]），最多 50 快照。
  每次有效变更时 push，Ctrl+Z/Y 弹出恢复。
  不使用 QUndoStack（简化实现、避免 QUndoCommand 的模板代码）。

导出
====
  读取当前步骤 + 标定值 → kinematics.build_sequence()
  → 弹出确认对话框（显示前 12 条元组 + 目标文件）
  → trajectory_planner.exporter.write_auto_sequence() 原子写入
  → 同时写入 TURN_TICKS_PER_DEG 常量（upsert_float_constant）
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QAction,
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .config import (
    CHASSIS_PROFILES,
    DEFAULT_FIELD_HEIGHT_CM,
    DEFAULT_FIELD_WIDTH_CM,
    DEFAULT_PROFILE_ID,
    STEP_SEQUENCES_DIR,
    default_calibration,
    get_profile,
)
from .field_view import FieldView
from .file_io import load_steps, next_filename, save_steps
from .kinematics import (
    build_sequence,
    estimate_total_distance,
    estimate_total_rotation,
)
from .models import CalibrationState, Pose, Step, StepType
from .step_editor import StepEditor
from .step_table import StepTableWidget

# 复用旧 exporter 的原子写入 + 常量 upsert
from trajectory_planner.exporter import (
    upsert_float_constant,
    upsert_int_constant,
    write_auto_sequence,
)


# =========================================================
#  标定向导对话框
# =========================================================
class CalibrationDialog(QDialog):
    """两标签页标定向导

    Tab 1 "直线标定":
      new_tpc = current_tpc × (command_cm / actual_cm)

    Tab 2 "旋转标定":
      new_ttd = current_ttd × (command_deg / actual_deg)
      — 或 —
      新值 = 固定 ticks 数 ÷ 实测角度
      （推荐发 enc_rot 1000 ticks，量实际角度）
    """

    def __init__(self, current_cal: CalibrationState,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("标定向导")
        self.setFixedWidth(420)
        self._result_cal = copy.deepcopy(current_cal)

        tabs = QTabWidget()

        # ── Tab 1: 直线标定 ──────────────────────────────────────
        t1 = QWidget()
        f1 = QFormLayout(t1)
        f1.setLabelAlignment(Qt.AlignRight)

        self._lbl_cur_tpc = QLabel(f"{current_cal.encoder_ticks_per_cm:.4f} °/cm")
        self._lbl_cur_tpc.setStyleSheet("color: #555;")
        f1.addRow("当前 ENCODER_TICKS_PER_CM:", self._lbl_cur_tpc)

        self._sp_cmd_cm = QDoubleSpinBox()
        self._sp_cmd_cm.setRange(1, 1000)
        self._sp_cmd_cm.setValue(100.0)
        self._sp_cmd_cm.setSuffix(" cm")
        self._sp_cmd_cm.setButtonSymbols(QAbstractSpinBox.NoButtons)
        f1.addRow("命令移动距离:", self._sp_cmd_cm)

        self._sp_act_cm = QDoubleSpinBox()
        self._sp_act_cm.setRange(0.1, 1000)
        self._sp_act_cm.setValue(100.0)
        self._sp_act_cm.setSuffix(" cm")
        self._sp_act_cm.setButtonSymbols(QAbstractSpinBox.NoButtons)
        f1.addRow("实测移动距离:", self._sp_act_cm)

        self._lbl_new_tpc = QLabel("—")
        self._lbl_new_tpc.setStyleSheet("font-weight: bold; color: #1B4E8A;")
        f1.addRow("新 ENCODER_TICKS_PER_CM:", self._lbl_new_tpc)

        hint1 = QLabel("新值 = 当前值 × (命令距离 / 实测距离)\n"
                        "可多次迭代：每次用上次结果作为新的当前值。")
        hint1.setWordWrap(True)
        hint1.setStyleSheet("color: #777; font-size: 11px;")
        f1.addRow("", hint1)

        self._sp_cmd_cm.valueChanged.connect(self._update_linear_preview)
        self._sp_act_cm.valueChanged.connect(self._update_linear_preview)
        self._update_linear_preview()
        tabs.addTab(t1, "直线标定")

        # ── Tab 2: 旋转标定 ──────────────────────────────────────
        t2 = QWidget()
        f2 = QFormLayout(t2)
        f2.setLabelAlignment(Qt.AlignRight)

        self._lbl_cur_ttd = QLabel(f"{current_cal.turn_ticks_per_deg:.4f} °/°")
        self._lbl_cur_ttd.setStyleSheet("color: #555;")
        f2.addRow("当前 TURN_TICKS_PER_DEG:", self._lbl_cur_ttd)

        self._sp_cmd_deg = QDoubleSpinBox()
        self._sp_cmd_deg.setRange(1, 3600)
        self._sp_cmd_deg.setValue(360.0)
        self._sp_cmd_deg.setSuffix(" °")
        self._sp_cmd_deg.setButtonSymbols(QAbstractSpinBox.NoButtons)
        f2.addRow("命令旋转角度:", self._sp_cmd_deg)

        self._sp_act_deg = QDoubleSpinBox()
        self._sp_act_deg.setRange(0.1, 3600)
        self._sp_act_deg.setValue(360.0)
        self._sp_act_deg.setSuffix(" °")
        self._sp_act_deg.setButtonSymbols(QAbstractSpinBox.NoButtons)
        f2.addRow("实测旋转角度:", self._sp_act_deg)

        self._lbl_new_ttd = QLabel("—")
        self._lbl_new_ttd.setStyleSheet("font-weight: bold; color: #1B4E8A;")
        f2.addRow("新 TURN_TICKS_PER_DEG:", self._lbl_new_ttd)

        hint2 = QLabel(
            "推荐流程：\n"
            "1. 先导出一个旋转标定序列（从主窗口「旋转标定导出」按钮）\n"
            "2. 烧录，按 + 运行，用量角器量实际转角\n"
            "3. 在此填入命令角度（导出时的角度）和实测角度\n"
            "新值 = 当前值 × (命令角度 / 实测角度)"
        )
        hint2.setWordWrap(True)
        hint2.setStyleSheet("color: #777; font-size: 11px;")
        f2.addRow("", hint2)

        self._sp_cmd_deg.valueChanged.connect(self._update_rotation_preview)
        self._sp_act_deg.valueChanged.connect(self._update_rotation_preview)
        self._update_rotation_preview()
        tabs.addTab(t2, "旋转标定")

        # ── 按钮 ─────────────────────────────────────────────────
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self._on_accept)
        btns.rejected.connect(self.reject)

        vbox = QVBoxLayout(self)
        vbox.addWidget(tabs)
        vbox.addWidget(btns)

        self._current_cal = current_cal

    def _update_linear_preview(self) -> None:
        cmd = self._sp_cmd_cm.value()
        act = self._sp_act_cm.value()
        if act > 0:
            new_val = self._current_cal.encoder_ticks_per_cm * (cmd / act)
            self._lbl_new_tpc.setText(f"{new_val:.4f} °/cm")
        else:
            self._lbl_new_tpc.setText("—")

    def _update_rotation_preview(self) -> None:
        cmd = self._sp_cmd_deg.value()
        act = self._sp_act_deg.value()
        if act > 0:
            new_val = self._current_cal.turn_ticks_per_deg * (cmd / act)
            self._lbl_new_ttd.setText(f"{new_val:.4f} °/°")
        else:
            self._lbl_new_ttd.setText("—")

    def _on_accept(self) -> None:
        cal = copy.deepcopy(self._current_cal)

        cmd_cm = self._sp_cmd_cm.value()
        act_cm = self._sp_act_cm.value()
        if act_cm > 0 and abs(cmd_cm - act_cm) > 0.01:
            cal.encoder_ticks_per_cm = cal.encoder_ticks_per_cm * (cmd_cm / act_cm)

        cmd_deg = self._sp_cmd_deg.value()
        act_deg = self._sp_act_deg.value()
        if act_deg > 0 and abs(cmd_deg - act_deg) > 0.01:
            cal.turn_ticks_per_deg = cal.turn_ticks_per_deg * (cmd_deg / act_deg)

        self._result_cal = cal
        self.accept()

    def result_calibration(self) -> CalibrationState:
        return copy.deepcopy(self._result_cal)


# =========================================================
#  导出确认对话框
# =========================================================
class ExportConfirmDialog(QDialog):
    def __init__(self, sequence: list[tuple], robot_file: Path,
                 cal: CalibrationState, parent=None):
        super().__init__(parent)
        self.setWindowTitle("确认导出")
        self.setMinimumWidth(460)

        vbox = QVBoxLayout(self)

        info = QLabel(
            f"目标文件：<b>{robot_file.name}</b><br>"
            f"ENCODER_TICKS_PER_CM = {cal.encoder_ticks_per_cm:.4f}<br>"
            f"TURN_TICKS_PER_DEG = {cal.turn_ticks_per_deg:.4f}<br>"
            f"共 <b>{len(sequence)}</b> 条指令（含末尾 enc_stop）"
        )
        info.setTextFormat(Qt.RichText)
        vbox.addWidget(info)

        preview_label = QLabel("生成的 AUTO_SEQUENCE（前 15 条）：")
        vbox.addWidget(preview_label)

        txt = QTextEdit()
        txt.setReadOnly(True)
        txt.setFont(QApplication.font())
        txt.setMaximumHeight(200)
        lines = []
        for i, tup in enumerate(sequence[:15]):
            lines.append(f"  {repr(tup)},")
        if len(sequence) > 15:
            lines.append(f"  ... ({len(sequence) - 15} 条未显示)")
        txt.setPlainText("\n".join(lines))
        vbox.addWidget(txt)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.button(QDialogButtonBox.Ok).setText("确认导出")
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        vbox.addWidget(btns)


# =========================================================
#  主窗口
# =========================================================
class StepMainWindow(QMainWindow):

    _MAX_UNDO = 50

    def __init__(self):
        super().__init__()
        self.setWindowTitle("步骤规划器 — MakeX 2026")
        self.resize(1400, 840)

        # ── 核心状态 ─────────────────────────────────────────────
        self._active_profile_id: str = DEFAULT_PROFILE_ID
        self._calibrations: dict[str, CalibrationState] = {
            p.profile_id: default_calibration(p.profile_id)
            for p in CHASSIS_PROFILES
        }
        self._field_w: float = DEFAULT_FIELD_WIDTH_CM
        self._field_h: float = DEFAULT_FIELD_HEIGHT_CM
        self._current_file: Optional[Path] = None

        # 撤销/重做快照（list of list[Step]）
        self._undo_stack: list[list[Step]] = []
        self._redo_stack: list[list[Step]] = []
        self._pre_edit_snapshot: Optional[list[Step]] = None
        # 防止右侧编辑器→表格→编辑器的循环更新
        self._editor_syncing: bool = False
        # debounce：右侧编辑器改值时延迟 120ms 再回写表格，避免连续击键闪烁
        self._editor_pending_step: Optional[Step] = None
        self._editor_debounce = QTimer(self)
        self._editor_debounce.setSingleShot(True)
        self._editor_debounce.setInterval(120)
        self._editor_debounce.timeout.connect(self._flush_editor_change)

        # ── 构建 UI ──────────────────────────────────────────────
        self._build_central()
        self._build_toolbar()
        self._build_statusbar()
        self._wire_signals()
        self._wire_shortcuts()

        # 初始加载
        self._load_profile(DEFAULT_PROFILE_ID)
        self._refresh_status()
        self._refresh_preview()

    # =========================================================
    #  UI 构建
    # =========================================================
    def _build_central(self) -> None:
        splitter = QSplitter(Qt.Horizontal)

        # ── 左区：步骤表格 + 标定状态栏 ─────────────────────────
        left = QWidget()
        left.setMinimumWidth(500)
        lv = QVBoxLayout(left)
        lv.setContentsMargins(4, 4, 4, 4)
        lv.setSpacing(4)

        self.table = StepTableWidget()
        lv.addWidget(self.table, 1)

        # 表格下方工具行
        tbl_tools = QHBoxLayout()
        self._btn_add    = QPushButton("➕ 添加步骤")
        self._btn_delete = QPushButton("🗑 删除")
        self._btn_up     = QPushButton("↑")
        self._btn_down   = QPushButton("↓")
        self._btn_dup    = QPushButton("复制")
        for btn in (self._btn_add, self._btn_delete, self._btn_up,
                    self._btn_down, self._btn_dup):
            btn.setFixedHeight(28)
            tbl_tools.addWidget(btn)
        tbl_tools.addStretch(1)
        lv.addLayout(tbl_tools)

        # 标定状态行
        cal_row = QHBoxLayout()
        self._lbl_cal = QLabel()
        self._lbl_cal.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._lbl_cal.setStyleSheet("color: #555; font-size: 11px;")
        cal_row.addWidget(self._lbl_cal, 1)
        self._btn_open_cal = QPushButton("标定向导…")
        self._btn_open_cal.setFixedWidth(90)
        cal_row.addWidget(self._btn_open_cal)
        lv.addLayout(cal_row)

        # ── 右区：场地预览 + 起始位姿 + 步骤编辑器 ─────────────
        right = QWidget()
        right.setMinimumWidth(360)
        rv = QVBoxLayout(right)
        rv.setContentsMargins(4, 4, 4, 4)
        rv.setSpacing(6)

        self.field_view = FieldView()
        self.field_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        rv.addWidget(self.field_view, 3)

        # 起始位姿行
        pose_row = QHBoxLayout()
        pose_row.addWidget(QLabel("起点 X:"))
        self._sp_x = self._make_dspin(0, DEFAULT_FIELD_WIDTH_CM,  1, " cm")
        self._sp_x.setValue(50.0)
        pose_row.addWidget(self._sp_x)
        pose_row.addWidget(QLabel("Y:"))
        self._sp_y = self._make_dspin(0, DEFAULT_FIELD_HEIGHT_CM, 1, " cm")
        self._sp_y.setValue(50.0)
        pose_row.addWidget(self._sp_y)
        pose_row.addWidget(QLabel("朝向:"))
        self._sp_h = self._make_dspin(-360, 360, 1, "°")
        self._sp_h.setValue(0.0)
        pose_row.addWidget(self._sp_h)
        btn_reset_pose = QPushButton("重置")
        btn_reset_pose.setFixedWidth(48)
        btn_reset_pose.clicked.connect(self._reset_start_pose)
        pose_row.addWidget(btn_reset_pose)
        rv.addLayout(pose_row)

        # 步骤编辑器（右侧参数面板）
        self.step_editor = StepEditor()
        self.step_editor.setMaximumHeight(280)
        rv.addWidget(self.step_editor, 0)

        # 标定参数区（内联，折叠在右侧下方）
        cal_group = QGroupBox("标定 / 硬件参数")
        cal_group.setCheckable(False)
        calf = QFormLayout(cal_group)
        calf.setLabelAlignment(Qt.AlignRight)

        # 标定值
        self._sp_tpc = self._make_dspin(0.01, 999.0, 4, " °/cm")
        self._sp_ttd = self._make_dspin(0.01, 999.0, 4, " °/°")
        calf.addRow("线性 (ENCODER_TICKS_PER_CM):", self._sp_tpc)
        calf.addRow("旋转 (TURN_TICKS_PER_DEG):",   self._sp_ttd)

        # 轴反转复选框
        invert_row = QHBoxLayout()
        self._chk_inv_vx    = QCheckBox("反转 Vx（左右）")
        self._chk_inv_vy    = QCheckBox("反转 Vy（前后）")
        self._chk_inv_omega = QCheckBox("反转 Ω（旋转）")
        for chk in (self._chk_inv_vx, self._chk_inv_vy, self._chk_inv_omega):
            invert_row.addWidget(chk)
        invert_row.addStretch(1)
        calf.addRow("轴反转:", invert_row)

        # 旋转偏量修正
        self._sp_fbc = self._make_dspin(-1.0, 1.0, 4, "")
        self._sp_fbc.setSingleStep(0.005)
        self._sp_fbc.setToolTip(
            "FRONT_BACK_COMPENSATION\n"
            "正值 = 压制后轮功率（斜走时顺时针漂移用正值修正）\n"
            "负值 = 压制前轮功率（逆时针漂移用负值修正）\n"
            "建议每次调 ±0.01 ~ 0.02，导出后实测")
        calf.addRow("旋转偏量修正:", self._sp_fbc)

        btn_rot_cal = QPushButton("导出旋转标定序列…")
        btn_rot_cal.clicked.connect(self._on_export_rotation_cal)
        calf.addRow("", btn_rot_cal)
        rv.addWidget(cal_group, 0)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([780, 520])
        self.setCentralWidget(splitter)

    def _build_toolbar(self) -> None:
        tb = self.addToolBar("主工具栏")
        tb.setMovable(False)
        tb.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)

        def act(text, slot, shortcut=None, tip=None):
            a = QAction(text, self)
            a.triggered.connect(slot)
            if shortcut:
                a.setShortcut(shortcut)
            if tip:
                a.setToolTip(tip)
            tb.addAction(a)
            return a

        act("📄 新建",   self.on_new,   "Ctrl+N")
        act("📂 导入",   self.on_open,  "Ctrl+O")
        act("💾 保存",   self.on_save,  "Ctrl+S")
        tb.addSeparator()

        self._act_undo = act("↩ 撤销", self.on_undo, "Ctrl+Z")
        self._act_redo = act("↪ 重做", self.on_redo, "Ctrl+Y")
        self._act_undo.setEnabled(False)
        self._act_redo.setEnabled(False)
        tb.addSeparator()

        # 底盘选择
        tb.addWidget(QLabel("底盘: "))
        self._chassis_combo = QComboBox()
        self._chassis_combo.setFixedWidth(200)
        for p in CHASSIS_PROFILES:
            self._chassis_combo.addItem(p.display_name, userData=p.profile_id)
        tb.addWidget(self._chassis_combo)
        tb.addSeparator()

        export_act = act("🚀 导出到机器人", self.on_export, "Ctrl+E")
        # 绿色高亮
        for w in tb.findChildren(QWidget):
            pass   # 样式通过 stylesheet 处理

    def _build_statusbar(self) -> None:
        self._sb_steps   = QLabel("步骤: 0")
        self._sb_dist    = QLabel("行程: 0 cm")
        self._sb_chassis = QLabel()
        self.statusBar().addWidget(self._sb_steps)
        self.statusBar().addWidget(QLabel("  |  "))
        self.statusBar().addWidget(self._sb_dist)
        self.statusBar().addPermanentWidget(self._sb_chassis)

    def _wire_signals(self) -> None:
        # 表格
        self.table.steps_changed.connect(self._on_steps_changed)
        self.table.step_selected.connect(self._on_step_selected)

        # 步骤编辑器（右侧面板改参数 → 同步到表格）
        self.step_editor.step_changed.connect(self._on_editor_step_changed)

        # 起始位姿 spinbox
        self._sp_x.valueChanged.connect(self._on_pose_changed)
        self._sp_y.valueChanged.connect(self._on_pose_changed)
        self._sp_h.valueChanged.connect(self._on_pose_changed)

        # 场地视图手柄拖动
        self.field_view.field_scene.start_handle_moved.connect(
            self._on_start_handle_moved)

        # 标定 spinbox + 反转复选框 + 偏量修正
        self._sp_tpc.valueChanged.connect(self._on_cal_spinbox_changed)
        self._sp_ttd.valueChanged.connect(self._on_cal_spinbox_changed)
        self._sp_fbc.valueChanged.connect(self._on_cal_spinbox_changed)
        self._chk_inv_vx.stateChanged.connect(self._on_cal_spinbox_changed)
        self._chk_inv_vy.stateChanged.connect(self._on_cal_spinbox_changed)
        self._chk_inv_omega.stateChanged.connect(self._on_cal_spinbox_changed)

        # 底盘切换
        self._chassis_combo.currentIndexChanged.connect(self._on_chassis_changed)

        # 按钮
        self._btn_add.clicked.connect(self._add_default_step)
        self._btn_delete.clicked.connect(self._delete_selected)
        self._btn_up.clicked.connect(lambda: self._move_selected(-1))
        self._btn_down.clicked.connect(lambda: self._move_selected(+1))
        self._btn_dup.clicked.connect(self._duplicate_selected)
        self._btn_open_cal.clicked.connect(self._open_calibration_dialog)

    def _wire_shortcuts(self) -> None:
        QShortcut(QKeySequence("Delete"),       self, self._delete_selected)
        QShortcut(QKeySequence("Backspace"),    self, self._delete_selected)
        QShortcut(QKeySequence("Ctrl+D"),       self, self._duplicate_selected)
        QShortcut(QKeySequence("Ctrl+Up"),      self, lambda: self._move_selected(-1))
        QShortcut(QKeySequence("Ctrl+Down"),    self, lambda: self._move_selected(+1))
        QShortcut(QKeySequence("Ctrl+Shift+S"), self, self.on_save_as)

    # =========================================================
    #  辅助：控件工厂
    # =========================================================
    @staticmethod
    def _make_dspin(lo: float, hi: float, dec: int,
                    suffix: str = "") -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setDecimals(dec)
        w.setSuffix(suffix)
        w.setButtonSymbols(QAbstractSpinBox.NoButtons)
        return w

    # =========================================================
    #  信号处理
    # =========================================================
    def _on_steps_changed(self) -> None:
        """表格任意编辑 → 同步右侧编辑器 + 刷新预览 + 状态栏"""
        # 若是由 _on_editor_step_changed 触发的回写，跳过重载编辑器，避免循环
        if not self._editor_syncing:
            row = self.table.current_row()
            step = self.table.current_step()
            if step is not None:
                self.step_editor.load_step(step, row)
        self._refresh_preview()
        self._refresh_status()

    def _on_step_selected(self, row: int) -> None:
        """选中行 → 步骤编辑器显示该步骤 + 预览高亮"""
        step = self.table.current_step()
        self.step_editor.load_step(step, row)
        self._refresh_preview()

    def _on_editor_step_changed(self, new_step: Step) -> None:
        """右侧编辑器修改参数 → 延迟 120ms 后单行更新表格（保持 spinbox 焦点）"""
        self._editor_pending_step = new_step
        self._editor_debounce.start()   # 已在跑则重置计时

    def _flush_editor_change(self) -> None:
        """debounce 到期后真正写回表格"""
        new_step = self._editor_pending_step
        if new_step is None:
            return
        row = self.table.current_row()
        if row < 0:
            return
        self._editor_syncing = True
        try:
            self.table.update_step_at(row, new_step)
        finally:
            self._editor_syncing = False
        self._refresh_preview()
        self._refresh_status()

    def _on_pose_changed(self) -> None:
        self._refresh_preview()

    def _on_start_handle_moved(self, x_cm: float, y_cm: float) -> None:
        """手柄拖动 → 更新起始 X/Y spinbox（不触发 _on_pose_changed 死循环）"""
        self._sp_x.blockSignals(True)
        self._sp_y.blockSignals(True)
        self._sp_x.setValue(x_cm)
        self._sp_y.setValue(y_cm)
        self._sp_x.blockSignals(False)
        self._sp_y.blockSignals(False)
        self._refresh_preview()

    def _on_cal_spinbox_changed(self) -> None:
        """标定 spinbox / 复选框修改 → 写入 _calibrations"""
        cal = self._calibrations[self._active_profile_id]
        cal.encoder_ticks_per_cm    = self._sp_tpc.value()
        cal.turn_ticks_per_deg      = self._sp_ttd.value()
        cal.invert_vx               = self._chk_inv_vx.isChecked()
        cal.invert_vy               = self._chk_inv_vy.isChecked()
        cal.invert_omega            = self._chk_inv_omega.isChecked()
        cal.front_back_compensation = self._sp_fbc.value()
        self._refresh_cal_label()

    def _on_chassis_changed(self, _: int) -> None:
        new_id = self._chassis_combo.currentData()
        if new_id == self._active_profile_id:
            return
        # 保存当前底盘的标定值（已由 _on_cal_spinbox_changed 实时同步）
        self._active_profile_id = new_id
        self._load_profile(new_id)
        self._refresh_status()
        self._refresh_preview()

    # =========================================================
    #  刷新逻辑
    # =========================================================
    def _refresh_preview(self) -> None:
        steps  = self.table.get_steps()
        start  = Pose(self._sp_x.value(), self._sp_y.value(), self._sp_h.value())
        sel    = self.table.current_row()
        self.field_view.field_scene.update_preview(steps, start, sel)
        # 同步手柄位置
        self.field_view.field_scene.set_start_pose(start.x, start.y)

    def _refresh_status(self) -> None:
        steps  = self.table.get_steps()
        dist   = estimate_total_distance(steps)
        rot    = estimate_total_rotation(steps)
        n      = len(steps)
        self._sb_steps.setText(f"步骤: {n}")
        self._sb_dist.setText(f"行程: ~{dist:.0f} cm  旋转: ~{rot:.0f}°")
        profile = get_profile(self._active_profile_id)
        self._sb_chassis.setText(f"底盘: {profile.display_name}")

        self._act_undo.setEnabled(bool(self._undo_stack))
        self._act_redo.setEnabled(bool(self._redo_stack))

    def _refresh_cal_label(self) -> None:
        cal = self._calibrations[self._active_profile_id]
        inv = []
        if cal.invert_vx:    inv.append("Vx↑")
        if cal.invert_vy:    inv.append("Vy↑")
        if cal.invert_omega: inv.append("Ω↑")
        inv_str = " ".join(inv) if inv else "—"
        self._lbl_cal.setText(
            f"线性: {cal.encoder_ticks_per_cm:.4f}  "
            f"旋转: {cal.turn_ticks_per_deg:.4f}  "
            f"补偿: {cal.front_back_compensation:+.4f}  "
            f"反转: {inv_str}"
        )

    def _load_profile(self, profile_id: str) -> None:
        """切换底盘 → 更新标定控件 + 状态栏"""
        cal = self._calibrations[profile_id]
        for w in (self._sp_tpc, self._sp_ttd, self._sp_fbc,
                  self._chk_inv_vx, self._chk_inv_vy, self._chk_inv_omega):
            w.blockSignals(True)
        self._sp_tpc.setValue(cal.encoder_ticks_per_cm)
        self._sp_ttd.setValue(cal.turn_ticks_per_deg)
        self._sp_fbc.setValue(cal.front_back_compensation)
        self._chk_inv_vx.setChecked(cal.invert_vx)
        self._chk_inv_vy.setChecked(cal.invert_vy)
        self._chk_inv_omega.setChecked(cal.invert_omega)
        for w in (self._sp_tpc, self._sp_ttd, self._sp_fbc,
                  self._chk_inv_vx, self._chk_inv_vy, self._chk_inv_omega):
            w.blockSignals(False)
        self._refresh_cal_label()

        profile = get_profile(profile_id)
        self._sb_chassis.setText(f"底盘: {profile.display_name}")

    # =========================================================
    #  步骤操作（含撤销快照）
    # =========================================================
    def _push_undo(self) -> None:
        snapshot = self.table.get_steps()
        self._undo_stack.append(snapshot)
        if len(self._undo_stack) > self._MAX_UNDO:
            self._undo_stack.pop(0)
        self._redo_stack.clear()
        self._act_undo.setEnabled(True)
        self._act_redo.setEnabled(False)

    def _add_default_step(self) -> None:
        self._push_undo()
        row = self.table.current_row()
        self.table.add_step(Step(), row)

    def _delete_selected(self) -> None:
        row = self.table.current_row()
        if row < 0:
            return
        self._push_undo()
        self.table.delete_step(row)

    def _duplicate_selected(self) -> None:
        row = self.table.current_row()
        if row < 0:
            return
        step = self.table.current_step()
        if step is None:
            return
        self._push_undo()
        self.table.add_step(copy.deepcopy(step), row)

    def _move_selected(self, delta: int) -> None:
        row = self.table.current_row()
        if row < 0:
            return
        self._push_undo()
        self.table.move_step(row, delta)

    def _reset_start_pose(self) -> None:
        self._sp_x.setValue(50.0)
        self._sp_y.setValue(50.0)
        self._sp_h.setValue(0.0)

    # =========================================================
    #  撤销 / 重做
    # =========================================================
    def on_undo(self) -> None:
        if not self._undo_stack:
            return
        self._redo_stack.append(self.table.get_steps())
        steps = self._undo_stack.pop()
        self.table.set_steps(steps)
        self._act_undo.setEnabled(bool(self._undo_stack))
        self._act_redo.setEnabled(True)

    def on_redo(self) -> None:
        if not self._redo_stack:
            return
        self._undo_stack.append(self.table.get_steps())
        steps = self._redo_stack.pop()
        self.table.set_steps(steps)
        self._act_undo.setEnabled(True)
        self._act_redo.setEnabled(bool(self._redo_stack))

    # =========================================================
    #  文件操作
    # =========================================================
    def on_new(self) -> None:
        if not self._confirm_discard():
            return
        self.table.set_steps([])
        self._current_file = None
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.setWindowTitle("步骤规划器 — MakeX 2026")
        self._refresh_status()

    def on_open(self) -> None:
        if not self._confirm_discard():
            return
        folder = str(STEP_SEQUENCES_DIR)
        path, _ = QFileDialog.getOpenFileName(
            self, "导入步骤序列", folder, "JSON 文件 (*.json)")
        if not path:
            return
        try:
            steps, cal, start_pose, profile_id, fw, fh = load_steps(Path(path))
        except Exception as e:
            QMessageBox.critical(self, "导入失败", str(e))
            return

        # 切换底盘
        for i in range(self._chassis_combo.count()):
            if self._chassis_combo.itemData(i) == profile_id:
                self._chassis_combo.blockSignals(True)
                self._chassis_combo.setCurrentIndex(i)
                self._chassis_combo.blockSignals(False)
                break
        self._active_profile_id = profile_id
        self._calibrations[profile_id] = cal

        self._field_w = fw
        self._field_h = fh
        self.field_view.field_scene.set_field_size(fw, fh)
        self._sp_x.setRange(0, fw)
        self._sp_y.setRange(0, fh)
        self._sp_x.setValue(start_pose.x)
        self._sp_y.setValue(start_pose.y)
        self._sp_h.setValue(start_pose.heading)

        self._load_profile(profile_id)
        self.table.set_steps(steps)
        self._current_file = Path(path)
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.setWindowTitle(f"步骤规划器 — {Path(path).name}")
        self._refresh_status()

    def on_save(self) -> None:
        if self._current_file:
            self._do_save(self._current_file)
        else:
            self.on_save_as()

    def on_save_as(self) -> None:
        folder = str(STEP_SEQUENCES_DIR)
        default = str(next_filename())
        path, _ = QFileDialog.getSaveFileName(
            self, "保存步骤序列", default, "JSON 文件 (*.json)")
        if not path:
            return
        self._do_save(Path(path))

    def _do_save(self, path: Path) -> None:
        cal        = self._calibrations[self._active_profile_id]
        start_pose = Pose(self._sp_x.value(), self._sp_y.value(), self._sp_h.value())
        try:
            save_steps(
                steps       = self.table.get_steps(),
                cal         = cal,
                start_pose  = start_pose,
                profile_id  = self._active_profile_id,
                field_w     = self._field_w,
                field_h     = self._field_h,
                target_path = path,
            )
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))
            return
        self._current_file = path
        self.setWindowTitle(f"步骤规划器 — {path.name}")
        self.statusBar().showMessage(f"已保存到 {path.name}", 3000)

    def _confirm_discard(self) -> bool:
        """如果有未保存内容，询问是否丢弃。"""
        if not self.table.get_steps():
            return True
        reply = QMessageBox.question(
            self, "确认",
            "当前步骤序列还未保存，继续操作将丢失更改。是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return reply == QMessageBox.Yes

    # =========================================================
    #  导出
    # =========================================================
    def on_export(self) -> None:
        steps = self.table.get_steps()
        if not steps:
            QMessageBox.warning(self, "无步骤", "请先添加至少一条步骤。")
            return

        cal     = self._calibrations[self._active_profile_id]
        profile = get_profile(self._active_profile_id)

        if cal.encoder_ticks_per_cm <= 0:
            QMessageBox.warning(self, "未标定",
                                "ENCODER_TICKS_PER_CM 为 0，请先完成线性标定。")
            return
        if cal.turn_ticks_per_deg <= 0:
            QMessageBox.warning(self, "未标定",
                                "TURN_TICKS_PER_DEG 为 0，请先完成旋转标定。")
            return

        robot_file = profile.file_path
        if not robot_file.exists():
            QMessageBox.critical(self, "找不到文件",
                                 f"机器人源文件不存在：\n{robot_file}")
            return

        sequence = build_sequence(steps, cal,
                                  use_face_rotations=(profile.profile_id == "omni3"))

        # 弹出确认对话框
        dlg = ExportConfirmDialog(sequence, robot_file, cal, self)
        if dlg.exec_() != QDialog.Accepted:
            return

        # 执行写入
        try:
            write_auto_sequence(
                sequence,
                robot_file           = robot_file,
                source_name          = str(self._current_file or "(step_planner)"),
                mode                 = "step_sequence",
                cm_per_s_at_p50      = 0.0,
                auto_power           = 0,
                encoder_based        = True,
                encoder_ticks_per_cm = cal.encoder_ticks_per_cm,
                front_back_compensation = cal.front_back_compensation,
            )
            # 追加写入 TURN_TICKS_PER_DEG（仅标定值，不写入 INVERT_* 以免影响遥控）
            text = robot_file.read_text(encoding="utf-8")
            text = upsert_float_constant(text, "TURN_TICKS_PER_DEG", cal.turn_ticks_per_deg)
            robot_file.write_text(text, encoding="utf-8")
        except Exception as e:
            QMessageBox.critical(self, "导出失败", str(e))
            return

        QMessageBox.information(
            self, "导出成功",
            f"已成功写入 {robot_file.name}\n"
            f"共 {len(sequence)} 条指令（含末尾 enc_stop）。\n"
            f"原文件已备份为 {robot_file.name}.bak"
        )
        self.statusBar().showMessage(f"导出成功 → {robot_file.name}", 5000)

    # =========================================================
    #  旋转标定导出
    # =========================================================
    def _on_export_rotation_cal(self) -> None:
        """生成一条旋转标定序列并写入机器人文件。

        序列内容：enc_rot 1000 ticks → enc_stop
        写入后烧录，量实际转角，再用标定向导录入结果。
        """
        profile    = get_profile(self._active_profile_id)
        robot_file = profile.file_path
        if not robot_file.exists():
            QMessageBox.critical(self, "找不到文件",
                                 f"机器人源文件不存在：\n{robot_file}")
            return

        cal_seq = [("enc_rot", 1000, 0, 50), ("enc_stop", 0, 0, 0)]

        reply = QMessageBox.question(
            self, "导出旋转标定序列",
            f"将向 {robot_file.name} 写入：\n"
            f"  ('enc_rot', 1000, 0, 50)\n"
            f"  ('enc_stop', 0, 0, 0)\n\n"
            "烧录后按 + 键运行，机器人会原地旋转，\n"
            "用量角器量实际角度，然后在「标定向导→旋转标定」页录入。\n\n"
            "继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if reply != QMessageBox.Yes:
            return

        try:
            write_auto_sequence(
                cal_seq,
                robot_file      = robot_file,
                source_name     = "(rotation_calibration)",
                mode            = "rotation_cal",
                cm_per_s_at_p50 = 0.0,
                auto_power      = 0,
                encoder_based   = True,
            )
        except Exception as e:
            QMessageBox.critical(self, "导出失败", str(e))
            return

        QMessageBox.information(
            self, "已导出",
            f"旋转标定序列已写入 {robot_file.name}。\n"
            "请烧录后运行，量角度，再用「标定向导」录入结果。"
        )

    # =========================================================
    #  标定向导
    # =========================================================
    def _open_calibration_dialog(self) -> None:
        cal = self._calibrations[self._active_profile_id]
        dlg = CalibrationDialog(cal, self)
        if dlg.exec_() == QDialog.Accepted:
            new_cal = dlg.result_calibration()
            self._calibrations[self._active_profile_id] = new_cal
            self._load_profile(self._active_profile_id)
            self.statusBar().showMessage("标定值已更新", 3000)
