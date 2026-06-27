"""主窗口：工具栏 + 画布 + 右侧控制面板"""
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets

from .config import (
    DEFAULT_FIELD_WIDTH_CM,
    DEFAULT_FIELD_HEIGHT_CM,
    DEFAULT_CM_PER_SEC_AT_P50,
    DEFAULT_DEG_PER_SEC_AT_OMEGA50,
    DEFAULT_AUTO_POWER,
    DEFAULT_OMEGA_POWER,
    DEFAULT_SMOOTH_ITER,
    DEFAULT_RESAMPLE_CM,
    DEFAULT_INVERT_X,
    DEFAULT_INVERT_Y,
    DEFAULT_RAMP_MS,
    DEFAULT_DRIFT_LEFT_OMEGA,
    DEFAULT_DRIFT_RIGHT_OMEGA,
    MODE_TRANSLATION,
    MODE_HEADING,
    POWER_MIN,
    POWER_MAX,
    TRAJECTORIES_DIR,
    ROBOT_FILE,
    STOP_BUFFER,
)
from .field_view import FieldScene, FieldView
from .kinematics import build_sequence
from .file_io import build_payload, save_trajectory, load_trajectory, next_filename
from .exporter import write_auto_sequence
from .obstacle_item import ObstacleItem


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("轨迹规划器 — 三轮全向机器人")
        self.resize(1280, 800)
        self._current_file = None

        self.scene = FieldScene(DEFAULT_FIELD_WIDTH_CM, DEFAULT_FIELD_HEIGHT_CM)
        self.view = FieldView(self.scene)
        self.scene.smooth_iter = DEFAULT_SMOOTH_ITER
        self.scene.resample_step = DEFAULT_RESAMPLE_CM

        self.setCentralWidget(self.view)

        self._build_toolbar()
        self._build_dock()
        self._build_statusbar()
        self._wire_signals()
        self._refresh_title()

    # ---- 工具栏 ----
    def _build_toolbar(self):
        tb = self.addToolBar("主工具栏")
        tb.setMovable(False)
        tb.setIconSize(QtCore.QSize(20, 20))

        act_new = tb.addAction("新建")
        act_new.triggered.connect(self.on_new)

        act_open = tb.addAction("导入")
        act_open.triggered.connect(self.on_open)

        act_save = tb.addAction("保存")
        act_save.triggered.connect(self.on_save)

        tb.addSeparator()

        act_export = tb.addAction("导出到机器人")
        act_export.triggered.connect(self.on_export)

        act_calib = tb.addAction("标定测试段")
        act_calib.triggered.connect(self.on_calibration_export)

        tb.addSeparator()

        act_add_obs = tb.addAction("加障碍物")
        act_add_obs.triggered.connect(self.on_add_obstacle)

        act_del = tb.addAction("删除选中")
        act_del.triggered.connect(self.on_delete_selected)

        act_clear_path = tb.addAction("清空轨迹")
        act_clear_path.triggered.connect(self.on_clear_path)

    # ---- 右侧控件 ----
    def _build_dock(self):
        dock = QtWidgets.QDockWidget("控制面板", self)
        dock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea | QtCore.Qt.LeftDockWidgetArea)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

        container = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(container)
        outer.setContentsMargins(8, 8, 8, 8)

        # --- 场地 ---
        g_field = QtWidgets.QGroupBox("场地尺寸 (cm)")
        f_field = QtWidgets.QFormLayout(g_field)
        self.sp_field_w = QtWidgets.QDoubleSpinBox()
        self.sp_field_w.setRange(50, 1500); self.sp_field_w.setDecimals(1)
        self.sp_field_w.setValue(DEFAULT_FIELD_WIDTH_CM); self.sp_field_w.setSuffix(" cm")
        self.sp_field_h = QtWidgets.QDoubleSpinBox()
        self.sp_field_h.setRange(50, 1500); self.sp_field_h.setDecimals(1)
        self.sp_field_h.setValue(DEFAULT_FIELD_HEIGHT_CM); self.sp_field_h.setSuffix(" cm")
        f_field.addRow("宽:", self.sp_field_w)
        f_field.addRow("高:", self.sp_field_h)
        outer.addWidget(g_field)

        # --- 速度标定 ---
        g_cal = QtWidgets.QGroupBox("速度标定")
        f_cal = QtWidgets.QFormLayout(g_cal)
        self.sp_cm_per_s = QtWidgets.QDoubleSpinBox()
        self.sp_cm_per_s.setRange(1, 300); self.sp_cm_per_s.setDecimals(2)
        self.sp_cm_per_s.setValue(DEFAULT_CM_PER_SEC_AT_P50); self.sp_cm_per_s.setSuffix(" cm/s")
        self.sp_deg_per_s = QtWidgets.QDoubleSpinBox()
        self.sp_deg_per_s.setRange(5, 720); self.sp_deg_per_s.setDecimals(1)
        self.sp_deg_per_s.setValue(DEFAULT_DEG_PER_SEC_AT_OMEGA50); self.sp_deg_per_s.setSuffix(" deg/s")
        f_cal.addRow("功率50时:", self.sp_cm_per_s)
        f_cal.addRow("ω50时:", self.sp_deg_per_s)
        outer.addWidget(g_cal)

        # --- 运动参数 ---
        g_motion = QtWidgets.QGroupBox("运动参数")
        f_motion = QtWidgets.QFormLayout(g_motion)
        self.sp_auto_power = QtWidgets.QSpinBox()
        self.sp_auto_power.setRange(POWER_MIN, POWER_MAX); self.sp_auto_power.setValue(DEFAULT_AUTO_POWER)
        self.sp_omega_power = QtWidgets.QSpinBox()
        self.sp_omega_power.setRange(POWER_MIN, POWER_MAX); self.sp_omega_power.setValue(DEFAULT_OMEGA_POWER)
        self.btn_mode = QtWidgets.QPushButton("模式: 纯平移")
        self.btn_mode.setCheckable(True)
        self.btn_mode.clicked.connect(self._on_toggle_mode)
        self.chk_invert_x = QtWidgets.QCheckBox("反转 X 轴（左右装反时勾选）")
        self.chk_invert_x.setChecked(DEFAULT_INVERT_X)
        self.chk_invert_y = QtWidgets.QCheckBox("反转 Y 轴（前后装反时勾选）")
        self.chk_invert_y.setChecked(DEFAULT_INVERT_Y)
        f_motion.addRow("移动功率:", self.sp_auto_power)
        f_motion.addRow("旋转功率:", self.sp_omega_power)
        f_motion.addRow(self.btn_mode)
        f_motion.addRow(self.chk_invert_x)
        f_motion.addRow(self.chk_invert_y)
        outer.addWidget(g_motion)

        # --- 执行端：速度过渡 ---
        g_ramp = QtWidgets.QGroupBox("速度过渡（写入机器人 AUTO_RAMP_MS）")
        f_ramp = QtWidgets.QFormLayout(g_ramp)
        self.sp_ramp_ms = QtWidgets.QSpinBox()
        self.sp_ramp_ms.setRange(0, 1000); self.sp_ramp_ms.setSuffix(" ms")
        self.sp_ramp_ms.setValue(DEFAULT_RAMP_MS)
        self.sp_ramp_ms.setSingleStep(20)
        self.sp_ramp_ms.setToolTip("步间速度线性插值时长。0=立即切换；100ms=常用；200ms=最软")
        f_ramp.addRow("插值时长:", self.sp_ramp_ms)
        outer.addWidget(g_ramp)

        # --- 漂移补偿 ---
        g_drift = QtWidgets.QGroupBox("平移漂移补偿 (omega)")
        f_drift = QtWidgets.QFormLayout(g_drift)
        self.sp_drift_left = QtWidgets.QSpinBox()
        self.sp_drift_left.setRange(-30, 30); self.sp_drift_left.setValue(DEFAULT_DRIFT_LEFT_OMEGA)
        self.sp_drift_left.setToolTip(
            "向左平移时机身若往右偏，填正数（+omega 顺时针补偿）。\n"
            "强度按 |Vx|/auto_power 线性缩放。一般 1~5 起调。")
        self.sp_drift_right = QtWidgets.QSpinBox()
        self.sp_drift_right.setRange(-30, 30); self.sp_drift_right.setValue(DEFAULT_DRIFT_RIGHT_OMEGA)
        self.sp_drift_right.setToolTip(
            "向右平移时机身若往左偏，填负数；往右偏，填正数。\n"
            "强度按 |Vx|/auto_power 线性缩放。")
        f_drift.addRow("左移补偿:", self.sp_drift_left)
        f_drift.addRow("右移补偿:", self.sp_drift_right)
        outer.addWidget(g_drift)

        # --- 平滑 ---
        g_smooth = QtWidgets.QGroupBox("平滑参数")
        f_smooth = QtWidgets.QFormLayout(g_smooth)
        self.sp_smooth = QtWidgets.QSpinBox()
        self.sp_smooth.setRange(0, 6); self.sp_smooth.setValue(DEFAULT_SMOOTH_ITER)
        self.sp_resample = QtWidgets.QDoubleSpinBox()
        self.sp_resample.setRange(1, 50); self.sp_resample.setDecimals(1)
        self.sp_resample.setValue(DEFAULT_RESAMPLE_CM); self.sp_resample.setSuffix(" cm")
        btn_resmooth = QtWidgets.QPushButton("重新平滑")
        btn_resmooth.clicked.connect(self.on_resmooth)
        f_smooth.addRow("迭代次数:", self.sp_smooth)
        f_smooth.addRow("采样间隔:", self.sp_resample)
        f_smooth.addRow(btn_resmooth)
        outer.addWidget(g_smooth)

        # --- 障碍物选中信息 ---
        g_obs = QtWidgets.QGroupBox("选中的障碍物")
        f_obs = QtWidgets.QFormLayout(g_obs)
        self.sp_obs_w = QtWidgets.QDoubleSpinBox()
        self.sp_obs_w.setRange(2, 1000); self.sp_obs_w.setDecimals(1)
        self.sp_obs_w.setSuffix(" cm")
        self.sp_obs_h = QtWidgets.QDoubleSpinBox()
        self.sp_obs_h.setRange(2, 1000); self.sp_obs_h.setDecimals(1)
        self.sp_obs_h.setSuffix(" cm")
        self.sp_obs_w.setEnabled(False)
        self.sp_obs_h.setEnabled(False)
        f_obs.addRow("宽:", self.sp_obs_w)
        f_obs.addRow("高:", self.sp_obs_h)
        outer.addWidget(g_obs)

        # --- 提示 ---
        self.lbl_face = QtWidgets.QLabel(
            "⚠ 提示：\n"
            "1) 运行前确认机器人在 Face 0（M1-M2 边朝前）\n"
            "2) 任何参数改了之后必须点「导出到机器人」才会生效\n"
            "3) 漂移补偿是在 GUI 视角下：勾不勾反转都按你画图时的左右"
        )
        self.lbl_face.setWordWrap(True)
        self.lbl_face.setStyleSheet("color: #b56500; padding:6px; background:#fff7e0; border:1px solid #e0c080; border-radius:4px;")
        outer.addWidget(self.lbl_face)

        outer.addStretch(1)
        dock.setWidget(container)
        dock.setMinimumWidth(280)

    def _build_statusbar(self):
        self.statusBar().showMessage("就绪 — 在画布上按住左键画轨迹")

    def _wire_signals(self):
        self.sp_field_w.valueChanged.connect(self._on_field_size_changed)
        self.sp_field_h.valueChanged.connect(self._on_field_size_changed)
        self.scene.selectionChanged.connect(self._on_selection_changed)
        self.scene.path_finalized.connect(self._on_path_finalized)
        self.sp_obs_w.valueChanged.connect(self._on_obs_size_changed)
        self.sp_obs_h.valueChanged.connect(self._on_obs_size_changed)

    # ---- 状态同步 ----
    def _on_field_size_changed(self, *_):
        self.scene.set_field_size(self.sp_field_w.value(), self.sp_field_h.value())
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _on_path_finalized(self):
        n_raw = len(self.scene.path_item.raw_points)
        n_smooth = len(self.scene.path_item.smoothed_points)
        self.statusBar().showMessage("绘制完成 — 原始 %d 点 / 平滑 %d 点" % (n_raw, n_smooth))

    def _on_selection_changed(self):
        sel = [it for it in self.scene.selectedItems() if isinstance(it, ObstacleItem)]
        if len(sel) == 1:
            obs = sel[0]
            _, _, w, h = obs.rect_scene()
            self.sp_obs_w.blockSignals(True); self.sp_obs_h.blockSignals(True)
            self.sp_obs_w.setValue(w)
            self.sp_obs_h.setValue(h)
            self.sp_obs_w.blockSignals(False); self.sp_obs_h.blockSignals(False)
            self.sp_obs_w.setEnabled(True)
            self.sp_obs_h.setEnabled(True)
        else:
            self.sp_obs_w.setEnabled(False)
            self.sp_obs_h.setEnabled(False)

    def _on_obs_size_changed(self, *_):
        sel = [it for it in self.scene.selectedItems() if isinstance(it, ObstacleItem)]
        if len(sel) != 1:
            return
        sel[0].set_size(self.sp_obs_w.value(), self.sp_obs_h.value())

    def _on_toggle_mode(self):
        if self.btn_mode.isChecked():
            self.btn_mode.setText("模式: 车头跟随")
        else:
            self.btn_mode.setText("模式: 纯平移")

    def _current_mode(self):
        return MODE_HEADING if self.btn_mode.isChecked() else MODE_TRANSLATION

    # ---- 工具栏动作 ----
    def on_new(self):
        if not self._confirm("新建会清空当前轨迹与障碍物，确定？"):
            return
        self.scene.clear_path()
        self.scene.clear_obstacles()
        self._current_file = None
        self._refresh_title()
        self.statusBar().showMessage("已新建")

    def on_save(self):
        path = self.scene.path_item
        if len(path.raw_points) < 2:
            QtWidgets.QMessageBox.information(self, "保存", "还没有画轨迹，无法保存。")
            return

        payload = self._build_current_payload()
        target = next_filename()
        try:
            saved = save_trajectory(payload, target)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "保存失败", str(e))
            return
        self._current_file = saved
        self._refresh_title()
        self.statusBar().showMessage("已保存: %s" % saved)

    def on_open(self):
        TRAJECTORIES_DIR.mkdir(parents=True, exist_ok=True)
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入轨迹", str(TRAJECTORIES_DIR), "轨迹文件 (*.json)"
        )
        if not path:
            return
        try:
            data = load_trajectory(path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "导入失败", str(e))
            return
        self._apply_payload(data)
        self._current_file = Path(path)
        self._refresh_title()
        self.statusBar().showMessage("已导入: %s" % path)

    def on_export(self):
        path = self.scene.path_item
        if len(path.smoothed_points) < 2:
            QtWidgets.QMessageBox.information(self, "导出", "请先画一条轨迹。")
            return
        sequence = build_sequence(
            path.smoothed_points,
            self._current_mode(),
            self.sp_cm_per_s.value(),
            self.sp_deg_per_s.value(),
            self.sp_auto_power.value(),
            self.sp_omega_power.value(),
            invert_x=self.chk_invert_x.isChecked(),
            invert_y=self.chk_invert_y.isChecked(),
            drift_left_omega=self.sp_drift_left.value(),
            drift_right_omega=self.sp_drift_right.value(),
        )
        if not self._confirm_export(sequence):
            return
        try:
            backup, _ = write_auto_sequence(
                sequence,
                source_name=str(self._current_file or "(unsaved)"),
                mode=self._current_mode(),
                cm_per_s_at_p50=self.sp_cm_per_s.value(),
                auto_power=self.sp_auto_power.value(),
                ramp_ms=self.sp_ramp_ms.value(),
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "导出失败", str(e))
            return
        QtWidgets.QMessageBox.information(
            self, "导出成功",
            "已写入 %s\n共 %d 步 | AUTO_RAMP_MS=%d\n备份: %s" % (
                ROBOT_FILE, len(sequence), self.sp_ramp_ms.value(), backup.name),
        )
        self.statusBar().showMessage("导出完成 — %d 步" % len(sequence))

    def on_calibration_export(self):
        """生成 2 秒前进的测试段，让用户实测距离反算速度"""
        if not self._confirm(
            "将向 mecanum_forward.py 写入一段标定测试：\n"
            "  (2.0s, 0, 50, 0) — 前进 2 秒\n"
            "  (0.1s, 0, 0, 0)  — 停止\n\n"
            "运行机器人，用尺子量实际走的距离 D（cm），\n"
            "然后在「功率50时」填入 D / 2.0。\n\n确认写入？"
        ):
            return
        seq = [(2.0, 0, 50, 0), STOP_BUFFER]
        try:
            backup, _ = write_auto_sequence(
                seq, source_name="(calibration)",
                mode="translation",
                cm_per_s_at_p50=self.sp_cm_per_s.value(),
                auto_power=50,
                ramp_ms=0,  # 标定时禁用插值，量到的就是纯净速度
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "写入失败", str(e))
            return
        QtWidgets.QMessageBox.information(
            self, "标定写入成功",
            "已写入 %s\n备份: %s\nAUTO_RAMP_MS 已设为 0（标定时不插值）\n\n"
            "烧录后按 + 键运行，量距离反推 cm/s。" % (ROBOT_FILE, backup.name),
        )

    def on_add_obstacle(self):
        size = max(20.0, min(self.scene.field_size()) * 0.1)
        obs = self.scene.add_obstacle(w=size, h=size)
        self.scene.clearSelection()
        obs.setSelected(True)

    def on_delete_selected(self):
        for it in list(self.scene.selectedItems()):
            if isinstance(it, ObstacleItem):
                self.scene.remove_obstacle(it)

    def on_clear_path(self):
        if not self._confirm("清空当前轨迹？"):
            return
        self.scene.clear_path()

    def on_resmooth(self):
        self.scene.resmooth(self.sp_smooth.value(), self.sp_resample.value())
        self._on_path_finalized()

    # ---- 数据交换 ----
    def _build_current_payload(self):
        raw = self.scene.path_item.raw_points
        smooth = self.scene.path_item.smoothed_points
        w, h = self.scene.field_size()
        calibration = {
            "cm_per_second_at_power_50": self.sp_cm_per_s.value(),
            "deg_per_second_at_omega_50": self.sp_deg_per_s.value(),
            "auto_power": self.sp_auto_power.value(),
            "omega_power": self.sp_omega_power.value(),
            "drift_left_omega": self.sp_drift_left.value(),
            "drift_right_omega": self.sp_drift_right.value(),
        }
        settings = {
            "mode": self._current_mode(),
            "smoothing_iterations": self.sp_smooth.value(),
            "resample_step_cm": self.sp_resample.value(),
            "invert_x": self.chk_invert_x.isChecked(),
            "invert_y": self.chk_invert_y.isChecked(),
            "ramp_ms": self.sp_ramp_ms.value(),
        }
        obstacles = [o.to_dict() for o in self.scene.obstacles()]
        return build_payload(w, h, calibration, settings, raw, smooth, obstacles)

    def _apply_payload(self, data):
        field = data.get("field", {})
        self.sp_field_w.setValue(float(field.get("width_cm", DEFAULT_FIELD_WIDTH_CM)))
        self.sp_field_h.setValue(float(field.get("height_cm", DEFAULT_FIELD_HEIGHT_CM)))

        cal = data.get("calibration", {})
        self.sp_cm_per_s.setValue(float(cal.get("cm_per_second_at_power_50", DEFAULT_CM_PER_SEC_AT_P50)))
        self.sp_deg_per_s.setValue(float(cal.get("deg_per_second_at_omega_50", DEFAULT_DEG_PER_SEC_AT_OMEGA50)))
        self.sp_auto_power.setValue(int(cal.get("auto_power", DEFAULT_AUTO_POWER)))
        self.sp_omega_power.setValue(int(cal.get("omega_power", DEFAULT_OMEGA_POWER)))
        self.sp_drift_left.setValue(int(cal.get("drift_left_omega", DEFAULT_DRIFT_LEFT_OMEGA)))
        self.sp_drift_right.setValue(int(cal.get("drift_right_omega", DEFAULT_DRIFT_RIGHT_OMEGA)))

        settings = data.get("settings", {})
        mode = settings.get("mode", MODE_TRANSLATION)
        self.btn_mode.setChecked(mode == MODE_HEADING)
        self._on_toggle_mode()
        self.sp_smooth.setValue(int(settings.get("smoothing_iterations", DEFAULT_SMOOTH_ITER)))
        self.sp_resample.setValue(float(settings.get("resample_step_cm", DEFAULT_RESAMPLE_CM)))
        self.chk_invert_x.setChecked(bool(settings.get("invert_x", DEFAULT_INVERT_X)))
        self.chk_invert_y.setChecked(bool(settings.get("invert_y", DEFAULT_INVERT_Y)))
        self.sp_ramp_ms.setValue(int(settings.get("ramp_ms", DEFAULT_RAMP_MS)))

        self.scene.clear_path()
        self.scene.clear_obstacles()

        path = data.get("path", {})
        raw = [tuple(p) for p in path.get("raw_points_cm", [])]
        smooth = [tuple(p) for p in path.get("smoothed_points_cm", [])]
        if raw:
            self.scene.path_item.set_points(raw, smooth or raw)
            self.scene._refresh_collision()

        for od in data.get("obstacles", []):
            self.scene.add_obstacle_item(ObstacleItem.from_dict(od))

    # ---- 杂项 ----
    def _refresh_title(self):
        base = "轨迹规划器"
        if self._current_file:
            self.setWindowTitle("%s — %s" % (base, Path(self._current_file).name))
        else:
            self.setWindowTitle(base + " — (未命名)")

    def _confirm(self, text):
        return QtWidgets.QMessageBox.question(
            self, "确认", text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        ) == QtWidgets.QMessageBox.Yes

    def _confirm_export(self, sequence):
        # 统计补偿/反转是否生效
        nonzero_omega = sum(1 for d, vx, vy, w in sequence if w != 0)
        max_abs_omega = max((abs(w) for d, vx, vy, w in sequence), default=0)

        flags = []
        if self.chk_invert_x.isChecked():
            flags.append("✓ 反转 X 轴")
        if self.chk_invert_y.isChecked():
            flags.append("✓ 反转 Y 轴")
        if self.sp_drift_left.value() or self.sp_drift_right.value():
            flags.append("✓ 漂移补偿 L=%+d R=%+d" % (
                self.sp_drift_left.value(), self.sp_drift_right.value()))
        if self.sp_ramp_ms.value() > 0:
            flags.append("✓ 速度过渡 %d ms" % self.sp_ramp_ms.value())
        flags_text = "\n  ".join(flags) if flags else "（无补偿/反转，原样导出）"

        # 预览前几步
        preview = "\n".join(
            "  (%5.2fs, Vx=%4d, Vy=%4d, ω=%4d)" % (d, vx, vy, w)
            for d, vx, vy, w in sequence[:8]
        )
        if len(sequence) > 8:
            preview += "\n  ... (共 %d 步)" % len(sequence)

        msg = (
            "当前补偿设置：\n  " + flags_text + "\n\n"
            + "导出序列：%d 步\n" % len(sequence)
            + "其中带 ω 旋转的步: %d 步（最大 |ω|=%d）\n\n" % (nonzero_omega, max_abs_omega)
            + "前 8 步预览：\n" + preview + "\n\n"
            + "确认补偿已生效后，点 Yes 写入 mecanum_forward.py（自动 .bak 备份）"
        )
        return self._confirm(msg)
