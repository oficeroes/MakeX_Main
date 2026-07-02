"""主窗口：工具栏 + 画布 + 右侧控制面板

职责
====
MainWindow 是整个 GUI 的控制中心。它把各个子模块（field_view / kinematics /
file_io / exporter）串联起来，处理用户交互并在它们之间传递数据。

控件布局
========
  工具栏（顶部）：新建 / 导入 / 保存 / 导出到机器人 / 标定测试段 / 加障碍物 / 删除选中 / 清空轨迹
  画布（中央）  ：FieldView（见 field_view.py）
  控制面板（右侧 QDockWidget）：
    ┌─ 底盘型号    QComboBox（从 CHASSIS_PROFILES 动态填充）
    ├─ 场地尺寸    宽 / 高（cm）
    ├─ 速度标定    功率50时速度 / omega50时角速度
    ├─ 运动参数    移动功率 / 旋转功率 / 模式切换 / 反转 X / 反转 Y
    ├─ 速度过渡    AUTO_RAMP_MS（写入机器人）
    ├─ 漂移补偿    左移 omega 补偿 / 右移 omega 补偿
    ├─ 平滑参数    Chaikin 迭代次数 / 等弧长采样间隔 / 重新平滑
    ├─ 障碍物信息  选中障碍物的宽 / 高（联动编辑）
    └─ 提示标签    随底盘切换动态更新（face 概念 / 目标文件名）

底盘切换
========
combo_chassis 的 userData 存 profile_id（字符串）。
切换时 _on_chassis_changed 刷新描述标签和提示标签。
导出时 _current_profile().file_path 决定写入哪个文件。
底盘 profile_id 随轨迹 JSON 一起存储和恢复（settings.chassis_profile_id）。

数据流
======
  绘制 → scene.path_item.smoothed_points
       → build_sequence(points, mode, 标定值, 反转/补偿参数)
       → write_auto_sequence(sequence, robot_file=当前 profile 路径, ramp_ms=...)

  导出确认对话框（_confirm_export）显示活跃的补偿/反转标志 + 前 8 步预览，
  让用户在写入前确认参数已正确配置。

注意
====
- _build_current_payload / _apply_payload 完整保存/恢复所有控件状态（含 chassis_profile_id）。
- on_calibration_export 强制 ramp_ms=0，使标定测量结果不受插值干扰。
- on_export 中 ROBOT_FILE（硬编码别名）已弃用，请始终通过 _current_profile().file_path。
- 漂移补偿在 GUI 视角下有意义（不受反转影响），详见 kinematics.py 的注释。
"""
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
    CHASSIS_PROFILES,
    DEFAULT_PROFILE_ID,
    get_profile,
)
from .field_view import FieldScene, FieldView
from .kinematics import build_sequence
from .file_io import build_payload, save_trajectory, load_trajectory, next_filename
from .exporter import write_auto_sequence
from .obstacle_item import ObstacleItem
from .action_block_item import ActionBlockItem


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("轨迹规划器")
        self.resize(1280, 800)
        self._current_file = None
        self._undo_stack = []   # list of payload dicts (最多 30 步)
        self._redo_stack = []

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
        tb.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        tb.setIconSize(QtCore.QSize(20, 20))

        # 文件操作
        act_new = tb.addAction("📄 新建")
        act_new.setToolTip("清空画布，新建轨迹 (Ctrl+N)")
        act_new.setShortcut("Ctrl+N")
        act_new.triggered.connect(self.on_new)

        act_open = tb.addAction("📂 导入")
        act_open.setToolTip("从 JSON 文件导入轨迹 (Ctrl+O)")
        act_open.setShortcut("Ctrl+O")
        act_open.triggered.connect(self.on_open)

        act_save = tb.addAction("💾 保存")
        act_save.setToolTip("保存轨迹到 JSON (Ctrl+S)")
        act_save.setShortcut("Ctrl+S")
        act_save.triggered.connect(self.on_save)

        tb.addSeparator()

        # 撤销 / 重做
        self._act_undo = tb.addAction("↩ 撤销")
        self._act_undo.setToolTip("撤销上一步操作 (Ctrl+Z)")
        self._act_undo.setShortcut("Ctrl+Z")
        self._act_undo.setEnabled(False)
        self._act_undo.triggered.connect(self.on_undo)

        self._act_redo = tb.addAction("↪ 重做")
        self._act_redo.setToolTip("重做已撤销的操作 (Ctrl+Y)")
        self._act_redo.setShortcut("Ctrl+Y")
        self._act_redo.setEnabled(False)
        self._act_redo.triggered.connect(self.on_redo)

        tb.addSeparator()

        # 底盘选择（直接放工具栏）
        lbl_chassis = QtWidgets.QLabel("  底盘: ")
        lbl_chassis.setStyleSheet("font-weight: bold;")
        tb.addWidget(lbl_chassis)
        self.combo_chassis = QtWidgets.QComboBox()
        self.combo_chassis.setMinimumWidth(180)
        self.combo_chassis.setToolTip("选择当前机器人底盘型号，影响导出目标文件")
        for p in CHASSIS_PROFILES:
            self.combo_chassis.addItem(p.display_name, userData=p.profile_id)
        default_idx = next(
            (i for i, p in enumerate(CHASSIS_PROFILES) if p.profile_id == DEFAULT_PROFILE_ID), 0
        )
        self.combo_chassis.setCurrentIndex(default_idx)
        self.combo_chassis.currentIndexChanged.connect(self._on_chassis_changed)
        tb.addWidget(self.combo_chassis)

        tb.addSeparator()

        # 导出（视觉强调）
        act_export = tb.addAction("🚀 导出到机器人")
        act_export.setToolTip("将轨迹转换为 AUTO_SEQUENCE 并写入机器人源文件 (Ctrl+E)")
        act_export.setShortcut("Ctrl+E")
        act_export.triggered.connect(self.on_export)
        # 让导出按钮加粗显示
        for child in tb.children():
            if isinstance(child, QtWidgets.QToolButton) and child.defaultAction() == act_export:
                f = child.font(); f.setBold(True); child.setFont(f)
                child.setStyleSheet("QToolButton { color: #1a6b1a; font-weight: bold; }")
                break

        act_calib = tb.addAction("📏 标定测试")
        act_calib.setToolTip("导出 2 秒直走测试段，用尺子量距离反推速度标定值")
        act_calib.triggered.connect(self.on_calibration_export)

        tb.addSeparator()

        # 画布操作
        act_add_obs = tb.addAction("⬛ 加障碍物")
        act_add_obs.setToolTip("在场地中央添加一个可拖拽障碍物方块")
        act_add_obs.triggered.connect(self.on_add_obstacle)

        act_del = tb.addAction("🗑 删除选中")
        act_del.setToolTip("删除选中的路段 / 障碍物 / 执行块 (Del)")
        act_del.setShortcut("Del")
        act_del.triggered.connect(self.on_delete_selected)

        act_clear_path = tb.addAction("✕ 清空轨迹")
        act_clear_path.setToolTip("清除当前画的轨迹（不影响障碍物）")
        act_clear_path.triggered.connect(self.on_clear_path)

    # ---- 右侧控件 ----
    def _build_dock(self):
        dock = QtWidgets.QDockWidget("参数面板", self)
        dock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea | QtCore.Qt.LeftDockWidgetArea)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

        # 滚动区域：小屏不截断
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        container = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(container)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        # --- 底盘描述（工具栏里已有选择器，这里只显示描述） ---
        self.lbl_chassis_desc = QtWidgets.QLabel()
        self.lbl_chassis_desc.setWordWrap(True)
        self.lbl_chassis_desc.setStyleSheet(
            "color: #444; font-size: 11px; padding: 4px 6px;"
            "background: #f0f4ff; border: 1px solid #c8d4f0; border-radius: 4px;"
        )
        outer.addWidget(self.lbl_chassis_desc)

        # --- 场地尺寸 ---
        g_field = QtWidgets.QGroupBox("场地尺寸")
        f_field = QtWidgets.QFormLayout(g_field)
        f_field.setRowWrapPolicy(QtWidgets.QFormLayout.DontWrapRows)
        self.sp_field_w = QtWidgets.QDoubleSpinBox()
        self.sp_field_w.setRange(50, 1500); self.sp_field_w.setDecimals(0)
        self.sp_field_w.setValue(DEFAULT_FIELD_WIDTH_CM); self.sp_field_w.setSuffix(" cm")
        self.sp_field_h = QtWidgets.QDoubleSpinBox()
        self.sp_field_h.setRange(50, 1500); self.sp_field_h.setDecimals(0)
        self.sp_field_h.setValue(DEFAULT_FIELD_HEIGHT_CM); self.sp_field_h.setSuffix(" cm")
        f_field.addRow("宽:", self.sp_field_w)
        f_field.addRow("高:", self.sp_field_h)
        outer.addWidget(g_field)

        # --- 运动参数（合并最重要的参数） ---
        g_motion = QtWidgets.QGroupBox("运动参数")
        f_motion = QtWidgets.QFormLayout(g_motion)

        self.sp_auto_power = QtWidgets.QSpinBox()
        self.sp_auto_power.setRange(POWER_MIN, POWER_MAX)
        self.sp_auto_power.setValue(DEFAULT_AUTO_POWER)
        self.sp_auto_power.setToolTip("自动程序平移功率（%）。功率越高越快，越难控制精度")
        f_motion.addRow("移动功率:", self.sp_auto_power)

        self.sp_omega_power = QtWidgets.QSpinBox()
        self.sp_omega_power.setRange(POWER_MIN, POWER_MAX)
        self.sp_omega_power.setValue(DEFAULT_OMEGA_POWER)
        self.sp_omega_power.setToolTip("车头跟随模式时的旋转功率（%）")
        f_motion.addRow("旋转功率:", self.sp_omega_power)

        # 模式按钮（颜色区分）
        self.btn_mode = QtWidgets.QPushButton("模式: 纯平移")
        self.btn_mode.setCheckable(True)
        self.btn_mode.setToolTip(
            "纯平移：车头固定，不跟随路径方向（推荐）\n"
            "车头跟随：机器人自动旋转对准行进方向（需要额外标定 ω 速度）"
        )
        self.btn_mode.clicked.connect(self._on_toggle_mode)
        f_motion.addRow(self.btn_mode)

        # 轴反转（两个 checkbox 放一行）
        inv_row = QtWidgets.QWidget()
        inv_h = QtWidgets.QHBoxLayout(inv_row)
        inv_h.setContentsMargins(0, 0, 0, 0)
        self.chk_invert_x = QtWidgets.QCheckBox("反转 X")
        self.chk_invert_x.setToolTip("勾选：所有 Vx 取反（左右安装方向反了时用）")
        self.chk_invert_x.setChecked(DEFAULT_INVERT_X)
        self.chk_invert_y = QtWidgets.QCheckBox("反转 Y")
        self.chk_invert_y.setToolTip("勾选：所有 Vy 取反（前后安装方向反了时用）")
        self.chk_invert_y.setChecked(DEFAULT_INVERT_Y)
        inv_h.addWidget(self.chk_invert_x)
        inv_h.addWidget(self.chk_invert_y)
        inv_h.addStretch()
        f_motion.addRow("轴反转:", inv_row)
        outer.addWidget(g_motion)

        # --- 速度标定 ---
        g_cal = QtWidgets.QGroupBox("速度标定")
        f_cal = QtWidgets.QFormLayout(g_cal)
        self.sp_cm_per_s = QtWidgets.QDoubleSpinBox()
        self.sp_cm_per_s.setRange(1, 300); self.sp_cm_per_s.setDecimals(1)
        self.sp_cm_per_s.setValue(DEFAULT_CM_PER_SEC_AT_P50); self.sp_cm_per_s.setSuffix(" cm/s")
        self.sp_cm_per_s.setToolTip("功率=50 时机器人前进速度（cm/s）。用「标定测试」按钮实测后填入")
        self.sp_deg_per_s = QtWidgets.QDoubleSpinBox()
        self.sp_deg_per_s.setRange(5, 720); self.sp_deg_per_s.setDecimals(0)
        self.sp_deg_per_s.setValue(DEFAULT_DEG_PER_SEC_AT_OMEGA50); self.sp_deg_per_s.setSuffix(" °/s")
        self.sp_deg_per_s.setToolTip("omega=50 时机器人自转角速度（°/s）。车头跟随模式才需要标定")
        f_cal.addRow("P50 直行:", self.sp_cm_per_s)
        f_cal.addRow("P50 旋转:", self.sp_deg_per_s)
        outer.addWidget(g_cal)

        # --- 高级参数：速度过渡 + 漂移补偿（合并一组） ---
        g_adv = QtWidgets.QGroupBox("高级参数")
        f_adv = QtWidgets.QFormLayout(g_adv)

        self.sp_ramp_ms = QtWidgets.QSpinBox()
        self.sp_ramp_ms.setRange(0, 1000); self.sp_ramp_ms.setSuffix(" ms")
        self.sp_ramp_ms.setValue(DEFAULT_RAMP_MS); self.sp_ramp_ms.setSingleStep(20)
        self.sp_ramp_ms.setToolTip(
            "步间速度插值时长（写入 AUTO_RAMP_MS）。\n"
            "0=立即切换  100ms=常用  200ms=最平滑\n"
            "标定测试时自动设为 0。"
        )
        f_adv.addRow("速度过渡:", self.sp_ramp_ms)

        self.sp_drift_left = QtWidgets.QSpinBox()
        self.sp_drift_left.setRange(-30, 30); self.sp_drift_left.setValue(DEFAULT_DRIFT_LEFT_OMEGA)
        self.sp_drift_left.setToolTip(
            "向左平移时机身往右偏 → 填正数（顺时针补偿）\n"
            "向左平移时机身往左偏 → 填负数\n"
            "建议 1~5 起调，强度按 |Vx|/power 自动缩放"
        )
        self.sp_drift_right = QtWidgets.QSpinBox()
        self.sp_drift_right.setRange(-30, 30); self.sp_drift_right.setValue(DEFAULT_DRIFT_RIGHT_OMEGA)
        self.sp_drift_right.setToolTip(
            "向右平移时机身往左偏 → 填负数\n"
            "向右平移时机身往右偏 → 填正数\n"
            "建议 1~5 起调，强度按 |Vx|/power 自动缩放"
        )
        # 两个漂移补偿放一行
        drift_row = QtWidgets.QWidget()
        drift_h = QtWidgets.QHBoxLayout(drift_row)
        drift_h.setContentsMargins(0, 0, 0, 0); drift_h.setSpacing(4)
        drift_h.addWidget(QtWidgets.QLabel("左"))
        drift_h.addWidget(self.sp_drift_left)
        drift_h.addWidget(QtWidgets.QLabel("右"))
        drift_h.addWidget(self.sp_drift_right)
        f_adv.addRow("漂移补偿:", drift_row)
        outer.addWidget(g_adv)

        # --- 平滑参数 ---
        g_smooth = QtWidgets.QGroupBox("曲线平滑")
        f_smooth = QtWidgets.QFormLayout(g_smooth)
        self.sp_smooth = QtWidgets.QSpinBox()
        self.sp_smooth.setRange(0, 6); self.sp_smooth.setValue(DEFAULT_SMOOTH_ITER)
        self.sp_smooth.setToolTip("Chaikin 迭代次数（0=不平滑，3=推荐，5=最圆滑）")
        self.sp_resample = QtWidgets.QDoubleSpinBox()
        self.sp_resample.setRange(1, 50); self.sp_resample.setDecimals(1)
        self.sp_resample.setValue(DEFAULT_RESAMPLE_CM); self.sp_resample.setSuffix(" cm")
        self.sp_resample.setToolTip("等弧长重采样间隔。越小步骤越多路径越精确，越大导出步骤越少")
        btn_resmooth = QtWidgets.QPushButton("重新平滑")
        btn_resmooth.setToolTip("用当前平滑参数重新处理已画的轨迹")
        btn_resmooth.clicked.connect(self.on_resmooth)
        smooth_row = QtWidgets.QWidget()
        smooth_h = QtWidgets.QHBoxLayout(smooth_row)
        smooth_h.setContentsMargins(0, 0, 0, 0); smooth_h.setSpacing(4)
        smooth_h.addWidget(QtWidgets.QLabel("迭代"))
        smooth_h.addWidget(self.sp_smooth)
        smooth_h.addWidget(QtWidgets.QLabel("间隔"))
        smooth_h.addWidget(self.sp_resample)
        f_smooth.addRow(smooth_row)
        f_smooth.addRow(btn_resmooth)
        outer.addWidget(g_smooth)

        # --- 动作序列 ---
        g_seq = QtWidgets.QGroupBox("动作序列")
        v_seq = QtWidgets.QVBoxLayout(g_seq)
        v_seq.setSpacing(4)

        # 添加按钮行
        btn_row = QtWidgets.QWidget()
        btn_h = QtWidgets.QHBoxLayout(btn_row)
        btn_h.setContentsMargins(0, 0, 0, 0); btn_h.setSpacing(4)
        btn_add_servo = QtWidgets.QPushButton("⚙ 舵机")
        btn_add_servo.setToolTip("在最后一段路径后添加舵机动作块")
        btn_add_servo.clicked.connect(self.on_add_servo_block)
        btn_add_drive = QtWidgets.QPushButton("→ 直走")
        btn_add_drive.setToolTip("在最后一段路径后添加直走 N cm 块")
        btn_add_drive.clicked.connect(self.on_add_drive_block)
        btn_add_delay = QtWidgets.QPushButton("⏱ 延时")
        btn_add_delay.setToolTip("在最后一段路径后添加延时块")
        btn_add_delay.clicked.connect(self.on_add_delay_block)
        btn_h.addWidget(btn_add_servo)
        btn_h.addWidget(btn_add_drive)
        btn_h.addWidget(btn_add_delay)
        v_seq.addWidget(btn_row)

        # 序列树（路段为父节点，block 为子节点）
        self.seq_tree = QtWidgets.QTreeWidget()
        self.seq_tree.setHeaderHidden(True)
        self.seq_tree.setMinimumHeight(100)
        self.seq_tree.setMaximumHeight(200)
        self.seq_tree.setRootIsDecorated(True)
        self.seq_tree.setAlternatingRowColors(True)
        v_seq.addWidget(self.seq_tree)

        # 操作按钮行
        ctrl_row = QtWidgets.QWidget()
        ctrl_h = QtWidgets.QHBoxLayout(ctrl_row)
        ctrl_h.setContentsMargins(0, 0, 0, 0); ctrl_h.setSpacing(4)
        btn_edit_block = QtWidgets.QPushButton("编辑")
        btn_edit_block.setToolTip("编辑选中的执行块")
        btn_edit_block.clicked.connect(self.on_edit_block)
        btn_del_block = QtWidgets.QPushButton("删除")
        btn_del_block.setToolTip("删除选中的执行块")
        btn_del_block.clicked.connect(self.on_delete_block)
        btn_up_block = QtWidgets.QPushButton("↑")
        btn_up_block.setToolTip("上移选中的执行块")
        btn_up_block.setMaximumWidth(30)
        btn_up_block.clicked.connect(self.on_move_block_up)
        btn_dn_block = QtWidgets.QPushButton("↓")
        btn_dn_block.setToolTip("下移选中的执行块")
        btn_dn_block.setMaximumWidth(30)
        btn_dn_block.clicked.connect(self.on_move_block_down)
        ctrl_h.addWidget(btn_edit_block)
        ctrl_h.addWidget(btn_del_block)
        ctrl_h.addWidget(btn_up_block)
        ctrl_h.addWidget(btn_dn_block)
        v_seq.addWidget(ctrl_row)
        outer.addWidget(g_seq)

        # --- 障碍物选中信息 ---
        g_obs = QtWidgets.QGroupBox("选中的障碍物")
        f_obs = QtWidgets.QFormLayout(g_obs)
        self.sp_obs_w = QtWidgets.QDoubleSpinBox()
        self.sp_obs_w.setRange(2, 1000); self.sp_obs_w.setDecimals(1); self.sp_obs_w.setSuffix(" cm")
        self.sp_obs_h = QtWidgets.QDoubleSpinBox()
        self.sp_obs_h.setRange(2, 1000); self.sp_obs_h.setDecimals(1); self.sp_obs_h.setSuffix(" cm")
        self.sp_obs_w.setEnabled(False); self.sp_obs_h.setEnabled(False)
        obs_row = QtWidgets.QWidget()
        obs_h = QtWidgets.QHBoxLayout(obs_row)
        obs_h.setContentsMargins(0, 0, 0, 0); obs_h.setSpacing(4)
        obs_h.addWidget(QtWidgets.QLabel("宽"))
        obs_h.addWidget(self.sp_obs_w)
        obs_h.addWidget(QtWidgets.QLabel("高"))
        obs_h.addWidget(self.sp_obs_h)
        f_obs.addRow(obs_row)
        outer.addWidget(g_obs)

        # --- 提示标签 ---
        self.lbl_face = QtWidgets.QLabel()
        self.lbl_face.setWordWrap(True)
        self.lbl_face.setStyleSheet(
            "color: #7a4000; padding: 6px; font-size: 11px;"
            "background: #fff7e0; border: 1px solid #e0c080; border-radius: 4px;"
        )
        self._refresh_hint_label()
        outer.addWidget(self.lbl_face)

        outer.addStretch(1)
        scroll.setWidget(container)
        dock.setWidget(scroll)
        dock.setMinimumWidth(310)

    def _build_statusbar(self):
        sb = self.statusBar()
        self._lbl_status = QtWidgets.QLabel("就绪 — 在画布上按住左键画轨迹")
        sb.addWidget(self._lbl_status, 1)
        # 右侧：鼠标坐标显示
        self._lbl_coord = QtWidgets.QLabel("X: --  Y: --")
        self._lbl_coord.setStyleSheet("color: #666; padding-right: 8px; font-family: monospace;")
        sb.addPermanentWidget(self._lbl_coord)

    def _wire_signals(self):
        self.sp_field_w.valueChanged.connect(self._on_field_size_changed)
        self.sp_field_h.valueChanged.connect(self._on_field_size_changed)
        self.scene.selectionChanged.connect(self._on_selection_changed)
        self.scene.path_finalized.connect(self._on_path_finalized)
        self.scene.mouse_pos_cm.connect(self._on_mouse_pos)
        self.sp_obs_w.valueChanged.connect(self._on_obs_size_changed)
        self.sp_obs_h.valueChanged.connect(self._on_obs_size_changed)
        self.seq_tree.itemDoubleClicked.connect(self._on_seq_tree_double_clicked)
        # 每段路径完成时推一个快照到撤销栈
        self.scene.path_finalized.connect(self._push_undo)

    # ---- 撤销 / 重做 ----
    def _push_undo(self, *_):
        """把当前状态快照压栈，清空重做栈"""
        try:
            snap = self._build_current_payload()
        except Exception:
            return
        self._undo_stack.append(snap)
        if len(self._undo_stack) > 30:
            self._undo_stack.pop(0)
        self._redo_stack.clear()
        self._act_undo.setEnabled(True)
        self._act_redo.setEnabled(False)

    def _restore_snapshot(self, snap):
        self._apply_payload(snap)
        self._refresh_seq_tree()
        self._refresh_title()

    def on_undo(self):
        if not self._undo_stack:
            return
        # 把当前状态推入重做栈
        try:
            cur = self._build_current_payload()
            self._redo_stack.append(cur)
        except Exception:
            pass
        snap = self._undo_stack.pop()
        self._restore_snapshot(snap)
        self._act_undo.setEnabled(bool(self._undo_stack))
        self._act_redo.setEnabled(True)
        self._lbl_status.setText("已撤销（还可撤销 %d 步）" % len(self._undo_stack))

    def on_redo(self):
        if not self._redo_stack:
            return
        try:
            cur = self._build_current_payload()
            self._undo_stack.append(cur)
        except Exception:
            pass
        snap = self._redo_stack.pop()
        self._restore_snapshot(snap)
        self._act_undo.setEnabled(True)
        self._act_redo.setEnabled(bool(self._redo_stack))
        self._lbl_status.setText("已重做（还可重做 %d 步）" % len(self._redo_stack))

    # ---- 状态同步 ----
    def _on_field_size_changed(self, *_):
        self.scene.set_field_size(self.sp_field_w.value(), self.sp_field_h.value())
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _on_path_finalized(self, seg_idx):
        n_raw = len(self.scene.path_segments[seg_idx].raw_points)
        n_smooth = len(self.scene.path_segments[seg_idx].smoothed_points)
        self._lbl_status.setText("段 %d 绘制完成 — 原始 %d 点 / 平滑 %d 点" % (
            seg_idx + 1, n_raw, n_smooth))
        self._refresh_seq_tree()

    def _on_mouse_pos(self, x, y):
        self._lbl_coord.setText("X: %6.1f  Y: %6.1f cm" % (x, y))

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

    def _current_profile(self):
        profile_id = self.combo_chassis.currentData()
        return get_profile(profile_id)

    def _on_chassis_changed(self, index):
        profile = self._current_profile()
        self.lbl_chassis_desc.setText(profile.description)
        self._refresh_hint_label()

    def _refresh_hint_label(self):
        profile = self._current_profile()
        lines = []
        if profile.has_face_concept:
            lines.append("1) 运行前确认机器人在 Face 0（M1-M2 边朝前）")
        else:
            lines.append("1) 四轮麦克纳姆前后对称，无需切换正面")
        lines.append("2) 任何参数改了之后必须点「导出到机器人」才会生效")
        lines.append("3) 漂移补偿是在 GUI 视角下：勾不勾反转都按你画图时的左右")
        lines.append("4) 导出目标文件: %s" % profile.file_path.name)
        self.lbl_face.setText("提示：\n" + "\n".join(lines))

    # ---- 工具栏动作 ----
    def on_new(self):
        if not self._confirm("新建会清空当前轨迹与障碍物，确定？"):
            return
        self.scene.clear_path()
        self.scene.clear_obstacles()
        self._current_file = None
        self._refresh_title()
        self._refresh_seq_tree()
        self._lbl_status.setText("已新建")

    def on_save(self):
        if not self.scene.path_segments or not any(
            len(s.raw_points) >= 2 for s in self.scene.path_segments
        ):
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
        self._lbl_status.setText("已保存: %s" % saved)

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
        self._lbl_status.setText("已导入: %s" % path)

    def on_export(self):
        if not self.scene.path_segments:
            QtWidgets.QMessageBox.information(self, "导出", "请先画一条轨迹。")
            return
        has_path = any(
            len(seg.smoothed_points) >= 2 for seg in self.scene.path_segments
        )
        if not has_path:
            QtWidgets.QMessageBox.information(self, "导出", "请先画一条轨迹。")
            return
        sequence = self._build_combined_sequence()
        if not self._confirm_export(sequence):
            return
        profile = self._current_profile()
        try:
            backup, _ = write_auto_sequence(
                sequence,
                robot_file=profile.file_path,
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
                profile.file_path.name, len(sequence), self.sp_ramp_ms.value(), backup.name),
        )
        self._lbl_status.setText("导出完成 — %d 步 → %s" % (len(sequence), profile.file_path.name))

    def on_calibration_export(self):
        """生成 2 秒前进的测试段，让用户实测距离反算速度"""
        profile = self._current_profile()
        if not self._confirm(
            "将向 %s 写入一段标定测试：\n"
            "  (2.0s, 0, 50, 0) — 前进 2 秒\n"
            "  (0.1s, 0, 0, 0)  — 停止\n\n"
            "运行机器人，用尺子量实际走的距离 D（cm），\n"
            "然后在「功率50时」填入 D / 2.0。\n\n确认写入？" % profile.file_path.name
        ):
            return
        seq = [(2.0, 0, 50, 0), STOP_BUFFER]
        try:
            backup, _ = write_auto_sequence(
                seq, robot_file=profile.file_path,
                source_name="(calibration)",
                mode="translation",
                cm_per_s_at_p50=self.sp_cm_per_s.value(),
                auto_power=50,
                ramp_ms=0,
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "写入失败", str(e))
            return
        QtWidgets.QMessageBox.information(
            self, "标定写入成功",
            "已写入 %s\n备份: %s\nAUTO_RAMP_MS 已设为 0（标定时不插值）\n\n"
            "烧录后按 + 键运行，量距离反推 cm/s。" % (profile.file_path.name, backup.name),
        )

    def on_add_obstacle(self):
        size = max(20.0, min(self.scene.field_size()) * 0.1)
        obs = self.scene.add_obstacle(w=size, h=size)
        self.scene.clearSelection()
        obs.setSelected(True)

    def on_delete_selected(self):
        """删除当前选中的项目：路段（含其执行块）/ 执行块 / 障碍物"""
        selected = list(self.scene.selectedItems())
        if not selected:
            return
        self._push_undo()   # 删除前先存快照

        from .path_item import PathItem
        for it in selected:
            if isinstance(it, ObstacleItem):
                self.scene.remove_obstacle(it)
            elif isinstance(it, PathItem):
                # 找到是第几段，连同该段执行链一起删除
                if it in self.scene.path_segments:
                    seg_idx = self.scene.path_segments.index(it)
                    # 删执行块
                    for block in list(self.scene.action_chains[seg_idx]):
                        self.scene.removeItem(block)
                    del self.scene.action_chains[seg_idx]
                    self.scene.removeItem(it)
                    del self.scene.path_segments[seg_idx]
                    self.scene._refresh_collision()
            elif isinstance(it, ActionBlockItem):
                # 找到属于哪段
                for seg_idx, chain in enumerate(self.scene.action_chains):
                    if it in chain:
                        self.scene.remove_action_block(seg_idx, it)
                        break

        self._refresh_seq_tree()
        self._lbl_status.setText("已删除选中项目")

    def on_clear_path(self):
        if not self._confirm("清空当前轨迹？"):
            return
        self.scene.clear_path()
        self._refresh_seq_tree()

    def on_resmooth(self):
        self.scene.resmooth(self.sp_smooth.value(), self.sp_resample.value())
        self._refresh_seq_tree()

    # ---- 执行块操作 ----
    def _target_seg_idx(self):
        """添加执行块的目标段索引：选中段，或最后一段"""
        sel = self.seq_tree.currentItem()
        if sel:
            # 如果选的是 block 节点，用其父段
            parent = sel.parent()
            node = parent if parent else sel
            idx = node.data(0, QtCore.Qt.UserRole)
            if idx is not None:
                return int(idx)
        return len(self.scene.path_segments) - 1

    def _add_block(self, action_type):
        idx = self._target_seg_idx()
        if idx < 0:
            QtWidgets.QMessageBox.information(self, "提示", "请先在场地上画一段轨迹。")
            return
        block = ActionBlockItem(action_type)
        if not block.open_edit_dialog(self):
            return
        self.scene.add_action_block(idx, block)
        self._refresh_seq_tree()
        self._lbl_status.setText("已添加 %s 块到路段 %d" % (block.type_label(), idx + 1))

    def on_add_servo_block(self):
        self._add_block(ActionBlockItem.TYPE_SERVO)

    def on_add_drive_block(self):
        self._add_block(ActionBlockItem.TYPE_DRIVE)

    def on_add_delay_block(self):
        self._add_block(ActionBlockItem.TYPE_DELAY)

    def on_edit_block(self):
        item = self.seq_tree.currentItem()
        if item is None or item.parent() is None:
            return
        block = item.data(0, QtCore.Qt.UserRole + 1)
        if block and block.open_edit_dialog(self):
            item.setText(0, block.label_text())
            self._lbl_status.setText("已更新执行块")

    def on_delete_block(self):
        item = self.seq_tree.currentItem()
        if item is None or item.parent() is None:
            return
        seg_idx = int(item.parent().data(0, QtCore.Qt.UserRole))
        block = item.data(0, QtCore.Qt.UserRole + 1)
        self.scene.remove_action_block(seg_idx, block)
        self._refresh_seq_tree()

    def on_move_block_up(self):
        item = self.seq_tree.currentItem()
        if item is None or item.parent() is None:
            return
        seg_idx = int(item.parent().data(0, QtCore.Qt.UserRole))
        pos = item.parent().indexOfChild(item)
        if pos > 0:
            self.scene.move_action_block(seg_idx, pos, pos - 1)
            self._refresh_seq_tree()

    def on_move_block_down(self):
        item = self.seq_tree.currentItem()
        if item is None or item.parent() is None:
            return
        seg_idx = int(item.parent().data(0, QtCore.Qt.UserRole))
        pos = item.parent().indexOfChild(item)
        chain_len = len(self.scene.action_chains[seg_idx])
        if pos < chain_len - 1:
            self.scene.move_action_block(seg_idx, pos, pos + 1)
            self._refresh_seq_tree()

    def _on_seq_tree_double_clicked(self, item, col):
        if item.parent() is not None:
            block = item.data(0, QtCore.Qt.UserRole + 1)
            if block and block.open_edit_dialog(self):
                item.setText(0, block.label_text())

    def _refresh_seq_tree(self):
        self.seq_tree.clear()
        for i, seg in enumerate(self.scene.path_segments):
            n_pts = len(seg.smoothed_points)
            seg_item = QtWidgets.QTreeWidgetItem(
                self.seq_tree, ["路段 %d  (%d 点)" % (i + 1, n_pts)])
            seg_item.setData(0, QtCore.Qt.UserRole, i)
            seg_item.setExpanded(True)
            for block in self.scene.action_chains[i]:
                block_item = QtWidgets.QTreeWidgetItem(seg_item, [block.label_text()])
                block_item.setData(0, QtCore.Qt.UserRole + 1, block)
                block_item.setForeground(0, QtGui.QBrush(block.border_color()))

    # ---- 构建合并序列 ----
    def _build_combined_sequence(self):
        """把所有路段 + 执行块按执行顺序合并为 AUTO_SEQUENCE 元组列表"""
        seq = []
        cm_per_s = self.sp_cm_per_s.value()
        deg_per_s = self.sp_deg_per_s.value()
        auto_power = self.sp_auto_power.value()
        omega_power = self.sp_omega_power.value()
        mode = self._current_mode()
        inv_x = self.chk_invert_x.isChecked()
        inv_y = self.chk_invert_y.isChecked()
        drift_l = self.sp_drift_left.value()
        drift_r = self.sp_drift_right.value()

        for i, seg in enumerate(self.scene.path_segments):
            if len(seg.smoothed_points) >= 2:
                seg_seq = build_sequence(
                    seg.smoothed_points, mode, cm_per_s, deg_per_s,
                    auto_power, omega_power,
                    invert_x=inv_x, invert_y=inv_y,
                    drift_left_omega=drift_l, drift_right_omega=drift_r,
                    add_stop=False,
                )
                seq.extend(seg_seq)
            for block in self.scene.action_chains[i]:
                seq.extend(block.to_sequence_steps(cm_per_s, auto_power))

        from .config import STOP_BUFFER
        seq.append(STOP_BUFFER)
        return seq


    # ---- 数据交换 ----
    def _build_current_payload(self):
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
            "chassis_profile_id": self._current_profile().profile_id,
        }
        obstacles = [o.to_dict() for o in self.scene.obstacles()]
        path_segments = [
            (seg.raw_points, seg.smoothed_points)
            for seg in self.scene.path_segments
        ]
        action_chains = [
            [block.to_dict() for block in chain]
            for chain in self.scene.action_chains
        ]
        # 向后兼容：raw/smooth 传第一段（或空）
        first_raw = path_segments[0][0] if path_segments else []
        first_smooth = path_segments[0][1] if path_segments else []
        return build_payload(
            w, h, calibration, settings, first_raw, first_smooth, obstacles,
            path_segments=path_segments, action_chains=action_chains,
        )

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
        profile_id = settings.get("chassis_profile_id", DEFAULT_PROFILE_ID)
        for i in range(self.combo_chassis.count()):
            if self.combo_chassis.itemData(i) == profile_id:
                self.combo_chassis.setCurrentIndex(i)
                break

        self.scene.clear_path()
        self.scene.clear_obstacles()

        # 优先读新格式 segments，降级到旧格式 path
        if "segments" in data:
            segments_data = data["segments"]
            action_chains_data = [
                seg.get("action_chain", []) for seg in segments_data
            ]
            self.scene.rebuild_from_segments(segments_data, action_chains_data)
        else:
            path = data.get("path", {})
            raw = [tuple(p) for p in path.get("raw_points_cm", [])]
            smooth = [tuple(p) for p in path.get("smoothed_points_cm", [])]
            if raw:
                self.scene.rebuild_from_segments(
                    [{"raw_points_cm": [[x, y] for x, y in raw],
                      "smoothed_points_cm": [[x, y] for x, y in smooth]}],
                    [[]]
                )

        for od in data.get("obstacles", []):
            self.scene.add_obstacle_item(ObstacleItem.from_dict(od))

        self._refresh_seq_tree()

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
        # 只统计标准运动步骤（4元组且第1元不是字符串）
        motion_steps = [s for s in sequence if not isinstance(s[0], str)]
        nonzero_omega = sum(1 for d, vx, vy, w in motion_steps if w != 0)
        max_abs_omega = max((abs(w) for d, vx, vy, w in motion_steps), default=0)
        action_steps = len(sequence) - len(motion_steps)

        flags = []
        if self.chk_invert_x.isChecked():
            flags.append("反转 X 轴")
        if self.chk_invert_y.isChecked():
            flags.append("反转 Y 轴")
        if self.sp_drift_left.value() or self.sp_drift_right.value():
            flags.append("漂移补偿 L=%+d R=%+d" % (
                self.sp_drift_left.value(), self.sp_drift_right.value()))
        if self.sp_ramp_ms.value() > 0:
            flags.append("速度过渡 %d ms" % self.sp_ramp_ms.value())
        flags_text = "\n  ".join(flags) if flags else "（无补偿/反转，原样导出）"

        # 预览前 8 步（混合显示）
        preview_lines = []
        for step in sequence[:8]:
            if isinstance(step[0], str):
                preview_lines.append("  %r" % (step,))
            else:
                d, vx, vy, w = step
                preview_lines.append("  (%5.2fs, Vx=%4d, Vy=%4d, w=%4d)" % (d, vx, vy, w))
        preview = "\n".join(preview_lines)
        if len(sequence) > 8:
            preview += "\n  ... (共 %d 步)" % len(sequence)

        msg = (
            "当前补偿设置：\n  " + flags_text + "\n\n"
            + "导出序列：%d 步（运动 %d + 动作 %d）\n" % (
                len(sequence), len(motion_steps), action_steps)
            + "其中旋转步: %d（最大 |w|=%d）\n\n" % (nonzero_omega, max_abs_omega)
            + "前 8 步预览：\n" + preview + "\n\n"
            + "确认后写入 %s（自动 .bak 备份）" % self._current_profile().file_path.name
        )
        return self._confirm(msg)
