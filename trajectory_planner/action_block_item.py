"""可拖拽的"动作执行块" — 吸附在轨迹末端，构成有序执行列表

块类型（block_type）
===================
  "drive"   -- 行走 N cm
  "servo"   -- 舵机动作 (servo_id, angle, speed, wait_ms)
  "delay"   -- 延时 N 秒
"""
from PyQt5 import QtCore, QtGui, QtWidgets

BLOCK_W = 36.0
BLOCK_H = 16.0
BLOCK_RADIUS = 3.0
_SNAP_GAP_X = 8.0    # 路径末端到第一个块的横向间距（cm）
_SNAP_GAP_Y = 4.0    # 块与块之间的纵向间距（cm）


def block_anchor(end_x, end_y, index):
    """返回第 index 个执行块在场景坐标中的吸附位置 (x, y)。

    视图使用 scale(1,-1) 翻转 Y 轴，屏幕"向下"= 场景 Y 递减。
    块排列在路径末端右侧，从上往下（= 场景 Y 递减）依次摆放。
    """
    x = end_x + _SNAP_GAP_X
    y = end_y - index * (BLOCK_H + _SNAP_GAP_Y)
    return x, y

_COLORS = {
    "drive":    ("#1565C0", "#E3F2FD"),
    "servo":    ("#2E7D32", "#E8F5E9"),
    "delay":    ("#E65100", "#FFF3E0"),
    "spin":     ("#6A1B9A", "#F3E5F5"),
    "motor":    ("#C62828", "#FFEBEE"),
    "dc_motor": ("#00838F", "#E0F7FA"),
    "oscillate": ("#BF360C", "#FBE9E7"),
}
_ICON = {"drive": ">>", "servo": "SV", "delay": "T", "spin": "⟳", "motor": "M", "dc_motor": "DC", "oscillate": "↔"}


def _type_name(block_type):
    return {"drive": "行走", "servo": "舵机", "delay": "延时", "spin": "自旋",
            "motor": "编码电机", "dc_motor": "直流电机",
            "oscillate": "震荡"}.get(block_type, block_type)


def _default_params(block_type):
    if block_type == "drive":
        return {"distance_cm": 30.0}
    if block_type == "servo":
        return {"servo_id": "S1", "angle": 90, "speed": 50, "wait_ms": 500}
    if block_type == "delay":
        return {"duration_s": 1.0}
    if block_type == "spin":
        return {"degrees": 90, "omega_power": 40}
    if block_type == "motor":
        return {"motor_id": "M1", "power": 50}
    if block_type == "dc_motor":
        return {"dc_port": "DC1", "power": 100}
    if block_type == "oscillate":
        return {"angle_deg": 45.0, "cycles": 3, "speed_power": 50,
                "strafe_cm": 0.0, "strafe_power": 40, "drift_vy": 0}
    return {}


class ActionBlockItem(QtWidgets.QGraphicsObject):
    """场景内的动作执行块，双击弹出参数编辑框"""

    # 类型常量（供 main_window 用）
    TYPE_DRIVE = "drive"
    TYPE_SERVO = "servo"
    TYPE_DELAY = "delay"
    TYPE_SPIN  = "spin"
    TYPE_MOTOR = "motor"
    TYPE_DC_MOTOR = "dc_motor"
    TYPE_OSCILLATE = "oscillate"

    block_changed = QtCore.pyqtSignal()

    def __init__(self, block_type="drive", params=None, order=0, parent=None):
        super().__init__(parent)
        self.block_type = block_type
        self.order = order
        self.params = dict(params) if params else _default_params(block_type)

        self.setFlags(
            QtWidgets.QGraphicsItem.ItemIsMovable
            | QtWidgets.QGraphicsItem.ItemIsSelectable
            | QtWidgets.QGraphicsItem.ItemSendsGeometryChanges
        )
        self.setZValue(25)
        self.setCursor(QtCore.Qt.SizeAllCursor)
        self._drag_start_pos = None

    def mousePressEvent(self, event):
        self._drag_start_pos = self.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        # 拖拽结束后弹回原位（块应该吸附在路径末端，不允许自由放置）
        if self._drag_start_pos is not None:
            self.setPos(self._drag_start_pos)
            self._drag_start_pos = None
        super().mouseReleaseEvent(event)

    def boundingRect(self):
        return QtCore.QRectF(0, 0, BLOCK_W, BLOCK_H)

    def paint(self, painter, option, widget=None):
        border_hex, fill_hex = _COLORS.get(self.block_type, ("#555", "#eee"))
        border_color = QtGui.QColor(border_hex)
        fill_color = QtGui.QColor(fill_hex)
        if self.isSelected():
            fill_color = fill_color.darker(112)

        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(border_color, 0.9 if not self.isSelected() else 1.6))
        painter.setBrush(QtGui.QBrush(fill_color))
        painter.drawRoundedRect(QtCore.QRectF(0, 0, BLOCK_W, BLOCK_H),
                                BLOCK_RADIUS, BLOCK_RADIUS)

        # 序号徽章（左上角小圆）
        badge_r = 3.5
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QBrush(border_color))
        painter.drawEllipse(QtCore.QPointF(badge_r + 0.8, badge_r + 0.8), badge_r, badge_r)

        # 以下文字用 painter transform 转换到像素坐标绘制，避免 QFont 浮点字号问题
        t = painter.transform()
        # 场景 1 cm 对应的像素数
        scale_x = abs(t.m11()) if abs(t.m11()) > 1e-6 else 1.0
        scale_y = abs(t.m22()) if abs(t.m22()) > 1e-6 else 1.0

        def draw_text_px(rect_cm, text, bold=False, pt=8, color=None):
            rx = rect_cm.x() * scale_x
            ry = rect_cm.y() * scale_y
            rw = rect_cm.width() * scale_x
            rh = rect_cm.height() * scale_y
            painter.save()
            painter.resetTransform()
            # 重建平移（把块原点映射回视口坐标）
            origin = t.map(QtCore.QPointF(0, 0))
            painter.translate(origin)
            f = QtGui.QFont("Arial", pt)
            f.setBold(bold)
            painter.setFont(f)
            c = color if color else border_color
            painter.setPen(QtGui.QPen(c, 0.3))
            painter.drawText(QtCore.QRectF(rx, ry, rw, rh),
                             QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter, text)
            painter.restore()

        # 序号
        draw_text_px(
            QtCore.QRectF(0.8, 0.8, badge_r * 2, badge_r * 2),
            str(self.order + 1), bold=True, pt=7,
            color=QtGui.QColor("white"),
        )
        # 类型行
        draw_text_px(
            QtCore.QRectF(2, 0.5, BLOCK_W - 3, BLOCK_H * 0.48),
            "[%s] %s" % (_ICON.get(self.block_type, "?"), _type_name(self.block_type)),
            bold=True, pt=8,
        )
        # 参数行
        draw_text_px(
            QtCore.QRectF(2, BLOCK_H * 0.5, BLOCK_W - 3, BLOCK_H * 0.46),
            self._summary(), bold=False, pt=7,
            color=border_color.darker(140),
        )

    def mouseDoubleClickEvent(self, event):
        dlg = ActionBlockEditor(self.block_type, self.params)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            self.params = dlg.get_params()
            self.update()
            self.block_changed.emit()
        event.accept()

    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemPositionHasChanged:
            self.block_changed.emit()
        return super().itemChange(change, value)

    def _summary(self):
        p = self.params
        if self.block_type == "drive":
            return "%.0f cm" % p.get("distance_cm", 0)
        if self.block_type == "servo":
            return "%s %d deg spd%d" % (
                p.get("servo_id", "S1"), p.get("angle", 90), p.get("speed", 50))
        if self.block_type == "delay":
            return "%.1f s" % p.get("duration_s", 1.0)
        if self.block_type == "spin":
            return "%d° @P%d" % (p.get("degrees", 90), p.get("omega_power", 40))
        if self.block_type == "motor":
            return "%s @%d%%" % (p.get("motor_id", "M1"), p.get("power", 50))
        if self.block_type == "dc_motor":
            return "%s @%d%%" % (p.get("dc_port", "DC1"), p.get("power", 100))
        if self.block_type == "oscillate":
            s = "±%.0f° ×%d @P%d" % (
                p.get("angle_deg", 45), p.get("cycles", 3), p.get("speed_power", 50))
            sc = p.get("strafe_cm", 0.0)
            if sc > 0.5:
                s += " ←%.0fcm" % sc
                dv = p.get("drift_vy", 0)
                if dv != 0:
                    s += " Vy%+d" % dv
            return s
        return "?"

    def type_label(self):
        """用于状态栏提示的类型名称"""
        return _type_name(self.block_type)

    def label_text(self):
        """用于 QTreeWidget 行显示的文字"""
        return "[%s] %s" % (_type_name(self.block_type), self._summary())

    def border_color(self):
        """返回此块的主题颜色（QColor），用于树形列表着色"""
        return QtGui.QColor(_COLORS.get(self.block_type, ("#555", "#eee"))[0])

    def open_edit_dialog(self, parent=None):
        """弹出编辑对话框，返回 True 表示用户点了 OK"""
        dlg = ActionBlockEditor(self.block_type, self.params, parent)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            self.params = dlg.get_params()
            self.update()
            self.block_changed.emit()
            return True
        return False

    def to_sequence_steps(self, cm_per_s_at_p50=30.0, auto_power=50):
        """返回一或多个 AUTO_SEQUENCE 兼容元组"""
        p = self.params
        if self.block_type == "drive":
            dist = float(p.get("distance_cm", 0))
            if dist < 0.01:
                return []
            speed = cm_per_s_at_p50 * auto_power / 50.0
            dur = dist / max(speed, 0.1)
            return [(round(dur, 2), 0, int(auto_power), 0)]
        if self.block_type == "servo":
            return [("servo",
                     str(p.get("servo_id", "S1")),
                     int(p.get("angle", 90)),
                     int(p.get("speed", 50)),
                     int(p.get("wait_ms", 500)))]
        if self.block_type == "delay":
            return [("delay", round(float(p.get("duration_s", 1.0)), 2))]
        if self.block_type == "spin":
            return [("spin",
                     int(p.get("degrees", 90)),
                     int(p.get("omega_power", 40)))]
        if self.block_type == "motor":
            return [("motor",
                     str(p.get("motor_id", "M1")),
                     int(p.get("power", 50)))]
        if self.block_type == "dc_motor":
            return [("dc_motor",
                     str(p.get("dc_port", "DC1")),
                     int(p.get("power", 100)))]
        if self.block_type == "oscillate":
            return self._build_oscillate_sequence(p)
        return []

    def _build_oscillate_sequence(self, p):
        """构建震荡序列。

        strafe_cm == 0（纯旋转）：使用 enc_rot 编码器闭环自旋扫掠。
        strafe_cm > 0（旋转+左平移）：使用时间步 (dur, Vx, Vy, omega) 组合运动。
        drift_vy：左移时的前向补偿功率，抵消机械后溜（正=前推）。
        """
        angle_deg = float(p.get("angle_deg", 45.0))
        cycles = int(p.get("cycles", 3))
        power = int(p.get("speed_power", 50))
        strafe_cm = float(p.get("strafe_cm", 0.0))
        strafe_power = int(p.get("strafe_power", 40))
        drift_vy = int(p.get("drift_vy", 0))

        if angle_deg < 1.0 or cycles < 1:
            return []

        if strafe_cm < 0.5:
            return self._build_pure_oscillate(angle_deg, cycles, power)
        else:
            return self._build_strafe_oscillate(angle_deg, cycles, power,
                                                 strafe_cm, strafe_power, drift_vy)

    def _build_pure_oscillate(self, angle_deg, cycles, power):
        """纯旋转震荡：enc_rot 编码器闭环 + 自动收球"""
        from .config import ENCODER_TICKS_PER_CM
        ticks_per_cm = ENCODER_TICKS_PER_CM

        tick_half = int(round(abs(angle_deg) * ticks_per_cm * 0.5))
        tick_full = tick_half * 2
        if tick_half < 1:
            return []

        seq = []
        # 震荡开始 → 自动开启收球电机
        seq.append(("dc_motor", "DC1", -100))
        seq.append(("dc_motor", "DC2", -100))

        seq.append(("enc_rot", -tick_half, 0, power))
        for i in range(cycles):
            if i % 2 == 0:
                seq.append(("enc_rot", +tick_full, 0, power))
            else:
                seq.append(("enc_rot", -tick_full, 0, power))
        last_dir = -1 if cycles % 2 == 1 else 1
        seq.append(("enc_rot", last_dir * tick_half, 0, power))
        seq.append(("enc_stop", 0, 0, 0))

        # 震荡结束 → 关闭收球电机
        seq.append(("dc_motor", "DC1", 0))
        seq.append(("dc_motor", "DC2", 0))
        return seq

    def _build_strafe_oscillate(self, angle_deg, cycles, power,
                                 strafe_cm, strafe_power, drift_vy=0):
        """旋转震荡 + 左平移：时间步 (dur, Vx, Vy, omega)

        drift_vy: 前向补偿功率（正=前推），抵消左移时的机械后溜。
        """
        from .config import (DEFAULT_DEG_PER_SEC_AT_OMEGA50,
                             DEFAULT_CM_PER_SEC_AT_P50, STOP_BUFFER)

        # 旋转速度估算：power=50 时 90°/s，线性外推
        rot_deg_per_s = power * DEFAULT_DEG_PER_SEC_AT_OMEGA50 / 50.0
        rot_deg_per_s = max(rot_deg_per_s, 1.0)

        # 半程时间（转 angle_deg° 的耗时）
        t_half = angle_deg / rot_deg_per_s
        # 全程时间（转 2×angle_deg°，穿过中点）
        t_full = t_half * 2.0

        # 总旋转量 = 首半程 + cycles×2全程 + 尾半程 = 2×angle×(cycles+1)
        total_rot_deg = 2.0 * angle_deg * (cycles + 1)
        total_time = total_rot_deg / rot_deg_per_s

        # 平移速度：需要在 total_time 内走完 strafe_cm
        strafe_speed_cm_s = strafe_cm / max(total_time, 0.01)
        # 平移功率：power=50 时 30cm/s，线性外推
        cm_per_s_per_power = DEFAULT_CM_PER_SEC_AT_P50 / 50.0
        vx_power = int(round(strafe_speed_cm_s / max(cm_per_s_per_power, 0.01)))
        vx_power = max(25, min(95, vx_power))  # 左移，Vx 负

        # 如果 strafe_power 不足以达到所需速度，用 strafe_power 作为上限
        vx_power = -min(abs(vx_power), abs(strafe_power))

        seq = []
        # 震荡开始 → 自动开启收球电机
        seq.append(("dc_motor", "DC1", -100))
        seq.append(("dc_motor", "DC2", -100))

        # 第1步：初始半程左转 + 左移 + 前向补偿
        seq.append((round(t_half, 2), vx_power, drift_vy, -power))
        # 中间 cycles 步：全程扫过中点（交替右/左转）
        for i in range(cycles):
            if i % 2 == 0:
                seq.append((round(t_full, 2), vx_power, drift_vy, +power))
            else:
                seq.append((round(t_full, 2), vx_power, drift_vy, -power))
        # 最后一步：归中
        last_omega = -power if cycles % 2 == 1 else power
        seq.append((round(t_half, 2), vx_power, drift_vy, last_omega))
        seq.append(STOP_BUFFER)

        # 震荡结束 → 关闭收球电机
        seq.append(("dc_motor", "DC1", 0))
        seq.append(("dc_motor", "DC2", 0))
        return seq

    def to_dict(self):
        return {
            "block_type": self.block_type,
            "params": dict(self.params),
            "order": self.order,
            "x_cm": round(self.pos().x(), 2),
            "y_cm": round(self.pos().y(), 2),
        }

    @classmethod
    def from_dict(cls, d):
        item = cls(
            block_type=d.get("block_type", "drive"),
            params=d.get("params"),
            order=d.get("order", 0),
        )
        item.setPos(float(d.get("x_cm", 0)), float(d.get("y_cm", 0)))
        return item


# ================================================================
#  编辑对话框
# ================================================================
class ActionBlockEditor(QtWidgets.QDialog):
    def __init__(self, block_type, params, parent=None):
        super().__init__(parent)
        self.block_type = block_type
        self.setWindowTitle("编辑动作块 — " + _type_name(block_type))
        self.setMinimumWidth(300)
        self._widgets = {}
        layout = QtWidgets.QFormLayout(self)

        if block_type == "drive":
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(1, 3000); sp.setDecimals(1); sp.setSuffix(" cm")
            sp.setValue(float(params.get("distance_cm", 30)))
            layout.addRow("行走距离:", sp)
            self._widgets["distance_cm"] = sp

        elif block_type == "servo":
            sid = QtWidgets.QComboBox()
            sid.addItems(["S1", "S2", "S3", "S4", "S5", "S6"])
            cur = str(params.get("servo_id", "S1"))
            i = sid.findText(cur)
            if i >= 0:
                sid.setCurrentIndex(i)
            layout.addRow("舵机编号:", sid)
            self._widgets["servo_id_combo"] = sid

            angle = QtWidgets.QSpinBox()
            angle.setRange(0, 270); angle.setSuffix(" deg")
            angle.setValue(int(params.get("angle", 90)))
            layout.addRow("目标角度:", angle)
            self._widgets["angle"] = angle

            speed = QtWidgets.QSpinBox()
            speed.setRange(1, 100); speed.setSuffix(" %")
            speed.setValue(int(params.get("speed", 50)))
            layout.addRow("速度:", speed)
            self._widgets["speed"] = speed

            wait = QtWidgets.QSpinBox()
            wait.setRange(0, 10000); wait.setSuffix(" ms")
            wait.setValue(int(params.get("wait_ms", 500)))
            layout.addRow("等待时间:", wait)
            self._widgets["wait_ms"] = wait

        elif block_type == "spin":
            deg = QtWidgets.QSpinBox()
            deg.setRange(-3600, 3600); deg.setSuffix(" deg")
            deg.setValue(int(params.get("degrees", 90)))
            deg.setToolTip("正数=顺时针自旋，负数=逆时针自旋")
            layout.addRow("旋转角度:", deg)
            self._widgets["degrees"] = deg

            wp = QtWidgets.QSpinBox()
            wp.setRange(1, 100); wp.setSuffix(" %")
            wp.setValue(int(params.get("omega_power", 40)))
            wp.setToolTip("旋转时 omega 功率（%）")
            layout.addRow("旋转功率:", wp)
            self._widgets["omega_power"] = wp

        elif block_type == "delay":
            dur = QtWidgets.QDoubleSpinBox()
            dur.setRange(0.01, 60); dur.setDecimals(2); dur.setSuffix(" s")
            dur.setValue(float(params.get("duration_s", 1.0)))
            layout.addRow("延时时长:", dur)
            self._widgets["duration_s"] = dur

        elif block_type == "motor":
            mid = QtWidgets.QComboBox()
            mid.addItems(["M1", "M2", "M3", "M4", "M5", "M6"])
            cur = str(params.get("motor_id", "M1"))
            i = mid.findText(cur)
            if i >= 0:
                mid.setCurrentIndex(i)
            layout.addRow("电机编号:", mid)
            self._widgets["motor_id_combo"] = mid

            mpw = QtWidgets.QSpinBox()
            mpw.setRange(-100, 100); mpw.setSuffix(" %")
            mpw.setValue(int(params.get("power", 50)))
            mpw.setToolTip("正数=正转，负数=反转。设置后电机会一直转！")
            layout.addRow("电机功率:", mpw)
            self._widgets["power"] = mpw

        elif block_type == "dc_motor":
            dport = QtWidgets.QComboBox()
            dport.addItems(["DC1", "DC2", "DC3"])
            cur = str(params.get("dc_port", "DC1"))
            i = dport.findText(cur)
            if i >= 0:
                dport.setCurrentIndex(i)
            layout.addRow("直流端口:", dport)
            self._widgets["dc_port_combo"] = dport

            dpw = QtWidgets.QSpinBox()
            dpw.setRange(-100, 100); dpw.setSuffix(" %")
            dpw.setValue(int(params.get("power", 100)))
            dpw.setToolTip("正数=正转，负数=反转。设置后会一直转！")
            layout.addRow("电机功率:", dpw)
            self._widgets["power"] = dpw

        elif block_type == "oscillate":
            amp = QtWidgets.QDoubleSpinBox()
            amp.setRange(5, 3600); amp.setDecimals(0); amp.setSuffix(" °")
            amp.setValue(float(params.get("angle_deg", 45.0)))
            amp.setToolTip("单侧最大转角。震荡时小车从 0°→左 angle°→右 angle°→左… 穿过中点来回扫掠")
            layout.addRow("旋转角度:", amp)
            self._widgets["angle_deg"] = amp

            cyc = QtWidgets.QSpinBox()
            cyc.setRange(1, 100); cyc.setSuffix(" 次")
            cyc.setValue(int(params.get("cycles", 3)))
            cyc.setToolTip("完整来回次数（1次=左→右→左 扫过一个完整周期）")
            layout.addRow("震荡次数:", cyc)
            self._widgets["cycles"] = cyc

            spd = QtWidgets.QSpinBox()
            spd.setRange(20, 100); spd.setSuffix(" %")
            spd.setValue(int(params.get("speed_power", 50)))
            spd.setToolTip("自旋时的 omega 功率（%）。越高转得越快")
            layout.addRow("旋转功率:", spd)
            self._widgets["speed_power"] = spd

            # ── 左平移（边震荡边向左移动）──
            sc = QtWidgets.QDoubleSpinBox()
            sc.setRange(0, 500); sc.setDecimals(1); sc.setSuffix(" cm")
            sc.setValue(float(params.get("strafe_cm", 0.0)))
            sc.setToolTip("震荡同时向左平移的距离。0=纯旋转震荡，>0=边转边向左移")
            layout.addRow("左移距离:", sc)
            self._widgets["strafe_cm"] = sc

            sp = QtWidgets.QSpinBox()
            sp.setRange(20, 100); sp.setSuffix(" %")
            sp.setValue(int(params.get("strafe_power", 40)))
            sp.setToolTip("左平移时的 Vx 功率上限（%）。实际功率=匀速所需功率，不超过此值")
            layout.addRow("左移功率:", sp)
            self._widgets["strafe_power"] = sp

            dv = QtWidgets.QSpinBox()
            dv.setRange(-30, 30); dv.setSuffix(" %")
            dv.setValue(int(params.get("drift_vy", 0)))
            dv.setToolTip("左移漂移补偿（Vy 前向功率）。左移时小车会后溜偏左下 → 填正数前推抵消")
            layout.addRow("漂移补偿 Vy:", dv)
            self._widgets["drift_vy"] = dv

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addRow(btns)

    def get_params(self):
        p = {}
        if self.block_type == "drive":
            p["distance_cm"] = self._widgets["distance_cm"].value()
        elif self.block_type == "servo":
            p["servo_id"] = self._widgets["servo_id_combo"].currentText()
            p["angle"] = self._widgets["angle"].value()
            p["speed"] = self._widgets["speed"].value()
            p["wait_ms"] = self._widgets["wait_ms"].value()
        elif self.block_type == "spin":
            p["degrees"] = self._widgets["degrees"].value()
            p["omega_power"] = self._widgets["omega_power"].value()
        elif self.block_type == "delay":
            p["duration_s"] = self._widgets["duration_s"].value()
        elif self.block_type == "motor":
            p["motor_id"] = self._widgets["motor_id_combo"].currentText()
            p["power"] = self._widgets["power"].value()
        elif self.block_type == "dc_motor":
            p["dc_port"] = self._widgets["dc_port_combo"].currentText()
            p["power"] = self._widgets["power"].value()
        elif self.block_type == "oscillate":
            p["angle_deg"] = self._widgets["angle_deg"].value()
            p["cycles"] = self._widgets["cycles"].value()
            p["speed_power"] = self._widgets["speed_power"].value()
            p["strafe_cm"] = self._widgets["strafe_cm"].value()
            p["strafe_power"] = self._widgets["strafe_power"].value()
            p["drift_vy"] = self._widgets["drift_vy"].value()
        return p
