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
    "drive":  ("#1565C0", "#E3F2FD"),
    "servo":  ("#2E7D32", "#E8F5E9"),
    "delay":  ("#E65100", "#FFF3E0"),
}
_ICON = {"drive": ">>", "servo": "SV", "delay": "T"}


def _type_name(block_type):
    return {"drive": "行走", "servo": "舵机", "delay": "延时"}.get(block_type, block_type)


def _default_params(block_type):
    if block_type == "drive":
        return {"distance_cm": 30.0}
    if block_type == "servo":
        return {"servo_id": "S1", "angle": 90, "speed": 50, "wait_ms": 500}
    if block_type == "delay":
        return {"duration_s": 1.0}
    return {}


class ActionBlockItem(QtWidgets.QGraphicsObject):
    """场景内的动作执行块，双击弹出参数编辑框"""

    # 类型常量（供 main_window 用）
    TYPE_DRIVE = "drive"
    TYPE_SERVO = "servo"
    TYPE_DELAY = "delay"

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
        return []

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

        elif block_type == "delay":
            dur = QtWidgets.QDoubleSpinBox()
            dur.setRange(0.01, 60); dur.setDecimals(2); dur.setSuffix(" s")
            dur.setValue(float(params.get("duration_s", 1.0)))
            layout.addRow("延时时长:", dur)
            self._widgets["duration_s"] = dur

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
        elif self.block_type == "delay":
            p["duration_s"] = self._widgets["duration_s"].value()
        return p
