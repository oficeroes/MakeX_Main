"""可拖拽、可缩放的障碍物方块（cm 单位）

外观：半透明红色矩形 + 角落 8 个小手柄；选中后手柄出现，未选中只显示矩形。
手柄拖拽时实时更新矩形。
"""
from PyQt5 import QtCore, QtGui, QtWidgets


HANDLE_SIZE_CM = 4.0  # 手柄边长（场景单位）
HANDLE_HALF = HANDLE_SIZE_CM / 2.0

# 8 个手柄位置代号
HANDLES = [
    "tl", "tm", "tr",
    "ml",       "mr",
    "bl", "bm", "br",
]


class _Handle(QtWidgets.QGraphicsRectItem):
    """单个手柄。把鼠标事件转发给父障碍物"""

    def __init__(self, key, parent):
        super().__init__(-HANDLE_HALF, -HANDLE_HALF, HANDLE_SIZE_CM, HANDLE_SIZE_CM, parent)
        self.key = key
        self.setBrush(QtGui.QBrush(QtGui.QColor(40, 40, 40)))
        self.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255), 0.3))
        self.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, False)
        self.setCursor(self._cursor_for(key))
        self.setZValue(30)
        self._drag_start_scene = None
        self._orig_rect = None

    @staticmethod
    def _cursor_for(key):
        return {
            "tl": QtCore.Qt.SizeFDiagCursor, "br": QtCore.Qt.SizeFDiagCursor,
            "tr": QtCore.Qt.SizeBDiagCursor, "bl": QtCore.Qt.SizeBDiagCursor,
            "tm": QtCore.Qt.SizeVerCursor, "bm": QtCore.Qt.SizeVerCursor,
            "ml": QtCore.Qt.SizeHorCursor, "mr": QtCore.Qt.SizeHorCursor,
        }[key]

    def mousePressEvent(self, event):
        self._drag_start_scene = event.scenePos()
        self._orig_rect = self.parentItem().rect_scene()
        event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_start_scene is None:
            return
        delta = event.scenePos() - self._drag_start_scene
        self.parentItem().resize_by_handle(self.key, self._orig_rect, delta)
        event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_start_scene = None
        self._orig_rect = None
        event.accept()


class ObstacleItem(QtWidgets.QGraphicsRectItem):
    """障碍物：QGraphicsRectItem 的 rect() 始终用 (0,0,w,h)，位移用 pos()

    场景坐标用 mapToScene 转换。提供 rect_scene() 拿场景矩形（x,y,w,h，含位置）。
    """

    BRUSH_NORMAL = QtGui.QBrush(QtGui.QColor(220, 80, 80, 100))
    BRUSH_SELECTED = QtGui.QBrush(QtGui.QColor(220, 80, 80, 160))
    PEN = QtGui.QPen(QtGui.QColor(180, 30, 30), 0.5)

    def __init__(self, x_cm, y_cm, w_cm, h_cm, label=""):
        super().__init__(0, 0, max(2.0, w_cm), max(2.0, h_cm))
        self.setPos(x_cm, y_cm)
        self.setBrush(self.BRUSH_NORMAL)
        self.setPen(self.PEN)
        self.setFlags(
            QtWidgets.QGraphicsItem.ItemIsMovable
            | QtWidgets.QGraphicsItem.ItemIsSelectable
            | QtWidgets.QGraphicsItem.ItemSendsGeometryChanges
        )
        self.setZValue(10)
        self.label = label or "obs"
        self._handles = {}
        for key in HANDLES:
            h = _Handle(key, self)
            h.setVisible(False)
            self._handles[key] = h
        self._reposition_handles()

    # ---- 几何工具 ----
    def rect_scene(self):
        """返回当前矩形在场景坐标系的位置 (x, y, w, h)，cm"""
        r = self.rect()
        return (self.pos().x() + r.x(), self.pos().y() + r.y(), r.width(), r.height())

    def set_scene_rect(self, x, y, w, h):
        w = max(2.0, float(w))
        h = max(2.0, float(h))
        self.setPos(float(x), float(y))
        self.setRect(0, 0, w, h)
        self._reposition_handles()
        self._notify_changed()

    def set_size(self, w_cm, h_cm):
        x, y, _, _ = self.rect_scene()
        self.set_scene_rect(x, y, w_cm, h_cm)

    def _reposition_handles(self):
        r = self.rect()
        coords = {
            "tl": (r.left(),  r.top()),
            "tm": (r.center().x(), r.top()),
            "tr": (r.right(), r.top()),
            "ml": (r.left(),  r.center().y()),
            "mr": (r.right(), r.center().y()),
            "bl": (r.left(),  r.bottom()),
            "bm": (r.center().x(), r.bottom()),
            "br": (r.right(), r.bottom()),
        }
        for k, (x, y) in coords.items():
            self._handles[k].setPos(x, y)

    def resize_by_handle(self, key, orig_scene_rect, delta):
        """根据被拖拽的手柄重算矩形"""
        x, y, w, h = orig_scene_rect
        dx, dy = delta.x(), delta.y()
        left, top, right, bottom = x, y, x + w, y + h

        if "l" in key:
            left = min(x + dx, right - 2.0)
        if "r" in key:
            right = max(x + w + dx, left + 2.0)
        if "t" in key:
            top = min(y + dy, bottom - 2.0)
        if "b" in key:
            bottom = max(y + h + dy, top + 2.0)

        self.set_scene_rect(left, top, right - left, bottom - top)

    # ---- 选中 & 变化通知 ----
    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemSelectedChange:
            visible = bool(value)
            self.setBrush(self.BRUSH_SELECTED if visible else self.BRUSH_NORMAL)
            for h in self._handles.values():
                h.setVisible(visible)
        elif change == QtWidgets.QGraphicsItem.ItemPositionHasChanged:
            self._notify_changed()
        return super().itemChange(change, value)

    def _notify_changed(self):
        scene = self.scene()
        if scene is not None and hasattr(scene, "obstacle_changed"):
            scene.obstacle_changed(self)

    # ---- 序列化 ----
    def to_dict(self):
        x, y, w, h = self.rect_scene()
        return {
            "x_cm": round(x, 2),
            "y_cm": round(y, 2),
            "width_cm": round(w, 2),
            "height_cm": round(h, 2),
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            float(d.get("x_cm", 0)),
            float(d.get("y_cm", 0)),
            float(d.get("width_cm", 20)),
            float(d.get("height_cm", 20)),
            label=d.get("label", "obs"),
        )
