"""轨迹视觉项：绘制平滑曲线 + 起终点标记 + 与障碍物的碰撞高亮

目的
====
PathItem 是 QGraphicsPathItem 的子类，负责在场地画布上渲染轨迹。
它持有两套点：raw_points（鼠标原始采样）和 smoothed_points（平滑后）。
绘制时优先使用 smoothed_points；绘制过程中（鼠标还没抬起）调 preview_raw() 用原始点实时预览。

坐标系
======
场景单位 = cm，+Y 朝上（FieldView 用 scale(1,-1) 翻转屏幕 Y 轴）。
PathItem 不关心坐标系细节，直接用传入的 (x, y) 画线。

颜色语义
========
  COLOR_NORMAL  = 蓝色 — 正常，不与任何障碍物相交
  COLOR_COLLIDE = 红色 — 路径与至少一个障碍物相交（仅视觉警告，不阻止导出）

标记
====
  _start_marker  绿色小圆，标记轨迹起点
  _end_marker    红色小圆，标记轨迹终点

注意
====
- set_collision() 由 FieldScene._refresh_collision() 调用，外部不应直接调用。
- raw_points 存储原始采样，导出 JSON 时也保存，以便用新参数重新平滑。
- 标记尺寸固定为 2.5 cm 半径，与场地缩放无关（场景坐标）。

API
===
  set_points(raw, smoothed)        更新双套点并重绘
  append_raw(x, y)                 绘制中追加采样点
  preview_raw()                    只用 raw_points 预览（绘制中调用）
  clear()                          清空所有点
  set_collision(collide: bool)     切换碰撞高亮颜色
"""
from PyQt5 import QtCore, QtGui, QtWidgets

from .config import PATH_PEN_WIDTH_CM
from . import curves


ANCHOR_HANDLE_R = 3.0   # 锚点手柄半径（场景 cm）


class _AnchorHandle(QtWidgets.QGraphicsEllipseItem):
    """样条 / 直线的可拖拽控制锚点。拖动时把新位置回传给父 PathItem。"""

    def __init__(self, index, parent):
        super().__init__(-ANCHOR_HANDLE_R, -ANCHOR_HANDLE_R,
                         2 * ANCHOR_HANDLE_R, 2 * ANCHOR_HANDLE_R, parent)
        self.index = index
        self.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        self.setPen(QtGui.QPen(QtGui.QColor(40, 40, 40), 0.5))
        self.setZValue(30)
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsScenePositionChanges, True)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self._active = False

    def itemChange(self, change, value):
        if (change == QtWidgets.QGraphicsItem.ItemPositionHasChanged
                and self._active):
            self.parentItem().on_anchor_dragged(self.index, self.pos())
        return super().itemChange(change, value)


class PathItem(QtWidgets.QGraphicsPathItem):
    """显示平滑后的轨迹。raw_points / smoothed_points 都用场景坐标（cm，+Y 上）"""

    COLOR_NORMAL = QtGui.QColor(40, 120, 220)
    COLOR_COLLIDE = QtGui.QColor(220, 60, 60)

    def __init__(self, color=None):
        super().__init__()
        self.raw_points = []
        self.smoothed_points = []
        self._collide = False
        # 绘制类型："freehand"（手绘）/ "spline"（样条）/ "line"（直线）
        self.draw_kind = curves.KIND_FREEHAND
        # 控制锚点（仅 spline / line 有意义）——用户点的点
        self.anchor_points = []
        self._anchor_handles = []
        self._max_spacing = 5.0   # 生成曲线时的加密间距（由场景注入）
        # 行进中自旋：整段路径从起点到终点边走边转的总角度（度，0=不自旋）
        self.travel_spin_deg = 0.0
        # 车型颜色（None → 使用默认蓝）
        self._vehicle_color = QtGui.QColor(color) if color else None
        self._configure_pen()
        self.setZValue(20)
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)

        # 起点 / 终点小圆圈
        self._start_marker = QtWidgets.QGraphicsEllipseItem(self)
        self._start_marker.setPen(QtGui.QPen(QtGui.QColor(20, 180, 20), 0.3))
        self._start_marker.setBrush(QtGui.QBrush(QtGui.QColor(20, 200, 20)))
        self._start_marker.setZValue(21)

        self._end_marker = QtWidgets.QGraphicsEllipseItem(self)
        self._end_marker.setPen(QtGui.QPen(QtGui.QColor(200, 30, 30), 0.3))
        self._end_marker.setBrush(QtGui.QBrush(QtGui.QColor(220, 50, 50)))
        self._end_marker.setZValue(21)

    def contextMenuEvent(self, event):
        """右键菜单：为本路段设置行进中自旋角度（方案 C）"""
        menu = QtWidgets.QMenu()
        cur = float(getattr(self, "travel_spin_deg", 0.0) or 0.0)
        act_set = menu.addAction("设置行进中自旋... (当前 %.0f°)" % cur)
        act_clear = menu.addAction("清除自旋")
        act_clear.setEnabled(abs(cur) >= 1.0)
        chosen = menu.exec_(event.screenPos())
        if chosen is act_set:
            val, ok = QtWidgets.QInputDialog.getDouble(
                None, "行进中自旋",
                "从起点到终点，整段要旋转的总角度（+顺时针 -逆时针，度）",
                cur, -3600, 3600, 0)
            if ok:
                self.travel_spin_deg = float(val)
                self._notify_spin_changed()
        elif chosen is act_clear:
            self.travel_spin_deg = 0.0
            self._notify_spin_changed()
        event.accept()

    def _notify_spin_changed(self):
        """通知场景重新计算导出、刷新状态。"""
        scene = self.scene()
        if scene is not None and hasattr(scene, "on_path_spin_changed"):
            scene.on_path_spin_changed(self)
        self.update()

    def _configure_pen(self):
        if self._collide:
            color = self.COLOR_COLLIDE
        elif self.isSelected():
            color = QtGui.QColor(255, 160, 0)   # 橙色高亮
        elif self._vehicle_color:
            color = self._vehicle_color
        else:
            color = self.COLOR_NORMAL
        width = PATH_PEN_WIDTH_CM * (1.8 if self.isSelected() else 1.0)
        pen = QtGui.QPen(color, width)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        self.setPen(pen)

    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemSelectedChange:
            self._configure_pen()
            # 选中且是控制点曲线时显示锚点手柄，取消选中时隐藏
            self._set_handles_visible(bool(value)
                                      and self.draw_kind in
                                      (curves.KIND_SPLINE, curves.KIND_LINE))
        return super().itemChange(change, value)

    def set_collision(self, collide):
        if collide == self._collide:
            return
        self._collide = collide
        self._configure_pen()

    def set_points(self, raw, smoothed):
        self.raw_points = [(float(x), float(y)) for x, y in raw]
        self.smoothed_points = [(float(x), float(y)) for x, y in smoothed]
        self._rebuild_path()

    def append_raw(self, x, y):
        self.raw_points.append((float(x), float(y)))

    def clear(self):
        self.raw_points = []
        self.smoothed_points = []
        self._rebuild_path()

    def _rebuild_path(self):
        path = QtGui.QPainterPath()
        pts = self.smoothed_points if self.smoothed_points else self.raw_points
        if pts:
            path.moveTo(pts[0][0], pts[0][1])
            for x, y in pts[1:]:
                path.lineTo(x, y)
        self.setPath(path)
        self._update_markers(pts)

    def _update_markers(self, pts):
        r = 2.5  # cm
        if pts:
            x0, y0 = pts[0]
            self._start_marker.setRect(x0 - r, y0 - r, 2 * r, 2 * r)
            self._start_marker.setVisible(True)
            xn, yn = pts[-1]
            self._end_marker.setRect(xn - r, yn - r, 2 * r, 2 * r)
            self._end_marker.setVisible(True)
        else:
            self._start_marker.setVisible(False)
            self._end_marker.setVisible(False)

    def preview_raw(self):
        """绘制中：只走原始点，不平滑（实时反馈）"""
        self._rebuild_path()

    # ---- 控制点曲线（样条 / 直线） ----
    def set_max_spacing(self, spacing):
        """设置生成曲线时的加密间距（cm），由场景在绘制前注入。"""
        try:
            self._max_spacing = max(0.5, float(spacing))
        except (TypeError, ValueError):
            self._max_spacing = 5.0

    def set_curve(self, anchors, kind, max_spacing=None):
        """用控制锚点生成曲线，填充 raw/smoothed 点并重建手柄。

        anchors:      用户点的控制点列表 [(x, y), ...]
        kind:         curves.KIND_SPLINE / KIND_LINE
        max_spacing:  加密间距（None = 用已设置的 _max_spacing）

        生成的点同时写入 raw_points 和 smoothed_points——控制点曲线
        本身已经规整，不再需要 Chaikin 平滑；导出管线照常复用。
        """
        if max_spacing is not None:
            self.set_max_spacing(max_spacing)
        self.draw_kind = kind
        self.anchor_points = [(float(x), float(y)) for x, y in anchors]
        pts = curves.build_curve(kind, self.anchor_points, self._max_spacing)
        self.raw_points = [(float(x), float(y)) for x, y in pts]
        self.smoothed_points = list(self.raw_points)
        self._rebuild_path()
        self._rebuild_anchor_handles()

    def _regenerate_from_anchors(self):
        """锚点变动后重算曲线点（不重建手柄，避免拖动时抖动）。"""
        pts = curves.build_curve(self.draw_kind, self.anchor_points,
                                 self._max_spacing)
        self.raw_points = [(float(x), float(y)) for x, y in pts]
        self.smoothed_points = list(self.raw_points)
        self._rebuild_path()

    def on_anchor_dragged(self, index, new_pos):
        """某个锚点手柄被拖动 → 更新锚点并实时重算曲线。"""
        if 0 <= index < len(self.anchor_points):
            self.anchor_points[index] = (new_pos.x(), new_pos.y())
            self._regenerate_from_anchors()
            scene = self.scene()
            if scene is not None and hasattr(scene, "on_path_anchor_edited"):
                scene.on_path_anchor_edited(self)

    def _rebuild_anchor_handles(self):
        """按当前锚点重建手柄集合。"""
        for h in self._anchor_handles:
            h._active = False
            if h.scene() is not None:
                h.scene().removeItem(h)
            elif h.parentItem() is self:
                h.setParentItem(None)
        self._anchor_handles = []
        if self.draw_kind not in (curves.KIND_SPLINE, curves.KIND_LINE):
            return
        for i, (x, y) in enumerate(self.anchor_points):
            h = _AnchorHandle(i, self)
            h.setPos(x, y)
            h._active = True
            h.setVisible(self.isSelected())
            self._anchor_handles.append(h)

    def _set_handles_visible(self, visible):
        for h in self._anchor_handles:
            h.setVisible(visible)

    def has_anchors(self):
        return bool(self.anchor_points) and self.draw_kind in (
            curves.KIND_SPLINE, curves.KIND_LINE)

    # ---- 序列化辅助 ----
    def curve_to_dict(self):
        """返回绘制类型 + 锚点，供 JSON 存储（freehand 时锚点为空）。"""
        return {
            "draw_kind": self.draw_kind,
            "anchor_points_cm": [[round(x, 2), round(y, 2)]
                                 for x, y in self.anchor_points],
            "travel_spin_deg": round(float(self.travel_spin_deg), 2),
        }

    def apply_curve_dict(self, d):
        """从 JSON 恢复绘制类型 + 锚点。无字段时保持 freehand。"""
        kind = d.get("draw_kind", curves.KIND_FREEHAND)
        anchors = [(float(p[0]), float(p[1]))
                   for p in d.get("anchor_points_cm", [])]
        self.draw_kind = kind
        self.anchor_points = anchors
        try:
            self.travel_spin_deg = float(d.get("travel_spin_deg", 0.0))
        except (TypeError, ValueError):
            self.travel_spin_deg = 0.0
        if anchors and kind in (curves.KIND_SPLINE, curves.KIND_LINE):
            self._rebuild_anchor_handles()
            self._set_handles_visible(self.isSelected())
