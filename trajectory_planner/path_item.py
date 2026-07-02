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


class PathItem(QtWidgets.QGraphicsPathItem):
    """显示平滑后的轨迹。raw_points / smoothed_points 都用场景坐标（cm，+Y 上）"""

    COLOR_NORMAL = QtGui.QColor(40, 120, 220)
    COLOR_COLLIDE = QtGui.QColor(220, 60, 60)

    def __init__(self):
        super().__init__()
        self.raw_points = []        # 原始鼠标采样点（cm）
        self.smoothed_points = []   # 平滑后的点（cm）
        self._collide = False
        self._configure_pen()
        self.setZValue(20)
        # 允许选中（选中后可用 Del 删除）
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

    def _configure_pen(self):
        color = self.COLOR_COLLIDE if self._collide else self.COLOR_NORMAL
        # 选中时加粗、半透明橙色覆盖
        if self.isSelected():
            color = QtGui.QColor(255, 160, 0)
        pen = QtGui.QPen(color, PATH_PEN_WIDTH_CM * (1.8 if self.isSelected() else 1.0))
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        self.setPen(pen)

    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemSelectedChange:
            self._configure_pen()
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
