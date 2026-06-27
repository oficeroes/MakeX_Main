"""轨迹视觉项：绘制平滑曲线 + 起终点标记 + 与障碍物的碰撞高亮"""
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
        pen = QtGui.QPen(color, PATH_PEN_WIDTH_CM)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        self.setPen(pen)

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
