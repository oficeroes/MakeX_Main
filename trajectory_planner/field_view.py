"""场地画布：QGraphicsScene/View 单位 = cm，+Y 朝上，自动保持纵横比

坐标系设计
==========
场景单位直接等于厘米，消除像素↔cm 换算。
FieldView 用 scale(1, -1) 翻转 Y 轴，使屏幕"上方"对应 +Y（机器人前进方向）。
窗口缩放时 resizeEvent → fitInView(KeepAspectRatio) 保持场地比例，
多余区域显示深灰色 letterbox。

FieldScene（QGraphicsScene）
============================
职责：场地边框 + 网格 + 障碍物列表 + 单条轨迹 + 碰撞检测

  场地边框：白色矩形，zValue=-100
  网格：每 GRID_SPACING_CM(=10) cm 一条浅灰线，50 cm 倍数的线稍深，zValue=-99
  障碍物：ObstacleItem 列表，zValue=10
  轨迹：单个 PathItem，zValue=20
  标记（起/终点小圆）：zValue=21，PathItem 的子项

  碰撞检测：path().intersects(QRectF) 逐障碍判断，结果设给 PathItem.set_collision()。
  每次障碍物移动/缩放（obstacle_changed 回调）和轨迹更新（finish_path/resmooth）都刷新。

绘制流程（鼠标）
================
  mousePressEvent  → start_path(scene_pos)   : 清旧轨迹，开始新采样
  mouseMoveEvent   → extend_path(scene_pos)  : 追加采样点 + preview_raw()
  mouseReleaseEvent→ finish_path()           : Chaikin 平滑 + 重采样 + 刷新碰撞 + 发 path_finalized 信号

  点到 ObstacleItem 或 _Handle 时，改为走 Qt 默认事件流（拖拽/选中），不开始画线。

FieldView（QGraphicsView）
==========================
  - Antialiasing + SmoothPixmapTransform 渲染
  - 禁用滚动条；fitInView 完全负责缩放
  - _is_background(item)：判断命中项是否是"背景"（网格/底板），是则可以开始画线

信号
====
  path_finalized      鼠标抬起、平滑完成后发射，main_window 用来更新状态栏
  obstacles_changed   障碍物增删或几何变化后发射（预留，当前 main_window 未接）

注意
====
- 同一时刻只有一条轨迹（self.path_item），新建时 clear()。
- 平滑参数 smooth_iter / resample_step 由 main_window 在绘制前注入。
- resmooth() 用新参数重跑现有 raw_points，无需重新画。
"""
from PyQt5 import QtCore, QtGui, QtWidgets

from .config import GRID_SPACING_CM
from .obstacle_item import ObstacleItem
from .path_item import PathItem
from .smoothing import smooth_and_resample


class FieldScene(QtWidgets.QGraphicsScene):
    """场景：固定矩形场地 + 网格 + 障碍物 + 轨迹"""

    path_finalized = QtCore.pyqtSignal()   # 鼠标抬起后发射
    obstacles_changed = QtCore.pyqtSignal()  # 障碍物增删/几何变化

    def __init__(self, width_cm, height_cm, parent=None):
        super().__init__(parent)
        self._width = float(width_cm)
        self._height = float(height_cm)
        self.setSceneRect(0, 0, self._width, self._height)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(250, 250, 248)))

        self._field_rect = QtWidgets.QGraphicsRectItem(0, 0, self._width, self._height)
        self._field_rect.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        self._field_rect.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 0.6))
        self._field_rect.setZValue(-100)
        self.addItem(self._field_rect)

        self._grid_items = []
        self._rebuild_grid()

        self.path_item = PathItem()
        self.addItem(self.path_item)

        # 绘制中的临时点列（鼠标按下时生效）
        self._drawing = False

        # 平滑参数（外部注入）
        self.smooth_iter = 3
        self.resample_step = 5.0

    # ---- 场地尺寸 ----
    def set_field_size(self, w, h):
        self._width = float(w)
        self._height = float(h)
        self.setSceneRect(0, 0, self._width, self._height)
        self._field_rect.setRect(0, 0, self._width, self._height)
        self._rebuild_grid()
        self.update()

    def field_size(self):
        return self._width, self._height

    def _rebuild_grid(self):
        for it in self._grid_items:
            self.removeItem(it)
        self._grid_items = []
        pen_minor = QtGui.QPen(QtGui.QColor(230, 230, 230), 0.15)
        pen_major = QtGui.QPen(QtGui.QColor(200, 200, 200), 0.3)
        step = GRID_SPACING_CM
        x = step
        while x < self._width:
            line = QtWidgets.QGraphicsLineItem(x, 0, x, self._height)
            line.setPen(pen_major if int(x) % 50 == 0 else pen_minor)
            line.setZValue(-99)
            self.addItem(line)
            self._grid_items.append(line)
            x += step
        y = step
        while y < self._height:
            line = QtWidgets.QGraphicsLineItem(0, y, self._width, y)
            line.setPen(pen_major if int(y) % 50 == 0 else pen_minor)
            line.setZValue(-99)
            self.addItem(line)
            self._grid_items.append(line)
            y += step

    # ---- 障碍物 ----
    def add_obstacle(self, x=None, y=None, w=20.0, h=20.0, label=None):
        if x is None:
            x = (self._width - w) / 2.0
        if y is None:
            y = (self._height - h) / 2.0
        label = label or ("obs%d" % (self.obstacle_count() + 1))
        obs = ObstacleItem(x, y, w, h, label=label)
        self.addItem(obs)
        self.obstacle_changed(obs)
        self.obstacles_changed.emit()
        return obs

    def add_obstacle_item(self, obs):
        """从导入数据加入已构造好的障碍物"""
        self.addItem(obs)
        self.obstacle_changed(obs)
        self.obstacles_changed.emit()

    def obstacles(self):
        return [it for it in self.items() if isinstance(it, ObstacleItem)]

    def obstacle_count(self):
        return len(self.obstacles())

    def remove_obstacle(self, obs):
        self.removeItem(obs)
        self._refresh_collision()
        self.obstacles_changed.emit()

    def clear_obstacles(self):
        for obs in self.obstacles():
            self.removeItem(obs)
        self._refresh_collision()
        self.obstacles_changed.emit()

    def obstacle_changed(self, obs):
        """由 ObstacleItem 调用，触发碰撞检测刷新"""
        self._refresh_collision()
        self.obstacles_changed.emit()

    # ---- 碰撞检测 ----
    def _refresh_collision(self):
        if not self.path_item or not self.path_item.smoothed_points:
            self.path_item.set_collision(False) if self.path_item else None
            return
        path = self.path_item.path()
        collide = False
        for obs in self.obstacles():
            x, y, w, h = obs.rect_scene()
            rect = QtCore.QRectF(x, y, w, h)
            if path.intersects(rect):
                collide = True
                break
        self.path_item.set_collision(collide)

    # ---- 绘制轨迹（由 FieldView 转发鼠标事件）----
    def start_path(self, scene_pos):
        self._drawing = True
        self.path_item.clear()
        self.path_item.append_raw(scene_pos.x(), scene_pos.y())
        self.path_item.preview_raw()

    def extend_path(self, scene_pos):
        if not self._drawing:
            return
        self.path_item.append_raw(scene_pos.x(), scene_pos.y())
        self.path_item.preview_raw()

    def finish_path(self):
        if not self._drawing:
            return
        self._drawing = False
        raw = list(self.path_item.raw_points)
        if len(raw) >= 2:
            smoothed = smooth_and_resample(raw, self.smooth_iter, self.resample_step)
        else:
            smoothed = list(raw)
        self.path_item.set_points(raw, smoothed)
        self._refresh_collision()
        self.path_finalized.emit()

    def resmooth(self, iterations, step):
        """用新参数重新平滑现有 raw_points"""
        self.smooth_iter = iterations
        self.resample_step = step
        raw = list(self.path_item.raw_points)
        if len(raw) >= 2:
            smoothed = smooth_and_resample(raw, iterations, step)
        else:
            smoothed = list(raw)
        self.path_item.set_points(raw, smoothed)
        self._refresh_collision()

    def clear_path(self):
        self.path_item.clear()
        self._refresh_collision()


class FieldView(QtWidgets.QGraphicsView):
    """视图：自动保持场地纵横比；左键绘制轨迹（除非按到障碍物/手柄）"""

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        # Y 轴翻转：屏幕"上"对应场景 +Y
        self.scale(1, -1)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(60, 60, 65)))
        self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    # ---- 鼠标 ----
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            item = self.itemAt(event.pos())
            # 如果点到障碍物/手柄/障碍物子项，让 Qt 走正常选择/拖拽
            if item is not None and not self._is_background(item):
                super().mousePressEvent(event)
                return
            scene_pos = self.mapToScene(event.pos())
            self.scene().clearSelection()
            self.scene().start_path(scene_pos)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & QtCore.Qt.LeftButton and self.scene()._drawing:
            scene_pos = self.mapToScene(event.pos())
            self.scene().extend_path(scene_pos)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.scene()._drawing:
            self.scene().finish_path()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    @staticmethod
    def _is_background(item):
        """判断 itemAt 命中的是否是网格/底板，可以无视并直接开始画"""
        from .obstacle_item import ObstacleItem, _Handle
        if isinstance(item, (ObstacleItem, _Handle)):
            return False
        if isinstance(item, PathItem):
            return False
        return True
