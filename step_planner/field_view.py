"""step_planner.field_view — 场地预览（死算轨迹渲染）

FieldScene  (QGraphicsScene) — 场地 + 轨迹
FieldView   (QGraphicsView)  — 视口（Y 轴翻转、保持宽高比）

功能
====
  - 白色场地背景 + 网格（10 cm 细线，50 cm 粗线）
  - 蓝色轨迹折线（每个 enabled 步骤贡献一段）
  - 每步终点编号圆圈：
      蓝 = 移动步骤选中
      橙 = 旋转步骤
      绿 = 执行机构步骤
      灰 = 非选中
  - 机器人方向箭头（在选中步骤的终点显示当前朝向）
  - 可拖动的起始位置手柄（绿圈）
  - 越界点显示红色边框警告

信号
====
  FieldScene.start_handle_moved(x_cm, y_cm)  ← 手柄被拖动时发出
  FieldView.waypoint_clicked(int)             ← 点击路径点时发出（row index）
"""

from __future__ import annotations

import math
from typing import Optional

from PyQt5.QtCore import QPointF, Qt, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPen,
    QPolygonF,
    QTransform,
)
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QWidget,
)

from PyQt5.QtGui import QPainterPath

from .models import Pose, Step, StepType, MOTION_TYPES, LINEAR_TYPES
from .kinematics import dead_reckoning_poses


# ── 颜色常量 ────────────────────────────────────────────────────
C_BG          = QColor(50,  50,  55)      # 视口背景
C_FIELD       = QColor(255, 255, 255)     # 场地白色
C_GRID_MINOR  = QColor(210, 213, 220)     # 10 cm 细格
C_GRID_MAJOR  = QColor(160, 170, 185)     # 50 cm 粗格
C_PATH        = QColor(40,  120, 208)     # 轨迹线（蓝）
C_MOVE_PT     = QColor(40,  120, 208)     # 移动步骤路径点
C_ROTATE_PT   = QColor(210, 130,  20)     # 旋转步骤路径点
C_ACTION_PT   = QColor(40,  160,  80)     # 执行机构步骤路径点
C_DIM_PT      = QColor(190, 200, 215)     # 非选中路径点
C_ROBOT_SEL   = QColor(40,  120, 208)     # 选中行机器人箭头
C_ROBOT_DIM   = QColor(170, 190, 220)     # 未选中机器人箭头
C_START       = QColor(40,  180,  70)     # 起始手柄
C_OOB         = QColor(220,  60,  60)     # 越界警告

# ── Z 值分层 ────────────────────────────────────────────────────
Z_FIELD   = -100
Z_GRID    = -99
Z_PATH    =   0
Z_WAYPTS  =   5
Z_ROBOTS  =  10
Z_START   =  20

# ── 几何参数（cm） ───────────────────────────────────────────────
WAYPOINT_RADIUS = 4.0      # 路径点圆圈半径
ARROW_LENGTH    = 14.0     # 机器人箭头长
ARROW_WIDTH     = 6.0      # 机器人箭头最大宽
START_RADIUS    = 6.0      # 起始手柄圆圈半径


class _StartHandle(QGraphicsEllipseItem):
    """可拖动的起始位置手柄（绿色圆圈）。"""

    def __init__(self, x_cm: float, y_cm: float, scene: "FieldScene"):
        r = START_RADIUS
        super().__init__(-r, -r, r * 2, r * 2)
        self._scene = scene
        self._suppress = False   # 编程式移动时不发出信号
        self.setPos(x_cm, y_cm)
        self.setZValue(Z_START)
        self.setBrush(QBrush(C_START))
        self.setPen(QPen(C_START.darker(130), 1.0))
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.OpenHandCursor)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            pos = self.pos()
            # 钳制在场地范围内
            rect = self._scene.sceneRect()
            clamped_x = max(rect.left(),  min(rect.right(),  pos.x()))
            clamped_y = max(rect.bottom(), min(rect.top(),   pos.y()))
            # sceneRect 在 Y 轴翻转的 scene 里：bottom < top
            # 实际上 setSceneRect(0, 0, w, h) 后 top=h, bottom=0
            if pos.x() != clamped_x or pos.y() != clamped_y:
                self.setPos(clamped_x, clamped_y)
                return QPointF(clamped_x, clamped_y)
            if not self._suppress:
                self._scene.start_handle_moved.emit(pos.x(), pos.y())
        return super().itemChange(change, value)


class FieldScene(QGraphicsScene):
    """场地场景，所有渲染发生在这里。"""

    start_handle_moved = pyqtSignal(float, float)   # x_cm, y_cm
    waypoint_clicked   = pyqtSignal(int)             # step index (0-based)

    def __init__(self, width_cm: float = 465.5, height_cm: float = 305.5,
                 parent=None):
        super().__init__(parent)
        self._width  = float(width_cm)
        self._height = float(height_cm)
        self.setSceneRect(0, 0, self._width, self._height)

        self._path_item:    Optional[QGraphicsPathItem]    = None
        self._waypts:       list[QGraphicsItem]            = []
        self._robot_items:  list[QGraphicsItem]            = []
        self._handle:       Optional[_StartHandle]         = None

        # 静态元素
        self._draw_field()
        self._draw_grid()
        # 起始手柄默认位置
        self._handle = _StartHandle(50.0, 50.0, self)
        self.addItem(self._handle)

    # =========================================================
    #  公开 API
    # =========================================================
    def set_field_size(self, width_cm: float, height_cm: float) -> None:
        self._width  = float(width_cm)
        self._height = float(height_cm)
        self.setSceneRect(0, 0, self._width, self._height)
        # 重绘静态元素
        for item in self.items():
            if item is not self._handle and item not in self._waypts \
                    and item is not self._path_item \
                    and item not in self._robot_items:
                self.removeItem(item)
        self._draw_field()
        self._draw_grid()

    def set_start_pose(self, x_cm: float, y_cm: float) -> None:
        """从外部设置手柄位置（不触发 start_handle_moved 信号）。"""
        if self._handle:
            self._handle._suppress = True
            self._handle.setPos(x_cm, y_cm)
            self._handle._suppress = False

    def update_preview(self, steps: list[Step], start: Pose,
                       selected_row: int = -1) -> None:
        """完全重绘动态部分（路径、路径点、机器人箭头）。"""
        poses = dead_reckoning_poses(steps, start)
        self._rebuild_path(poses)
        self._rebuild_waypoints(steps, poses, selected_row)
        self._rebuild_robot_arrow(poses, selected_row)

    # =========================================================
    #  静态元素
    # =========================================================
    def _draw_field(self) -> None:
        rect = self.addRect(
            0, 0, self._width, self._height,
            QPen(QColor(100, 100, 110), 1.0),
            QBrush(C_FIELD),
        )
        rect.setZValue(Z_FIELD)

    def _draw_grid(self) -> None:
        w, h = self._width, self._height
        # 细格 10 cm
        pen_minor = QPen(C_GRID_MINOR, 0.3)
        x = 0.0
        while x <= w:
            line = self.addLine(x, 0, x, h, pen_minor)
            line.setZValue(Z_GRID)
            x += 10.0
        y = 0.0
        while y <= h:
            line = self.addLine(0, y, w, y, pen_minor)
            line.setZValue(Z_GRID)
            y += 10.0
        # 粗格 50 cm
        pen_major = QPen(C_GRID_MAJOR, 0.7)
        x = 0.0
        while x <= w:
            line = self.addLine(x, 0, x, h, pen_major)
            line.setZValue(Z_GRID)
            x += 50.0
        y = 0.0
        while y <= h:
            line = self.addLine(0, y, w, y, pen_major)
            line.setZValue(Z_GRID)
            y += 50.0

    # =========================================================
    #  动态元素
    # =========================================================
    def _rebuild_path(self, poses: list[Pose]) -> None:
        if self._path_item:
            self.removeItem(self._path_item)
            self._path_item = None

        if len(poses) < 2:
            return

        path = QPainterPath()
        path.moveTo(poses[0].x, poses[0].y)
        for p in poses[1:]:
            path.lineTo(p.x, p.y)

        item = self.addPath(path, QPen(C_PATH, 2.0))
        item.setZValue(Z_PATH)
        self._path_item = item

    def _rebuild_waypoints(self, steps: list[Step], poses: list[Pose],
                           selected_row: int) -> None:
        for old in self._waypts:
            self.removeItem(old)
        self._waypts.clear()

        # poses[0] = start（起始手柄），poses[i+1] = steps[i] 之后
        for i, step in enumerate(steps):
            pose = poses[i + 1]
            is_selected = (i == selected_row)

            # 越界检查
            out_of_bounds = (
                pose.x < 0 or pose.x > self._width or
                pose.y < 0 or pose.y > self._height
            )

            # 颜色
            if is_selected:
                if step.step_type in LINEAR_TYPES:
                    color = C_MOVE_PT
                elif step.step_type in (StepType.ROTATE_CW, StepType.ROTATE_CCW):
                    color = C_ROTATE_PT
                else:
                    color = C_ACTION_PT
            else:
                color = C_DIM_PT

            if out_of_bounds:
                border_pen = QPen(C_OOB, 1.5)
            else:
                border_pen = QPen(color.darker(120), 0.5)

            r = WAYPOINT_RADIUS
            circle = self.addEllipse(
                pose.x - r, pose.y - r, r * 2, r * 2,
                border_pen,
                QBrush(C_FIELD if not is_selected else color),
            )
            circle.setZValue(Z_WAYPTS)
            # 使圆圈可点击
            circle.setFlag(QGraphicsItem.ItemIsSelectable, False)
            self._waypts.append(circle)

            # 编号标签（在路径点上方显示步骤序号）
            if is_selected or len(steps) <= 30:
                lbl = QGraphicsSimpleTextItem(str(i + 1))
                lbl.setFont(QFont("Arial", 3))  # 3 cm 字体（scene 坐标）
                lbl.setBrush(QBrush(color if is_selected else C_DIM_PT))
                lbl.setPos(pose.x + r + 0.5, pose.y + r * 0.5)
                # 文字需要单独翻转（因为 View 做了 scale(1,-1)）
                lbl.setTransform(QTransform().scale(1, -1))
                lbl.setZValue(Z_WAYPTS + 1)
                self.addItem(lbl)
                self._waypts.append(lbl)

    def _rebuild_robot_arrow(self, poses: list[Pose],
                             selected_row: int) -> None:
        for old in self._robot_items:
            self.removeItem(old)
        self._robot_items.clear()

        # 起始箭头（总是显示，半透明）
        if poses:
            self._add_arrow(poses[0], highlighted=False)

        # 选中步骤后的箭头
        if 0 <= selected_row < len(poses) - 1:
            self._add_arrow(poses[selected_row + 1], highlighted=True)

    def _add_arrow(self, pose: Pose, highlighted: bool) -> None:
        """在 pose 位置绘制一个三角形机器人方向箭头。"""
        L, W = ARROW_LENGTH, ARROW_WIDTH
        tip   = QPointF(0,  L * 0.6)
        left  = QPointF(-W / 2, -L * 0.4)
        right = QPointF( W / 2, -L * 0.4)
        poly  = QPolygonF([tip, left, right])

        item = QGraphicsPolygonItem(poly)
        color = C_ROBOT_SEL if highlighted else C_ROBOT_DIM
        item.setBrush(QBrush(color))
        item.setPen(QPen(color.darker(130), 0.8))

        # heading=0 → 朝 +Y（已经对齐），顺时针旋转 heading 度
        item.setRotation(pose.heading)
        item.setPos(pose.x, pose.y)
        item.setZValue(Z_ROBOTS)
        self.addItem(item)
        self._robot_items.append(item)


class FieldView(QGraphicsView):
    """场地视口（保持宽高比，Y 轴翻转使 +Y 朝上）。"""

    waypoint_clicked = pyqtSignal(int)   # 传递给 main_window

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._field_scene = FieldScene()
        self.setScene(self._field_scene)

        # Y 轴翻转：场景 +Y 朝上，与场地坐标系一致
        self.setTransform(QTransform().scale(1, -1))

        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(C_BG))
        self.setDragMode(QGraphicsView.NoDrag)

    @property
    def field_scene(self) -> FieldScene:
        return self._field_scene

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.fitInView(self._field_scene.sceneRect(), Qt.KeepAspectRatio)
