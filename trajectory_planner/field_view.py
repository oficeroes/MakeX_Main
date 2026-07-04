"""场地画布：QGraphicsScene/View 单位 = cm，+Y 朝上，自动保持纵横比

坐标系设计
==========
场景单位直接等于厘米，消除像素↔cm 换算。
FieldView 用 scale(1, -1) 翻转 Y 轴，使屏幕"上方"对应 +Y（机器人前进方向）。
窗口缩放时 resizeEvent → fitInView(KeepAspectRatio) 保持场地比例，
多余区域显示深灰色 letterbox。

FieldScene（QGraphicsScene）
============================
职责：场地边框 + 网格 + 障碍物列表 + 多段轨迹 + 执行块链 + 碰撞检测

  场地边框：白色矩形，zValue=-100
  网格：每 GRID_SPACING_CM(=10) cm 一条浅灰线，50 cm 倍数的线稍深，zValue=-99
  障碍物：ObstacleItem 列表，zValue=10
  轨迹：path_segments 列表（每段一个 PathItem），zValue=20
  执行块：action_chains 列表（与 path_segments 平行），zValue=25

多段路径模型
============
  path_segments: list[PathItem]                  — 每次鼠标释放完成一段
  action_chains: list[list[ActionBlockItem]]     — 每段对应若干执行块，按顺序排列
  path_item property: 最后一段（或哑元 PathItem）— 向后兼容

执行块布局
==========
  块吸附在路段终点右侧，垂直向上堆叠（间距 = BLOCK_GAP cm）
  _reposition_blocks(seg_idx) 在任何改变后刷新位置

绘制流程（鼠标）
================
  mousePressEvent  → start_path(scene_pos)   : append 新段，开始采样
  mouseMoveEvent   → extend_path(scene_pos)  : 追加采样点 + preview_raw()
  mouseReleaseEvent→ finish_path()           : 平滑 + 重采样 + 刷新碰撞 + 发信号

信号
====
  path_finalized(int)  鼠标抬起、平滑完成后发射，携带段索引
  obstacles_changed    障碍物增删或几何变化后发射
  mouse_pos_cm         鼠标在场景中的 (x_cm, y_cm)
"""
from PyQt5 import QtCore, QtGui, QtWidgets

from .config import GRID_SPACING_CM
from .obstacle_item import ObstacleItem
from .path_item import PathItem
from .smoothing import smooth_and_resample
from .action_block_item import ActionBlockItem, block_anchor


class FieldScene(QtWidgets.QGraphicsScene):
    """场景：固定矩形场地 + 网格 + 障碍物 + 多段轨迹 + 执行块"""

    path_finalized = QtCore.pyqtSignal(int)    # 参数：完成的段索引
    obstacles_changed = QtCore.pyqtSignal()
    mouse_pos_cm = QtCore.pyqtSignal(float, float)

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

        self._corner_items = []
        self._rebuild_corners()

        # 多段路径（与 vehicle_ids 保持索引对齐）
        self.path_segments = []         # list[PathItem]
        self.action_chains = []         # list[list[ActionBlockItem]]
        self.segment_vehicle_ids = []   # list[str]  — 每段对应的 profile_id
        self._drawing = False
        self._drawing_seg = -1

        # 当前绘制车型（由 MainWindow 在画线前注入）
        self.active_vehicle_id = "omni3"
        self.active_vehicle_color = "#2878D0"

        # 哪些车型路径可见（空集 = 全可见）
        self._hidden_vehicles = set()

        # 共同路径叠加层（OverlapItem 列表）
        self._overlap_items = []

        self._dummy_path = PathItem()
        self.addItem(self._dummy_path)

        self.smooth_iter = 3
        self.resample_step = 5.0

    # ---- 向后兼容 ----
    @property
    def path_item(self):
        return self.path_segments[-1] if self.path_segments else self._dummy_path

    # ---- 场地尺寸 ----
    def set_field_size(self, w, h):
        self._width = float(w)
        self._height = float(h)
        self.setSceneRect(0, 0, self._width, self._height)
        self._field_rect.setRect(0, 0, self._width, self._height)
        self._rebuild_grid()
        self._rebuild_corners()
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

    def _rebuild_corners(self):
        """四个角落 500×500mm 正方形：上方红色、下方蓝色"""
        for it in self._corner_items:
            self.removeItem(it)
        self._corner_items = []

        side = 50.0   # 500 mm
        red = QtGui.QColor(200, 40, 40, 140)
        blue = QtGui.QColor(40, 80, 200, 140)
        pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 80), 0.5)

        corners = [
            # (x, y, color) — 场景坐标，+Y 朝上
            (0,                    self._height - side, red),   # 左上（红）
            (self._width - side,   self._height - side, red),   # 右上（红）
            (0,                    0,                    blue),  # 左下（蓝）
            (self._width - side,   0,                    blue),  # 右下（蓝）
        ]
        for x, y, color in corners:
            r = QtWidgets.QGraphicsRectItem(x, y, side, side)
            r.setBrush(QtGui.QBrush(color))
            r.setPen(pen)
            r.setZValue(-98)   # 网格上方、场地下方
            r.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, False)
            self.addItem(r)
            self._corner_items.append(r)

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
        self._refresh_collision()
        self.obstacles_changed.emit()

    # ---- 碰撞检测 ----
    def _refresh_collision(self):
        obs_rects = [obs.rect_scene() for obs in self.obstacles()]
        for seg in self.path_segments:
            if not seg.smoothed_points:
                seg.set_collision(False)
                continue
            p = seg.path()
            collide = any(
                p.intersects(QtCore.QRectF(x, y, w, h))
                for x, y, w, h in obs_rects
            )
            seg.set_collision(collide)

    # ---- 绘制轨迹 ----
    def start_path(self, scene_pos):
        self._drawing = True
        seg = PathItem(color=self.active_vehicle_color)
        self.addItem(seg)
        self.path_segments.append(seg)
        self.action_chains.append([])
        self.segment_vehicle_ids.append(self.active_vehicle_id)
        self._drawing_seg = len(self.path_segments) - 1
        seg.append_raw(scene_pos.x(), scene_pos.y())
        seg.preview_raw()

    def extend_path(self, scene_pos):
        if not self._drawing or self._drawing_seg < 0:
            return
        seg = self.path_segments[self._drawing_seg]
        seg.append_raw(scene_pos.x(), scene_pos.y())
        seg.preview_raw()

    def finish_path(self):
        if not self._drawing or self._drawing_seg < 0:
            return
        self._drawing = False
        idx = self._drawing_seg
        self._drawing_seg = -1
        seg = self.path_segments[idx]
        raw = list(seg.raw_points)
        if len(raw) >= 2:
            smoothed = smooth_and_resample(raw, self.smooth_iter, self.resample_step)
        else:
            smoothed = list(raw)
        seg.set_points(raw, smoothed)
        self._reposition_blocks(idx)
        self._refresh_collision()
        self.recompute_overlaps()
        self.path_finalized.emit(idx)

    def resmooth(self, iterations, step, seg_idx=None):
        self.smooth_iter = iterations
        self.resample_step = step
        indices = range(len(self.path_segments)) if seg_idx is None else [seg_idx]
        for i in indices:
            seg = self.path_segments[i]
            raw = list(seg.raw_points)
            if len(raw) >= 2:
                smoothed = smooth_and_resample(raw, iterations, step)
            else:
                smoothed = list(raw)
            seg.set_points(raw, smoothed)
            self._reposition_blocks(i)
        self._refresh_collision()

    def clear_path(self):
        for seg in self.path_segments:
            self.removeItem(seg)
        for chain in self.action_chains:
            for block in chain:
                self.removeItem(block)
        for ov in self._overlap_items:
            self.removeItem(ov)
        self.path_segments = []
        self.action_chains = []
        self.segment_vehicle_ids = []
        self._overlap_items = []
        self._drawing = False
        self._drawing_seg = -1
        self._refresh_collision()

    # ---- 车型显隐 ----
    def set_vehicle_visible(self, vehicle_id, visible):
        """显示或隐藏某个车型的所有路段及其执行块"""
        if visible:
            self._hidden_vehicles.discard(vehicle_id)
        else:
            self._hidden_vehicles.add(vehicle_id)
        for idx, seg in enumerate(self.path_segments):
            vid = self.segment_vehicle_ids[idx] if idx < len(self.segment_vehicle_ids) else ""
            is_visible = vid not in self._hidden_vehicles
            seg.setVisible(is_visible)
            for block in self.action_chains[idx]:
                block.setVisible(is_visible)

    def is_vehicle_visible(self, vehicle_id):
        return vehicle_id not in self._hidden_vehicles

    # ---- 执行块 ----
    def add_action_block(self, seg_idx, block):
        if seg_idx < 0 or seg_idx >= len(self.path_segments):
            return
        self.addItem(block)
        self.action_chains[seg_idx].append(block)
        self._reposition_blocks(seg_idx)

    def remove_action_block(self, seg_idx, block):
        if seg_idx < 0 or seg_idx >= len(self.action_chains):
            return
        chain = self.action_chains[seg_idx]
        if block in chain:
            chain.remove(block)
            self.removeItem(block)
            self._reposition_blocks(seg_idx)

    def move_action_block(self, seg_idx, old_pos, new_pos):
        chain = self.action_chains[seg_idx]
        if 0 <= old_pos < len(chain) and 0 <= new_pos < len(chain):
            chain.insert(new_pos, chain.pop(old_pos))
            self._reposition_blocks(seg_idx)

    def _reposition_blocks(self, seg_idx):
        if seg_idx < 0 or seg_idx >= len(self.path_segments):
            return
        seg = self.path_segments[seg_idx]
        pts = seg.smoothed_points or seg.raw_points
        if not pts:
            return
        end_x, end_y = pts[-1]
        for i, block in enumerate(self.action_chains[seg_idx]):
            x, y = block_anchor(end_x, end_y, i)
            block.setPos(x, y)

    def action_blocks_flat(self):
        result = []
        for i, chain in enumerate(self.action_chains):
            for block in chain:
                result.append((i, block))
        return result

    def rebuild_from_segments(self, segments_data, action_chains_data=None):
        """从 JSON 数据重建多段路径 + 执行块（含 vehicle_id）"""
        self.clear_path()
        if action_chains_data is None:
            action_chains_data = [[] for _ in segments_data]

        from .config import get_profile
        for seg_data, chain_data in zip(segments_data, action_chains_data):
            raw = [tuple(p) for p in seg_data.get("raw_points_cm", [])]
            smooth = [tuple(p) for p in seg_data.get("smoothed_points_cm", [])]
            if not raw:
                continue
            vid = seg_data.get("vehicle_id", self.active_vehicle_id)
            profile = get_profile(vid)
            seg = PathItem(color=profile.color)
            self.addItem(seg)
            seg.set_points(raw, smooth or raw)
            self.path_segments.append(seg)
            self.segment_vehicle_ids.append(vid)
            # 若该车型被隐藏，恢复时也隐藏
            if vid in self._hidden_vehicles:
                seg.setVisible(False)

            chain = []
            for bd in chain_data:
                block = ActionBlockItem.from_dict(bd)
                self.addItem(block)
                if vid in self._hidden_vehicles:
                    block.setVisible(False)
                chain.append(block)
            self.action_chains.append(chain)
            self._reposition_blocks(len(self.path_segments) - 1)

        self._refresh_collision()
        self.recompute_overlaps()

    # ---- 共同路径检测 ----
    def recompute_overlaps(self, threshold_cm=8.0):
        """找出不同车型路径中相互靠近的片段，用黄色粗线高亮叠加"""
        # 清除旧叠加层
        for ov in self._overlap_items:
            self.removeItem(ov)
        self._overlap_items = []

        # 按车型分组，每组收集所有平滑点
        from collections import defaultdict
        vehicle_pts = defaultdict(list)   # vehicle_id -> list of (x,y)
        for idx, seg in enumerate(self.path_segments):
            vid = self.segment_vehicle_ids[idx] if idx < len(self.segment_vehicle_ids) else ""
            vehicle_pts[vid].extend(seg.smoothed_points or seg.raw_points)

        vehicles = list(vehicle_pts.keys())
        if len(vehicles) < 2:
            return   # 只有一辆车，无需对比

        # 对每对车型找重叠点
        overlap_pts = []
        for i in range(len(vehicles)):
            for j in range(i + 1, len(vehicles)):
                pts_a = vehicle_pts[vehicles[i]]
                pts_b = vehicle_pts[vehicles[j]]
                for ax, ay in pts_a:
                    for bx, by in pts_b:
                        if (ax - bx) ** 2 + (ay - by) ** 2 <= threshold_cm ** 2:
                            overlap_pts.append(((ax + bx) / 2, (ay + by) / 2))

        if not overlap_pts:
            return

        # 用 DBSCAN 风格简单聚类，把相邻点连成折线段
        clusters = _cluster_points(overlap_pts, eps=threshold_cm * 1.5)
        pen = QtGui.QPen(QtGui.QColor(255, 220, 0, 200), 5.0)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            path = QtGui.QPainterPath()
            path.moveTo(cluster[0][0], cluster[0][1])
            for x, y in cluster[1:]:
                path.lineTo(x, y)
            item = QtWidgets.QGraphicsPathItem(path)
            item.setPen(pen)
            item.setZValue(22)   # 在路径上方
            item.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, False)
            self.addItem(item)
            self._overlap_items.append(item)


def _cluster_points(pts, eps):
    """简单贪心聚类：把距离 <= eps 的相邻点串成一条折线。
    返回 list[list[(x,y)]]，每个子列表是一条重叠折线。
    """
    if not pts:
        return []
    visited = [False] * len(pts)
    clusters = []
    for i, (x0, y0) in enumerate(pts):
        if visited[i]:
            continue
        cluster = [(x0, y0)]
        visited[i] = True
        # 贪心：从当前末端找最近的未访问点
        while True:
            cx, cy = cluster[-1]
            best_j, best_d2 = -1, eps * eps
            for j, (xj, yj) in enumerate(pts):
                if visited[j]:
                    continue
                d2 = (cx - xj) ** 2 + (cy - yj) ** 2
                if d2 <= best_d2:
                    best_d2 = d2
                    best_j = j
            if best_j < 0:
                break
            cluster.append(pts[best_j])
            visited[best_j] = True
        clusters.append(cluster)
    return clusters


class FieldView(QtWidgets.QGraphicsView):
    """视图：自动保持场地纵横比；左键绘制轨迹（除非按到障碍物/手柄/执行块）"""

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        self.scale(1, -1)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(60, 60, 65)))
        self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # 全视口更新：消除 ItemIsMovable 拖拽时的边框鬼影
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.FullViewportUpdate)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            # Shift + 拖拽 → 橡皮筋框选
            if event.modifiers() & QtCore.Qt.ShiftModifier:
                self.setDragMode(QtWidgets.QGraphicsView.RubberBandDrag)
                super().mousePressEvent(event)
                return

            item = self.itemAt(event.pos())
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
        scene_pos = self.mapToScene(event.pos())
        self.scene().mouse_pos_cm.emit(scene_pos.x(), scene_pos.y())
        # 橡皮筋框选进行中 → Qt 接管
        if self.dragMode() == QtWidgets.QGraphicsView.RubberBandDrag:
            super().mouseMoveEvent(event)
            return
        if event.buttons() & QtCore.Qt.LeftButton and self.scene()._drawing:
            self.scene().extend_path(scene_pos)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            # 橡皮筋框选结束 → 重置为 NoDrag
            if self.dragMode() == QtWidgets.QGraphicsView.RubberBandDrag:
                super().mouseReleaseEvent(event)
                self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
                event.accept()
                return
            if self.scene()._drawing:
                self.scene().finish_path()
                event.accept()
                return
        super().mouseReleaseEvent(event)

    @staticmethod
    def _is_background(item):
        from .obstacle_item import ObstacleItem, _Handle
        from .path_item import PathItem as _PathItem
        if isinstance(item, (ObstacleItem, _Handle)):
            return False
        if isinstance(item, _PathItem):
            return False
        if isinstance(item, ActionBlockItem):
            return False
        return True
