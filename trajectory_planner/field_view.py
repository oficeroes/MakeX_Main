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
from .smoothing import normalize_drawn_path, smooth_and_resample
from .action_block_item import ActionBlockItem, block_anchor, block_anchor_head
from . import curves


class FieldScene(QtWidgets.QGraphicsScene):
    """场景：固定矩形场地 + 网格 + 障碍物 + 多段轨迹 + 执行块"""

    path_finalized = QtCore.pyqtSignal(int)    # 参数：完成的段索引
    obstacles_changed = QtCore.pyqtSignal()
    mouse_pos_cm = QtCore.pyqtSignal(float, float)
    path_anchor_edited = QtCore.pyqtSignal()    # 控制点曲线锚点被拖动后重算
    block_anchor_changed = QtCore.pyqtSignal()  #块拖拽后头/尾归属改变

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

        self._center_safe_item = QtWidgets.QGraphicsRectItem()
        safe_pen = QtGui.QPen(QtGui.QColor(220, 40, 40, 190), 0.8, QtCore.Qt.DashLine)
        self._center_safe_item.setPen(safe_pen)
        self._center_safe_item.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))
        self._center_safe_item.setZValue(-97)
        self._center_safe_item.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, False)
        self.addItem(self._center_safe_item)
        self._safe_margin_x = 0.0
        self._safe_margin_y = 0.0
        self._safe_robot_width = 0.0
        self._safe_robot_length = 0.0
        self._safe_extra_margin = 0.0

        # 多段路径（与 vehicle_ids 保持索引对齐）
        self.path_segments = []         # list[PathItem]
        self.action_chains = []         # list[list[ActionBlockItem]]
        self.segment_vehicle_ids = []   # list[str]  — 每段对应的 profile_id
        self._drawing = False
        self._drawing_seg = -1

        # 绘制工具："freehand"（手绘）/ "spline"（样条）/ "line"（直线）
        # spline / line 是"点锚点"模式：单击落点，双击/回车完成。
        self.draw_tool = "freehand"
        self._anchor_pts = []           # 正在放置的锚点（场景坐标）
        self._anchor_preview = None     # 放置中的预览 PathItem
        self._placing = False           # 是否正在放锚点

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

    # ---- 绘制采样 ----
    def _draw_sample_spacing(self):
        return max(0.2, min(float(self.resample_step) * 0.12, 0.75))

    def _append_draw_sample(self, seg, scene_pos, force=False):
        x = float(scene_pos.x())
        y = float(scene_pos.y())
        if not seg.raw_points:
            seg.append_raw(x, y)
            return True

        last_x, last_y = seg.raw_points[-1]
        dx = x - last_x
        dy = y - last_y
        dist2 = dx * dx + dy * dy

        if force:
            if dist2 > 1e-9:
                seg.append_raw(x, y)
                return True
            return False

        spacing = self._draw_sample_spacing()
        if dist2 >= spacing * spacing:
            seg.append_raw(x, y)
            return True
        return False

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
        self.set_center_safe_area(
            self._safe_robot_width,
            self._safe_robot_length,
            self._safe_extra_margin,
        )
        self.update()

    def field_size(self):
        return self._width, self._height

    def set_center_safe_area(self, robot_width_cm, robot_length_cm, safety_margin_cm=0.0):
        """显示小车中心点允许活动的矩形区域。"""
        self._safe_robot_width = float(robot_width_cm)
        self._safe_robot_length = float(robot_length_cm)
        self._safe_extra_margin = float(safety_margin_cm)
        self._safe_margin_x = max(0.0, float(robot_width_cm) / 2.0 + float(safety_margin_cm))
        self._safe_margin_y = max(0.0, float(robot_length_cm) / 2.0 + float(safety_margin_cm))
        x = self._safe_margin_x
        y = self._safe_margin_y
        w = max(0.0, self._width - x * 2.0)
        h = max(0.0, self._height - y * 2.0)
        self._center_safe_item.setRect(x, y, w, h)
        self._center_safe_item.setVisible(w > 0 and h > 0)

    def _grid_spacing(self):
        """网格间距自适应：大场地时放大间距，避免生成上万条线卡界面。

        目标是任一方向的网格线数量不超过 ~200 条。基准间距 GRID_SPACING_CM，
        必要时按 1/2/5/10… 的档位逐级放大。major 线间隔恒为 minor 的 5 倍。
        """
        base = GRID_SPACING_CM
        longest = max(self._width, self._height)
        target_max_lines = 200
        step = base
        # 逐级放大到线数达标（1,2,5,10,20,50,... × base）
        mults = [1, 2, 5]
        decade = 1
        while longest / step > target_max_lines:
            step = base * decade * mults[0]
            for m in mults:
                cand = base * decade * m
                if longest / cand <= target_max_lines:
                    step = cand
                    break
            else:
                decade *= 10
                continue
            break
        return step

    def _rebuild_grid(self):
        for it in self._grid_items:
            self.removeItem(it)
        self._grid_items = []
        pen_minor = QtGui.QPen(QtGui.QColor(230, 230, 230), 0.15)
        pen_major = QtGui.QPen(QtGui.QColor(200, 200, 200), 0.3)
        step = self._grid_spacing()
        major_every = step * 5   # 每 5 格一条深线
        x = step
        while x < self._width:
            line = QtWidgets.QGraphicsLineItem(x, 0, x, self._height)
            is_major = abs((x / major_every) - round(x / major_every)) < 1e-6
            line.setPen(pen_major if is_major else pen_minor)
            line.setZValue(-99)
            self.addItem(line)
            self._grid_items.append(line)
            x += step
        y = step
        while y < self._height:
            line = QtWidgets.QGraphicsLineItem(0, y, self._width, y)
            is_major = abs((y / major_every) - round(y / major_every)) < 1e-6
            line.setPen(pen_major if is_major else pen_minor)
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
        self._append_draw_sample(seg, scene_pos, force=True)
        seg.preview_raw()

    def extend_path(self, scene_pos):
        if not self._drawing or self._drawing_seg < 0:
            return
        seg = self.path_segments[self._drawing_seg]
        if self._append_draw_sample(seg, scene_pos):
            seg.preview_raw()

    def finish_path(self, scene_pos=None):
        if not self._drawing or self._drawing_seg < 0:
            return
        self._drawing = False
        idx = self._drawing_seg
        self._drawing_seg = -1
        seg = self.path_segments[idx]
        if scene_pos is not None:
            self._append_draw_sample(seg, scene_pos, force=True)
        raw = list(seg.raw_points)
        if len(raw) >= 2:
            raw = normalize_drawn_path(raw, self.resample_step)
            smoothed = smooth_and_resample(raw, self.smooth_iter, self.resample_step)
        else:
            smoothed = list(raw)
        seg.set_points(raw, smoothed)
        self._reposition_blocks(idx)
        self._refresh_collision()
        self.recompute_overlaps()
        self.path_finalized.emit(idx)

    # ---- anchor curve placement (spline / line tools) ----
    def begin_anchor_placement(self, scene_pos):
        """Start a control-point curve: drop the first anchor."""
        self._placing = True
        self._anchor_pts = [(float(scene_pos.x()), float(scene_pos.y()))]
        if self._anchor_preview is None:
            self._anchor_preview = PathItem(color=self.active_vehicle_color)
            self.addItem(self._anchor_preview)
        self._anchor_preview.set_max_spacing(self.resample_step)
        self._refresh_anchor_preview(scene_pos)

    def add_anchor(self, scene_pos):
        """Drop another anchor while placing (skip near-duplicate of last)."""
        if not self._placing:
            return
        x, y = float(scene_pos.x()), float(scene_pos.y())
        if self._anchor_pts:
            lx, ly = self._anchor_pts[-1]
            if (x - lx) ** 2 + (y - ly) ** 2 < 1.0:   # < 1cm: treat as same click
                self._refresh_anchor_preview(scene_pos)
                return
        self._anchor_pts.append((x, y))
        self._refresh_anchor_preview(scene_pos)

    def update_anchor_preview(self, scene_pos):
        """Rubber-band: preview the curve through placed anchors + cursor."""
        if not self._placing:
            return
        self._refresh_anchor_preview(scene_pos)

    def _refresh_anchor_preview(self, cursor_pos):
        if self._anchor_preview is None:
            return
        pts = list(self._anchor_pts)
        if cursor_pos is not None:
            cx, cy = float(cursor_pos.x()), float(cursor_pos.y())
            if not pts or (pts[-1][0] != cx or pts[-1][1] != cy):
                pts = pts + [(cx, cy)]
        if len(pts) >= 2:
            self._anchor_preview.set_curve(pts, self.draw_tool,
                                           max_spacing=self.resample_step)
        else:
            self._anchor_preview.set_points(pts, pts)

    def finish_anchor_placement(self):
        """Commit the placed anchors as a real path segment. Returns True if kept."""
        if not self._placing:
            return False
        self._placing = False
        anchors = list(self._anchor_pts)
        self._anchor_pts = []
        if self._anchor_preview is not None:
            self.removeItem(self._anchor_preview)
            self._anchor_preview = None
        if len(anchors) < 2:
            return False

        seg = PathItem(color=self.active_vehicle_color)
        self.addItem(seg)
        seg.set_max_spacing(self.resample_step)
        seg.set_curve(anchors, self.draw_tool, max_spacing=self.resample_step)
        self.path_segments.append(seg)
        self.action_chains.append([])
        self.segment_vehicle_ids.append(self.active_vehicle_id)
        idx = len(self.path_segments) - 1
        self._reposition_blocks(idx)
        self._refresh_collision()
        self.recompute_overlaps()
        self.path_finalized.emit(idx)
        return True

    def cancel_anchor_placement(self):
        """Abort placing (Esc). Returns True if something was aborted."""
        if not self._placing:
            return False
        self._placing = False
        self._anchor_pts = []
        if self._anchor_preview is not None:
            self.removeItem(self._anchor_preview)
            self._anchor_preview = None
        return True

    def on_path_anchor_edited(self, seg):
        """A PathItem anchor handle was dragged: refresh collision + overlaps."""
        self._refresh_collision()
        self.recompute_overlaps()
        if seg in self.path_segments:
            self._reposition_blocks(self.path_segments.index(seg))
        self.path_anchor_edited.emit()

    def resmooth(self, iterations, step, seg_idx=None):
        self.smooth_iter = iterations
        self.resample_step = step
        indices = range(len(self.path_segments)) if seg_idx is None else [seg_idx]
        for i in indices:
            seg = self.path_segments[i]
            # 控制点曲线：按新的加密间距从锚点重算，不做 Chaikin（会破坏样条）
            if seg.has_anchors():
                seg.set_max_spacing(step)
                seg.set_curve(seg.anchor_points, seg.draw_kind, step)
                self._reposition_blocks(i)
                continue
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

    def cancel_path(self):
        """取消正在绘制的路段（Esc）。返回 True 表示确实取消了一段。"""
        if not self._drawing or self._drawing_seg < 0:
            return False
        idx = self._drawing_seg
        self._drawing = False
        self._drawing_seg = -1
        if 0 <= idx < len(self.path_segments):
            self.removeItem(self.path_segments[idx])
            del self.path_segments[idx]
            del self.action_chains[idx]
            del self.segment_vehicle_ids[idx]
        return True

    def select_all(self):
        """全选：所有路段 + 障碍物 + 执行块（Ctrl+A）"""
        for seg in self.path_segments:
            if seg.isVisible():
                seg.setSelected(True)
        for obs in self.obstacles():
            obs.setSelected(True)
        for chain in self.action_chains:
            for block in chain:
                if block.isVisible():
                    block.setSelected(True)

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
        block.drop_requested.connect(self._on_block_dropped)
        block.anchor_requested.connect(self._on_block_anchor_requested)
        self.action_chains[seg_idx].append(block)
        self._reposition_blocks(seg_idx)

    def _seg_idx_of_block(self, block):
        for i, chain in enumerate(self.action_chains):
            if block in chain:
                return i
        return None

    def _set_block_anchor(self, block, seg_idx, new_anchor):
        """把块设为指定锚点（头/尾），重排并在改变时发信号。"""
        if new_anchor != getattr(block, "anchor", ActionBlockItem.ANCHOR_TAIL):
            block.anchor = new_anchor
            # 移到链内同锚点组的末尾，保持竖排顺序 = 执行顺序
            chain = self.action_chains[seg_idx]
            chain.remove(block)
            chain.append(block)
            self._reorder_chain_by_anchor(seg_idx)
            self.block_anchor_changed.emit()
        self._reposition_blocks(seg_idx)

    def _on_block_anchor_requested(self, block, anchor):
        """右键菜单直接把块设为头部/尾部。"""
        seg_idx = self._seg_idx_of_block(block)
        if seg_idx is None:
            return
        self._set_block_anchor(block, seg_idx, anchor)

    def _on_block_dropped(self, block, drop_pos):
        """块拖拽结束：按落点相对路径起点/终点的远近，归到头部或尾部。

        落点更靠近某段起点 → 头部（开画前执行）；更靠近末端 → 尾部（跑完后执行）。
        归属改变后重排并通知窗口刷新序列树。
        """
        seg_idx = self._seg_idx_of_block(block)
        if seg_idx is None:
            return
        seg = self.path_segments[seg_idx]
        pts = seg.smoothed_points or seg.raw_points
        if not pts:
            return
        sx, sy = pts[0]
        ex, ey = pts[-1]
        dx_s, dy_s = drop_pos.x() - sx, drop_pos.y() - sy
        dx_e, dy_e = drop_pos.x() - ex, drop_pos.y() - ey
        d_start = dx_s * dx_s + dy_s * dy_s
        d_end = dx_e * dx_e + dy_e * dy_e
        new_anchor = (ActionBlockItem.ANCHOR_HEAD if d_start < d_end
                      else ActionBlockItem.ANCHOR_TAIL)
        self._set_block_anchor(block, seg_idx, new_anchor)

    def _reorder_chain_by_anchor(self, seg_idx):
        """把链重排成 [所有头部块…, 所有尾部块…]，各组内保持相对顺序。"""
        chain = self.action_chains[seg_idx]
        heads = [b for b in chain if getattr(b, "anchor", ActionBlockItem.ANCHOR_TAIL)
                 == ActionBlockItem.ANCHOR_HEAD]
        tails = [b for b in chain if getattr(b, "anchor", ActionBlockItem.ANCHOR_TAIL)
                 != ActionBlockItem.ANCHOR_HEAD]
        self.action_chains[seg_idx] = heads + tails

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
        start_x, start_y = pts[0]
        end_x, end_y = pts[-1]
        # 头部块排在起点左侧（开画前执行），尾部块排在末端右侧（跑完后执行）。
        # order 在各自组内从 0 开始，保证竖排编号连续。
        head_i = 0
        tail_i = 0
        for block in self.action_chains[seg_idx]:
            if getattr(block, "anchor", ActionBlockItem.ANCHOR_TAIL) == ActionBlockItem.ANCHOR_HEAD:
                x, y = block_anchor_head(start_x, start_y, head_i)
                block.order = head_i
                head_i += 1
            else:
                x, y = block_anchor(end_x, end_y, tail_i)
                block.order = tail_i
                tail_i += 1
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
            seg.set_max_spacing(self.resample_step)
            self.addItem(seg)
            seg.set_points(raw, smooth or raw)
            # 恢复控制点曲线（若该段是样条/直线）
            curve_d = seg_data.get("curve")
            if curve_d:
                seg.apply_curve_dict(curve_d)
            self.path_segments.append(seg)
            self.segment_vehicle_ids.append(vid)
            # 若该车型被隐藏，恢复时也隐藏
            if vid in self._hidden_vehicles:
                seg.setVisible(False)

            chain = []
            for bd in chain_data:
                block = ActionBlockItem.from_dict(bd)
                self.addItem(block)
                block.drop_requested.connect(self._on_block_dropped)
                block.anchor_requested.connect(self._on_block_anchor_requested)
                if vid in self._hidden_vehicles:
                    block.setVisible(False)
                chain.append(block)
            self.action_chains.append(chain)
            self._reorder_chain_by_anchor(len(self.path_segments) - 1)
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
    """视图：自动保持场地纵横比；左键绘制轨迹（除非按到障碍物/手柄/执行块）

    绘图软件式交互
    ==============
      Ctrl + 滚轮 / 滚轮  缩放（以鼠标为中心）
      中键拖拽 / 空格+左键拖拽  平移画布
      Ctrl+0   缩放适应窗口
      Ctrl+= / Ctrl+-  放大 / 缩小
      Esc      取消正在绘制的路段
    """

    ZOOM_STEP = 1.15
    ZOOM_MIN = 0.5    # 相对 fit 的倍率
    ZOOM_MAX = 12.0

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        self.scale(1, -1)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(38, 41, 48)))
        self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # 全视口更新：消除 ItemIsMovable 拖拽时的边框鬼影
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.FullViewportUpdate)

        self._zoom = 1.0            # 相对 fit 的缩放倍率
        self._user_zoomed = False   # 用户手动缩放后不再自动 fit
        self._panning = False       # 中键 / 空格拖拽平移中
        self._pan_start = QtCore.QPoint()
        self._space_held = False

    # ---- 缩放 ----
    def zoom_fit(self):
        """缩放至场地充满窗口（Ctrl+0）"""
        self._zoom = 1.0
        self._user_zoomed = False
        self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def zoom_by(self, factor, anchor_under_mouse=True):
        new_zoom = self._zoom * factor
        if new_zoom < self.ZOOM_MIN or new_zoom > self.ZOOM_MAX:
            return
        self._zoom = new_zoom
        self._user_zoomed = True
        if anchor_under_mouse:
            self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        else:
            self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.scale(factor, factor)

    def zoom_in(self):
        self.zoom_by(self.ZOOM_STEP, anchor_under_mouse=False)

    def zoom_out(self):
        self.zoom_by(1.0 / self.ZOOM_STEP, anchor_under_mouse=False)

    def current_zoom(self):
        return self._zoom

    def wheelEvent(self, event):
        # 滚轮缩放（主流绘图软件行为；Ctrl 可按可不按）
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = self.ZOOM_STEP if delta > 0 else 1.0 / self.ZOOM_STEP
        self.zoom_by(factor, anchor_under_mouse=True)
        event.accept()

    # ---- 平移 ----
    def _begin_pan(self, global_pos):
        self._panning = True
        self._pan_start = global_pos
        self.viewport().setCursor(QtCore.Qt.ClosedHandCursor)

    def _do_pan(self, global_pos):
        delta = global_pos - self._pan_start
        self._pan_start = global_pos
        h = self.horizontalScrollBar()
        v = self.verticalScrollBar()
        h.setValue(h.value() - delta.x())
        v.setValue(v.value() - delta.y())

    def _end_pan(self):
        self._panning = False
        self.viewport().setCursor(
            QtCore.Qt.OpenHandCursor if self._space_held else QtCore.Qt.ArrowCursor)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space and not event.isAutoRepeat():
            self._space_held = True
            if not self._panning:
                self.viewport().setCursor(QtCore.Qt.OpenHandCursor)
            event.accept()
            return
        if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            # 回车：结束控制点曲线的锚点放置
            if self.scene().finish_anchor_placement():
                event.accept()
                return
        if event.key() == QtCore.Qt.Key_Backspace and self.scene()._placing:
            # Backspace: undo the last placed anchor while placing
            self.scene().undo_last_anchor()
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Escape:
            # Esc 优先取消正在放置的锚点曲线
            if self.scene().cancel_anchor_placement():
                event.accept()
                return
            if self.scene().cancel_path():
                event.accept()
                return
            self.scene().clearSelection()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space and not event.isAutoRepeat():
            self._space_held = False
            if not self._panning:
                self.viewport().setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._user_zoomed:
            self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        if not self._user_zoomed:
            self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def mousePressEvent(self, event):
        # 中键拖拽平移
        if event.button() == QtCore.Qt.MiddleButton:
            self._begin_pan(event.globalPos())
            event.accept()
            return
        if event.button() == QtCore.Qt.LeftButton:
            # 空格 + 左键 → 平移（Photoshop 式抓手）
            if self._space_held:
                self._begin_pan(event.globalPos())
                event.accept()
                return
            # Shift + 拖拽 → 橡皮筋框选
            if event.modifiers() & QtCore.Qt.ShiftModifier:
                self.setDragMode(QtWidgets.QGraphicsView.RubberBandDrag)
                super().mousePressEvent(event)
                return

            # 控制点工具（样条 / 直线）优先：单击落锚点，不被路径/预览线拦截
            if self.scene().draw_tool != "freehand":
                scene_pos = self.mapToScene(event.pos())
                if self.scene()._placing:
                    # 已在放点 → 追加一个锚点
                    self.scene().add_anchor(scene_pos)
                    event.accept()
                    return
                # 未在放点：若点到已有曲线的控制手柄则交给 Qt 拖动，否则开始放点
                from .path_item import _AnchorHandle
                hit = self.itemAt(event.pos())
                if isinstance(hit, _AnchorHandle):
                    super().mousePressEvent(event)
                    return
                self.scene().clearSelection()
                self.scene().begin_anchor_placement(scene_pos)
                event.accept()
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
        if self._panning:
            self._do_pan(event.globalPos())
            event.accept()
            return
        scene_pos = self.mapToScene(event.pos())
        self.scene().mouse_pos_cm.emit(scene_pos.x(), scene_pos.y())
        # 控制点工具放置中 → 实时橡皮筋预览下一段
        if self.scene()._placing:
            self.scene().update_anchor_preview(scene_pos)
            event.accept()
            return
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
        if self._panning and event.button() in (
                QtCore.Qt.MiddleButton, QtCore.Qt.LeftButton):
            self._end_pan()
            event.accept()
            return
        if event.button() == QtCore.Qt.LeftButton:
            # 橡皮筋框选结束 → 重置为 NoDrag
            if self.dragMode() == QtWidgets.QGraphicsView.RubberBandDrag:
                super().mouseReleaseEvent(event)
                self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
                event.accept()
                return
            if self.scene()._drawing:
                scene_pos = self.mapToScene(event.pos())
                self.scene().finish_path(scene_pos)
                event.accept()
                return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        # 控制点曲线：双击结束放锚点并生成曲线
        if (event.button() == QtCore.Qt.LeftButton
                and self.scene().draw_tool != "freehand"
                and self.scene()._placing):
            # 把双击位置也当作一个锚点加进去（重复点会被去重），避免丢失终点
            self.scene().add_anchor(self.mapToScene(event.pos()))
            self.scene().finish_anchor_placement()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

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
