"""轨迹平滑：Chaikin 角点切割 + 等弧长重采样

目的
====
将用户徒手绘制的折线（采样率不均、噪点多）变成平滑且等间距的点列，
再交给 kinematics.py 计算每段的运动时间和方向。

算法
====
1. Chaikin 角点切割（iterations 次，默认 3）
   每次迭代把每条线段替换成两个新点：
     Q = 3/4 * P0 + 1/4 * P1
     R = 1/4 * P0 + 3/4 * P1
   多次迭代后逼近二次 B 样条（近似弧形），仅用乘法，无 scipy 依赖。
   端点固定（起点/终点不移动）。

2. 等弧长重采样（step = 5 cm 默认）
   沿曲线累计行进距离，每走 step 厘米插值取一个点。
   结果：相邻点间距均等，kinematics 每段运动时间 = step / speed，简洁直接。

注意
====
- Chaikin 迭代数 > 5 时平滑效果边际递减，且点数以 ~2× 增长，没必要超过 5。
- resample 末端逻辑：最后一个原始端点若距输出末点 > 0.3 * step，则强制追加，
  避免因浮点累积导致终点丢失。
- 纯标准库实现（math 模块），可在无 numpy/scipy 的环境运行。

API
===
  chaikin(points, iterations=3) -> list[(x,y)]
  resample(points, step) -> list[(x,y)]
  smooth_and_resample(raw_points, iterations, step) -> list[(x,y)]
"""
import math


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def remove_near_duplicates(points, min_dist):
    """Keep spatial samples only; mouse event frequency must not affect path."""
    if len(points) < 2 or min_dist <= 0:
        return list(points)

    pts = [(float(x), float(y)) for x, y in points]
    out = [pts[0]]
    for p in pts[1:-1]:
        if _dist(p, out[-1]) >= min_dist:
            out.append(p)

    last = pts[-1]
    if _dist(last, out[-1]) > 1e-9:
        out.append(last)

    if len(out) < 2 and len(pts) >= 2:
        return [pts[0], pts[-1]]
    return out


def _point_line_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-12:
        return _dist(p, a)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def simplify_polyline(points, tolerance):
    """Ramer-Douglas-Peucker simplification, preserving endpoints."""
    if len(points) < 3 or tolerance <= 0:
        return list(points)

    keep = [False] * len(points)
    keep[0] = True
    keep[-1] = True
    stack = [(0, len(points) - 1)]

    while stack:
        start, end = stack.pop()
        a = points[start]
        b = points[end]
        best_i = -1
        best_d = tolerance
        for i in range(start + 1, end):
            d = _point_line_distance(points[i], a, b)
            if d > best_d:
                best_d = d
                best_i = i
        if best_i >= 0:
            keep[best_i] = True
            stack.append((start, best_i))
            stack.append((best_i, end))

    return [p for p, k in zip(points, keep) if k]


def densify_polyline(points, max_spacing):
    """Insert geometry-based support points so smoothing is not event-based."""
    if len(points) < 2 or max_spacing <= 0:
        return list(points)

    out = [points[0]]
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-9:
            continue
        parts = max(1, int(math.ceil(seg_len / max_spacing)))
        for j in range(1, parts):
            t = j / float(parts)
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        out.append((x1, y1))
    return out


def normalize_drawn_path(raw_points, step):
    """Convert mouse samples into a geometry-only path before smoothing.

    Short pauses create many almost-identical mouse events.  Those points are
    not trajectory intent, so remove them before Chaikin and resampling.
    """
    if len(raw_points) < 2:
        return list(raw_points)

    try:
        step = float(step)
    except (TypeError, ValueError):
        step = 5.0
    if step <= 0:
        step = 5.0
    spacing = max(0.2, min(step * 0.20, 1.0))
    tolerance = max(0.5, min(step * 0.30, 1.5))
    support_spacing = max(1.0, min(step, 5.0))

    pts = remove_near_duplicates(raw_points, spacing)
    pts = simplify_polyline(pts, tolerance)
    if len(pts) < 2:
        return [(float(raw_points[0][0]), float(raw_points[0][1])),
                (float(raw_points[-1][0]), float(raw_points[-1][1]))]
    return densify_polyline(pts, support_spacing)


def chaikin(points, iterations=3):
    """Chaikin 角点切割算法：每次迭代用 1/4 + 3/4 内插，曲线变平滑

    points: [(x, y), ...]
    返回新点列；端点保留
    """
    if iterations <= 0 or len(points) < 3:
        return list(points)

    pts = list(points)
    for _ in range(iterations):
        new_pts = [pts[0]]
        for i in range(len(pts) - 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            new_pts.append(q)
            new_pts.append(r)
        new_pts.append(pts[-1])
        pts = new_pts
    return pts


def resample(points, step):
    """等弧长重采样：沿曲线每 step 距离取一个点

    points: [(x, y), ...]，step: 间隔（cm）
    返回新点列；起点保留，末端总是包含原终点
    """
    if len(points) < 2 or step <= 0:
        return list(points)

    out = [points[0]]
    carry = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-9:
            continue
        traveled = -carry
        while traveled + step <= seg_len:
            traveled += step
            t = traveled / seg_len
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        carry = seg_len - traveled
    last = points[-1]
    if math.hypot(out[-1][0] - last[0], out[-1][1] - last[1]) > step * 0.3:
        out.append(last)
    return out


def smooth_and_resample(raw_points, iterations, step):
    """组合平滑：先 Chaikin 再等弧长重采样"""
    normalized = normalize_drawn_path(raw_points, step)
    smoothed = chaikin(normalized, iterations)
    return resample(smoothed, step)
