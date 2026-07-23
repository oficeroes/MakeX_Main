"""控制点曲线：直线段 + Catmull-Rom 样条（纯标准库）

目的
====
让用户像绘图软件的钢笔工具那样，只点几个"锚点"就得到规整的路径，
而不是徒手抖动画线。生成的点列直接填进 PathItem.raw_points，
后续平滑 / 重采样 / 运动学导出完全复用，不需要改动导出管线。

两种工具
========
  line     锚点之间直线连接（折线），再按 max_spacing 加密
  spline   Catmull-Rom 样条，曲线"穿过"每一个锚点，段间自动平滑相切

为什么用 Catmull-Rom
====================
Catmull-Rom 是插值样条——曲线严格经过每个控制点，用户点哪它过哪，
所见即所得。相比之下 B 样条不过控制点，钢笔工具手感差。
端点用镜像虚拟点处理，保证首尾切线自然。

坐标系
======
和 smoothing.py 一致：场景单位 = cm，直接对 (x, y) 运算，+Y 方向无关紧要。

API
===
  line_path(anchors, max_spacing=5.0) -> [(x, y), ...]
  catmull_rom_spline(anchors, samples_per_seg=None, max_spacing=5.0) -> [(x, y), ...]
  build_curve(kind, anchors, max_spacing=5.0) -> [(x, y), ...]
"""
import math

# 曲线类型常量（与 PathItem.draw_kind 对齐）
KIND_FREEHAND = "freehand"
KIND_SPLINE = "spline"
KIND_LINE = "line"

# Catmull-Rom 张力：0.5 = 标准 centripetal 风格的均匀参数化近似，
# 值越大曲线越"紧贴"锚点连线，越小越圆滑膨出。
_CR_TENSION = 0.5


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dedupe(anchors):
    """去掉相邻重复锚点（用户误双击同一处会产生零长段）。"""
    if not anchors:
        return []
    out = [(float(anchors[0][0]), float(anchors[0][1]))]
    for p in anchors[1:]:
        fp = (float(p[0]), float(p[1]))
        if _dist(fp, out[-1]) > 1e-6:
            out.append(fp)
    return out


def line_path(anchors, max_spacing=5.0):
    """锚点直线连接，并按 max_spacing 在每段内插支撑点。

    加密的目的：让后续等弧长重采样和碰撞检测有足够密的点，
    否则一条很长的直线只有两个端点，重采样会漏掉中间。
    """
    pts = _dedupe(anchors)
    if len(pts) < 2:
        return list(pts)
    try:
        max_spacing = float(max_spacing)
    except (TypeError, ValueError):
        max_spacing = 5.0
    if max_spacing <= 0:
        max_spacing = 5.0

    out = [pts[0]]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        parts = max(1, int(math.ceil(seg_len / max_spacing)))
        for j in range(1, parts + 1):
            t = j / float(parts)
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out


def catmull_rom_spline(anchors, samples_per_seg=None, max_spacing=5.0):
    """Catmull-Rom 插值样条：曲线穿过每个锚点，段间平滑相切。

    anchors:          用户点的控制点 [(x, y), ...]
    samples_per_seg:  每段采样数（None = 按段长和 max_spacing 自适应）
    max_spacing:      自适应采样时的目标点间距（cm）

    端点用镜像虚拟点（2*P0 - P1）保证首尾切线自然。
    2 个锚点时退化为直线。
    """
    pts = _dedupe(anchors)
    n = len(pts)
    if n < 3:
        return line_path(pts, max_spacing)

    try:
        max_spacing = float(max_spacing)
    except (TypeError, ValueError):
        max_spacing = 5.0
    if max_spacing <= 0:
        max_spacing = 5.0

    # 构造带镜像端点的扩展序列： P[-1], P0, P1, ..., Pn-1, P[n]
    first = (2 * pts[0][0] - pts[1][0], 2 * pts[0][1] - pts[1][1])
    last = (2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1])
    ext = [first] + pts + [last]

    out = [pts[0]]
    tau = _CR_TENSION
    for i in range(1, len(ext) - 2):
        p0 = ext[i - 1]
        p1 = ext[i]
        p2 = ext[i + 1]
        p3 = ext[i + 2]

        # 该段采样数：优先用显式值，否则按段长 / max_spacing 自适应
        if samples_per_seg is not None:
            steps = max(2, int(samples_per_seg))
        else:
            seg_len = _dist(p1, p2)
            steps = max(2, int(math.ceil(seg_len / max_spacing)))

        # Catmull-Rom 基函数（uniform 参数化，张力 tau）
        # m1, m2 是 p1, p2 处的切向量
        m1x = tau * (p2[0] - p0[0])
        m1y = tau * (p2[1] - p0[1])
        m2x = tau * (p3[0] - p1[0])
        m2y = tau * (p3[1] - p1[1])

        for s in range(1, steps + 1):
            t = s / float(steps)
            t2 = t * t
            t3 = t2 * t
            # Hermite 基
            h00 = 2 * t3 - 3 * t2 + 1
            h10 = t3 - 2 * t2 + t
            h01 = -2 * t3 + 3 * t2
            h11 = t3 - t2
            x = h00 * p1[0] + h10 * m1x + h01 * p2[0] + h11 * m2x
            y = h00 * p1[1] + h10 * m1y + h01 * p2[1] + h11 * m2y
            out.append((x, y))
    return out


def build_curve(kind, anchors, max_spacing=5.0):
    """按工具类型生成点列。未知类型退化为直线。"""
    if kind == KIND_SPLINE:
        return catmull_rom_spline(anchors, max_spacing=max_spacing)
    return line_path(anchors, max_spacing)
