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
    smoothed = chaikin(raw_points, iterations)
    return resample(smoothed, step)
