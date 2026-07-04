"""轨迹绘制工具 — 多底盘机器人路径规划 GUI

功能概述
========
用 PyQt5 绘制场地轨迹 → 平滑 → 转换为 AUTO_SEQUENCE 元组列表 → 写入机器人源文件。

支持的底盘（见 config.CHASSIS_PROFILES）
  - omni3        三轮全向（120° 对称），mecanum_forward.py
  - mecanum_4w   四轮 X 型麦克纳姆（含收球/滚球），mecanum_drive.py

运行方式
========
  python -m trajectory_planner.main

模块依赖关系
============
  main.py
    └─ main_window.py          # 主窗口控制逻辑
         ├─ field_view.py      # 画布 (QGraphicsScene/View)
         │    ├─ path_item.py
         │    └─ obstacle_item.py
         ├─ kinematics.py      # 路径 → AUTO_SEQUENCE
         ├─ smoothing.py       # Chaikin + 等弧长重采样
         ├─ file_io.py         # JSON 存/读
         ├─ exporter.py        # 写回机器人源文件
         └─ config.py          # 所有常量 + ChassisProfile 注册表

扩展新底盘
==========
只需两步：
1. 在 config.py 的 CHASSIS_PROFILES 里加一项 ChassisProfile
2. 创建对应的机器人源文件（含 AUTO_SEQUENCE + AUTO_RAMP_MS 块）
GUI 无需其他改动，详见 docs/EXTENDING_FOR_NEW_CHASSIS.md。
"""
__version__ = "0.2.0"
