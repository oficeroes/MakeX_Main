"""step_planner — 拉格子式机器人自动程序编辑器

用法：
    python -m step_planner.main

版本历史：
    v1.0   2026-07-08  初始版本。替代 trajectory_planner 中"绘路径→导出"的模式，
                        改用显式步骤表格，彻底消除路径几何近似误差。

设计原则：
    每步意图 100% 透明：用户填"前进 50 cm 功率 70%"，
    程序直接生成 ('enc_move', 474, 70, 0)，中间没有任何近似。
"""

__version__ = "1.0.0"
__all__ = ["main"]


def main():
    from .main import main as _main
    _main()
