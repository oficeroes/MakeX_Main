"""入口点：python -m trajectory_planner.main

用法
====
  cd D:\\UserData\\Desktop\\MakeX_Main
  python -m trajectory_planner.main

依赖
====
  pip install PyQt5
  （其他依赖均为标准库：json / math / ast / os / shutil / re / pathlib）

启动流程
========
  1. 检测 PyQt5，未安装则打印安装提示退出
  2. QApplication("Fusion" 主题) → MainWindow → show() → exec_()
"""
import sys

try:
    from PyQt5 import QtWidgets
except ImportError:
    print("缺少 PyQt5。请运行: pip install PyQt5", file=sys.stderr)
    sys.exit(1)

from .main_window import MainWindow
from .style import APP_STYLESHEET


def main():
    # 高 DPI 支持（必须在 QApplication 创建前设置）
    QtCore = None
    try:
        from PyQt5 import QtCore
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    except Exception:
        pass

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    # 全局字体加大：用于 4K / 高 DPI 屏幕可读性
    font = app.font()
    font.setPointSize(11)
    app.setFont(font)

    win = MainWindow()
    win.showMaximized()   # 默认最大化启动
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
