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


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
