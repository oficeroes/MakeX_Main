"""入口点：python -m trajectory_planner.main"""
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
