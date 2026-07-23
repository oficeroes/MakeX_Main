"""step_planner.main — 入口点

用法：
    python -m step_planner.main
    python -m step_planner          # __init__.py 里的 main() 也会调到这里
"""

import sys


def main():
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QApplication

    # 高 DPI 支持
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except AttributeError:
        pass

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setApplicationName("步骤规划器")
    app.setOrganizationName("MakeX")

    # 尝试复用 trajectory_planner 的样式表
    try:
        from trajectory_planner.style import APP_STYLESHEET
        app.setStyleSheet(APP_STYLESHEET + """
            QTableWidget {
                alternate-background-color: #F6F8FB;
                gridline-color: #E2E6EC;
            }
            QTableWidget::item:selected {
                background: #D9E7F8;
                color: #1B4E8A;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #CBD5E0;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 4px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                color: #2C5F8A;
            }
        """)
    except ImportError:
        pass

    from .main_window import StepMainWindow
    window = StepMainWindow()
    window.showMaximized()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
