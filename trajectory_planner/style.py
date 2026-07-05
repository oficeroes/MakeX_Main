"""全局 QSS 样式表：现代扁平浅色主题

设计原则
========
  主色     #2878D0（蓝）— 强调 / 选中 / 焦点
  成功色   #1F9D55（绿）— 导出按钮
  背景     #EEF1F5（浅灰蓝）
  面板     #FFFFFF（白）
  边框     #D8DCE3
  文字     #2B3440（近黑）/ #6B7480（次要）

用法
====
  from .style import APP_STYLESHEET
  app.setStyleSheet(APP_STYLESHEET)
"""

APP_STYLESHEET = """
/* ===== 全局 ===== */
QMainWindow, QDialog {
    background: #EEF1F5;
    color: #2B3440;
}
QWidget {
    color: #2B3440;
}
QToolTip {
    background: #333B47;
    color: #FFFFFF;
    border: 1px solid #22262E;
    padding: 6px 8px;
    border-radius: 4px;
}

/* ===== 菜单栏 ===== */
QMenuBar {
    background: #FFFFFF;
    border-bottom: 1px solid #D8DCE3;
    padding: 2px 4px;
}
QMenuBar::item {
    padding: 4px 10px;
    border-radius: 4px;
    background: transparent;
}
QMenuBar::item:selected {
    background: #E4EEF9;
    color: #2878D0;
}
QMenu {
    background: #FFFFFF;
    border: 1px solid #C9CFD8;
    border-radius: 6px;
    padding: 4px;
}
QMenu::item {
    padding: 5px 28px 5px 12px;
    border-radius: 4px;
}
QMenu::item:selected {
    background: #2878D0;
    color: #FFFFFF;
}
QMenu::separator {
    height: 1px;
    background: #E2E6EC;
    margin: 4px 8px;
}

/* ===== 工具栏 ===== */
QToolBar {
    background: #FFFFFF;
    border: none;
    border-bottom: 1px solid #D8DCE3;
    padding: 3px 6px;
    spacing: 2px;
}
QToolBar::separator {
    width: 1px;
    background: #E2E6EC;
    margin: 4px 6px;
}
QToolButton {
    background: transparent;
    border: 1px solid transparent;
    border-radius: 6px;
    padding: 5px 10px;
}
QToolButton:hover {
    background: #E8F0FB;
    border-color: #C4DAF4;
}
QToolButton:pressed {
    background: #D0E2F7;
}
QToolButton:checked {
    background: #2878D0;
    color: #FFFFFF;
}
QToolButton:disabled {
    color: #A8AFBA;
}
/* 导出按钮：绿色实心强调 */
QToolButton#btnExport {
    background: #1F9D55;
    color: #FFFFFF;
    font-weight: bold;
    padding: 5px 14px;
}
QToolButton#btnExport:hover {
    background: #178A49;
}
QToolButton#btnExport:pressed {
    background: #12753D;
}

/* ===== 停靠面板 ===== */
QDockWidget {
    titlebar-close-icon: none;
    font-weight: bold;
}
QDockWidget::title {
    background: #E4E9F0;
    padding: 6px 10px;
    border-bottom: 1px solid #D8DCE3;
}
QScrollArea {
    background: transparent;
    border: none;
}
QScrollArea > QWidget > QWidget {
    background: transparent;
}

/* ===== 分组框 ===== */
QGroupBox {
    background: #FFFFFF;
    border: 1px solid #D8DCE3;
    border-radius: 8px;
    margin-top: 12px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    top: 2px;
    padding: 0 5px;
    color: #2878D0;
    background: #FFFFFF;
    border-radius: 3px;
}

/* ===== 按钮 ===== */
QPushButton {
    background: #FFFFFF;
    border: 1px solid #C9CFD8;
    border-radius: 6px;
    padding: 5px 12px;
}
QPushButton:hover {
    background: #EEF4FC;
    border-color: #2878D0;
    color: #2878D0;
}
QPushButton:pressed {
    background: #D9E7F8;
}
QPushButton:checked {
    background: #2878D0;
    border-color: #2878D0;
    color: #FFFFFF;
}
QPushButton:disabled {
    color: #A8AFBA;
    background: #F3F4F6;
}

/* ===== 输入控件 ===== */
QSpinBox, QDoubleSpinBox, QComboBox, QLineEdit {
    background: #FFFFFF;
    border: 1px solid #C9CFD8;
    border-radius: 5px;
    padding: 3px 6px;
    min-height: 20px;
    selection-background-color: #2878D0;
}
QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus, QLineEdit:focus {
    border-color: #2878D0;
}
QSpinBox:disabled, QDoubleSpinBox:disabled {
    background: #F3F4F6;
    color: #A8AFBA;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    width: 16px;
    background: #F3F5F8;
    border: none;
}
QSpinBox::up-button, QDoubleSpinBox::up-button {
    border-top-right-radius: 4px;
    border-bottom: 1px solid #E2E6EC;
}
QSpinBox::down-button, QDoubleSpinBox::down-button {
    border-bottom-right-radius: 4px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background: #E0EAF8;
}
QComboBox::drop-down {
    width: 20px;
    border: none;
    border-top-right-radius: 4px;
    border-bottom-right-radius: 4px;
    background: #F3F5F8;
}
QComboBox QAbstractItemView {
    background: #FFFFFF;
    border: 1px solid #C9CFD8;
    border-radius: 4px;
    selection-background-color: #2878D0;
    selection-color: #FFFFFF;
}
QCheckBox {
    spacing: 6px;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #C9CFD8;
    border-radius: 4px;
    background: #FFFFFF;
}
QCheckBox::indicator:hover {
    border-color: #2878D0;
}
QCheckBox::indicator:checked {
    background: #2878D0;
    border-color: #2878D0;
    image: none;
}

/* ===== 树 / 列表 ===== */
QTreeWidget, QTreeView, QListWidget {
    background: #FFFFFF;
    border: 1px solid #D8DCE3;
    border-radius: 6px;
    alternate-background-color: #F6F8FB;
    padding: 2px;
}
QTreeWidget::item, QTreeView::item {
    padding: 3px 2px;
    border-radius: 4px;
}
QTreeWidget::item:selected, QTreeView::item:selected {
    background: #D9E7F8;
    color: #1B4E8A;
}
QTreeWidget::item:hover {
    background: #EEF4FC;
}

/* ===== 状态栏 ===== */
QStatusBar {
    background: #FFFFFF;
    border-top: 1px solid #D8DCE3;
    color: #4A5560;
}
QStatusBar::item {
    border: none;
}

/* ===== 滚动条 ===== */
QScrollBar:vertical {
    background: transparent;
    width: 10px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #C3C9D3;
    border-radius: 5px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background: #A7AFBC;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
QScrollBar:horizontal {
    background: transparent;
    height: 10px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: #C3C9D3;
    border-radius: 5px;
    min-width: 30px;
}
QScrollBar::handle:horizontal:hover {
    background: #A7AFBC;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* ===== 消息框 ===== */
QMessageBox {
    background: #FFFFFF;
}
"""
