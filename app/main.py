from main_window import MainWindow
from PyQt5 import QtWidgets
import sys
import qdarkstyle


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    lidar_camera_manual = MainWindow()
    lidar_camera_manual.showMaximized()
    sys.exit(app.exec_())

