from PyQt5 import QtGui, QtCore, QtWidgets
import ctypes
import xml.etree.ElementTree as ET
import os
import yaml
import cv2
import numpy as np

from sub_window import ImageWidget
from cpp_python_link import *
from utils import *

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.CALIBRATION_LAUNCH_ADDR = "../launch/calibration.launch"
        self.setWindowTitle("lidar camera manual caliibration@WHY")
        self.Initialize()
        # x为slider.value（正数）则实际表达数值等于var = (x + min) / factor
        self.intrinsic_setting = { 
                        "fx": {
                            "min": 0, "max": 500000, "factor": 100, "unit": "×100", "label": self.fx_label, "slider": self.fx_slider, "slot": self.ValueChangedFx
                         },
                         "fy": {
                            "min": 0, "max": 500000, "factor": 100, "unit": "×100", "label": self.fy_label, "slider": self.fy_slider, "slot": self.ValueChangedFy   
                         },
                         "cx": {
                            "min": 0, "max": 192000, "factor": 100, "unit": "×100", "label": self.cx_label, "slider": self.cx_slider, "slot": self.ValueChangedCx
                         },
                         "cy": {
                            "min": 0, "max": 108000, "factor": 100, "unit": "×100", "label": self.cy_label, "slider": self.cy_slider, "slot": self.ValueChangedCy
                         },
                         "k1": {
                            "min": -5000, "max": 5000, "factor": 100, "unit": "×100", "label": self.k1_label, "slider": self.k1_slider, "slot": self.ValueChangedK1
                         },
                         "k2": {
                            "min": -5000, "max": 5000, "factor": 100, "unit": "×100", "label": self.k2_label, "slider": self.k2_slider, "slot": self.ValueChangedK2
                         },
                         "p1": {
                            "min": -500000, "max": 500000, "factor": 1e5, "unit": "×1e5", "label": self.p1_label, "slider": self.p1_slider, "slot": self.ValueChangedP1
                         },
                         "p2": {
                            "min": -500000, "max": 500000, "factor": 1e5, "unit": "×1e5", "label": self.p2_label, "slider": self.p2_slider, "slot": self.ValueChangedP2
                         },
        }
        self.extrinsic_setting = {
                         "tx": {
                            "min": -2000, "max": 2000, "factor": 1000, "unit": "mm", "label": self.tx_label, "slider": self.tx_slider, "slot": self.ValueChangedTx
                         },
                         "ty": {
                            "min": -2000, "max": 2000, "factor": 1000, "unit": "mm", "label": self.ty_label, "slider": self.ty_slider, "slot": self.ValueChangedTy
                         },
                         "tz": {
                            "min": -2000, "max": 2000, "factor": 1000, "unit": "mm", "label": self.tz_label, "slider": self.tz_slider, "slot": self.ValueChangedTz
                         },
                         "roll": {
                            "min": 0, "max": 3600, "factor": 10, "unit": "0.1deg", "label": self.roll_label, "slider": self.roll_slider, "slot": self.ValueChangedRoll
                         },
                         "pitch": {
                            "min": 0, "max": 3600, "factor": 10, "unit": "0.1deg", "label": self.pitch_label, "slider": self.pitch_slider, "slot": self.ValueChangedPitch
                         },
                         "yaw": {
                            "min": 0, "max": 3600, "factor": 10, "unit": "0.1deg", "label": self.yaw_label, "slider": self.yaw_slider, "slot": self.ValueChangedYaw
                         },
        }
        self.SetUI()
        self.SetCPPLink()

    def Initialize(self):
        self.launch_file_label = QtWidgets.QLabel("Launch File")
        self.launch_file_lineedit = QtWidgets.QLineEdit(self.CALIBRATION_LAUNCH_ADDR)
        self.launch_file_lineedit.setReadOnly(True)
        self.camera_file_label = QtWidgets.QLabel("Camera File")
        self.camera_file_lineedit = QtWidgets.QLineEdit()
        self.camera_file_lineedit.setReadOnly(True)
        self.calib_file_label = QtWidgets.QLabel("Calib File")
        self.calib_file_lineedit = QtWidgets.QLineEdit()
        self.calib_file_lineedit.setReadOnly(True)
        self.img_dir_label = QtWidgets.QLabel("Image Address")
        self.img_dir_linedit = QtWidgets.QLineEdit()
        self.img_dir_linedit.setReadOnly(True)
        self.lidar_dir_label = QtWidgets.QLabel("Lidar Address")
        self.lidar_dir_linedit = QtWidgets.QLineEdit()
        self.lidar_dir_linedit.setReadOnly(True)
        self.lidar_type_lable = QtWidgets.QLabel("Lidar Type(true: CustomMsg, false: Pointcloud2)")
        self.lidar_type_lineedit = QtWidgets.QLineEdit()
        self.lidar_type_lineedit.setReadOnly(True)
        self.load_param_button = QtWidgets.QPushButton("Load Parameters")
        self.start_registration_button = QtWidgets.QPushButton("Start Registration")
        # ##################################################################3
        self.fx_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.fy_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.cx_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.cy_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.k1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.k2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.p1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.p2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.tx_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.ty_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.tz_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.roll_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.pitch_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.yaw_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.fx_label = QtWidgets.QLabel()
        self.fy_label = QtWidgets.QLabel()
        self.cx_label = QtWidgets.QLabel()
        self.cy_label = QtWidgets.QLabel()
        self.k1_label = QtWidgets.QLabel()
        self.k2_label = QtWidgets.QLabel()
        self.p1_label = QtWidgets.QLabel()
        self.p2_label = QtWidgets.QLabel()
        self.tx_label = QtWidgets.QLabel()
        self.ty_label = QtWidgets.QLabel()
        self.tz_label = QtWidgets.QLabel()
        self.roll_label = QtWidgets.QLabel()
        self.pitch_label = QtWidgets.QLabel()
        self.yaw_label = QtWidgets.QLabel()

    def SetUI(self):
        self.image_window = ImageWidget()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.image_window)
        grid_layout = QtWidgets.QGridLayout()
        grid_layout.addWidget(self.launch_file_label, 0, 0)
        grid_layout.addWidget(self.launch_file_lineedit, 0, 1)
        grid_layout.addWidget(self.camera_file_label, 0, 2)
        grid_layout.addWidget(self.camera_file_lineedit, 0, 3)
        grid_layout.addWidget(self.calib_file_label, 0, 4)
        grid_layout.addWidget(self.calib_file_lineedit, 0, 5)
        grid_layout.addWidget(self.load_param_button, 0, 6)
        # #############################################################
        grid_layout.addWidget(self.img_dir_label, 1, 0)
        grid_layout.addWidget(self.img_dir_linedit, 1, 1)
        grid_layout.addWidget(self.lidar_dir_label, 1, 2)
        grid_layout.addWidget(self.lidar_dir_linedit, 1, 3)
        grid_layout.addWidget(self.lidar_type_lable, 1, 4)
        grid_layout.addWidget(self.lidar_type_lineedit, 1, 5)
        grid_layout.addWidget(self.start_registration_button, 1, 6)
        layout.addLayout(grid_layout)
        # #############################################################
        tab_widget = QtWidgets.QTabWidget(self)
        intrinsic_widget = QtWidgets.QWidget()
        extrinsic_widget = QtWidgets.QWidget()
        tab_widget.addTab(intrinsic_widget, "intrinsic")
        tab_widget.addTab(extrinsic_widget, "extrinsic")
        layout.addWidget(tab_widget)
        intrin_layout = QtWidgets.QVBoxLayout(intrinsic_widget)   
        extrin_layout = QtWidgets.QVBoxLayout(extrinsic_widget)   
        for key in self.intrinsic_setting.keys():
            self.intrinsic_setting[key]["label"].setText("{}({}) ({}/{})".format(key, 
                                                                       self.intrinsic_setting[key]["unit"], 
                                                                       self.intrinsic_setting[key]["min"],  
                                                                       self.intrinsic_setting[key]["max"]))
            self.intrinsic_setting[key]["slider"].setMinimum(0)
            self.intrinsic_setting[key]["slider"].setMaximum(self.intrinsic_setting[key]["max"] - self.intrinsic_setting[key]["min"])
            self.intrinsic_setting[key]["slider"].setTickPosition(QtWidgets.QSlider.TicksBelow)
            self.intrinsic_setting[key]["slider"].valueChanged.connect(self.intrinsic_setting[key]["slot"])
            hlayout = QtWidgets.QHBoxLayout()
            hlayout.addWidget(self.intrinsic_setting[key]["label"])
            hlayout.addWidget(self.intrinsic_setting[key]["slider"])
            intrin_layout.addLayout(hlayout)
        for key in self.extrinsic_setting.keys():
            self.extrinsic_setting[key]["label"].setText("{}({}) ({}/{})".format(key, 
                                                                       self.extrinsic_setting[key]["unit"], 
                                                                       self.extrinsic_setting[key]["min"],  
                                                                       self.extrinsic_setting[key]["max"]))
            self.extrinsic_setting[key]["slider"].setMinimum(0)
            self.extrinsic_setting[key]["slider"].setMaximum(self.extrinsic_setting[key]["max"] - self.extrinsic_setting[key]["min"])
            self.extrinsic_setting[key]["slider"].setTickPosition(QtWidgets.QSlider.TicksBelow)
            self.extrinsic_setting[key]["slider"].valueChanged.connect(self.extrinsic_setting[key]["slot"])
            hlayout = QtWidgets.QHBoxLayout()
            hlayout.addWidget(self.extrinsic_setting[key]["label"])
            hlayout.addWidget(self.extrinsic_setting[key]["slider"])
            extrin_layout.addLayout(hlayout)

    def SetCPPLink(self):
        self.set_init_par_func = clib.SetInitialParameters
        self.set_init_par_func.argtypes = (ctypes.POINTER(InitialParametersStruct), ) 
        self.set_calib_par_func = clib.SetCalibrationParmaters
        self.set_calib_par_func.argtypes = (ctypes.POINTER(CalibrationParametersStruct), )
        self.set_calib_par_func.restype = ctypes.POINTER(StructImage)
        self.start_registration_button.clicked.connect(self.StartRegistration)
        self.load_param_button.clicked.connect(self.RegisterLaunch)

    def StartRegistration(self):
        calib_par_struct = CalibrationParametersStruct()
        calib_par_struct.fx = (self.fx_slider.value() + self.intrinsic_setting["fx"]["min"]) / self.intrinsic_setting["fx"]["factor"]
        calib_par_struct.fy = (self.fy_slider.value() + self.intrinsic_setting["fy"]["min"]) / self.intrinsic_setting["fy"]["factor"]
        calib_par_struct.cx = (self.cx_slider.value() + self.intrinsic_setting["cx"]["min"]) / self.intrinsic_setting["cx"]["factor"]
        calib_par_struct.cy = (self.cy_slider.value() + self.intrinsic_setting["cy"]["min"]) / self.intrinsic_setting["cy"]["factor"]
        calib_par_struct.k1 = (self.k1_slider.value() + self.intrinsic_setting["k1"]["min"]) / self.intrinsic_setting["k1"]["factor"]
        calib_par_struct.k2 = (self.k2_slider.value() + self.intrinsic_setting["k2"]["min"]) / self.intrinsic_setting["k2"]["factor"]
        calib_par_struct.p1 = (self.p1_slider.value() + self.intrinsic_setting["p1"]["min"]) / self.intrinsic_setting["p1"]["factor"]
        calib_par_struct.p2 = (self.p2_slider.value() + self.intrinsic_setting["p2"]["min"]) / self.intrinsic_setting["p2"]["factor"]
        calib_par_struct.tx = (self.tx_slider.value() + self.extrinsic_setting["tx"]["min"]) / self.extrinsic_setting["tx"]["factor"]
        calib_par_struct.ty = (self.ty_slider.value() + self.extrinsic_setting["ty"]["min"]) / self.extrinsic_setting["ty"]["factor"]
        calib_par_struct.tz = (self.tz_slider.value() + self.extrinsic_setting["tz"]["min"]) / self.extrinsic_setting["tz"]["factor"]
        calib_par_struct.roll = (self.roll_slider.value() + self.extrinsic_setting["roll"]["min"]) / self.extrinsic_setting["roll"]["factor"] / 180 * np.pi
        calib_par_struct.pitch = (self.pitch_slider.value() + self.extrinsic_setting["pitch"]["min"]) / self.extrinsic_setting["pitch"]["factor"] / 180 * np.pi
        calib_par_struct.yaw = (self.yaw_slider.value() + self.extrinsic_setting["yaw"]["min"]) / self.extrinsic_setting["yaw"]["factor"] / 180 * np.pi
        img_char = self.set_calib_par_func(calib_par_struct)
        row = img_char.contents.row
        col = img_char.contents.col
        channel = img_char.contents.channel
        img_array = np.array(img_char.contents.img)[:row * col * channel].reshape(row, col, channel)
        img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
        self.image_window.SetFrame(img_array)
        self.image_window.ShowImg()

    def RegisterLaunch(self):
        print("in RegisterLaunch")
        dom = ET.parse(self.CALIBRATION_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "lidar_dir":
                lidar_dir = node.get("value")
            elif node.get("name") == "img_dir":
                img_dir = node.get("value")
            elif node.get("name") == "lidar_type":
                lidar_type = node.get("value")
            elif node.get("name") == "camera_file":
                camera_file = node.get("value")
            elif node.get("name") == "calib_file":
                calib_file = node.get("value")
        self.img_dir_linedit.setText(img_dir)
        self.lidar_dir_linedit.setText(lidar_dir)
        self.lidar_type_lineedit.setText(lidar_type)
        self.camera_file_lineedit.setText(camera_file)
        self.calib_file_lineedit.setText(calib_file)
        if not os.path.exists(img_dir):
            QtWidgets.QMessageBox.information(self, "[WARNING]", "{} doesn't exist".format(img_dir), QtWidgets.QMessageBox.Ok)
        if not os.path.exists(lidar_dir):
            QtWidgets.QMessageBox.information(self, "[WARNING]", "{} doesn't exist".format(lidar_dir), QtWidgets.QMessageBox.Ok)
        if not os.path.exists(camera_file):
            QtWidgets.QMessageBox.information(self, "[WARNING]", "{} missing".format(camera_file), QtWidgets.QMessageBox.Ok)
        else:
            # opencv yaml
            cv_file = cv2.FileStorage(camera_file, cv2.FILE_STORAGE_READ)
            intrinsic_mat = cv_file.getNode("CameraMat").mat()
            dist_mat = cv_file.getNode("DistCoeffs").mat()
            print(intrinsic_mat, intrinsic_mat.dtype)
            self.fx_slider.setValue(int(intrinsic_mat[0, 0] * self.intrinsic_setting["fx"]["factor"] - self.intrinsic_setting["fx"]["min"]))
            self.fy_slider.setValue(int(intrinsic_mat[1, 1] * self.intrinsic_setting["fy"]["factor"] - self.intrinsic_setting["fy"]["min"]))
            self.cx_slider.setValue(int(intrinsic_mat[0, 2] * self.intrinsic_setting["cx"]["factor"] - self.intrinsic_setting["cx"]["min"]))
            self.cy_slider.setValue(int(intrinsic_mat[1, 2] * self.intrinsic_setting["cy"]["factor"] - self.intrinsic_setting["cy"]["min"]))
            self.k1_slider.setValue(int(dist_mat[0] * self.intrinsic_setting["k1"]["factor"] - self.intrinsic_setting["k1"]["min"]))
            self.k2_slider.setValue(int(dist_mat[1] * self.intrinsic_setting["k2"]["factor"] - self.intrinsic_setting["k2"]["min"]))
            self.p1_slider.setValue(int(dist_mat[2] * self.intrinsic_setting["p1"]["factor"] - self.intrinsic_setting["p1"]["min"]))
            self.p2_slider.setValue(int(dist_mat[3] * self.intrinsic_setting["p2"]["factor"] - self.intrinsic_setting["p2"]["min"]))
        if not os.path.exists(calib_file):
            QtWidgets.QMessageBox.information(self, "[WARNING]", "{} missing".format(calib_file), QtWidgets.QMessageBox.Ok)
        else:
            cv_file = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
            extrinsic_mat = cv_file.getNode("ExtrinsicMat").mat()
            rpy_angle = mat2euler(extrinsic_mat[:3, :3])
            print(rpy_angle)
            self.roll_slider.setValue(int(rpy_angle[0] * self.extrinsic_setting["roll"]["factor"] - self.extrinsic_setting["roll"]["min"]))
            self.pitch_slider.setValue(int(rpy_angle[1] * self.extrinsic_setting["pitch"]["factor"] - self.extrinsic_setting["pitch"]["min"]))
            self.yaw_slider.setValue(int(rpy_angle[2] * self.extrinsic_setting["yaw"]["factor"] - self.extrinsic_setting["yaw"]["min"]))
            self.tx_slider.setValue(int(extrinsic_mat[0, 3] * self.extrinsic_setting["tx"]["factor"] - self.extrinsic_setting["tx"]["min"]))
            self.ty_slider.setValue(int(extrinsic_mat[1, 3] * self.extrinsic_setting["ty"]["factor"] - self.extrinsic_setting["ty"]["min"]))
            self.tz_slider.setValue(int(extrinsic_mat[2, 3] * self.extrinsic_setting["tz"]["factor"] - self.extrinsic_setting["tz"]["min"]))
        init_par_struct = InitialParametersStruct()
        init_par_struct.calib_file_address = ctypes.c_char_p(calib_file.encode())
        init_par_struct.lidar_address = ctypes.c_char_p(lidar_dir.encode())
        init_par_struct.lidar_type = ctypes.c_char_p(lidar_type.encode())
        init_par_struct.img_address = ctypes.c_char_p(img_dir.encode())
        self.set_init_par_func(init_par_struct)

    def IntrinsicValueChanged(self, key):
        self.intrinsic_setting[key]["label"].setText("{}({}) ({}/{})".format(key, 
                                                                   self.intrinsic_setting[key]["unit"], 
                                                                   self.intrinsic_setting[key]["slider"].value() + self.intrinsic_setting[key]["min"],  
                                                                   self.intrinsic_setting[key]["max"]))

    def ExtrinsicValueChanged(self, key):
        self.extrinsic_setting[key]["label"].setText("{}({}) ({}/{})".format(key, 
                                                                   self.extrinsic_setting[key]["unit"], 
                                                                   self.extrinsic_setting[key]["slider"].value() + self.extrinsic_setting[key]["min"],  
                                                                   self.extrinsic_setting[key]["max"]))


    def ValueChangedFx(self):
        self.IntrinsicValueChanged("fx")

    def ValueChangedFy(self):
        self.IntrinsicValueChanged("fy")

    def ValueChangedCx(self):
        self.IntrinsicValueChanged("cx")

    def ValueChangedCy(self):
        self.IntrinsicValueChanged("cy")

    def ValueChangedK1(self):
        self.IntrinsicValueChanged("k1")

    def ValueChangedK2(self):
        self.IntrinsicValueChanged("k2")

    def ValueChangedP1(self):
        self.IntrinsicValueChanged("p1")

    def ValueChangedP2(self):
        self.IntrinsicValueChanged("p2")

    def ValueChangedTx(self):
        self.ExtrinsicValueChanged("tx")

    def ValueChangedTy(self):
        self.ExtrinsicValueChanged("ty")

    def ValueChangedTz(self):
        self.ExtrinsicValueChanged("tz")

    def ValueChangedRoll(self):
        self.ExtrinsicValueChanged("roll")

    def ValueChangedPitch(self):
        self.ExtrinsicValueChanged("pitch")
        
    def ValueChangedYaw(self):
        self.ExtrinsicValueChanged("yaw")




