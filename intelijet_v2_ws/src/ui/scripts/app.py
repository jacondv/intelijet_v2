#!/usr/bin/env python3
import os
#allow create file with full permission
os.umask(0)

import re
import time

import sys, subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QPushButton
from PyQt5.QtCore import pyqtSignal



from vtk_viewer import VTKViewer
from ros_thread import RosThread
from jobnumber_page_manager import JobNumberPageManager
from history_page_manager import HistoryPageManager

from data_binder import DataBinder
from ui.update_data_utils import DataBinder, load_config_to_ui
from ui.utils import ros_pointcloud2_to_o3d_to_vtk_polydata_voxel

from shared.pps_command import PPSCommand

from ui.setting_page_ui import Ui_setting_page
from ui.intelijet_ui import Ui_MainWindow 
import vtk

class App(QMainWindow):

    cloud_received_signal = pyqtSignal(object, str)
    ui_send_cmd_signal = pyqtSignal(int)
    ui_data_update = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        # --- UI chính ---
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.tab_mainview.setCurrentIndex(0)

        # --- Tab Setting ---
        self.setting_page_widget = QWidget()
        self.setting_page_ui = Ui_setting_page()
        self.setting_page_ui.setupUi(self.setting_page_widget)

        if self.ui.tab_setting.layout() is None:
            self.ui.tab_setting.setLayout(QVBoxLayout())
        self.ui.tab_setting.layout().addWidget(self.setting_page_widget)

        # ------Tab JobSetting ---
        self.jobsetting_page = JobNumberPageManager(self.ui.tab_jobnumber)
        if self.ui.tab_jobnumber.layout() is None:
            self.ui.tab_jobnumber.setLayout(QVBoxLayout())
        self.ui.tab_jobnumber.layout().addWidget(self.jobsetting_page)

        #-------ToolBox-----------
        #Page 1: Jobs view
        self.jobsetting_page_in_toolbox = JobNumberPageManager(self.ui.tboxPage1)
        self.jobsetting_page_in_toolbox.ui.widget_2.hide()
        btn_ok = QPushButton("OK", self.jobsetting_page_in_toolbox)
        btn_cancel = QPushButton("Cancel", self.jobsetting_page_in_toolbox)
        self.jobsetting_page_in_toolbox.ui.widget_3.layout().addWidget(btn_ok)
        self.jobsetting_page_in_toolbox.ui.widget_3.layout().addWidget(btn_cancel)

        self.jobsetting_page_in_toolbox.setVisible(False)

        if self.ui.tboxPage1.layout() is None:
            self.ui.tboxPage1.setLayout(QVBoxLayout())
        self.ui.tboxPage1.layout().insertWidget(1,self.jobsetting_page_in_toolbox)

        #Page 2: History view
        self.history_page_in_toolbox = HistoryPageManager(self.ui.tboxPage2)
        if self.ui.tboxPage2.layout() is None:
            self.ui.tboxPage2.setLayout(QVBoxLayout())
        self.ui.tboxPage2.layout().insertWidget(1,self.history_page_in_toolbox)

        self.history_page_in_toolbox.polydataSignal.connect(self.update_pointcloud_from_data)


        # --- VTK Viewer ---
        self.vtk_viewer = VTKViewer(self.ui.cloudFrame)

        # --- ROS Thread ---
        self.ros_thread = RosThread(self.cloud_received_signal,
                                    self.ui_send_cmd_signal,
                                    self.ui_data_update)
        self.ros_thread.start()

        # --- Signals ---
        self.cloud_received_signal.connect(self.update_pointcloud)
        self.ui_data_update.connect(self.update_data)
        self.ui_send_cmd_signal.connect(self.ros_thread.send_command)

        # --- Buttons ---
        self.ui.btnPreScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value))
        self.ui.btnPostScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_POSTSCAN.value))
        self.ui.btnCompare.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value))
        self.ui.btnCancel.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CANCEL_JOB.value))
        self.ui.btnOpenScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.OPEN_HOUSING.value))
        self.ui.btnCloseScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CLOSE_HOUSING.value))
        self.ui.btnShutdown.released.connect(self.on_shutdown)

        self.ui.btnSelectJob.released.connect(self.on_select_job_clicked)
        btn_ok.clicked.connect(self.accept_job)
        btn_cancel.clicked.connect(self.accept_job_cancel)


        # --- Status bar ---
        self.lblNotification = QLabel("Ready")
        self.lblNotification.setStyleSheet("margin-left: 5px;")  
        self.ui.statusbar.addWidget(self.lblNotification)

        # --- Data binder ---
        self.data_binder = DataBinder(self.ui.centralFrame)
        load_config_to_ui(self.ui.tab_setting)

        # --- Show main window ---
        self.showMaximized()
        # polydata = self.__load_sample()
        # if polydata:
        #     self.vtk_viewer.update(polydata)

    def on_select_job_clicked(self):
        self.jobsetting_page_in_toolbox.load_jobs_from_disk()
        if not self.jobsetting_page_in_toolbox.isVisible():
            self.jobsetting_page_in_toolbox.setVisible(True)

    def accept_job(self):
        job_name = None
        job_name = self.jobsetting_page_in_toolbox.get_selected_job()
        self.ui.lblCurrentJob.setText(job_name)
        if job_name:
            self.jobsetting_page_in_toolbox.setVisible(False)

    def accept_job_cancel(self):
        self.jobsetting_page_in_toolbox.setVisible(False)


    # --- Slot để cập nhật pointcloud ---
    def update_pointcloud(self, msg, topic_name):
        polydata = ros_pointcloud2_to_o3d_to_vtk_polydata_voxel(msg)
        self.vtk_viewer.update(polydata)
        if polydata:
            self.__save_pointcloud_polydata_as_ply(polydata, topic_name)

    def update_pointcloud_from_data(self, polydata):
        print("updated polydata from file")
        self.vtk_viewer.update(polydata)


    # --- Slot để cập nhật dữ liệu từ ROS ---
    def update_data(self, data):
        self.data_binder.update_ui_from_status(data)
        if "encoder_value_in_deg" in data:
            self.ui.lblEncoder.setText(f"{data['encoder_value_in_deg']:.2f}")
        if "notification" in data:
            self.lblNotification.setText(data["notification"])


    def closeEvent(self, event):
        subprocess.call(["rosnode", "kill", "-a"])
        subprocess.call("pkill -f ros", shell=True)
        subprocess.call(["rosclean", "purge", "-y"])
        event.accept()  

    # --- Shutdown handler ---
    def on_shutdown(self):
        from PyQt5.QtWidgets import QMessageBox
        msg = QMessageBox()
        msg.setWindowTitle("Confirmation")
        msg.setText("Are you sure you want to quit?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        msg.setWindowModality(True)
        reply = msg.exec_()
        if reply == QMessageBox.Yes:
            self.close()


    def __save_pointcloud_polydata_as_ply(self,polydata, topic_name):
        try:
            jobs_root = self.jobsetting_page.jobs_root
            job_number = self.ui.lblCurrentJob.text()
            safe_topic = re.sub(r'[^a-zA-Z0-9_-]', '', topic_name)
            folder = os.path.join(jobs_root, job_number)

            os.makedirs(folder, exist_ok=True)
            # print("Local:", time.strftime("%Y%m%d_%H%M%S", time.localtime()))
            # print("UTC:",   time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            filename = os.path.join(folder, f"{job_number}_{timestamp_str}_{safe_topic}.ply")

            #Color process
            colors = polydata.GetPointData().GetScalars()
            if colors is not None:
                colors.SetName("RGB")  # đặt tên cho array
                polydata.GetPointData().SetScalars(colors)

            writer = vtk.vtkPLYWriter()
            writer.SetFileName(filename)
            writer.SetInputData(polydata)
            writer.SetFileTypeToBinary()  # hoặc SetFileTypeToBinary() để tiết kiệm dung lượng
            writer.SetColorModeToDefault()  # ghi màu nếu có
            writer.SetArrayName("RGB")
            writer.Update()
            writer.Write()

        except Exception as e:
            print("Error occurred:", e)


    def __load_sample(self):
        try:
                                
            # Example point cloud

            import open3d as o3d
            import numpy as np
            import vtk
            from vtk.util import numpy_support

            #     # Hardcoded path
            # path = "/mnt/c/work/projects/pointcloud/post_scan_0_20250530_130626_afterporcess.ply"
            #path = "/root/intelijet_v2/post_scan_0_20250530_130626_afterporcess.ply"
            path = "/mnt/c/work/projects/intelijet_v2/post_scan_0_20250530_130626_afterporcess.ply"

            #Đọc PLY bằng Open3D
            pcd = o3d.io.read_point_cloud(path)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) 

            #  Chuyển sang vtkPolyData
            polydata = vtk.vtkPolyData()
            vtk_points = vtk.vtkPoints()
            vtk_points.SetData(numpy_support.numpy_to_vtk(points))
            polydata.SetPoints(vtk_points)

            return polydata           

        except Exception as e:
            print(e)
            return None

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = App()
    sys.exit(app.exec_())

