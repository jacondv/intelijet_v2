#!/usr/bin/env python3
import os
#allow create file with full permission
os.umask(0)

import re
import time

import sys, subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QPushButton
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMessageBox, QDialog



from vtk_viewer import VTKViewer
from ros_thread import RosThread
from jobnumber_page_manager import JobNumberPageManager
from history_page_manager import HistoryPageManager

from data_binder import DataBinder
from ui.update_data_utils import DataBinder, load_config_to_ui, load_ui_to_config   
from ui.utils import convert_pointcloud2_to_o3d_v2, o3d_to_vtk_polydata
from ui.compare_cloud_worker import cloud_compare

from shared.pps_command import PPSCommand

from ui.setting_page_ui import Ui_setting_page
from ui.intelijet_ui import Ui_MainWindow 

from ui.tunnel_report.report_controler import ReportGenerator

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

        self.setting_page_ui.btnUpdateHousingParam.released.connect(lambda: load_ui_to_config(self.ui.tab_setting))
        self.setting_page_ui.btnCancelHousingParam.released.connect(lambda: load_config_to_ui(self.ui.tab_setting))

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
        self.jobsetting_page_in_toolbox = JobNumberPageManager(self.ui.tboxPage1, mode="label")
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

        #Page 3: Compare page
        self.ui.btnCompare2.released.connect(self.on_compare)
        self.ui.btnViewReport.released.connect(self.on_viewreport_dlg)

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
        cloud_compare.compare_done.connect(self.update_pointcloud_from_data)
        cloud_compare.compare_done.connect(self.on_export_report)

        # --- Control Buttons ---
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
        o3d_cloud = convert_pointcloud2_to_o3d_v2(msg)
        polydata = o3d_to_vtk_polydata(o3d_cloud)
        # polydata = ros_pointcloud2_to_o3d_to_vtk_polydata_voxel(msg)
        self.vtk_viewer.update(polydata)
        if polydata:
            self.__save_cloud(o3d_cloud, topic_name)
            # self.__save_pointcloud_polydata_as_ply(polydata, topic_name)

    def update_pointcloud_from_data(self, data):
        from pps.data_converter import cloudconverter
        polydata = cloudconverter.o3d_to_vtk_polydata(data)
        print("updated polydata from file")
        self.vtk_viewer.update(polydata)

        report = ReportGenerator()
        report.export(pcd=data,output_path="/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/Test_3m_v4#20251007_005243#report.pdf")


    def on_compare(self):

        # import subprocess
        # subprocess.Popen(["evince", "/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/Test_3m_v4#20251007_005243#report.pdf"])

        from jobselect_dlg_manage import jobcompare_dlg
        if jobcompare_dlg.exec_() == QDialog.Accepted:
            data = jobcompare_dlg.get_result()
            print("Data:", data)

        pre = data['file1']
        post = data['file2']
        if pre is None and post is  None:
            return
        
        from pps.cloud_processing.align_manager import PointCloudAlignerManager
        from pps.cloud_processing.icp_aligner import ICPConfig
        from pps.data_converter import cloudconverter

        ICP_THRESHOLDS = [0.5, 0.3, 0.02]      # coarse → fine
        ICP_MAX_ITERS = [20, 20, 30]           # coarse → fine
        ICP_ALIGN_AREA = None                  # hoặc [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
        ICP_VOXEL_RADII = [0.25, 0.15, 0.01]  # coarse → fine
        aligner = PointCloudAlignerManager(
                    strategy="icp",
                    config=ICPConfig(
                        threshold=ICP_THRESHOLDS,
                        max_iters=ICP_MAX_ITERS,
                        align_area=ICP_ALIGN_AREA,
                        voxel_radii=ICP_VOXEL_RADII
                    )
                )

        pre = cloudconverter.load_ply(pre)
        pre = cloudconverter.tensor_to_o3d_legacy(pre)
        post = cloudconverter.load_ply(post)
        post = cloudconverter.tensor_to_o3d_legacy(post)

        aligner.align(post, pre)
        T = aligner.get_transformation_matrix()
        post.transform(T)
        cloud_compare.set_prescan(pre)
        cloud_compare.set_postscan(post)
        cloud_compare.compare()


    def on_viewreport_dlg(self):
        from reportselect_dlg_manager import reportselect_dlg
        reportselect_dlg.exec_()

    def on_export_report(self, data):
        pass
        

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
        msg = QMessageBox()
        msg.setWindowTitle("Confirmation")
        msg.setText("Are you sure you want to quit?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        msg.setWindowModality(True)
        reply = msg.exec_()
        if reply == QMessageBox.Yes:
            self.close()

    def _generate_filename(self, topic_name: str, ext = "ply") -> str:


        jobs_root = self.jobsetting_page.jobs_root
        job_number = self.ui.lblCurrentJob.text()
        
        # Chuẩn hóa tên topic
        safe_topic = re.sub(r'[^a-zA-Z0-9_-]', '', topic_name)

        # Tạo thư mục cho job nếu chưa tồn tại
        folder = os.path.join(jobs_root, job_number)
        os.makedirs(folder, exist_ok=True)
        index = sum(topic_name in f for f in os.listdir(folder)) +  1

        # Timestamp hiện tại
        timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())

        filename = os.path.join(folder, f"{job_number}#{safe_topic}_{index:02d}#{timestamp_str}.{ext}")

        return filename

    def __save_cloud(self, o3d_cloud, topic_name):
        """
        Lưu Open3D PointCloud (legacy hoặc tensor) ra .ply, giữ color và các field extra như 'distances' hoặc 'distance_mm'.
        Tên file: {job_number}_{YYYYmmdd_HHMMSS}_{safe_topic}.ply
        """

        from pps.data_converter import cloudconverter
        from ui.tunnel_report.report_data_model import ReportHeader

        try:
            filepath = self._generate_filename(topic_name, ext="ply")
            

            cloudconverter.o3d_to_ply(o3d_cloud, filepath)

            #Save job information to json file, it provides information for later visualization and report generation

            job_number = filepath.split("#")[0] if "#" in filepath else "--"

            header = ReportHeader(
                    site_name = job_number,
                    job_name= job_number,
                    applied_thickness = 30,
                    tolerance = 10,
                    operator = "Danh Vo"
            )

            header.save(path=filepath.replace(".ply", ".json"))

        except Exception as e:
            print("Error occurred while saving Open3D pointcloud:", e)
            return None

if __name__ == "__main__":

    app = QApplication(sys.argv)
    viewer = App()
    sys.exit(app.exec_())

