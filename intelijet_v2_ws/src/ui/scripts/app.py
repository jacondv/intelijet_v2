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
from ui.utils import o3d_to_vtk_polydata
from ui.compare_cloud_worker import cloud_compare

from shared.pps_command import PPSCommand

from ui.setting_page_ui import Ui_setting_page
from ui.intelijet_ui import Ui_MainWindow 

from ui.tunnel_report.report_controler import ReportGenerator

from shared.config_loader import CONFIG as cfg

from project_dlg_manager import ProjectManager 




BASE_DIR = cfg.BASE_DIR
CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC

DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")


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
        # self.jobsetting_page = JobNumberPageManager(self.ui.tab_jobnumber)
        # if self.ui.tab_jobnumber.layout() is None:
        #     self.ui.tab_jobnumber.setLayout(QVBoxLayout())
        # self.ui.tab_jobnumber.layout().addWidget(self.jobsetting_page)
        self.project_manager = ProjectManager()
        if self.ui.tab_jobnumber.layout() is None:
            self.ui.tab_jobnumber.setLayout(QVBoxLayout())
        self.ui.tab_jobnumber.layout().addWidget(self.project_manager)

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

        self.history_page_in_toolbox.polydataSignal.connect(lambda cloud: self.update_pointcloud_from_data(cloud, None))

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

        orig_show = self.ui.cbbJobSelect.showPopup
        def new_show():
            self.load_active_jobs(self.ui.cbbJobSelect, ACTIVE_JOB_FILE)
            orig_show()

        self.ui.cbbJobSelect.showPopup = new_show
        
        # --- Project and Job Manager ---
        # self.project_manager = None
        # self.open_project_manager()

        # self.job_select_manage = None
        # self.open_job_select_manager()


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


    # 1.--- Update commond data from ROS ---
    def update_data(self, data):
        self.data_binder.update_ui_from_status(data)
        if "encoder_value_in_deg" in data:
            self.ui.lblEncoder.setText(f"{data['encoder_value_in_deg']:.2f}")
        if "notification" in data:
            self.lblNotification.setText(data["notification"])

    # 2.--- Update pointcloud from reatime signal ---
    def update_pointcloud(self, msg, topic_name):

        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()
    
        # o3d_cloud = convert_pointcloud2_to_o3d_v2(msg)
        o3d_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)
        polydata = cloudconverter.o3d_to_vtk_polydata(o3d_cloud)

        self.vtk_viewer.update(polydata)
        if polydata:
            self.save_job(o3d_cloud, topic_name)

    # 3.--- Update pointcloud from available data---
    def update_pointcloud_from_data(self, data, filename):
        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()

        polydata = cloudconverter.o3d_to_vtk_polydata(data)
        if filename:
            print("updated polydata from file:", filename)
        self.vtk_viewer.update(polydata)


    # 4.--- Show report view dialog---
    def on_viewreport_dlg(self):
        from reportselect_dlg_manager import reportselect_dlg
        reportselect_dlg.exec_()


    # 5.--- Export report after compare done---
    def on_export_report(self, data, filename):
        try:
            from datetime import datetime
            from ui.models import JobInfo
            

            report = ReportGenerator()
            job_folder = os.path.dirname(filename)
            project_name = os.path.basename(job_folder) 
            basename = os.path.basename(filename)
            basename_parts = basename.split("#")
            job_name = basename_parts[0] if len(basename_parts) > 0 else "Unknown"    
            
            try:
                dt = datetime.strptime(basename_parts[1], "%Y%m%d_%H%M%S").date()
            except:
                dt = None
            try:
                tt = datetime.strptime(basename_parts[1], "%Y%m%d_%H%M%S").time()
            except:
                tt = None

            job_info = JobInfo.load(job_folder)
            if job_info is not None:
                report.set_info(
                    site_name = project_name,
                    job_name= job_name,
                    applied_thickness = job_info.parameters.get("target_thickness", 30),
                    tolerance = job_info.parameters.get("tolerance", 10),
                    operator = "Unknown"
                )
            else:
                report.set_info(
                    site_name = "Unknown",
                    job_name= job_name,
                    applied_thickness = 30,
                    tolerance = 10,
                    operator = "Unknown"
                )

            if filename.lower().endswith(".ply"):
                filename = filename.replace(".ply",".pdf")
                
            else:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"Test_report#{timestamp}.pdf"
                filename = f"{BASE_DIR}/data/reports/{filename}"
                
            report.export(pcd=data,output_path=filename)
        except Exception as e:
            print(f"[App] Failed to export report: {e}")

    # 6.--- Start compare 2 cloud selected for dialog---
    def on_compare(self):

        # import subprocess
        # subprocess.Popen(["evince", "/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/Test_3m_v4#20251007_005243#report.pdf"])

        from jobselect_dlg_manage import jobcompare_dlg
        if jobcompare_dlg.exec_() == QDialog.Accepted:
            data = jobcompare_dlg.get_result()
            print("Data:", data)
            pre = data['file1']
            post = data['file2']
            if pre is None or post is None:
                return
        else:
            return

        
        cloud_compare.set_prescan(pre)
        cloud_compare.set_postscan(post)
        cloud_compare.align()
        cloud_compare.compare() #--> output signal compare_done the cloud result.


    # 7.--- Close event handler ---
    def closeEvent(self, event):
        subprocess.call(["rosnode", "kill", "-a"])
        subprocess.call("pkill -f ros", shell=True)
        subprocess.call(["rosclean", "purge", "-y"])
        event.accept()  


    # 8.--- Shutdown handler ---
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


    # 9.--- Save job to disk ---
    def save_job(self, o3d_cloud, topic_name):
        """
        Lưu Open3D PointCloud (legacy hoặc tensor) ra .ply, giữ color và các field extra như 'distances' hoặc 'distance_mm'.
        Tên file: {job_number}_{YYYYmmdd_HHMMSS}_{safe_topic}.ply
        """
        def _generate_filename(topic_name: str, ext = "ply") -> str:

            # jobs_root = self.jobsetting_page.jobs_root
            # job_number = self.ui.lblCurrentJob.text()
            
            project_name = self.ui.cbbJobSelect.currentText().split(" / ")[0]
            job_number = self.ui.cbbJobSelect.currentText().split(" / ")[1]
            jobs_root = os.path.join(PROJECT_DIR, project_name)

            # Chuẩn hóa tên topic
            safe_topic = re.sub(r'[^a-zA-Z0-9_-]', '', topic_name)

            # Tạo thư mục cho job nếu chưa tồn tại
            folder = os.path.join(jobs_root, job_number)
            os.makedirs(folder, exist_ok=True)
            index = sum(safe_topic in f for f in os.listdir(folder) if f.endswith('.ply')) +  1

            # Timestamp hiện tại
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())

            filename = os.path.join(folder, f"{job_number}#{timestamp_str}#{safe_topic}_{index:02d}.{ext}")

            return filename

        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()

        from ui.tunnel_report.report_data_model import ReportHeader

        try:
            filepath = _generate_filename(topic_name, ext="ply")
            fname = os.path.basename(filepath)
            cloudconverter.o3d_to_ply(o3d_cloud, filepath) #save cloud to ply file.

            #Save job information to json file, it provides information for later visualization and report generation

            job_number = fname.split("#")[0] if "#" in filepath else "--"

            header = ReportHeader(
                    site_name = "Jacon Equipment",
                    job_name= job_number,
                    applied_thickness = 30,
                    tolerance = 10,
                    operator = "Danh Vo"
            )

            header.save(path=filepath.replace(".ply", ".json"))

            if topic_name in CLOUD_COMPARED_TOPIC:
                from ui.tunnel_report.report_controler import ReportGenerator
                report = ReportGenerator()
                report.set_info(
                    site_name = header.site_name,
                    job_name= header.job_name,
                    applied_thickness = header.applied_thickness,
                    tolerance = header.tolerance
                )
                report.export(o3d_cloud,output_path=filepath.replace(".ply", ".pdf"))

        except Exception as e:
            print("Error occurred while saving Open3D pointcloud:", e)
            return None
        

    def load_active_jobs(self,comboBox, json_file="active_job.json"):
        import json
        comboBox.clear()  # xóa item cũ
        if not os.path.exists(json_file):
            return
        with open(json_file, "r") as f:
            jobs = json.load(f)

        for job in jobs:
            # text hiển thị trong combobox
            display_text = f"{job['project']}/{job['job']}"
            comboBox.addItem(display_text, job)  # lưu dict job vào data

if __name__ == "__main__":

    app = QApplication(sys.argv)
    viewer = App()
    sys.exit(app.exec_())

