#!/usr/bin/env python3
import os
#allow create file with full permission
os.umask(0)
from pathlib import Path

import re
import time

import sys, subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QPushButton, QComboBox
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMessageBox, QDialog
from PyQt5.QtCore import QSettings



from vtk_viewer import VTKViewer
from ros_thread import RosThread
#Import pages manager
from jobnumber_page_manager import JobNumberPageManager
from history_page_manager import HistoryPageManager
from project_dlg_manager import ProjectManager 
from setting_page_manager import SettingPageManager

from data_binder import DataBinder
from ui.update_data_utils import DataBinder, load_config_to_ui, load_ui_to_config   
from ui.compare_cloud_worker import cloud_compare

from shared.pps_command import PPSCommand

from ui.intelijet_ui import Ui_MainWindow 

from ui.tunnel_report.report_controler import ReportGenerator

from shared.config_loader import CONFIG as cfg


BASE_DIR = cfg.BASE_DIR
CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC

DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")
CURRENT_JOB_FILE = os.path.join(PROJECT_DIR, "current_job.json")

settings = QSettings("JaconEquipment", "Intelijet")

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
        self.setting_page = SettingPageManager()
        if self.ui.tab_setting.layout() is None:
            self.ui.tab_setting.setLayout(QVBoxLayout())

        self.ui.tab_setting.layout().addWidget(self.setting_page)


        # ------Tab JobSetting ---
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
        # self.cloud_received_signal.connect(self.update_pointcloud)
        #Receive cloud and Send align, compare request to ROS if cloud come from postcloud topic
        self.cloud_received_signal.connect(self.on_cloud_received)

        #Receive cloud check cloud is come from /compared topic --> export report
        # --- Signals ---
        self.ui_data_update.connect(self.update_data)
        self.ui_send_cmd_signal.connect(self.ros_thread.send_command)
        cloud_compare.compare_done.connect(self.update_pointcloud_from_data)
        cloud_compare.compare_done2.connect(self.on_manual_export_report)

        # --- Control Buttons ---
        self.ui.btnPreScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value))
        self.ui.btnPostScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_POSTSCAN.value))
        self.ui.btnCompare.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value))
        self.ui.btnCancel.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CANCEL_JOB.value))
        self.ui.btnOpenScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.OPEN_HOUSING.value))
        self.ui.btnCloseScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CLOSE_HOUSING.value))
        self.ui.btnShutdown.released.connect(self.on_shutdown)

        # self.ui.btnSelectJob.released.connect(self.on_select_job_clicked)
        # btn_ok.clicked.connect(self.accept_job)
        # btn_cancel.clicked.connect(self.accept_job_cancel)
        self.ui.btnFullScreen.released.connect(self.toggle_max)

        # --- Select Job to work process ---
        self.load_active_jobs(self.ui.cbbJobSelect, ACTIVE_JOB_FILE)
        self.load_current_job()
        # self.ui.cbbJobSelect.currentIndexChanged.connect(self.on_job_changed)
        self.ui.cbbJobSelect.activated.connect(self.on_job_changed)
        self.current_job_index = self.ui.cbbJobSelect.currentIndex()
        
        
        orig_show = self.ui.cbbJobSelect.showPopup
        def new_show():
            self.load_active_jobs(self.ui.cbbJobSelect, ACTIVE_JOB_FILE)
            orig_show()

        self.ui.cbbJobSelect.showPopup = new_show
        # self.ui.cbbJobSelect.mousePressEvent = self.on_combo_click

        # --- Status bar ---
        self.lblNotification = QLabel("Ready")
        self.lblNotification.setStyleSheet("margin-left: 5px;")  
        self.ui.statusbar.addWidget(self.lblNotification)

        # --- Data binder ---
        self.data_binder = DataBinder(self.ui.centralFrame)
        load_config_to_ui(self.ui.tab_setting)

        # --- Show main window ---
        # self.showMaximized()
        # polydata = self.__load_sample()
        # if polydata:
        #     self.vtk_viewer.update(polydata)

        self.setting_page.txtEncodeValueRaw.setText("NaN")

        #Load ui state
        self.load_ui_state()

    # Setting parameter
    def save_ui_state(self):
        settings.setValue("cbbAutoCompare_index", self.ui.cbbAutoCompare.currentIndex())

    def load_ui_state(self):
        index = settings.value("cbbAutoCompare_index", 0, type=int)
        self.ui.cbbAutoCompare.setCurrentIndex(index)

    # 1.0--- Update commond data from ROS ---
    def on_cloud_received(self, msg, topic_name):
        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()
        o3d_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)

        # Show pointcloud
        polydata = cloudconverter.o3d_to_vtk_polydata(o3d_cloud)
        self.vtk_viewer.update(polydata)

        # Save cloud to file ply
        if polydata:
            self.save_job(o3d_cloud, topic_name)

        # Emit align command to ROS
        if topic_name in POST_SCAN_CLOUD_TOPIC:
            if self.ui.cbbAutoCompare.currentIndex()==0: 
                self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value)
            

        # Export Report
        if topic_name in CLOUD_COMPARED_TOPIC:
            if self.ui.cbbAutoCompare.currentIndex()==1 or self.ui.cbbAutoReport.currentIndex()==1:
                return # only export report when auto compare is on nad auto report is on. (1 is OFF)

            try:
                currnet_job = self.load_current_job(text_only=True)
                project_name = currnet_job.split("/")[0]
                job_number = currnet_job.split("/")[1]
                jobs_folder = os.path.join(PROJECT_DIR, project_name,job_number)
                # Chuẩn hóa tên topic
                safe_topic = re.sub(r'[^a-zA-Z0-9_-]', '', topic_name)
                index = sum(safe_topic in f for f in os.listdir(jobs_folder) if f.endswith('.pdf')) +  1
                # Timestamp hiện tại
                timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
                filename = os.path.join(jobs_folder, f"{job_number}#{timestamp_str}#{safe_topic}_{index:02d}.pdf")
                
                self.export_report(o3d_cloud,filename)

            except Exception as e:
                # in toàn bộ thông tin lỗi
                print(f"[Error] at on_cloud_received Export Report : {e}")

    def toggle_max(self):
        if not self.isFullScreen():
            self.showFullScreen()
            self.ui.btnFullScreen.setText("Exit Full Screen")   # đổi text khi full
        else:
            self.showMaximized()
            self.ui.btnFullScreen.setText("Full Screen")

    # 1.1--- Update commond data from ROS ---
    def update_data(self, data):

        self.data_binder.update_ui_from_status(data)
        if "encoder_value_in_deg" in data:
            self.ui.lblEncoder.setText(f"{data['encoder_value_in_deg']:.2f}")
        if "notification" in data:
            self.lblNotification.setText(data["notification"])
        if "encoder_value_raw" in data:
            value = str(data["encoder_value_raw"])
            self.ui.lblEncoderRawValue.setText(value)
            self.setting_page.txtEncodeValueRaw.setText(value)

        data = data['devices']
        if "encoder" in data:
            status = data['encoder']['device_state']
            self.ui.lblEncoderStatus.setText(status)
        else:
            self.ui.lblEncoderStatus.setText("unknown".upper())
    
        if 'lidar' in data:
            status = data['lidar']['device_state']
            self.ui.lblLidarStatus.setText(status)
        else:
            self.ui.lblLidarStatus.setText("unknown".upper())

        if 'pcan' in data:
            status = data['pcan']['device_state']
            self.ui.lblPCANStatus.setText(status)
        else:
            self.ui.lblPCANStatus.setText("unknown".upper())
 
        if 'plc' in data:
            status = data['plc']['device_state']
            self.ui.lblPLCStatus.setText(status)
        else:
            self.ui.lblPLCStatus.setText("unknown".upper())
               
               
    # # 1.2--- Update pointcloud from reatime signal ---
    # def update_pointcloud(self, msg, topic_name):

    #     from pps.data_converter import CloudConverter
    #     cloudconverter = CloudConverter()
    
    #     # o3d_cloud = convert_pointcloud2_to_o3d_v2(msg)
    #     o3d_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)


    #     polydata = cloudconverter.o3d_to_vtk_polydata(o3d_cloud)

    #     self.vtk_viewer.update(polydata)
    #     if polydata:
    #         self.save_job(o3d_cloud, topic_name)

    # 3.--- Update pointcloud from available data---
    def update_pointcloud_from_data(self, data, filename=None):
        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()

        polydata = cloudconverter.o3d_to_vtk_polydata(data)
        if filename:
            print("updated polydata from file:", filename)
        self.vtk_viewer.update(polydata)


    # 4.--- Show report view dialog---
    def on_viewreport_dlg(self):
        # from reportselect_dlg_manager import reportselect_dlg
        # reportselect_dlg.exec_()

        from report_view_dlg_manager import ReportViewManager
        dlg = ReportViewManager()

        parts = [p.strip() for p in self.ui.cbbJobSelect.currentText().split("/")]
        project, job = (parts + [None]*2)[:2]  # Nếu thiếu phần, job = None
        dlg.initialize(project,job)

        if dlg.exec_() == QDialog.Rejected:
            return



    #5.0 -- Manual export report handler---
    def on_manual_export_report(self, data, filename):
        if self.ui.cbbAutoReport.currentText().lower() == 'off':
            return # Auto report is off.

        self.export_report(data, filename)

    # 5.1--- Export report after compare done---
    def export_report(self, data, filename):
        try:
            from datetime import datetime
            from ui.models.job_info import JobInfo
            
            report = ReportGenerator()
            job_folder = os.path.dirname(filename)
            project_name = os.path.basename(os.path.dirname(job_folder)) 
            
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
                    applied_thickness = job_info.parameters.get("target_thickness", 10),
                    tolerance = job_info.parameters.get("tolerance", 10),
                    operator = "Unknown",
                    date = dt.strftime("%d/%m/%Y") if dt else None,
                    time = tt.strftime("%H:%M:%S") if tt else None
                )
            else:
                report.set_info(
                    site_name = "Unknown",
                    job_name= job_name,
                    applied_thickness = 40,
                    tolerance = 10,
                    operator = "Unknown",
                    date = dt.strftime("%d/%m/%Y") if dt else None,
                    time = tt.strftime("%H:%M:%S") if tt else None
                    
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

        from compare_dlg_manager import CompareManager
        jobcompare_dlg = CompareManager()
        # Initialize dialog with current selected project and job
        parts = [p.strip() for p in self.ui.cbbJobSelect.currentText().split("/")]
        project, job = (parts + [None]*2)[:2]  # Nếu thiếu phần, job = None
        jobcompare_dlg.initialize(project,job)
        filename = ""
        jobcompare_dlg.polydataSignal.connect(self.update_pointcloud_from_data)

        if jobcompare_dlg.exec_() == QDialog.Accepted:
            data = jobcompare_dlg.get_result()
            
            # pre = data['file1']
            # post = data['file2']
            pre, post, *_ = data
            if pre is None or post is None:
                return
            
            cloud_compare.set_prescan(pre)
            cloud_compare.set_postscan(post)
            if self.ui.cbbAutoAlign.currentText().lower() == "on":
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
        self.save_ui_state()
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
            
            project_name = self.ui.cbbJobSelect.currentText().split("/")[0]
            job_number = self.ui.cbbJobSelect.currentText().split("/")[1]
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
            print(f"Can not save file to {filepath}")
            return None
    
    
    #10. change current job

    def on_job_changed(self, index):
        import json
        if index < 0:
            return  # không chọn gì cả
        
        if index == self.current_job_index: # Chỉ hỏi khi item khác item hiện tại
            return

        value = self.ui.cbbJobSelect.itemText(index)

        # Hiển thị message box xác nhận
        reply = QMessageBox.question(
            self,
            "Confirm",
            f"Do you want to select job: {value}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Lưu giá trị vào file JSON
            try:
                with open(CURRENT_JOB_FILE, "w") as f:
                    json.dump({"current_job": value}, f, indent=4)
                self.current_job_index = self.ui.cbbJobSelect.currentIndex()
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Cannot save job: {e}")
        else:
            # Nếu user chọn No, quay lại giá trị cũ
            try:
                with open(CURRENT_JOB_FILE, "r") as f:
                    data = json.load(f)
                last_value = data.get("current_job", "")
                idx = self.ui.cbbJobSelect.findText(last_value)
                if idx >= 0:
                    self.ui.cbbJobSelect.blockSignals(True)
                    self.ui.cbbJobSelect.setCurrentIndex(idx)
                    self.ui.cbbJobSelect.blockSignals(False)
            except:
                pass

    def load_current_job(self, text_only=False):
        import json
        """Load giá trị hiện tại của job từ file hoặc gán giá trị đầu tiên."""
        last_job = None
        try:
            with open(CURRENT_JOB_FILE, "r") as f:
                data = json.load(f)
                last_job = data.get("current_job", None)
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"Error loading {CURRENT_JOB_FILE}: {e}")

        # Nếu có giá trị lưu trước đó và có trong combobox → chọn nó
        if text_only:
            return last_job
        
        if last_job:
            idx = self.ui.cbbJobSelect.findText(last_job)
            if idx >= 0:
                self.ui.cbbJobSelect.setCurrentIndex(idx)
                return

        # Nếu không có hoặc giá trị cũ không hợp lệ → chọn giá trị đầu tiên
        if self.ui.cbbJobSelect.count() > 0:
            self.ui.cbbJobSelect.setCurrentIndex(0)
        
        return last_job
            
    #######################################################
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
    # Load style QSS tại đây
    
    with open(f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/app.qss") as f:
        app.setStyleSheet(f.read())

    viewer = App()
    viewer.showMaximized()
    sys.exit(app.exec_())
