#!/usr/bin/env python3
import os
#allow create file with full permission
os.umask(0)
from pathlib import Path

import re
import time

import sys, subprocess
import rospy

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

# from data_binder import DataBinder
from ui.update_data_utils import DataBinder, load_config_to_ui, load_ui_to_config   
# from ui.compare_cloud_worker import cloud_compare

from shared.pps_command import PPSCommand

from ui.intelijet_ui import Ui_MainWindow 
from ui.keyboard import TouchKeyboard

from ui.tunnel_report.report_controler import ReportGenerator

from shared.config_loader import CONFIG as cfg


BASE_DIR = cfg.BASE_DIR
CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
CLOUD_COMPARED_UPSAMPLE_TOPIC = f"{CLOUD_COMPARED_TOPIC}/upsample"

CLOUD_COMPARED_TOPIC_MANUAL = cfg.CLOUD_COMPARED_TOPIC + "_manual"
CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL = f"{CLOUD_COMPARED_TOPIC_MANUAL}/upsample"


PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC

CURRENT_JOB_FILE_NAME = "current_job.json"
ACTIVE_JOB_FILE_NAME = "active_jobs.json"
PROJECT_FOLDER_NAME = "Projects"
JOBINFO_FILE_NAME = "job_info.json"

DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, PROJECT_FOLDER_NAME)
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, ACTIVE_JOB_FILE_NAME)
CURRENT_JOB_FILE = os.path.join(PROJECT_DIR, CURRENT_JOB_FILE_NAME)

THICKNESS_DEFAULT = cfg.thickness.target  # Target thickness in meter -> convert mm to m
TOLERANCE_DEFAULT = cfg.thickness.tolerance  # Allowable tolerance in meter of thickness

settings = QSettings("JaconEquipment", "Intelijet")

class App(QMainWindow):

    cloud_received_signal = pyqtSignal(object, str)
    ui_send_cmd_signal = pyqtSignal(int)
    ui_data_update = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.report_name = ""
        self.current_post_scan_path = ""


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
        self.ui.toolBox.currentChanged.connect(self.on_toolbox_changed)
        #Page 3: Compare page
        self.ui.btnCompare2.released.connect(self.on_compare)
        self.ui.btnViewReport.released.connect(self.on_viewreport_dlg)
        

        # --- VTK Viewer ---
        self.vtk_viewer = VTKViewer(self.ui.cloudFrame)

        #TODO
        
        # --- ROS Thread ---
        self.ros_thread = RosThread(self.cloud_received_signal,
                                    self.ui_send_cmd_signal,
                                    self.ui_data_update)
        self.ros_thread.start()

        # if not rospy.core.is_initialized():
        #     rospy.init_node("app_node", anonymous=False)


        # --- Signals ---
        # self.cloud_received_signal.connect(self.update_pointcloud)
        #Receive cloud and Send align, compare request to ROS if cloud come from postcloud topic
        self.cloud_received_signal.connect(self.on_cloud_received)
        #Receive cloud check cloud is come from /compared topic --> export report
        self.ui_data_update.connect(self.update_data)
        self.ui_send_cmd_signal.connect(self.ros_thread.send_command)
        # cloud_compare.compare_done.connect(self.update_pointcloud_from_data)
        # cloud_compare.compare_done2.connect(self.on_manual_export_report)

        # Set rntime parameter for ROS
        combo_boxes = [
            self.ui.cbbAutoAlign,
            self.ui.cbbAutoCompare,
            self.ui.cbbAutoReport,
            self.ui.cbbRemoveGround,
            self.ui.cbbUseKeypoint,
            self.ui.cbbUpsample
        ]
        self.ui.cbbAutoAlign.setEnabled(False)
        self.ui.cbbUpsample.setEnabled(False)

        for cb in combo_boxes:
            cb.currentIndexChanged.connect(self.update_param)

        # --- Control Buttons ---
        # self.ui.btnPreScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value))
        self.ui.btnPreScan.released.connect(self.confirm_and_send_prescan)
        self.ui.btnPostScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_POSTSCAN.value))
        self.ui.btnCompare.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value))
        self.ui.btnCancel.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CANCEL_JOB.value))
        self.ui.btnOpenScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.OPEN_HOUSING.value))
        self.ui.btnCloseScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CLOSE_HOUSING.value))
        self.ui.btnShutdown.released.connect(self.on_shutdown)

        self.ui.btnFullScreen.released.connect(self.toggle_max)

        self.ui.btnLogin.released.connect(self.on_login_clicked)

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

        self.setting_page.txtEncodeValueRaw.setText("NaN")

        #Load ui state
        self.load_ui_state()
        # Send parameters to the ROS on the first boot.
        self.update_param()

    # Setting parameter
    def save_ui_state(self):
        settings.setValue("cbbAutoAlign_index", self.ui.cbbAutoAlign.currentIndex())
        settings.setValue("cbbAutoCompare_index", self.ui.cbbAutoCompare.currentIndex())
        settings.setValue("cbbAutoReport_index", self.ui.cbbAutoReport.currentIndex())
        settings.setValue("cbbRemoveGround_index", self.ui.cbbRemoveGround.currentIndex())
        settings.setValue("cbbUseKeypoint_index", self.ui.cbbUseKeypoint.currentIndex())
        settings.setValue("cbbUpsample_index", self.ui.cbbUpsample.currentIndex())

    def load_ui_state(self):
        self.ui.cbbAutoAlign.setCurrentIndex(settings.value("cbbAutoAlign_index", 0, type=int))
        self.ui.cbbAutoCompare.setCurrentIndex(settings.value("cbbAutoCompare_index", 0, type=int))
        self.ui.cbbAutoReport.setCurrentIndex(settings.value("cbbAutoReport_index", 0, type=int))
        self.ui.cbbRemoveGround.setCurrentIndex(settings.value("cbbRemoveGround_index", 0, type=int))
        self.ui.cbbUseKeypoint.setCurrentIndex(settings.value("cbbUseKeypoint_index", 0, type=int))
        self.ui.cbbUpsample.setCurrentIndex(settings.value("cbbUpsample_index", 0, type=int))


    # Update runtime param to ROS
    def update_param(self):
        params = {
            "/runtime/do_align": self.ui.cbbAutoAlign.currentText().lower() == 'on',
            "/runtime/auto_compare": self.ui.cbbAutoCompare.currentText().lower() == 'on',
            "/runtime/auto_report": self.ui.cbbAutoReport.currentText().lower() == 'on',
            "/runtime/do_pre_process": self.ui.cbbRemoveGround.currentText().lower() == 'on',
            "/runtime/do_2d_keypoint": self.ui.cbbUseKeypoint.currentText().lower() == 'on',
            "/runtime/do_upsample": self.ui.cbbUpsample.currentText().lower() == 'on',
        }

        for key, value in params.items():
            rospy.set_param(key, value)


    #confirm_send_prescan_signal
    def confirm_and_send_prescan(self):
        reply = QMessageBox.question(
            self,
            "Confirm",
            "This will overwrite the existing Pre-Scan file. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Gửi signal nếu người dùng xác nhận
            self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value)

    # Reload data for history page when toolbox page 2 is activated
    def on_toolbox_changed(self, index):
        page = self.ui.toolBox.widget(index)

        if page is self.ui.tboxPage2:
            self.history_page_in_toolbox.load_jobs()
            parts = [p.strip() for p in self.ui.cbbJobSelect.currentText().split("/")]
            project, job = (parts + [None]*2)[:2] 
            if job:
                #Show file on detail view
                job_folder = os.path.join(PROJECT_DIR, project, job)
                self.history_page_in_toolbox.on_item_selected(None, job_folder)
                #Select current job in history view
                self.history_page_in_toolbox.select_job(job)


    # 1.0--- Update commond data from ROS ---
    def on_cloud_received(self, msg, topic_name):
        from pps.data_converter import CloudConverter
        from pps.helper import assign_colors

        cloudconverter = CloudConverter()
        o3d_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)
        print(f"Received cloud on topic {topic_name}")

        # define job_folder based on current selected job or manual compare mode
        if topic_name in [CLOUD_COMPARED_TOPIC_MANUAL, CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL]:
            jobs_folder = os.path.dirname(self.current_post_scan_path)
            job_number = jobs_folder.split("/")[-1]
        else:
            current_job = self.load_current_job(text_only=True)
            project_name = current_job.split("/")[0]
            job_number = current_job.split("/")[1]
            jobs_folder = os.path.join(PROJECT_DIR, project_name,job_number)
            
        # 1. Assign Color
        if topic_name in [CLOUD_COMPARED_TOPIC, CLOUD_COMPARED_UPSAMPLE_TOPIC, CLOUD_COMPARED_TOPIC_MANUAL, CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL]:
            try:
                from ui.models.job_info import JobInfo
                job_info = JobInfo.load(jobs_folder)

                if job_info:
                    target_thickness = job_info.parameters.get("target_thickness",THICKNESS_DEFAULT)
                    tolerance = job_info.parameters.get("tolerance",TOLERANCE_DEFAULT)
                else:
                    target_thickness = THICKNESS_DEFAULT
                    tolerance = TOLERANCE_DEFAULT

                highlight_range = [target_thickness-tolerance, target_thickness+tolerance]
                o3d_cloud = assign_colors(o3d_cloud, highlight_range=highlight_range)
                
            except Exception as e:
                print(f"[Error] at on_cloud_received() to re-assign color : {e}")
                pass

        # 2. Show pointcloud and Save Data
        if topic_name in [POST_SCAN_CLOUD_TOPIC, PRE_SCAN_CLOUD_TOPIC, CLOUD_COMPARED_TOPIC,CLOUD_COMPARED_TOPIC_MANUAL]:
            polydata = cloudconverter.o3d_to_vtk_polydata(o3d_cloud)
            self.vtk_viewer.update(polydata)
            self.ui.tab_mainview.setCurrentIndex(0)

            # Save cloud to file ply
            from ui.models.file_name  import generate_filename

            filepath = generate_filename(
                folder=jobs_folder,
                job=job_number,
                scan_type=topic_name,  # hoặc "postscan" tùy theo logic của bạn
                ext="ply"
            )
            if polydata:
                f_name = self.save_job(o3d_cloud, filepath=filepath)

            if topic_name in [CLOUD_COMPARED_TOPIC, CLOUD_COMPARED_TOPIC_MANUAL]:
                self.report_name = f_name

  


        # 3. Emit to ROS to call Compare Cloud Action
        if topic_name == POST_SCAN_CLOUD_TOPIC:
            if self.ui.cbbAutoCompare.currentIndex()==0: 
                self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value)
            

        # 4. Export Report
        if topic_name in [CLOUD_COMPARED_UPSAMPLE_TOPIC, CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL]:
                
            if self.ui.cbbAutoCompare.currentText().lower() == 'off' or self.ui.cbbAutoReport.currentText().lower() == 'off':
                return # only export report when auto compare is on nad auto report is on. (1 is OFF)

            try:

                print(f"Export Report=========================>")

                cloud_compared_upsample = o3d_cloud
                filename = self.report_name
                filename = filename.replace(".ply", ".pdf")
                self.export_report(cloud_compared_upsample,filename)



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

    def on_login_clicked(self):
        
        from ui.widgets.security_manager import security
        from ui.widgets.login_dialog_view import LoginDialog
        from PyQt5.QtGui import QIcon


        def _on_auth_changed(level):
            self.ui.cbbAutoAlign.setEnabled(level >= security.ADMIN)
            self.ui.cbbUpsample.setEnabled(level >= security.ADMIN)
            self.ui.cbbAutoCompare.setEnabled(level >= security.ADMIN)
            self.ui.cbbUseKeypoint.setEnabled(level >= security.ADMIN)
            

        from PyQt5.QtWidgets import QMessageBox
        if security.level() != security.VIEWER:
            security.logout()
            _on_auth_changed(level=security.VIEWER)
            self.ui.btnLogin.setIcon(QIcon(":/icon/icon/user.png"))
            return

        dlg = LoginDialog(self)
        if dlg.exec_() != dlg.Accepted:
            return

        level = security.login(dlg.password())

        if not level:
            QMessageBox.warning(self, "Error", "Wrong password")
            return

        _on_auth_changed(level=level)
        # self.ui.btnLogin.setText("Logout")
        self.ui.btnLogin.setIcon(QIcon(":/icon/icon/user-logout.png"))



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
               
               
    # 3.--- Update pointcloud from available data---
    def update_pointcloud_from_data(self, data, filename=None):
        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()

        polydata = cloudconverter.o3d_to_vtk_polydata(data)
        if filename:
            print("updated polydata from file:", filename)
        self.vtk_viewer.update(polydata)
        self.ui.tab_mainview.setCurrentIndex(0)


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

    # def on_manual_export_report(self, data, filename):
    #     if self.ui.cbbAutoReport.currentText().lower() == 'off':
    #         return # Auto report is off.
    #     self.export_report(data, filename)

    # 5.1--- Export report after compare done---
    def export_report(self, data, filename):
        print(f'Start releasing the Report: {filename}')
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
                
            # else:
            #     timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            #     filename = f"Test_report#{timestamp}.pdf"
            #     filename = f"{BASE_DIR}/data/reports/{filename}"
                
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
        jobcompare_dlg.polydataSignal.connect(self.update_pointcloud_from_data)

        if jobcompare_dlg.exec_() == QDialog.Accepted:
            data = jobcompare_dlg.get_result()
            
            prescan_path, postscan_path, *_ = data
            if prescan_path is None or postscan_path is None:
                return
             
            # cloud_compare.set_prescan(pre)
            # cloud_compare.set_postscan(post)
            # if self.ui.cbbAutoAlign.currentText().lower() == "on":
            #     cloud_compare.align()
            # cloud_compare.compare() #--> output signal compare_done the cloud result.

            # ✅ chạy trong main thread → OK
            from ui.compare_cloud_worker import CompareWorker
            # ✅ tạo worker, TRUYỀN PATH

            do_align        = rospy.get_param("/runtime/do_align", True)
            do_pre_process  = rospy.get_param("/runtime/do_pre_process", True)
            do_2d_keypoint  = rospy.get_param("/runtime/do_2d_keypoint", False)
            do_upsample     = rospy.get_param("/runtime/do_upsample", False)
            do_post_process     = rospy.get_param("/runtime/do_post_process", False)

            self.worker = CompareWorker(
                prescan_path=prescan_path,
                postscan_path=postscan_path,
                do_2d_keypoint=do_2d_keypoint,
                do_pre_process=do_pre_process,
                do_align=do_align,
                do_post_process=do_post_process,
                do_upsample=do_upsample
            )

            # ✅ connect signal
            self.current_post_scan_path = postscan_path
            self.worker.progress.connect(self.on_compare_process)
            self.worker.finished.connect(self.on_compare_done)

            # ✅ start thread
            self.worker.start()
     
       

    def on_compare_process(self, progress, stage):

        def make_progress_bar(progress, width=20):
            progress = max(0.0, min(1.0, progress))  # clamp
            filled = int(progress * width)
            bar = "=" * filled + " " * (width - filled)
            return f"COMPARE [{bar}] {int(progress * 100):3d}%"

        _string = make_progress_bar(progress)
        print(_string)  
        self.lblNotification.setText(_string)
 

    def on_compare_done(self, success, job_id):
        
        if success:
            print("✅COMPARE DONE ",job_id)
            _string = "✅COMPARE DONE "
        else:
            print("❌COMPARE FAILED ", job_id)
            _string = "❌COMPARE FAILED "
        
        self.lblNotification.setText(_string)

    # 7.--- Close event handler ---
    def closeEvent(self, event):
        # subprocess.call(["/mnt/c/work/projects/intelijet_v2/shutdown.sh"])
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
    def save_job(self, o3d_cloud, filepath):
        from pps.data_converter import CloudConverter
        cloudconverter = CloudConverter()

        try:
            cloudconverter.o3d_to_ply(o3d_cloud, filepath) #save cloud to ply file.
            print(f"Saved cloud to {filepath}")
            return filepath
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
            try:
                with open(CURRENT_JOB_FILE, "w") as f:
                    value = self.ui.cbbJobSelect.itemText(index)
                    json.dump({"current_job": value}, f, indent=4)
                self.current_job_index = self.ui.cbbJobSelect.currentIndex()
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Cannot save job: {e}")

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


    def load_job_info_from_file(self, filepath):
        from ui.models.job_info import JobInfo
        job_folder = os.path.dirname(filepath)
        job_info = JobInfo.load(job_folder)
        return job_info

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

    keyboard_filter = TouchKeyboard()
    app.installEventFilter(keyboard_filter)
    
    with open(f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/app.qss") as f:
        app.setStyleSheet(f.read())

    viewer = App()
    viewer.showFullScreen()
    sys.exit(app.exec_())