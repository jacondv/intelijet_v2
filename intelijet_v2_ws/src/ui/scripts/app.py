#!/usr/bin/env python3
import os


#allow create file with full permission
os.umask(0)
from pathlib import Path

import re
import time

import sys, subprocess
import rospy

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QSettings


from vtk_viewer import VTKViewer
from ros_thread import RosThread
#Import pages manager
from project_dlg_manager import ProjectManager
from report_page_manager import ReportPageManager

from ui.status_binder import StatusBinder

from shared.pps_command import PPSCommand

from ui.intelijet_ui import Ui_MainWindow, draw_app_icon
from ui.keyboard import TouchKeyboard

from ui.notification_center import NotificationCenter, LEVEL_COLORS
from ui.widgets.picker_item_delegate import PickerItemDelegate
from ui.notification_history_dialog import NotificationHistoryDialog
from ui.diagnostics_tab import DiagnosticsTab

from ui.services.job_store import JobStore
from ui.services import project_repository
from ui.services.cloud_pipeline import CloudPipelineService
from ui.services.report_service import ReportService
from ui.scan_pipeline_worker import ScanPipelineWorker

from shared.config_loader import CONFIG as cfg, load_config as load_yaml_config


BASE_DIR = cfg.BASE_DIR
CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
CLOUD_COMPARED_UPSAMPLE_TOPIC = f"{CLOUD_COMPARED_TOPIC}/upsample"

CLOUD_COMPARED_TOPIC_MANUAL = cfg.CLOUD_COMPARED_TOPIC + "_manual"
CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL = f"{CLOUD_COMPARED_TOPIC_MANUAL}/upsample"


PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC

DATA_DIR = cfg.DATA_DIR
# PROJECT_DIR/ACTIVE_JOB_FILE/CURRENT_JOB_FILE: single source of truth is
# ui.services.project_repository - see that module's docstring for why
# (this used to be redeclared independently in 5 different files).
PROJECT_DIR = project_repository.PROJECT_DIR
ACTIVE_JOB_FILE = project_repository.ACTIVE_JOB_FILE
CURRENT_JOB_FILE = project_repository.CURRENT_JOB_FILE

THICKNESS_DEFAULT = cfg.thickness.target  # Target thickness in meter -> convert mm to m
TOLERANCE_DEFAULT = cfg.thickness.tolerance  # Allowable tolerance in meter of thickness

# NotificationCenter history cap - shared by lblNotification, NotificationHistoryDialog,
# and the DIAGNOSTICS tab (DiagnosticsTab), so all three read from the same store.
DIAGNOSTICS_HISTORY_CAP = 500

settings = QSettings("JaconEquipment", "Intelijet")

class App(QMainWindow):

    cloud_received_signal = pyqtSignal(object, str)
    ui_send_cmd_signal = pyqtSignal(int)
    ui_data_update = pyqtSignal(object)  # ui.system_status.SystemStatus
    # (source, message, level, code) - see shared/notify.py. code is "" when
    # the notify() call didn't pass one (see shared/error_codes.py).
    # Replaces the old /rosout-JSON "notification" key inside
    # ui_data_update's dict.
    notification_received = pyqtSignal(str, str, str, str)

    def __init__(self):
        super().__init__()
        self.report_name = ""
        self.current_post_scan_path = ""
        self.isManualCompare = False


        # --- UI chính ---
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.show_3d_main_page()

        # --- Side-nav -> QStackedWidget page switching ---
        self.ui.btnNav3DMain.clicked.connect(self.show_3d_main_page)
        self.ui.btnNavJob.clicked.connect(lambda: self.ui.stacked.setCurrentWidget(self.ui.tab_jobnumber))
        self.ui.btnNavSystem.clicked.connect(lambda: self.ui.stacked.setCurrentWidget(self.ui.tab_system))
        self.ui.btnNavAlarm.clicked.connect(lambda: self.ui.stacked.setCurrentWidget(self.diagnostics_tab))
        self.ui.btnNavReport.clicked.connect(lambda: self.ui.stacked.setCurrentWidget(self.ui.tab_report))

        # ------Tab JobSetting ---
        # JobStore built here (not at its previous spot further down) so
        # ProjectManager can share this exact instance/cache instead of
        # opening a second independent JobStore on the same active_jobs.json.
        self.job_store = JobStore(ACTIVE_JOB_FILE, CURRENT_JOB_FILE)
        self.project_manager = ProjectManager(job_store=self.job_store)
        # Refresh the header's CURRENT JOB combobox the instant a job is
        # Scheduled/Removed in the JOB tab, instead of waiting for
        # _refresh_active_jobs' own 30s polling timer below to catch up.
        self.project_manager.active_jobs_changed.connect(self._refresh_active_jobs)
        if self.ui.tab_jobnumber.layout() is None:
            self.ui.tab_jobnumber.setLayout(QVBoxLayout())
        self.ui.tab_jobnumber.layout().addWidget(self.project_manager)

        # ------Tab REPORT (new merged Compare+Report screen) ---
        # Built alongside the old Compare/Report dialogs without touching
        # them - review this first, delete the old dialog code after.
        self.report_page_manager = ReportPageManager(
            job_store=self.job_store,
            on_view_3d=self.on_report_view_3d,
            on_start_compare=self.on_report_start_compare,
        )
        if self.ui.tab_report.layout() is None:
            self.ui.tab_report.setLayout(QVBoxLayout())
        self.ui.tab_report.layout().addWidget(self.report_page_manager)

        # --- VTK Viewer ---

        self.vtk_viewer = VTKViewer(self.ui.cloudFrame)
        self.ui.btnZoomCenter.released.connect(self.vtk_viewer.restore_initial_view)

        #TODO

        # --- ROS Thread ---
        self.ros_thread = RosThread(self.cloud_received_signal,
                                    self.ui_send_cmd_signal,
                                    self.ui_data_update,
                                    self.notification_received)
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

        # Set rntime parameter for ROS
        processing_switches = [
            self.ui.cbbAutoAlign,
            self.ui.cbbAutoCompare,
            self.ui.cbbAutoReport,
            self.ui.cbbRemoveGround,
            self.ui.cbbUseKeypoint,
            self.ui.cbbUpsample
        ]

        for item in processing_switches:
            item.setEnabled(False)
        self.ui.cbbRemoveGround.setEnabled(True)
        self.ui.cbbAutoReport.setEnabled(True)


        for cb in processing_switches:
            cb.toggled.connect(self.update_param)

        # --- Control Buttons ---
        # Pre/Post-Scan need Encoder + Scanner actually connected - start
        # disabled and let status_binder.py's STATUS_BINDINGS enable them
        # once the first device status snapshot confirms that.
        self.ui.btnPreScan.setEnabled(False)
        self.ui.btnPostScan.setEnabled(False)
        # self.ui.btnPreScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value))
        self.ui.btnPreScan.released.connect(self.confirm_and_send_prescan)
        self.ui.btnPostScan.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_POSTSCAN.value))
        # self.ui.btnCompare.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value))
        self.ui.btnCancel.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CANCEL_JOB.value))

        self.ui.btnOpenScanner.pressed.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.OPEN_HOUSING.value))
        self.ui.btnOpenScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.PAUSE_HOUSING.value))

        self.ui.btnCloseScanner.pressed.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.CLOSE_HOUSING.value))
        self.ui.btnCloseScanner.released.connect(lambda: self.ui_send_cmd_signal.emit(PPSCommand.PAUSE_HOUSING.value))
        self.ui.btnShutdown.released.connect(self.on_shutdown)

        self.ui.btnFullScreen.released.connect(self.toggle_full_screen)

        self.ui.btnLogin.released.connect(self.on_login_clicked)

        self.ui.btnSetHome.released.connect(self.confirm_and_send_sethome)

        # --- Select Job to work process ---
        # Same popup-row delegate as the REPORT tab's Project/Job pickers -
        # marks the current selection with a checkmark, gives rows real
        # spacing/height (QSS ::item rules don't reliably apply in this Qt
        # build - see PickerItemDelegate's docstring).
        self.ui.cbbJobSelect.setItemDelegate(PickerItemDelegate(self.ui.cbbJobSelect))
        self._populate_job_combobox()
        self.load_current_job()
        # self.ui.cbbJobSelect.currentIndexChanged.connect(self.on_job_changed)
        self.ui.cbbJobSelect.activated.connect(self.on_job_changed)
        self.current_job_index = self.ui.cbbJobSelect.currentIndex()

        # active_jobs.json can also be rewritten by another process (the two
        # tablets sync it via Syncthing), so refresh periodically instead of
        # only when the dropdown is opened.
        self._job_refresh_timer = QTimer(self)
        self._job_refresh_timer.timeout.connect(self._refresh_active_jobs)
        self._job_refresh_timer.start(30000)

        # --- Cloud/report services ---
        self.cloud_pipeline = CloudPipelineService()
        self.report_service = ReportService()

        # --- Status bar / notifications ---
        # Left: a "Connected"/"Disconnected" pill badge - driven live off
        # actual device connection state by status_binder.py (registered
        # into self.status_binder below), not hardcoded text. (A plain
        # colored-dot QLabel was tried first but wasn't visibly rendering
        # for the user, so a styled text badge is used instead.)
        self.lblSystemStatus = QLabel("Connected")
        self.lblSystemStatus.setObjectName("lblSystemStatus")
        self.ui.statusbar.addWidget(self.lblSystemStatus)

        # Right (permanent, stays put regardless of the transient
        # notification message): Mode/Version, hand-edited via
        # config/hmi_display.yaml rather than hardcoded here.
        try:
            hmi_display = load_yaml_config("hmi_display.yaml")
            mode = getattr(hmi_display, "mode", "--")
            version = getattr(hmi_display, "version", "--")
        except Exception as e:
            rospy.logwarn(f"Could not load hmi_display.yaml: {e}")
            mode, version = "--", "--"
        self.lblModeVersion = QLabel(f"Mode: {mode}  |  System Version: {version}")
        self.lblModeVersion.setStyleSheet(
            "color: #ffffff; background: transparent; border: none; font-size: 20px; margin-right: 8px;"
        )
        self.ui.statusbar.addPermanentWidget(self.lblModeVersion)

        self.lblNotification = QLabel("No Notifications")
        self.lblNotification.setStyleSheet("margin-left: 5px;")
        # A long message (e.g. a full error string) makes this QLabel's
        # sizeHint() grow past the window width, since it's added directly
        # to the QStatusBar layout with no cap - that widens the status
        # bar (and with it the whole window) past the screen, breaking
        # full-screen mode. Cap it and elide instead; full text is still
        # available via the tooltip and via clicking through to
        # NotificationHistoryDialog (_open_notification_history below).
        # Nothing else shares the status bar, so this can safely take most
        # of the screen width instead of the old 600px (which elided/lost
        # normal-length messages on this 2560px-wide screen).
        self.lblNotification.setMaximumWidth(2000)
        self.ui.statusbar.addWidget(self.lblNotification)

        self.notification_center = NotificationCenter(max_history=DIAGNOSTICS_HISTORY_CAP, parent=self)
        self.notification_center.label_changed.connect(self._on_notification_label_changed)
        self.notification_received.connect(
            lambda source, message, level, code: self.notification_center.push(
                source, message, level, code or None
            )
        )
        self.lblNotification.mousePressEvent = self._open_notification_history

        # --- Tab Diagnostics (HMI-style alarm log, read-only) ---
        self.diagnostics_tab = DiagnosticsTab(self.notification_center, parent=self.ui.stacked)
        self.ui.stacked.addWidget(self.diagnostics_tab)

        # --- Scan pipeline worker (runs convert/color/VTK/save/report off the GUI thread) ---
        self.scan_worker = ScanPipelineWorker(
            cloud_pipeline=self.cloud_pipeline,
            report_service=self.report_service,
            job_store=self.job_store,
            topics={
                "pre_scan": PRE_SCAN_CLOUD_TOPIC,
                "post_scan": POST_SCAN_CLOUD_TOPIC,
                "compared": CLOUD_COMPARED_TOPIC,
                "compared_upsample": CLOUD_COMPARED_UPSAMPLE_TOPIC,
                "compared_manual": CLOUD_COMPARED_TOPIC_MANUAL,
                "compared_upsample_manual": CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL,
            },
            project_dir=PROJECT_DIR,
            thickness_default=THICKNESS_DEFAULT,
            tolerance_default=TOLERANCE_DEFAULT,
            parent=self,
        )
        self.scan_worker.cloud_ready.connect(self._on_scan_cloud_ready)
        self.scan_worker.compare_requested.connect(
            lambda: self.ui_send_cmd_signal.emit(PPSCommand.START_COMPARE.value)
        )
        self.scan_worker.report_done.connect(self._on_scan_report_done)
        self.scan_worker.report_failed.connect(self._on_scan_report_failed)
        self.scan_worker.notify.connect(
            lambda source, message, level, code: self.notification_center.push(
                source, message, level, code or None
            )
        )

        # --- Data binder ---
        self.status_binder = StatusBinder(self.ui.centralFrame)
        # lblSystemStatus lives on the QMainWindow's own QStatusBar, not
        # under centralFrame, so it's not picked up by StatusBinder's
        # automatic findChildren() scan - register it by hand so
        # STATUS_BINDINGS can still drive it.
        self.status_binder.register("lblSystemStatus", self.lblSystemStatus)

        #Load ui state
        self.load_ui_state()
        # Send parameters to the ROS on the first boot.
        self.update_param()

        # Load last prescan path to runtime param
        self._load_last_prescan()

        self._mark_ui_ready()

    def _mark_ui_ready(self):
        """Touch a marker file the moment the main window is about to show.
        run_docker.sh watches for this (via the host bind mount at
        data/.ui_ready) to know when to close the startup terminal - see
        run_docker.sh. Best-effort: startup must never fail because of this."""
        try:
            ready_file = os.path.join(BASE_DIR, DATA_DIR, ".ui_ready")
            os.makedirs(os.path.dirname(ready_file), exist_ok=True)
            Path(ready_file).touch()
        except OSError as e:
            rospy.logwarn(f"Could not write UI-ready marker: {e}")

    # Setting parameter
    def save_ui_state(self):
        settings.setValue("cbbAutoAlign_on", self.ui.cbbAutoAlign.isChecked())
        settings.setValue("cbbAutoCompare_on", self.ui.cbbAutoCompare.isChecked())
        settings.setValue("cbbAutoReport_on", self.ui.cbbAutoReport.isChecked())
        settings.setValue("cbbRemoveGround_on", self.ui.cbbRemoveGround.isChecked())
        settings.setValue("cbbUseKeypoint_on", self.ui.cbbUseKeypoint.isChecked())
        settings.setValue("cbbUpsample_on", self.ui.cbbUpsample.isChecked())

    def load_ui_state(self):
        # New keys (cbb*_on, bool) - deliberately not reusing the old
        # cbb*_index keys (QComboBox.currentIndex(), 0="On"/1="Off"): a
        # leftover "0" read back with type=bool via QSettings/QVariant
        # coerces to False, which would silently flip every switch to OFF
        # on first run after this combo->checkbox migration.
        self.ui.cbbAutoAlign.setChecked(settings.value("cbbAutoAlign_on", True, type=bool))
        self.ui.cbbAutoCompare.setChecked(settings.value("cbbAutoCompare_on", True, type=bool))
        self.ui.cbbAutoReport.setChecked(settings.value("cbbAutoReport_on", True, type=bool))
        self.ui.cbbRemoveGround.setChecked(settings.value("cbbRemoveGround_on", True, type=bool))
        self.ui.cbbUseKeypoint.setChecked(settings.value("cbbUseKeypoint_on", True, type=bool))
        self.ui.cbbUpsample.setChecked(settings.value("cbbUpsample_on", True, type=bool))


    # Update runtime param to ROS
    def update_param(self):
        params = {
            "/runtime/do_align": self.ui.cbbAutoAlign.isChecked(),
            "/runtime/auto_compare": self.ui.cbbAutoCompare.isChecked(),
            "/runtime/auto_report": self.ui.cbbAutoReport.isChecked(),
            "/runtime/do_pre_process": self.ui.cbbRemoveGround.isChecked(),
            "/runtime/do_2d_keypoint": self.ui.cbbUseKeypoint.isChecked(),
            "/runtime/do_upsample": self.ui.cbbUpsample.isChecked(),
        }

        for key, value in params.items():
            rospy.set_param(key, value)


    #confirm_send_prescan_signal
    def confirm_and_send_prescan(self):
        # Pre-Scan never overwrites a file on disk (generate_filename always
        # stamps a fresh timestamp + a new scan_id) - what actually happens
        # is a new segment starts, and any not-yet-compared Post-Scans from
        # the PREVIOUS segment stop being this job's "current" Pre-Scan (see
        # compare_cloud_action_server.py's last_prescan_path fallback), so
        # they'd no longer auto-pair correctly. That's the real thing worth
        # confirming, not "overwrite".
        reply = QMessageBox.question(
            self,
            "Start New Segment?",
            "This starts a new segment with a fresh Pre-Scan.\n\n"
            "If the current job still has Post-Scans that haven't been "
            "compared yet, they will no longer be matched against this new "
            "Pre-Scan automatically. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Gửi signal nếu người dùng xác nhận
            self.ui_send_cmd_signal.emit(PPSCommand.START_PRESCAN.value)

    def confirm_and_send_sethome(self):
        reply = QMessageBox.question(
            self,
            "Confirm",
            "This will set current angle to zero. /nDo you want to continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Gửi signal nếu người dùng xác nhận
            self.ui_send_cmd_signal.emit(PPSCommand.PLC_SET_HOME_POSITION.value)

    # Load last prescan at startup, and put to topic compare_cloud_action_server can use it to continue compare when prescan cloud is missing.
    def _load_last_prescan(self):
        last_prescan_path = settings.value("last_prescan_path", "")
        if last_prescan_path:
            rospy.set_param("/runtime/last_prescan_path", last_prescan_path)
            

    # 1.0--- Update commond data from ROS (packages the job and hands it to
    # ScanPipelineWorker - the actual convert/color/save/report work runs
    # off the GUI thread, see scan_pipeline_worker.py) ---
    def on_cloud_received(self, msg, topic_name):
        print(f"Received cloud on topic {topic_name}")
        job = {
            "msg": msg,
            "topic_name": topic_name,
            # Snapshot App state now (GUI thread) so the worker never reads
            # self.* directly - avoids racing on_compare()/a later cloud
            # arrival that might change these before the worker gets to run.
            "is_manual": self.isManualCompare,
            "post_scan_path_snapshot": self.current_post_scan_path,
            "auto_compare_on": self.ui.cbbAutoCompare.isChecked(),
            "auto_compare_off": not self.ui.cbbAutoCompare.isChecked(),
            "auto_report_off": not self.ui.cbbAutoReport.isChecked(),
        }
        self.scan_worker.submit(job)

    def _on_scan_cloud_ready(self, polydata, metadata):
        self.vtk_viewer.update(polydata)
        # If a report export is about to run for this cloud, hold off
        # switching to 3D MAIN until it's actually done (see
        # _on_scan_report_done/_on_scan_report_failed) - otherwise switch
        # right away, same as before.
        if not metadata["report_pending"]:
            self.show_3d_main_page()

        if metadata["report_name"] is not None:
            self.report_name = metadata["report_name"]
        if metadata["reset_is_manual_compare"]:
            self.isManualCompare = False
        if metadata["reset_post_scan_path"]:
            self.current_post_scan_path = ""
        if metadata["last_prescan_path"] is not None:
            settings.setValue("last_prescan_path", metadata["last_prescan_path"])

    def _on_scan_report_done(self, final_path):
        self.notification_center.push("report", f"Report exported: {os.path.basename(final_path)}", "info")
        if getattr(self, "_compare_done_pending_report", False):
            self._compare_done_pending_report = False
            self.notification_center.push("compare", "✅COMPARE DONE ", "info")
        self.show_3d_main_page()

    def _on_scan_report_failed(self, error_message):
        rospy.logerr(f"[App] Failed to export report: {error_message}")
        self.notification_center.push("report", f"Report export failed: {error_message}", "error")
        if getattr(self, "_compare_done_pending_report", False):
            self._compare_done_pending_report = False
            self.notification_center.push("compare", "⚠️COMPARE DONE (report export failed) ", "warning")
        self.show_3d_main_page()


    def show_3d_main_page(self):
        """Switch the stacked content to 3D MAIN and keep the side-nav
        highlight in sync - used both by the nav button click and by
        code paths that jump here programmatically (a scan/compare cloud
        becoming ready)."""
        self.ui.stacked.setCurrentWidget(self.ui.tab_operator)
        self.ui.btnNav3DMain.setChecked(True)

    def toggle_full_screen(self):
        if not self.isFullScreen():
            self.showFullScreen()
            self.ui.btnFullScreen.setToolTip("Exit Full Screen")
        else:
            # Some window managers won't go straight from FullScreen to
            # Maximized (they just restore to the last windowed geometry
            # instead) - showNormal() first forces a clean state so
            # showMaximized() actually lands maximized.
            self.showNormal()
            self.showMaximized()
            self.ui.btnFullScreen.setToolTip("Full Screen")

    def on_login_clicked(self):
        
        from ui.widgets.security_manager import security
        from ui.widgets.login_dialog_view import LoginDialog
        from PyQt5.QtGui import QIcon


        def _on_auth_changed(level):
            self.ui.cbbAutoAlign.setEnabled(level >= security.ADMIN)
            self.ui.cbbUpsample.setEnabled(level >= security.ADMIN)
            self.ui.cbbAutoCompare.setEnabled(level >= security.ADMIN)
            self.ui.cbbUseKeypoint.setEnabled(level >= security.ADMIN)
            # self.ui.cbbAutoReport.setEnabled(level >= security.ADMIN)
            # self.ui.cbbRemoveGround.setEnabled(level >= security.ADMIN)
            

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
    def update_data(self, status):
        # status: ui.system_status.SystemStatus - see ros_thread.py's
        # emit_ui_data_update(). Device-state-change notifications are
        # published from the ROS side now (device_monitor.py's on_transition
        # hook), not diffed here. All widget pushes live in status_binder.py's
        # declarative STATUS_BINDINGS/PPS_BUTTON_STAGE_TABLE tables.
        self.status_binder.apply(status)

    # 3.--- Update pointcloud from available data---
    def update_pointcloud_from_data(self, data, filename=None):
        polydata = self.cloud_pipeline.to_vtk(data)
        if filename:
            print("updated polydata from file:", filename)
        self.vtk_viewer.update(polydata)
        self.show_3d_main_page()


    # --- New REPORT tab (ReportPageManager) callbacks ---
    def on_report_view_3d(self, filepath):
        """"3D View" button on a REPORT-tab row - load that .ply straight
        into the viewport, same conversion CompareManager.on_file_opened
        used for the old dialog's equivalent action."""
        from pps.data_converter import cloudconverter
        o3d_cloud = cloudconverter.load_ply(filepath)
        if o3d_cloud is None:
            self.notification_center.push("report", f"Failed to load point cloud: {filepath}", "error")
            return
        self.update_pointcloud_from_data(o3d_cloud, filepath)

    def on_report_start_compare(self, prescan_path, postscan_path):
        """"Compare Selected" on the REPORT tab - identical manual-compare
        pipeline as the old on_compare(), just sourced from the new
        page's own file picker instead of the CompareManager dialog."""
        self.isManualCompare = True
        self.current_post_scan_path = postscan_path

        from ui.compare_cloud_worker import CompareWorker

        do_align = rospy.get_param("/runtime/do_align", True)
        do_pre_process = rospy.get_param("/runtime/do_pre_process", True)
        do_2d_keypoint = rospy.get_param("/runtime/do_2d_keypoint", False)
        do_upsample = rospy.get_param("/runtime/do_upsample", False)
        do_post_process = rospy.get_param("/runtime/do_post_process", False)

        self.worker = CompareWorker(
            prescan_path=prescan_path,
            postscan_path=postscan_path,
            do_2d_keypoint=do_2d_keypoint,
            do_pre_process=do_pre_process,
            do_align=do_align,
            do_post_process=do_post_process,
            do_upsample=do_upsample
        )
        self.worker.progress.connect(self.on_compare_process)
        self.worker.finished.connect(self.on_compare_done)
        self.worker.start()

    def on_compare_process(self, progress, stage):
        progress = max(0.0, min(1.0, progress))  # clamp
        _string = f"COMPARE {int(progress * 100)}%"
        print(_string)
        self.notification_center.push_transient(_string, "info")


    def on_compare_done(self, success, job_id):

        if not success:
            print("❌COMPARE FAILED ", job_id)
            self.notification_center.push("compare", "❌COMPARE FAILED ", "error", "COMPARE-006")
            return

        print("✅COMPARE DONE ", job_id)

        if not self.ui.cbbAutoReport.isChecked():
            # No PDF export coming for this compare (scan_worker's report
            # step is skipped when auto-report is off) - nothing to wait
            # on, show the status right away like before.
            self.notification_center.push("compare", "✅COMPARE DONE ", "info")
        else:
            # A report export is about to run (scan_worker._process(),
            # triggered once the compared-cloud ROS message arrives) -
            # hold off on "Compare Done" until that actually finishes, so
            # the status bar doesn't say "done" while the PDF is still
            # being generated. _on_scan_report_done/_on_scan_report_failed
            # push the deferred message once the real outcome is known.
            self._compare_done_pending_report = True

    def _on_notification_label_changed(self, text, level):
        color = LEVEL_COLORS.get(level, LEVEL_COLORS["info"])
        self.lblNotification.setStyleSheet(f"margin-left: 5px; color: {color}; font-weight: bold;")
        elided = self.lblNotification.fontMetrics().elidedText(
            text, Qt.ElideRight, self.lblNotification.maximumWidth()
        )
        self.lblNotification.setText(elided)
        self.lblNotification.setToolTip(text)

    def _open_notification_history(self, event):
        dlg = NotificationHistoryDialog(self.notification_center, parent=self)
        dlg.exec_()

    # 7.--- Close event handler ---
    def closeEvent(self, event):
        # Give a possibly still-running scan pipeline job (report export can
        # take a while) a chance to finish cleanly; don't hang the app if it
        # doesn't - just move on and shut down anyway.
        if self.scan_worker.isRunning():
            if not self.scan_worker.wait(5000):
                rospy.logwarn("[App] closeEvent: scan_worker still running after 5s, shutting down anyway")

        # subprocess.call(["/mnt/c/work/projects/intelijet_v2/shutdown.sh"])
        subprocess.call(["rosnode", "kill", "-a"])
        subprocess.call("pkill -f ros", shell=True)
        subprocess.call(["rosclean", "purge", "-y"])
        event.accept()


    # 8.--- Shutdown handler ---
    def on_shutdown(self):
        msg = QMessageBox()
        msg.setWindowTitle("")
        msg.setText("Are you sure you want to quit?")
        self.save_ui_state()
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        msg.setWindowModality(True)
        reply = msg.exec_()
        if reply == QMessageBox.Yes:
            self.close()


    #10. change current job
    def on_job_changed(self, index):
        if index < 0:
            return  # không chọn gì cả

        if index == self.current_job_index: # Chỉ hỏi khi item khác item hiện tại
            value = self.ui.cbbJobSelect.itemText(index)
            try:
                self.job_store.set_current_job(value)
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
            try:
                self.job_store.set_current_job(value)
                self.current_job_index = self.ui.cbbJobSelect.currentIndex()
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Cannot save job: {e}")
        else:
            # Nếu user chọn No, quay lại giá trị cũ
            last_value = self.job_store.get_current_job()
            if last_value:
                idx = self.ui.cbbJobSelect.findText(last_value)
                if idx >= 0:
                    self.ui.cbbJobSelect.blockSignals(True)
                    self.ui.cbbJobSelect.setCurrentIndex(idx)
                    self.ui.cbbJobSelect.blockSignals(False)


    def load_job_info_from_file(self, filepath):
        from ui.models.job_info import JobInfo
        job_folder = os.path.dirname(filepath)
        job_info = JobInfo.load(job_folder)
        return job_info

    def load_current_job(self, text_only=False):
        """Load giá trị hiện tại của job từ file hoặc gán giá trị đầu tiên."""
        last_job = self.job_store.get_current_job()

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
    def _populate_job_combobox(self):
        self.ui.cbbJobSelect.clear()
        for job in self.job_store.list_active_jobs():
            display_text = f"{job['project']}/{job['job']}"
            self.ui.cbbJobSelect.addItem(display_text, job)

    def _refresh_active_jobs(self):
        """Periodic refresh (active_jobs.json can be rewritten by another
        process/tablet) - preserves the current selection instead of
        resetting it, since this runs regardless of user interaction."""
        current_text = self.ui.cbbJobSelect.currentText()
        self.job_store.reload()
        self.ui.cbbJobSelect.blockSignals(True)
        self._populate_job_combobox()
        idx = self.ui.cbbJobSelect.findText(current_text)
        if idx >= 0:
            self.ui.cbbJobSelect.setCurrentIndex(idx)
        self.ui.cbbJobSelect.blockSignals(False)
        self.current_job_index = self.ui.cbbJobSelect.currentIndex()

if __name__ == "__main__":

    app = QApplication(sys.argv)
    # Load style QSS tại đây

    app_icon = draw_app_icon()
    app.setWindowIcon(app_icon)

    keyboard_filter = TouchKeyboard()
    app.installEventFilter(keyboard_filter)

    with open(f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/app.qss") as f:
        app.setStyleSheet(f.read())

    viewer = App()
    viewer.setWindowIcon(app_icon)
    viewer.showFullScreen()
    sys.exit(app.exec_())