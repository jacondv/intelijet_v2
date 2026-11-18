#!/usr/bin/env python3
"""Re-capture every screenshot in docs/manual/assets/ from the real UI.

Run it through capture_screenshots.sh (it starts the app's Docker image with
the repo mounted and sources the catkin workspace) - not directly on the host.

What it does: builds the real App main window (ui/scripts/app.py) at the
tablet's 2560x1600, without ROS hardware, then fakes the live state each shot
needs (device badges, scan-button frames, network indicator, notifications)
and saves QWidget.grab() images.

Read-only on purpose - capturing must not change the machine's data:
  - the CURRENT JOB is whatever current_job.json / active_jobs.json already
    say; JobStore.set_current_job is blocked,
  - notifications go to a temp log dir, not data/logs/,
  - QSettings switch positions, .debug_mode and .ui_ready are never written,
  - New Project / New Job / Rename panels are opened but never saved.

Usage (inside the container): python3 capture_screenshots.py [out_dir]
"""
import os
import sys
import tempfile
import time

REPO = os.environ.get("REPO_DIR", "/root/intelijet_v2")
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "docs/manual/assets")
WIDTH, HEIGHT = 2560, 1600

sys.path.insert(0, os.path.join(REPO, "intelijet_v2_ws/src/ui/scripts"))

import rospy  # noqa: E402

rospy.set_param = lambda *a, **k: None  # no ROS master while capturing

from ui import notification_center as nc  # noqa: E402

nc.LOG_DIR = tempfile.mkdtemp(prefix="manual_capture_logs_")

from PyQt5.QtCore import Qt, QTimer  # noqa: E402
from PyQt5.QtTest import QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402

import app as app_module  # noqa: E402
from shared.msg import DeviceStatus  # noqa: E402
from ui.services.job_store import JobStore  # noqa: E402
from ui.system_status import DeviceState, SystemStatus  # noqa: E402
from ui.services import project_repository as repo  # noqa: E402


class _NoRos:
    """Stands in for RosThread - nothing is subscribed/published."""

    def __init__(self, *a, **k):
        pass

    def start(self):
        pass

    def send_command(self, *a):
        pass


def _blocked(*a, **k):
    raise RuntimeError("capture_screenshots must not change the current job")


app_module.RosThread = _NoRos
app_module.App.save_ui_state = lambda self: None
app_module.App._mark_ui_ready = lambda self: None
app_module.App._on_debug_mode_toggled = lambda self, checked: None
JobStore.set_current_job = _blocked


# ---------------------------------------------------------------- helpers
def wait(ms=300):
    QTest.qWait(ms)


def save(widget, name):
    wait()
    path = os.path.join(OUT_DIR, f"{name}.png")
    widget.grab().save(path)
    print(f"  {name}.png  {widget.width()}x{widget.height()}")


def save_with_3d(win, name):
    """QWidget.grab() leaves the VTK (OpenGL) view blank - paint the real
    screen pixels of just that view over the normal grab, then the cloud-info
    label that floats above it."""
    from PyQt5.QtCore import QPoint
    from PyQt5.QtGui import QPainter
    wait()
    image = win.grab()
    vtk_widget = win.vtk_viewer.vtkWidget
    gl = QApplication.primaryScreen().grabWindow(vtk_widget.winId())
    painter = QPainter(image)
    painter.drawPixmap(vtk_widget.mapTo(win, QPoint(0, 0)), gl)
    info = win.ui.lblCloudInfo
    if info.isVisible():
        painter.drawPixmap(info.mapTo(win, QPoint(0, 0)), info.grab())
    painter.end()
    image.save(os.path.join(OUT_DIR, f"{name}.png"))
    print(f"  {name}.png  {image.width()}x{image.height()} (with 3D view)")


def status(disconnected=(), errors=(), pps=DeviceStatus.IDLE, encoder_deg=0.4):
    def state(name):
        if name in disconnected:
            return DeviceStatus.DISCONNECTED
        if name in errors:
            return DeviceStatus.ERROR
        return DeviceStatus.CONNECTED

    devices = {
        n: DeviceState(name=n, device_state=state(n), process_state="", mode="", detail="")
        for n in ("plc", "pcan", "lidar", "encoder")
    }
    devices["pps"] = DeviceState(name="pps", device_state=pps, process_state="", mode="", detail="")
    used_gb = repo.project_dir_size_bytes() / (1024 ** 3)
    return SystemStatus(
        devices=devices, encoder_deg=encoder_deg, encoder_raw=int(encoder_deg * 72817) + 29880000,
        storage_used_gb=used_gb, storage_max_gb=100,
        project_count=len(repo.list_projects()), project_max=50,
    )


def set_network(win, wifi=True, lan=False, connected=True, level=3, ssid="JACON_PPS"):
    ind = win.ui.networkIndicator
    ind._timer.stop()
    try:
        ind._signals.done.disconnect()  # drop a real reading still in flight
    except TypeError:
        pass
    ind._apply({
        "wifi": {"ssid": ssid, "signal_dbm": -52, "level": level} if wifi else None,
        "lan": lan,
        "connected": connected,
    })


def message_box(parent, title, text, buttons=QMessageBox.Yes | QMessageBox.No, icon=QMessageBox.Question):
    box = QMessageBox(icon, title, text, buttons, parent)
    box.setDefaultButton(QMessageBox.No)
    box.show()
    return box


def grab_dialog(dlg, name):
    dlg.show()
    save(dlg, name)
    dlg.close()


def nav(win, button):
    button.click()
    wait(500)


def find_button(root, text):
    from PyQt5.QtWidgets import QPushButton
    for b in root.findChildren(QPushButton):
        if b.text().strip() == text and b.isVisible():
            return b
    return None


# ---------------------------------------------------------------- capture
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    qapp = QApplication(sys.argv)
    with open(os.path.join(REPO, "intelijet_v2_ws/src/ui/src/ui/app.qss")) as f:
        qapp.setStyleSheet(f.read())

    win = app_module.App()
    # The status-bar message label's minimum width follows its text (up to
    # 2000px), so a long message widens the window past the 2560px screen.
    # Let it shrink here so every shot is the real tablet width.
    win.lblNotification.setMinimumWidth(1)
    win.resize(WIDTH, HEIGHT)
    win.showFullScreen()
    win._clock_timer.stop()
    win._job_refresh_timer.stop()
    win._update_clock()
    set_network(win)
    win.update_data(status())
    wait(1500)

    current_job = win.job_store.get_current_job()
    print(f"Current job (unchanged): {current_job!r}; header shows {win.ui.cbbJobSelect.currentText()!r}")

    # ---- 3D MAIN with the newest heatmap of the current job
    project, job = repo.parse_job_ref(current_job) if current_job else (None, None)
    heatmap = None
    if project and job:
        job_dir = os.path.join(repo.PROJECT_DIR, project, job)
        clouds = sorted(f for f in os.listdir(job_dir) if "cloud_compared" in f and f.endswith(".ply"))
        if clouds:
            heatmap = os.path.join(job_dir, clouds[-1])
    if heatmap:
        win.on_report_view_3d(heatmap)
        wait(2000)
        win.vtk_viewer.restore_initial_view()
        if win.vtk_viewer.box_widget is not None:
            win.vtk_viewer.box_widget.Off()  # its centre handle can cover the view
        win.vtk_viewer.vtkWidget.GetRenderWindow().Render()
    if heatmap:
        win.notification_center.push("report", f"Report exported: {os.path.basename(heatmap)[:-4]}.pdf", "info")
    wait(1500)
    save_with_3d(win, "01_main_3d")

    # ---- header / side nav / control panel / status bar
    save(win.ui.header, "02_header")
    save(win.ui.networkIndicator, "02c_network_wifi")
    set_network(win, wifi=True, lan=True)
    save(win.ui.networkIndicator, "02d_network_wifi_lan")
    set_network(win, wifi=False, lan=True)
    save(win.ui.networkIndicator, "02e_network_lan")
    set_network(win, wifi=False, lan=False, connected=False)
    save(win.ui.networkIndicator, "02f_network_none")
    save(win.ui.header, "02g_header_no_network")
    set_network(win)
    save(win.ui.lblClock, "02h_clock")

    win.ui.cbbJobSelect.showPopup()
    wait(500)
    save(win.ui.cbbJobSelect.view().window(), "02b_current_job_dropdown")
    win.ui.cbbJobSelect.hidePopup()

    save(win.ui.side_nav, "03_side_nav")
    save(win.ui.control_panel, "04_control_panel")
    win.update_data(status(pps=DeviceStatus.PRESCAN, encoder_deg=74.0))
    save(win.ui.control_panel, "04b_control_panel_prescan_running")
    win.update_data(status(pps=DeviceStatus.PRESCAN_ERROR))
    save(win.ui.control_panel, "04c_control_panel_prescan_error")
    win.update_data(status(disconnected=("lidar",)))
    save(win.ui.control_panel, "04d_control_panel_scan_locked")
    win.update_data(status())
    save(win.ui.statusbar, "05_statusbar_connected")

    win.update_data(status(disconnected=("pcan", "plc", "encoder")))
    win.notification_center.push("device", "PCAN Ethernet Gateway disconnected", "error", "DEVICE-PCAN-001")
    wait(500)
    save(win.ui.statusbar, "05b_statusbar_disconnected_error")
    win.update_data(status())
    win._on_notification_label_changed("No Notifications", "info")

    # ---- JOB tab
    pm = win.project_manager
    nav(win, win.ui.btnNavJob)
    save(win, "10_job_tab")
    save(pm.projectsCard, "11_job_projects_list")
    save(pm.scheduleCardOuter, "12_job_work_schedule")
    pm.new_project()
    wait(500)
    save(pm.projectsCard, "13_job_new_project")
    pm._cancel_new_project()
    if project:
        pm.new_job(project)
        wait(500)
        save(pm.projectsCard, "14_job_new_job_panel")
        pm._cancel_job_panel()
        pm.render_projects()
    unscheduled = [p for p in repo.list_projects()
                   if not any(j["project"] == p for j in win.job_store.list_active_jobs())]
    if unscheduled:
        pm._renaming_project = unscheduled[0]
        pm.render_projects()
        wait(500)
        save(pm.projectsCard, "15_job_rename_project")
        pm._cancel_rename_project()
        pm.render_projects()

    # ---- SYSTEM tab
    nav(win, win.ui.btnNavSystem)
    save(win, "20_system_tab")
    save(win.ui.deviceCard, "21_system_device_card")
    win.update_data(status(disconnected=("pcan", "plc", "encoder")))
    save(win.ui.deviceCard, "21b_system_device_card_disconnected")
    win.update_data(status(errors=("lidar",)))
    save(win.ui.deviceCard, "21c_system_device_card_lidar_error")
    win.update_data(status())
    save(win.ui.processingCardScroll, "22_system_processing_card")
    for cb in (win.ui.cbbAutoAlign, win.ui.cbbAutoCompare, win.ui.cbbUseKeypoint, win.ui.cbbTrimToPrescan):
        cb.setEnabled(True)
    save(win.ui.processingCardScroll, "22b_system_processing_card_admin")
    for cb in (win.ui.cbbAutoAlign, win.ui.cbbAutoCompare, win.ui.cbbUseKeypoint, win.ui.cbbTrimToPrescan):
        cb.setEnabled(False)
    save(win.ui.storageCard, "23_system_storage_card")

    # ---- ALARM tab: a realistic set of messages (temp log dir, see top)
    pushes = [
        ("hmi", "Starting Pre-Scan", "info", None),
        ("scan", "Scan completed", "info", None),
        ("hmi", "Starting Post-Scan", "info", None),
        ("scan", "Encoder did not reach target angle while opening the housing", "warning", "SCAN-002"),
        ("compare", "Compare failed (auto-compare): not enough overlap between Pre-Scan and Post-Scan", "error", "COMPARE-004"),
        ("device", "Lidar disconnected (network unreachable)", "error", "DEVICE-LIDAR-001"),
        ("device", "PCAN Ethernet Gateway disconnected", "error", "DEVICE-PCAN-001"),
        ("system", "App time zone does not match the tablet's time zone - file names and report times may be wrong",
         "warning", "SYSTEM-TIME-001"),
        ("report", "Report exported: JOB01#20260915_103151#cloud_compared_05#SCAN001.pdf", "info", None),
    ]
    for source, message, level, code in pushes:
        win.notification_center.push(source, message, level, code)
        time.sleep(1.1)  # NotificationCenter de-duplicates bursts
    tab = win.diagnostics_tab
    nav(win, win.ui.btnNavAlarm)
    tab._level_filter.setCurrentText("All Levels")
    wait(500)
    save(win, "30_alarm_tab")
    tab._level_filter.setCurrentText("Error")
    wait(500)
    save(win, "31_alarm_tab_errors_only")
    history = win.notification_center.history()
    for code, name in (("COMPARE-004", "32_alarm_detail"),
                       ("DEVICE-PCAN-001", "33_alarm_detail_device"),
                       ("SYSTEM-TIME-001", "34_alarm_detail_time")):
        item = next((i for i in history if i.get("code") == code), None)
        if item:
            tab._open_detail_dialog(item)
            wait(500)
            save(tab._detail_dlg, name)
            tab._detail_dlg.close()

    # ---- REPORT tab (opens on the current job)
    rp = win.report_page_manager
    nav(win, win.ui.btnNavReport)
    rp._jump_to_current_job()
    wait(800)
    save(win, "40_report_tab")
    save(rp.reportTopBar, "41_report_top_bar")
    files = sorted(f for f in os.listdir(os.path.join(repo.PROJECT_DIR, project, job))
                   if f.endswith(".ply") and ("pre_scan" in f or "post_scan" in f)) if project else []
    pre = next((f for f in files if "pre_scan" in f), None)
    post = next((f for f in files if "post_scan" in f), None)
    if pre and post:
        job_dir = os.path.join(repo.PROJECT_DIR, project, job)
        rp._on_row_checked(os.path.join(job_dir, pre), True)
        rp._on_row_checked(os.path.join(job_dir, post), True)
        wait(500)
        save(win, "42_report_compare_selected")
        save(rp.compareBar, "43_report_compare_bar")
        rp._checked_files = []
        rp.render_files()
    save(rp.filesCard, "44_report_segment_rows")

    # ---- dialogs
    nav(win, win.ui.btnNav3DMain)
    from ui.widgets.login_dialog_view import LoginDialog
    grab_dialog(LoginDialog(win), "50_dialog_login")
    box = message_box(win, "Start New Segment?",
                      "This starts a new segment with a fresh Pre-Scan.\n\n"
                      "If the current job still has Post-Scans that haven't been "
                      "compared yet, they will no longer be matched against this new "
                      "Pre-Scan automatically. Continue?")
    save(box, "51_dialog_prescan_confirm"); box.close()
    box = message_box(win, "", "Are you sure you want to quit?", icon=QMessageBox.NoIcon)
    save(box, "52_dialog_quit_confirm"); box.close()
    box = message_box(win, "Confirm", f"Do you want to select job: {win.ui.cbbJobSelect.currentText()}?")
    save(box, "53_dialog_select_job"); box.close()
    box = message_box(win, "Confirm", "This will set current angle to zero. /nDo you want to continue?")
    save(box, "54_dialog_set_zero"); box.close()
    box = message_box(win, "Active Job", "Cannot edit an active job. Please remove it from active jobs first.",
                      QMessageBox.Ok, QMessageBox.Warning)
    save(box, "55_dialog_active_job_locked"); box.close()
    box = message_box(win, "Job Finished",
                      "Job 'JOB01' is already Finished.\nDo you want to reset it to Pending and schedule it again?")
    save(box, "56_dialog_reschedule_finished"); box.close()
    from ui.notification_history_dialog import NotificationHistoryDialog
    dlg = NotificationHistoryDialog(win.notification_center, parent=win)
    grab_dialog(dlg, "57_dialog_notification_history")

    print(f"Done. Current job still {win.job_store.get_current_job()!r}")
    QTimer.singleShot(0, qapp.quit)
    os._exit(0)  # skip App.closeEvent (it kills ROS nodes)


if __name__ == "__main__":
    main()
