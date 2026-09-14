# tunnel_ui/logic/cloud_manager.py
import os
import threading
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from pps.helper import load_ply,smooth_cloud,assign_colors

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing

from shared.config_loader import CONFIG as cfg

THICKNESS_TARGET = cfg.thickness.target  # Target thickness in meter -> convert mm to m
THICKNESS_TOLERANCE = cfg.thickness.tolerance  # Allowable tolerance in meter of thickness

from PyQt5.QtCore import QThread
import actionlib
import rospy
from pps.msg import CompareCloudAction, CompareCloudGoal

# Action server must accept the goal and finish within this long - if the
# connection drops mid-compare (network/PLC link loss, server process
# died), neither wait_for_server() nor wait_for_result() would otherwise
# ever return, hanging the UI indefinitely (see error_codes.yaml's
# COMPARE-002, previously defined but never actually enforced anywhere).
SERVER_WAIT_TIMEOUT_SEC = 120
RESULT_WAIT_TIMEOUT_SEC = 120


class CompareWorker(QThread):
    progress = pyqtSignal(float, str)
    finished = pyqtSignal(bool, str, str)  # success, postscan_path, error_code ("" on success)

    def __init__(self, prescan_path, postscan_path, do_pre_process, do_2d_keypoint, do_align, do_post_process, do_upsample):
        super().__init__()
        self.prescan_path = prescan_path
        self.postscan_path = postscan_path
        self.do_pre_process = do_pre_process
        self.do_align = do_align
        self.do_post_process=do_post_process
        self.do_2d_keypoint = do_2d_keypoint
        self.do_upsample = do_upsample
        self._result_lock = threading.Lock()
        self._result_emitted = False

    def run(self):
        # ✅ chạy trong worker thread

        from ui.models.job_info import JobInfo
        job_folder = os.path.dirname(self.postscan_path)
        job_info = JobInfo.load(job_folder)
        if job_info:
            self.target_thickness = job_info.parameters.get("target_thickness",THICKNESS_TARGET)/1000
            self.tolerance = job_info.parameters.get("tolerance",THICKNESS_TOLERANCE)/1000
        else:
            self.target_thickness = THICKNESS_TARGET
            self.tolerance = THICKNESS_TOLERANCE

        client = actionlib.SimpleActionClient(
            '/compare_cloud_manual',
            CompareCloudAction
        )

        if not client.wait_for_server(rospy.Duration(SERVER_WAIT_TIMEOUT_SEC)):
            self._emit_finished(False, "COMPARE-002")
            return

        goal = CompareCloudGoal()
        goal.prescan_path = self.prescan_path
        goal.postscan_path = self.postscan_path
        goal.do_pre_process = self.do_pre_process
        goal.do_2d_keypoint = self.do_2d_keypoint
        goal.do_post_process = self.do_post_process
        goal.do_align = self.do_align
        goal.do_upsample = self.do_upsample

        client.send_goal(
            goal,
            feedback_cb=self.on_feedback,
            done_cb=self.on_done
        )

        # rospy.spin()   # giữ thread sống SAI, ko dùng kiểu này
        # ✅ CHỈ chờ action xong - bounded, so a dropped connection/dead
        # server surfaces as a COMPARE-002 timeout instead of hanging the
        # UI forever (done_cb has already fired by the time this returns
        # True, so no duplicate finished.emit() here on the success path).
        finished_in_time = client.wait_for_result(rospy.Duration(RESULT_WAIT_TIMEOUT_SEC))
        if not finished_in_time:
            client.cancel_goal()
            self._emit_finished(False, "COMPARE-002")

    def on_feedback(self, fb):
        self.progress.emit(fb.progress, fb.stage)

    def on_done(self, status, result):
        # self.finished.emit(result.success, result.job_id)
        self._emit_finished(result.success, "" if result.success else "COMPARE-006")

    def _emit_finished(self, success, error_code):
        # done_cb (on_done) can still fire from actionlib's internal thread
        # right after wait_for_result() gives up and we cancel_goal() -
        # guard against emitting "finished" twice for the same run (the
        # second emit would let a second on_compare_done() decrement/reset
        # compare_in_progress state that the first one already handled).
        with self._result_lock:
            if self._result_emitted:
                return
            self._result_emitted = True
        self.finished.emit(success, self.postscan_path, error_code)


