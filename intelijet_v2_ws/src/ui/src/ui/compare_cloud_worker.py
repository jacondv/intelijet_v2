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

class CompareWorker(QThread):
    progress = pyqtSignal(float, str)
    finished = pyqtSignal(bool, str)

    def __init__(self, prescan_path, postscan_path, do_pre_process, do_2d_keypoint, do_align, do_post_process, do_upsample):
        super().__init__()
        self.prescan_path = prescan_path
        self.postscan_path = postscan_path
        self.do_pre_process = do_pre_process
        self.do_align = do_align
        self.do_post_process=do_post_process
        self.do_2d_keypoint = do_2d_keypoint
        self.do_upsample = do_upsample

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

        client.wait_for_server()
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
        # ✅ CHỈ chờ action xong
        client.wait_for_result()

    def on_feedback(self, fb):
        self.progress.emit(fb.progress, fb.stage)

    def on_done(self, status, result):
        # self.finished.emit(result.success, result.job_id)
        self.finished.emit(result.success, self.postscan_path)


