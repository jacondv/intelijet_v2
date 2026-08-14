# compare_base_client.py
import rospy
import actionlib
import time

from pps.msg import CompareCloudAction, CompareCloudGoal
from shared.config_loader import CONFIG as cfg

from shared.notify import notify


class CompareBaseClient:
    def __init__(self,
                 prescan_path="",
                 postscan_path="",
                 do_pre_process=True,
                 do_2d_keypoint=True,
                 do_post_process=False,
                 do_align=True,
                 do_upsample=True,
                 timeout=150.0):

        self.prescan_path = prescan_path
        self.postscan_path = postscan_path
        self.do_pre_process = do_pre_process
        self.do_2d_keypoint = do_2d_keypoint
        self.do_post_process = do_post_process
        self.do_align = do_align
        self.do_upsample = do_upsample
        self.timeout = timeout
        self._start_time = None
        self._timeout_timer = None

        self.client = actionlib.SimpleActionClient(
            "/compare_cloud",
            CompareCloudAction
        )
        rospy.loginfo("Waiting for /compare_cloud action server...")
        # No timeout here: at container startup this node can come up
        # before the action server node has finished initializing (ROS
        # master + several nodes starting together), so a short deadline
        # (e.g. 5s) logs a false "not available" error even though the
        # server appears moments later and later send_goal() calls work
        # fine anyway. Blocking here just means the UI waits a bit longer
        # once at startup instead of showing a misleading error.
        self.client.wait_for_server()
        rospy.loginfo("Connected to /compare_cloud")

    def set_prescan_path(self, path: str):
        self.prescan_path = path

    def set_postscan_path(self, path: str):
        self.postscan_path = path

    def set_pre_process(self, enable: bool):
        self.do_pre_process = enable

    def set_2d_keypoint(self, enable: bool):
        self.do_2d_keypoint = enable

    def set_post_process(self, enable: bool):
        self.do_post_process = enable

    def set_align(self, enable: bool):
        self.do_align = enable

    def set_upsample(self, enable: bool):
        self.do_upsample = enable

    def set_timeout(self, timeout: float):
        self.timeout = timeout

    def show_config(self):
        print("===== Current Config =====")
        print(f"Prescan path    : {self.prescan_path}")
        print(f"Postscan path   : {self.postscan_path}")
        print(f"Do pre-process  : {self.do_pre_process}")
        print(f"Do 2D keypoint  : {self.do_2d_keypoint}")
        print(f"Do post-process : {self.do_post_process}")
        print(f"Do align        : {self.do_align}")
        print(f"Do upsample     : {self.do_upsample}")
        print("==========================")

    # ---------- CALLBACKS ----------
    def _on_feedback(self, fb):
        # fb.stage: string
        # fb.progress: float [0-100]
        rospy.loginfo("COMPARE [%-12s] %3.0f%%", fb.stage, fb.progress*100)
        msg = f"[INFO] COMPARE [{fb.stage:<12}] {fb.progress*100:3.0f}%"
        notify(msg)


    def _on_done(self, state, result):
        if self._timeout_timer:
            self._timeout_timer.shutdown()

        state_str = actionlib.GoalStatus.to_string(state)
        rospy.loginfo("COMPARE DONE [%s] success=%s", state_str, result.success)   
        msg = "[INFO] COMPARE DONE [%s] success=%s" % (state_str, result.success)
        notify(msg)

    def _check_timeout(self, event):
        if self._start_time is None:
            return

        elapsed = time.time() - self._start_time
        state = self.client.get_state()

        # Nếu đã xong thì stop timer
        if state in [
            actionlib.GoalStatus.SUCCEEDED,
            actionlib.GoalStatus.ABORTED,
            actionlib.GoalStatus.REJECTED,
            actionlib.GoalStatus.PREEMPTED
        ]:
            self._timeout_timer.shutdown()
            return

        if elapsed > self.timeout:
            rospy.logerr("COMPARE TIMEOUT after %.1f seconds", elapsed)
            msg = f"[INFO] COMPARE TIMEOUT after {elapsed} seconds"
            notify(msg)
            self._timeout_timer.shutdown()
            self.client.cancel_goal()  


    # ---------- PUBLIC API ----------
    def send_goal(self):
        goal = CompareCloudGoal()
        goal.prescan_path = self.prescan_path
        goal.postscan_path = self.postscan_path
        goal.do_pre_process = self.do_pre_process
        goal.do_2d_keypoint = self.do_2d_keypoint
        goal.do_align = self.do_align
        goal.do_post_process = self.do_post_process
        goal.do_upsample = self.do_upsample

        self._start_time = time.time()

        self.client.send_goal(
            goal,
            feedback_cb=self._on_feedback,
            done_cb=self._on_done
        )

        # Check timeout mỗi 1s
        self._timeout_timer = rospy.Timer(
            rospy.Duration(5.0),
            self._check_timeout
        )


    def cancel(self):
        self.client.cancel_goal()
