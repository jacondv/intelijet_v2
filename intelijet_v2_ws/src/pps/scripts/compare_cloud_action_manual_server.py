#!/usr/bin/env python3
import rospy
import actionlib
import traceback
import uuid

from sensor_msgs.msg import PointCloud2
from pps.msg import CompareCloudAction, CompareCloudResult, CompareCloudFeedback
from pps.data_converter import cloudconverter
from pps.cloud_processing.compare_pipeline import CloudComparePipeline

from shared.config_loader import CONFIG as cfg
from shared.notify import notify

CLOUD_OUT = cfg.CLOUD_COMPARED_TOPIC + "_manual"


class CompareCloudManualServer:

    '''
    This server is used for manual comparison, which means it will load the pre and post scan from file path provided by the client, instead of subscribing to the cloud topic. This is useful for testing and debugging, as well as for comparing clouds that are not currently being scanned.
    '''

    def __init__(self):

        self.pipeline = CloudComparePipeline()

        self.server = actionlib.SimpleActionServer(
            "/compare_cloud_manual",
            CompareCloudAction,
            execute_cb=self.execute,
            auto_start=False
        )

        self.pub = rospy.Publisher(CLOUD_OUT, PointCloud2, queue_size=1, latch=True)

        self.server.start()
        rospy.loginfo("MANUAL compare server started")

    # ---------------- FEEDBACK ----------------
    def fb(self, stage, progress):
        f = CompareCloudFeedback()
        f.stage = stage
        f.progress = progress
        self.server.publish_feedback(f)

    # ---------------- EXECUTE ----------------
    def execute(self, goal):
        print(goal)
        job_id = uuid.uuid4().hex

        try:
            # ONLY FILE MODE
            self.fb("load", 0.1)

            # cloudconverter.load_ply (CloudConverter, not the pps.cloud_utils.io_utils
            # shim) never returns None on failure - it always raises
            # (FileNotFoundError/RuntimeError), caught by the except block below.
            pre = cloudconverter.load_ply(goal.prescan_path, as_legacy=True)
            post = cloudconverter.load_ply(goal.postscan_path, as_legacy=True)

            cloud, _dist = self.pipeline.run(pre, post, goal, feedback_cb=self.fb)

            self.fb("publish", 0.85)

            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, "base_link")
            self.pub.publish(msg)
            rospy.loginfo("Published compared cloud to %s", CLOUD_OUT)

            self.fb("done", 1.0)

            res = CompareCloudResult()
            res.success = True
            res.job_id = job_id
            self.server.set_succeeded(res)

        except Exception as e:
            # str(e) alone loses the exception type and, for some errors
            # (e.g. KeyError - str() is just repr() of the missing key,
            # so KeyError('') logs as the unhelpful ''), the real cause.
            # Full traceback makes every future failure here actually
            # diagnosable from the log instead of a bare message.
            rospy.logerr(f"{type(e).__name__}: {e}\n{traceback.format_exc()}")
            notify(message=f"Compare failed: {type(e).__name__}: {e}", code="COMPARE-005")
            self.server.set_aborted(CompareCloudResult(), f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    rospy.init_node("compare_cloud_manual")
    CompareCloudManualServer()
    rospy.spin()