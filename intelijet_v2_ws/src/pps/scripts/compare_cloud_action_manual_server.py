#!/usr/bin/env python3
import rospy
import actionlib
import traceback
import uuid

from sensor_msgs.msg import PointCloud2
from pps.msg import CompareCloudAction, CompareCloudResult, CompareCloudFeedback
from pps.data_converter import cloudconverter
from pps.cloud_processing.compare_pipeline import CloudComparePipeline
from pps.tunnel_processing import TunnelProcessing

from shared.config_loader import CONFIG as cfg

CLOUD_OUT = cfg.CLOUD_COMPARED_TOPIC + "_manual"
CLOUD_UP = f"{CLOUD_OUT}/upsample"


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
        self.pub2 = rospy.Publisher(CLOUD_UP, PointCloud2, queue_size=1, latch=True)

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

            pre = cloudconverter.load_ply(goal.prescan_path, as_legacy=True)
            post = cloudconverter.load_ply(goal.postscan_path, as_legacy=True)

            if pre is None or post is None:
                self.server.set_aborted(CompareCloudResult(), "Missing file")
                return

            cloud, dist = self.pipeline.run(pre, post, goal, feedback_cb=self.fb)

            self.fb("publish", 0.85)

            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, "base_link")
            self.pub.publish(msg)
            rospy.loginfo("Published compared cloud to %s", CLOUD_OUT)

            # if goal.do_upsample:
            #     self.fb("upsample", 0.9)

            #     up = TunnelProcessing(cloud).run_upsample(cloud)
            #     msg2 = cloudconverter.o3d_tensor_to_pointcloud2(up, "base_link")
            # else:
            #     msg2 = msg
            # self.pub2.publish(msg2)

            rospy.loginfo("Published compared cloud to %s", CLOUD_UP)

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
            self.server.set_aborted(CompareCloudResult(), f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    rospy.init_node("compare_cloud_manual")
    CompareCloudManualServer()
    rospy.spin()