#!/usr/bin/env python3
import rospy
import actionlib
import uuid

from sensor_msgs.msg import PointCloud2
from pps.msg import CompareCloudAction, CompareCloudResult, CompareCloudFeedback

from pps.data_converter import cloudconverter
from shared.config_loader import CONFIG as cfg
from pps.cloud_processing.compare_pipeline import CloudComparePipeline


PRE_SCAN_TOPIC = "/pre_scan_cloud"
POST_SCAN_TOPIC = "/post_scan_cloud"

CLOUD_OUT = cfg.CLOUD_COMPARED_TOPIC
CLOUD_UP = f"{CLOUD_OUT}/upsample"


class CompareCloudServer:

    def __init__(self):

        self.pre_cloud = None
        self.post_cloud = None

        self.pipeline = CloudComparePipeline()

        self.server = actionlib.SimpleActionServer(
            "/compare_cloud",
            CompareCloudAction,
            execute_cb=self.execute,
            auto_start=False
        )

        rospy.Subscriber(PRE_SCAN_TOPIC, PointCloud2, self._pre_cb)
        rospy.Subscriber(POST_SCAN_TOPIC, PointCloud2, self._post_cb)

        self.pub = rospy.Publisher(CLOUD_OUT, PointCloud2, queue_size=1, latch=True)
        self.pub2 = rospy.Publisher(CLOUD_UP, PointCloud2, queue_size=1, latch=True)

        self.server.start()
        rospy.loginfo("ONLINE compare server started")

    # ---------------- CALLBACK ----------------
    def _pre_cb(self, msg):
        self.pre_cloud = cloudconverter.pointcloud2_to_o3d(msg)

    def _post_cb(self, msg):
        self.post_cloud = cloudconverter.pointcloud2_to_o3d(msg)

    # ---------------- FEEDBACK ----------------
    def fb(self, stage, progress):
        f = CompareCloudFeedback()
        f.stage = stage
        f.progress = progress
        self.server.publish_feedback(f)

    # ---------------- EXECUTE ----------------
    def execute(self, goal):

        job_id = uuid.uuid4().hex

        try:
            pre = self.pre_cloud
            post = self.post_cloud

            if pre is None or post is None:
                self.server.set_aborted(CompareCloudResult(), "No cloud")
                return

            cloud, dist = self.pipeline.run(pre, post, goal, feedback_cb=self.fb)

            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, frame_id="base_link")
            self.pub.publish(msg)

            if goal.do_upsample:
                self.fb("upsample", 0.9)
                self.pub2.publish(msg)

            self.fb("done", 1.0)

            res = CompareCloudResult()
            res.success = True
            res.job_id = job_id
            self.server.set_succeeded(res)

        except Exception as e:
            rospy.logerr(str(e))
            self.server.set_aborted(CompareCloudResult(), str(e))


if __name__ == "__main__":
    rospy.init_node("compare_cloud_online")
    CompareCloudServer()
    rospy.spin()