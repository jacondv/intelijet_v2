#!/usr/bin/env python3
import rospy
import actionlib
import threading
import traceback
import uuid

from sensor_msgs.msg import PointCloud2
from pps.msg import CompareCloudAction, CompareCloudResult, CompareCloudFeedback

from pps.data_converter import cloudconverter
from shared.config_loader import CONFIG as cfg
from shared.notify import notify
from pps.cloud_processing.compare_pipeline import CloudComparePipeline

from pps.tunnel_processing import TunnelProcessing

POST_SCAN_TOPIC = "/post_scan_cloud"

CLOUD_OUT = cfg.CLOUD_COMPARED_TOPIC
CLOUD_UP = f"{CLOUD_OUT}/upsample"


class CompareCloudServer:

    def __init__(self):

        self.post_cloud = None
        # Set by _post_cb, waited on in execute() instead of busy-polling
        # with rospy.sleep(1). Cleared the moment execute() consumes it
        # (see execute() below) - a post-scan cloud is single-use per
        # compare cycle, and leaving this set after a failed/aborted
        # cycle used to make the NEXT cycle's wait() below return
        # instantly against that stale, already-consumed cloud instead of
        # genuinely waiting for its own post-scan message to arrive (the
        # actual cause of "compare ran before the post-scan had fully
        # arrived" reported from the field - not a slow file/message
        # load, but a leftover flag from a previous failure).
        self.post_cloud_event = threading.Event()

        self.pipeline = CloudComparePipeline()

        self.server = actionlib.SimpleActionServer(
            "/compare_cloud",
            CompareCloudAction,
            execute_cb=self.execute,
            auto_start=False
        )

        rospy.Subscriber(POST_SCAN_TOPIC, PointCloud2, self._post_cb)

        self.pub = rospy.Publisher(CLOUD_OUT, PointCloud2, queue_size=1, latch=True)
        self.pub2 = rospy.Publisher(CLOUD_UP, PointCloud2, queue_size=1, latch=True)

        self.server.start()
        rospy.loginfo("ONLINE compare server started")

    # ---------------- CALLBACK ----------------
    def _post_cb(self, msg):
        self.post_cloud = cloudconverter.pointcloud2_to_o3d(msg)
        self.post_cloud_event.set()

    def _load_latest_prescan(self):
        """Pre-Scan is no longer taken from an in-memory clouds kept by
        this node's own /pre_scan_cloud subscription - it's always
        reloaded from the most recently SAVED Pre-Scan file for whatever
        job is current (last_prescan_path, written by the UI right after
        every successful Pre-Scan - see app.py's update_data/_process in
        scan_pipeline_worker.py). Two reasons this is safer, not just
        different:
          - The in-memory cloud was whatever this node's OWN subscriber
            happened to see last, with no job/segment scoping at all - if
            the operator switched jobs between Pre-Scan and Post-Scan, it
            would silently keep comparing against the wrong job's cloud.
          - The two tablets share job data over Syncthing (files), not
            ROS params/topics - a Pre-Scan done on one tablet was never
            visible to the OTHER tablet's compare server under the old
            in-memory approach; the saved file is.
        Timing is safe: Post-Scan (which is what actually triggers a
        compare, see hmi_scan_command_handler.py) always happens well
        after its Pre-Scan is done and its file fully saved - there's no
        "file not finished writing yet" race here the way there is for
        the just-arrived post-scan cloud above.
        """
        path = rospy.get_param("/runtime/last_prescan_path", "")
        if not path:
            rospy.logwarn("No Pre-Scan recorded yet (/runtime/last_prescan_path is empty)")
            return None
        cloud = cloudconverter.load_ply(path, as_legacy=True)
        if cloud is None:
            rospy.logwarn(f"Failed to load Pre-Scan cloud from: {path}")
        return cloud

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
            if not self.post_cloud_event.wait(timeout=10):
                self.server.set_aborted(CompareCloudResult(), "Timeout waiting for post-scan cloud")
                return

            # Consume it now, before doing anything else that could fail -
            # see post_cloud_event's docstring above for why this can't
            # wait until after a successful run.
            post = self.post_cloud
            self.post_cloud = None
            self.post_cloud_event.clear()

            pre = self._load_latest_prescan()

            if pre is None or post is None:
                self.server.set_aborted(CompareCloudResult(), "No cloud")
                return

            cloud, dist = self.pipeline.run(pre, post, goal, feedback_cb=self.fb)

            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, frame_id="base_link")
            self.pub.publish(msg)

            # if goal.do_upsample:
            #     self.fb("upsample", 0.9)

            #     up = TunnelProcessing(cloud).run_upsample(cloud)
            #     msg2 = cloudconverter.o3d_tensor_to_pointcloud2(up, "base_link")
            # else:
            #     msg2 = msg
            
            # self.pub2.publish(msg2)

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
            notify(message=f"Compare failed: {type(e).__name__}: {e}", code="COMPARE-004")
            self.server.set_aborted(CompareCloudResult(), f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    rospy.init_node("compare_cloud_online")
    CompareCloudServer()
    rospy.spin()