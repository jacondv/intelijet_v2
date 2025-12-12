#!/usr/bin/env python3
import rospy
import actionlib
import uuid

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import compute_heatmap_to_plane
from shared.config_loader import CONFIG as cfg
from pps.cloud_processing.utils_align import align_cloud

from sensor_msgs.msg import PointCloud2
from pps.msg import (
    CompareCloudAction,
    CompareCloudResult,
    CompareCloudFeedback
)

CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC

class CompareCloudServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "compare_cloud",
            CompareCloudAction,
            execute_cb=self.execute,
            auto_start=False
        )
        #"compare_cloud/result_cloud"
        self.pub = rospy.Publisher(
            CLOUD_COMPARED_TOPIC,
            PointCloud2,
            queue_size=1,
            latch=True
        )

        self.server.start()
        rospy.loginfo("CompareCloud action server started")

    def execute(self, goal):
        rospy.logwarn("EXECUTE ENTERED")
        result = CompareCloudResult()
        feedback = CompareCloudFeedback()

        job_id = uuid.uuid4().hex
        rospy.loginfo(f"[{job_id}] Start compare")

        try:
            # ===== LOAD =====
            feedback.stage = "load"
            feedback.progress = 0.1
            self.server.publish_feedback(feedback)

            # TODO: load prescan & postscan cloud
            pre_cloud = cloudconverter.load_ply(goal.prescan_path, as_legacy=True)
            post_cloud = cloudconverter.load_ply(goal.postscan_path, as_legacy=True)

            # ===== PRE PROCESS=======#
            # This is auto crop ground and back side wall
            if goal.do_pre_process:
                feedback.stage = "pre-process"
                feedback.progress = 0.2

                pre_tunnel = TunnelProcessing(pre_cloud)
                pre_cloud = pre_tunnel.run_processing_pipeline()

                post_tunnel = TunnelProcessing(pre_cloud)
                pre_cloud = post_tunnel.run_processing_pipeline()

                self.server.publish_feedback(feedback)

            # ===== ALIGN =====
            if goal.do_align:
                feedback.stage = "align"
                feedback.progress = 0.4
                self.server.publish_feedback(feedback)

                # TODO: align cloud
                post_cloud = align_cloud(pre_cloud=pre_cloud, post_cloud=post_cloud)

                
            # ===== POST PROCESS =====
            # Recrop cloud after align
            if goal.do_post_process:
                feedback.stage = "post-process"
                feedback.progress = 0.6
                self.server.publish_feedback(feedback) 
                post_cloud = cloudconverter.crop_cloud_by_hull(pre_cloud,post_cloud)

            # ===== COMPARE =====
            feedback.stage = "compare"
            feedback.progress = 0.7
            self.server.publish_feedback(feedback)
          
            # TODO: compare + color

            cloud_compared, distance = compute_heatmap_to_plane(
                source=post_cloud, 
                target=pre_cloud, 
                target_thickness=30, 
                tolerance_thickness=10, 
                k=6
            )

            # ===== PUBLISH =====
            feedback.stage = "publish"
            feedback.progress = 0.9
            self.server.publish_feedback(feedback)

            # TODO: publish colored cloud
            # msg.header.frame_id = job_id
            # self.pub.publish(msg)
            frame_id = "base_link"
            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud_compared, frame_id=frame_id)
            self.pub.publish(msg)

            feedback.stage = "done"
            feedback.progress = 1.0
            self.server.publish_feedback(feedback)

            result.success = True
            result.job_id = job_id
            self.server.set_succeeded(result)

        except Exception as e:
            
            rospy.logerr(e)
            result.success = False
            result.job_id = job_id
            self.server.set_aborted(result)


if __name__ == "__main__":
    rospy.init_node("compare_cloud_server")
    CompareCloudServer()
    rospy.spin()
