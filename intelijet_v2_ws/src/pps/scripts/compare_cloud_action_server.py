#!/usr/bin/env python3
import rospy
import actionlib
import uuid
import numpy as np

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import compute_heatmap_to_plane
from shared.config_loader import CONFIG as cfg
from pps.cloud_processing.utils_align import align_cloud
from pps.image_processing.keypoint_processing_v2 import KeypointCloudAlignManager

PRE_SCAN_PROCESSED_TOPIC = "/pre_scan_cloud"
POST_SCAN_PROCESSED_TOPIC = "/post_scan_cloud"

from sensor_msgs.msg import PointCloud2
from pps.msg import (
    CompareCloudAction,
    CompareCloudResult,
    CompareCloudFeedback
)

CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
CLOUD_COMPARED_UPSAMPLE_TOPIC = f"{CLOUD_COMPARED_TOPIC}/upsample"

class CompareCloudServer:

    def __init__(self):

        # Save the cloud from topic
        self.pre_cloud = None
        self.post_cloud = None
        self.post_msg =None

        self.server = actionlib.SimpleActionServer(
            "/compare_cloud",
            CompareCloudAction,
            execute_cb=self.execute,
            auto_start=False
        )

        rospy.Subscriber(
            PRE_SCAN_PROCESSED_TOPIC,
            PointCloud2,
            self._pre_cloud_cb,
        )
        rospy.Subscriber(
            POST_SCAN_PROCESSED_TOPIC,
            PointCloud2,
            self._post_cloud_cb,
        )

        #"compare_cloud/result_cloud"
        self.pub = rospy.Publisher(
            CLOUD_COMPARED_TOPIC,
            PointCloud2,
            queue_size=1,
            latch=True
        )

        #"upsample cloud compared"
        self.pub2 = rospy.Publisher(
            CLOUD_COMPARED_UPSAMPLE_TOPIC,
            PointCloud2,
            queue_size=1,
            latch=True
        )

        self.server.start()
        rospy.loginfo("CompareCloud action server started")


    # Callbacks
    def _pre_cloud_cb(self, msg):
        self.pre_cloud = cloudconverter.pointcloud2_to_o3d(msg)

    def _post_cloud_cb(self, msg):
        # We should handle this later, as it takes a while and the compare command was called too early.
        self.post_msg = msg

    def execute(self, goal):
        rospy.logwarn("EXECUTE ENTERED")
        result = CompareCloudResult()
        feedback = CompareCloudFeedback()

        job_id = uuid.uuid4().hex
        # Wait 2 seconds to receive the self.post_cloud.
        rospy.loginfo(f"[{job_id}] Start compare")
        print(goal)
        self.post_cloud = cloudconverter.pointcloud2_to_o3d(self.post_msg)
        try:
            # ===== LOAD =====
            feedback.stage = "load"
            feedback.progress = 0.1
            self.server.publish_feedback(feedback)

            # TODO: load prescan & postscan cloud
            if goal.prescan_path:
                pre_cloud = cloudconverter.load_ply(goal.prescan_path, as_legacy=True)
                post_cloud = cloudconverter.load_ply(goal.postscan_path, as_legacy=True)
            else:
                # Use cloud from topic
                pre_cloud = self.pre_cloud
                post_cloud = self.post_cloud

            if pre_cloud is None or post_cloud is None:
                msg = "Missing cloud from topic"
                rospy.logerr(msg)
                result.success = False
                result.job_id = job_id
                self.server.set_aborted(result, msg)
                return

            # ===== PRE PROCESS=======#
            # This is auto crop ground and back side wall
            if goal.do_pre_process:
                feedback.stage = "pre-process"
                feedback.progress = 0.2
                self.server.publish_feedback(feedback)

                pre_tunnel = TunnelProcessing(pre_cloud)
                pre_cloud = pre_tunnel.run_processing_pipeline()

                post_tunnel = TunnelProcessing(post_cloud)
                post_cloud = post_tunnel.run_processing_pipeline()

            # ===== USE 2D KEYPOINT TO ALIGN=======#
            if goal.do_2d_keypoint:
                feedback.stage = "extract-2d-keypoint"
                feedback.progress = 0.2
                self.server.publish_feedback(feedback)

                #---- extract keypoint by image
                pre_image, intrinsic1, extrinsic1 = cloudconverter.cloud_to_image(filename=None,pcd=pre_cloud, rot_x=-90, rot_y=90, rot_z=0)
                post_image, intrinsic2, extrinsic2 = cloudconverter.cloud_to_image(filename=None,pcd=post_cloud, rot_x=-90, rot_y=90, rot_z=0)
 
                self.keypoint_manager = KeypointCloudAlignManager(camera_intrinsics=intrinsic1,
                                                    lidar_to_cam_extrinsic=extrinsic1,
                                                    dist_coeffs=np.zeros(5),
                                                    feature_method="SIFT",
                                                    pixel_radius=100,
                                                    cloud_radius=0.5,
                                                    match_ratio=0.5)
                
                self.keypoint_manager.set_cloud1(pre_cloud)
                self.keypoint_manager.set_cloud2(post_cloud)

                self.keypoint_manager.set_image1(pre_image)
                self.keypoint_manager.set_image2(post_image)

                if self.keypoint_manager.is_ready():    
                    target_patch, source_patch, T = self.keypoint_manager.get_result()
                    # _, source_patch, T = self.keypoint_manager.get_result()
                else:
                    source_patch = None
            else:
                source_patch=None
                target_patch=None

            # ===== ALIGN =====
            if goal.do_align:
                feedback.stage = "align"
                feedback.progress = 0.4
                self.server.publish_feedback(feedback)

                # TODO: align cloud
                if source_patch:
                    # cloudconverter.o3d_to_ply(source_patch,'/root/intelijet_v2/source_patch.ply')
                    # cloudconverter.o3d_to_ply(cloud1_target,'/root/intelijet_v2/cloud1_target.ply')
                    # cloudconverter.o3d_to_ply(post_cloud,'/root/intelijet_v2/post_cloud.ply')
                    # cloudconverter.o3d_to_ply(pre_cloud,'/root/intelijet_v2/pre_cloud.ply')
                    # post_cloud = align_cloud(pre_cloud=pre_cloud, post_cloud=post_cloud)
                    T = align_cloud(pre_cloud=target_patch, post_cloud=source_patch,return_transform_only=True)
                    post_cloud.transform(T)

                else:
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
            feedback.progress = 0.8
            self.server.publish_feedback(feedback)

            # TODO: publish colored cloud
            # msg.header.frame_id = job_id
            # self.pub.publish(msg)
            frame_id = "base_link"
            msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud_compared, frame_id=frame_id)
            self.pub.publish(msg)

            
            # ===== UPSAMPLE =====
            # Upsample and public cloud
            if goal.do_upsample:
                feedback.stage = "upsample"
                feedback.progress = 0.9
                self.server.publish_feedback(feedback)

                tunnel = TunnelProcessing(cloud_compared)
                cloud_compared_upsample = tunnel.run_upsample(cloud_compared)
                frame_id = "base_link"
                msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud_compared_upsample, frame_id=frame_id)
            # Sent cloud compared for report export
            self.pub2.publish(msg)


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
