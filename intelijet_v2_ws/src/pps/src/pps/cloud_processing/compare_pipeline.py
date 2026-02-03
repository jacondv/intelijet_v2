import numpy as np
import rospy

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import compute_heatmap_to_plane
from pps.cloud_processing.utils_align import align_cloud
from pps.image_processing.keypoint_processing_v2 import KeypointCloudAlignManager


class ComparePipeline:
    def __init__(self):
        self.keypoint_manager = None

    # ----------------------------
    def preprocess(self, pre_cloud, post_cloud, goal, feedback_cb=None):
        if feedback_cb:
            feedback_cb("pre-process", 0.2)

        # Crop ground + backside
        pre_cloud = TunnelProcessing(pre_cloud).run_processing_pipeline()
        post_cloud = TunnelProcessing(post_cloud).run_processing_pipeline()

        # Cloud -> Image
        pre_image, intrinsic1, extrinsic1 = cloudconverter.cloud_to_image(
            filename=None, pcd=pre_cloud, rot_x=-90, rot_y=90, rot_z=0
        )
        post_image, intrinsic2, extrinsic2 = cloudconverter.cloud_to_image(
            filename=None, pcd=post_cloud, rot_x=-90, rot_y=90, rot_z=0
        )

        self.keypoint_manager = KeypointCloudAlignManager(
            camera_intrinsics=intrinsic1,
            lidar_to_cam_extrinsic=extrinsic1,
            dist_coeffs=np.zeros(5),
            feature_method="SIFT",
            pixel_radius=50,
            cloud_radius=0.5,
            match_ratio=0.5
        )

        self.keypoint_manager.set_cloud1(pre_cloud)
        self.keypoint_manager.set_cloud2(post_cloud)
        self.keypoint_manager.set_image1(pre_image)
        self.keypoint_manager.set_image2(post_image)

        if self.keypoint_manager.is_ready():
            target_patch, source_patch, T = self.keypoint_manager.get_result()
        else:
            target_patch, source_patch = None, None

        return pre_cloud, post_cloud, target_patch, source_patch

    # ----------------------------
    def align(self, pre_cloud, post_cloud, target_patch, source_patch, goal, feedback_cb=None):
        if feedback_cb:
            feedback_cb("align", 0.4)

        if source_patch is not None:
            T = align_cloud(
                pre_cloud=target_patch,
                post_cloud=source_patch,
                return_transform_only=True
            )
            post_cloud.transform(T)
        else:
            post_cloud = align_cloud(
                pre_cloud=pre_cloud,
                post_cloud=post_cloud
            )

        return post_cloud

    # ----------------------------
    def postprocess(self, pre_cloud, post_cloud, goal, feedback_cb=None):
        if feedback_cb:
            feedback_cb("post-process", 0.6)

        try:
            post_cloud = cloudconverter.crop_cloud_by_hull(pre_cloud, post_cloud)
            
        except Exception as e: 
            print(f"Post process crop cloud by hull failed: {e}")
            
        return post_cloud

    # ----------------------------
    def compare(self, pre_cloud, post_cloud, goal, feedback_cb=None):
        if feedback_cb:
            feedback_cb("compare", 0.7)

        cloud_compared, distance = compute_heatmap_to_plane(
            source=post_cloud,
            target=pre_cloud,
            target_thickness=30,
            tolerance_thickness=10,
            k=6
        )

        return cloud_compared, distance

    # ----------------------------
    def run(self, pre_cloud, post_cloud, goal, feedback_cb=None):
        target_patch = source_patch = None

        if goal.do_pre_process:
            pre_cloud, post_cloud, target_patch, source_patch = \
                self.preprocess(pre_cloud, post_cloud, goal, feedback_cb)

        if goal.do_align:
            post_cloud = self.align(
                pre_cloud, post_cloud,
                target_patch, source_patch,
                goal, feedback_cb
            )

        if goal.do_post_process:
            post_cloud = self.postprocess(
                pre_cloud, post_cloud,
                goal, feedback_cb
            )

        cloud_compared, distance = self.compare(
            pre_cloud, post_cloud,
            goal, feedback_cb
        )

        return cloud_compared, distance



# from pps.cloud_processing.compare_pipeline import ComparePipeline

# class CompareCloudServer:

#     def __init__(self):
#         ...
#         self.pipeline = ComparePipeline()

#     def execute(self, goal):
#         result = CompareCloudResult()
#         feedback = CompareCloudFeedback()
#         job_id = uuid.uuid4().hex

#         def fb(stage, progress):
#             feedback.stage = stage
#             feedback.progress = progress
#             self.server.publish_feedback(feedback)

#         try:
#             fb("load", 0.1)

#             pre_cloud = cloudconverter.load_ply(goal.prescan_path, as_legacy=True)
#             post_cloud = cloudconverter.load_ply(goal.postscan_path, as_legacy=True)

#             cloud_compared, _ = self.pipeline.run(
#                 pre_cloud, post_cloud, goal, feedback_cb=fb
#             )

#             fb("publish", 0.9)
#             msg = cloudconverter.o3d_tensor_to_pointcloud2(
#                 cloud_compared, frame_id="base_link"
#             )
#             self.pub.publish(msg)

#             fb("done", 1.0)
#             result.success = True
#             result.job_id = job_id
#             self.server.set_succeeded(result)

#         except Exception as e:
#             rospy.logerr(e)
#             result.success = False
#             result.job_id = job_id
#             self.server.set_aborted(result)
