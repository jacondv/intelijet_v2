import numpy as np
import rospy

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import compute_heatmap_to_plane
from pps.cloud_processing.utils_align import align_cloud
from pps.image_processing.keypoint_processing_v3 import KeypointCloudAlignManager

from pps.helper import crop_pointcloud_by_box, check_transform
from pps.cloud_processing.utils_align import align_cloud, pre_align_cloud

class CloudComparePipeline:

    def run(self, pre_cloud, post_cloud, goal, feedback_cb=None):

        def fb(stage, progress):
            if feedback_cb:
                feedback_cb(stage, progress)

        # ===== PRE ALIGN =====
        fb("pre-align", 0.15)
        T_pre_align = pre_align_cloud(post_cloud=post_cloud, pre_cloud=pre_cloud)
        post_cloud.transform(T_pre_align)

        # ===== PRE PROCESS =====
        if goal.do_pre_process:
            fb("pre-process", 0.2)

            pre_cloud = TunnelProcessing(pre_cloud).run_processing_pipeline()
            post_cloud = TunnelProcessing(post_cloud).run_processing_pipeline()

        post_crop = crop_pointcloud_by_box(
            post_cloud,
            min_bound=(0, -10, 0),
            max_bound=(11, 10, 3.25)
        )

        # ===== 2D KEYPOINT =====
        source_patch = None
        target_patch = None

        if goal.do_2d_keypoint:
            fb("extract-2d-keypoint", 0.3)

            kpm = KeypointCloudAlignManager(
                camera_intrinsics=None,
                lidar_to_cam_extrinsic=None,
                dist_coeffs=np.zeros(5),
                feature_method="SIFT",
                pixel_radius=100,
                cloud_radius=0.5,
                match_ratio=0.5
            )

            kpm.set_cloud1(pre_cloud)
            kpm.set_cloud2(post_crop)

            if kpm.is_ready():
                target_patch, source_patch, T = kpm.get_result()

        # ===== ALIGN =====
        if goal.do_align:
            fb("align", 0.4)

            src = source_patch if source_patch is not None else post_crop

            T = align_cloud(
                pre_cloud=pre_cloud,
                post_cloud=src,
                return_transform_only=True
            )

            if check_transform(T):
                post_cloud.transform(T)

        # ===== POST PROCESS =====
        if goal.do_post_process:
            fb("post-process", 0.6)
            try:
                post_cloud = cloudconverter.crop_cloud_by_hull(pre_cloud, post_cloud)
            except Exception as e:
                rospy.logerr(f"post-process failed: {e}")

        # ===== COMPARE =====
        fb("compare", 0.7)

        cloud_compared, distance = compute_heatmap_to_plane(
            source=post_cloud,
            target=pre_cloud,
            target_thickness=30,
            tolerance_thickness=10,
            k=6
        )

        return cloud_compared, distance
