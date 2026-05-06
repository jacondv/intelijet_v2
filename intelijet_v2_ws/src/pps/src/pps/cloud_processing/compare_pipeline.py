import numpy as np
import rospy

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import run_compare, remove_small_clusters
# from pps.cloud_compare.compare_method_m3c2 import compute_heatmap_m3c2_ep as compute_heatmap_to_plane
from pps.cloud_processing.utils_align import align_cloud
from pps.image_processing.keypoint_processing_v3 import KeypointCloudAlignManager

from pps.helper import crop_pointcloud_by_box, check_transform
from pps.cloud_processing.utils_align import align_cloud, pre_align_cloud
# from pps.cloud_processing.ceres_aligner import ceres_refine_icp

class CloudComparePipeline:

    def run(self, pre_cloud, post_cloud, goal, feedback_cb=None):

        def fb(stage, progress):
            if feedback_cb:
                feedback_cb(stage, progress)

        # ===== PRE ALIGN =====
        fb("pre-align", 0.15)
        T_pre_align = pre_align_cloud(post_cloud=post_cloud, pre_cloud=pre_cloud)
        post_cloud.transform(T_pre_align)

        post_crop = crop_pointcloud_by_box(
            post_cloud,
            min_bound=(0, -10, -0.3),
            max_bound=(11, 10, 7)
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

            # ===== ALIGN 2nd TIME =====
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


        # ===== PRE PROCESS (Crop ground)=====
        if goal.do_pre_process:
            fb("pre-process", 0.5)
            # pre_cloud = TunnelProcessing(pre_cloud).run_processing_pipeline()
            post_cloud = TunnelProcessing(post_cloud).run_processing_pipeline()


        # ===== POST PROCESS =====
        if goal.do_post_process:
            fb("post-process", 0.6)
            try:
                post_cloud = cloudconverter.crop_cloud_by_hull(pre_cloud, post_cloud)
            except Exception as e:
                rospy.logerr(f"post-process failed: {e}")

        # ===== COMPARE =====
        fb("compare", 0.7)

        post_cloud = remove_small_clusters(post_cloud,eps=0.2,min_points=4,min_cluster_size=10000)
        cloud_compared, distance = run_compare(
            source=post_cloud,
            target=pre_cloud
        )


        # indices = np.where(np.abs(distance) < 25.0)[0]
        # new_post_cloud = post_cloud.select_by_index(indices)

        # T,sumary = ceres_refine_icp(
        #     src_cloud=new_post_cloud,
        #     tgt_cloud=pre_cloud,
        #     init_T=np.eye(4),
        #     max_iter=20)

        # print(f"Ceres ICP refine result:\nT:\n{T}\nSummary:\n{sumary}")
        # cloud_compared, distance = compute_heatmap_to_plane(
        #         source=pre_cloud,
        #         target=post_cloud,
        #         target_thickness=30,
        #         tolerance_thickness=10,
        # )

        return cloud_compared, distance
