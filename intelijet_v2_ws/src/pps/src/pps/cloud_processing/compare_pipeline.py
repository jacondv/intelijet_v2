import os
import time

import numpy as np
import rospy
from datetime import datetime
from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
from pps.helper import run_compare, keep_largest_cluster
from pps.cloud_processing.utils_align import align_cloud
from pps.image_processing.keypoint_processing_v3 import KeypointCloudAlignManager

from pps.helper import crop_pointcloud_by_box, check_transform
from pps.cloud_processing.utils_align import align_cloud, pre_align_cloud
from shared.config_loader import CONFIG as cfg
from shared.log_status import log_status


def _cfg(*names, default=None):
    """Safe nested getattr on CONFIG - returns `default` if any level is
    missing, so this keeps working even on a machine whose last_used.yaml
    predates the `compare_pipeline:` section in runtime.yaml."""
    obj = cfg
    for name in names:
        obj = getattr(obj, name, None)
        if obj is None:
            return default
    return obj


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
            min_bound=_cfg("compare_pipeline", "crop_box", "min", default=(0, -10, -0.3)),
            max_bound=_cfg("compare_pipeline", "crop_box", "max", default=(11, 10, 7))
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
                feature_method="SIFT", # SIFT is now not used.
                pixel_radius=_cfg("compare_pipeline", "keypoint", "pixel_radius", default=100),
                cloud_radius=_cfg("compare_pipeline", "keypoint", "cloud_radius", default=0.5),
                match_ratio=_cfg("compare_pipeline", "keypoint", "match_ratio", default=0.5)
            )

            kpm.set_cloud1(pre_cloud)
            kpm.set_cloud2(post_crop)

            if kpm.is_ready():
                target_patch, source_patch, T = kpm.get_result()
                import cv2
                image_out = kpm.draw_result()
                folder_path = f"{cfg.BASE_DIR}/{cfg.DATA_DIR}/log/{datetime.now().strftime('%Y%m%d')}/images"
                os.makedirs(folder_path, exist_ok=True)
                filename = f"{folder_path}/{int(time.time())}_keypoints.png"
                cv2.imwrite(filename, image_out)

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
                log_status(
                    name=cfg.NOTIFICATION,
                    message=f"[WARN] Post-process crop skipped (using uncropped cloud): {e}",
                    level="warning",
                )

        # ===== COMPARE =====
        fb("compare", 0.7)

        eps = _cfg("compare_pipeline", "clustering", "eps", default=0.1)
        min_points = _cfg("compare_pipeline", "clustering", "min_points", default=10)
        max_cluster_size_pass1 = _cfg("compare_pipeline", "clustering", "max_cluster_size_pass1", default=100)
        max_cluster_size_pass2 = _cfg("compare_pipeline", "clustering", "max_cluster_size_pass2", default=0)

        t0 = time.perf_counter()
        post_cloud = keep_largest_cluster(post_cloud, eps=eps, min_points=min_points, max_cluster_size=max_cluster_size_pass1)
        t1 = time.perf_counter()
        rospy.loginfo("[COMPARE] keep_largest_cluster pass1: %.2fs", t1 - t0)

        post_cloud = keep_largest_cluster(post_cloud, eps=eps, min_points=min_points, max_cluster_size=max_cluster_size_pass2)
        t2 = time.perf_counter()
        rospy.loginfo("[COMPARE] keep_largest_cluster pass2: %.2fs", t2 - t1)

        cloud_compared, distance = run_compare(
            source=post_cloud,
            target=pre_cloud
        )
        t3 = time.perf_counter()
        rospy.loginfo("[COMPARE] run_compare: %.2fs", t3 - t2)
        rospy.loginfo("[COMPARE] total: %.2fs", t3 - t0)

        return cloud_compared, distance
