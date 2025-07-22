import open3d as o3d
import numpy as np
from pps.cloud_processing.base_aligner import CloudAligner, AlignMethodConfig
from pps.helper import crop_pointcloud_by_box
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union


@dataclass
class RANSACConfig(AlignMethodConfig):
    voxel_size: float = 0.05  # Độ phân giải khi voxel downsampling
    max_correspondence_distance: Optional[float] = None  # Nếu không khai báo, sẽ tự tính theo voxel_size
    ransac_n: int = 4  # Số điểm cần cho mỗi lần nội suy RANSAC
    max_iteration: int = 100000  # Tổng số phép thử RANSAC tối đa
    max_validation: int = 1000  # Số lần xác nhận hợp lệ
    mutual_filter: bool = True  # Chỉ giữ các match đối ứng
    edge_length_checker: float = 0.9  # Ngưỡng kiểm tra độ dài cạnh
    distance_checker_factor: float = 1.5  # Nhân với voxel_size để thành ngưỡng khoảng cách

    def resolved_distance_threshold(self):
        """Nếu không thiết lập max_correspondence_distance, tự động tính theo voxel_size."""
        if self.max_correspondence_distance is not None:
            return self.max_correspondence_distance
        return self.voxel_size * self.distance_checker_factor


class RANSACCloudAligner(CloudAligner):
    def __init__(self, config: RANSACConfig):
        self.config = config
        self.__transform = np.eye(4)

    def get_transformation_matrix(self):
        return self.__transform

    def preprocess_point_cloud(self, pcd):
        pcd_down = pcd.voxel_down_sample(self.config.voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.config.voxel_size * 2, 
                max_nn=30
            )
        )
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.config.voxel_size * 5, 
                max_nn=100
            )
        )
        return pcd_down, pcd_fpfh

    def align(self, source, target, init_transform=np.eye(4)):
        # Crop theo config.align_area nếu có
        if self.config.align_area:
            min_bound, max_bound = self.config.align_area
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            source = source.crop(bbox)
            target = target.crop(bbox)

        src_down, src_fpfh = self.preprocess_point_cloud(source)
        tgt_down, tgt_fpfh = self.preprocess_point_cloud(target)

        distance_threshold = self.config.resolved_distance_threshold()

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down, tgt_down,
            src_fpfh, tgt_fpfh,
            mutual_filter=self.config.mutual_filter,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=self.config.ransac_n,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(self.config.edge_length_checker),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                self.config.max_iteration, 
                self.config.max_validation
            )
        )

        self.__transform = result.transformation
        return result
