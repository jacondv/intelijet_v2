import open3d as o3d
import numpy as np
from pps.cloud_processing.base_aligner import CloudAligner, AlignMethodConfig
from pps.helper import crop_pointcloud_by_box
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union


@dataclass
class ICPConfig(AlignMethodConfig):
    """
    Configuration parameters specific to the ICP (Iterative Closest Point) alignment method.

    Attributes:
        threshold (List[float]): 
            A list of distance thresholds used at each ICP level, typically from coarse to fine.
            For example: [1.0, 0.05, 0.01].

        max_iters (List[int]): 
            A list of maximum iteration counts corresponding to each threshold level.
            For example: [30, 30, 30] for three ICP refinement levels.

        align_area (Optional[Tuple[Union[np.ndarray, list], Union[np.ndarray, list]]]):
            An optional tuple of (min_bound, max_bound), used to crop the point cloud before alignment.
            Each bound can be a list or numpy array of shape (3,), representing [x, y, z] boundaries.
            If None, no cropping is applied.
    """
    threshold: List[float] = (1.0, 0.05, 0.01)
    max_iters: List[int] = (30, 30, 30)
    align_area: Optional[
        Tuple[Union[np.ndarray, list], Union[np.ndarray, list]]
    ] = None

class ICPAligner(CloudAligner):
        
    def __init__(self, config: ICPConfig):  
        super().__init__()
        self.config = config

        self._min_bound = None
        self._max_bound = None
        self._transformation_matrix = np.eye(4, dtype=np.float64)

        # Parameters for multi-scale ICP
        # self._voxel_radii = voxel_radii
        self._max_iters = config.max_iters
        self._max_corr_dist = config.threshold
        # self._max_nns = [max(min_nn, int(25 * (0.01 / v))) for v in self._voxel_radii]


        self.__set_align_area(config.align_area)

    def __preprocess(self, pcd):

        # PCD is expected to be an Open3D PointCloud object
        if not isinstance(pcd, o3d.geometry.PointCloud):
            raise TypeError(f"[ICPAligner.__preprocess] Expected PointCloud, got {type(pcd).__name__}")
        if not pcd.has_points():
            raise ValueError(f"[ICPAligner.__preprocess] PointCloud is empty: received 0 points.")
        
        # pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self._radius, max_nn=self._max_nn))
        return pcd
    
    def __set_align_area(self, align_area):
        """Set the area for alignment by cropping the point cloud."""
 
        if align_area is None:
            self._min_bound = None
            self._max_bound = None
            return

        min_bound, max_bound = map(np.array, align_area)
        
        if not isinstance(min_bound, (list, np.ndarray)) or not isinstance(max_bound, (list, np.ndarray)):
            raise TypeError("[ICPAligner.__set_align_area] min_bound and max_bound must be lists or numpy arrays.")
        
        if len(min_bound) != 3 or len(max_bound) != 3:
            raise ValueError("[ICPAligner.__set_align_area] min_bound and max_bound must be 3-dimensional vectors.")
        
        if np.any(np.array(min_bound) >= np.array(max_bound)):
            raise ValueError("[ICPAligner.__set_align_area] min_bound and max_bound must not contain NaN values.")
        
        if np.any(np.isnan(min_bound)) or np.any(np.isnan(max_bound)):
            raise ValueError("[ICPAligner.__set_align_area] min_bound and max_bound must not contain infinite values.")
        
        if np.any(np.isinf(min_bound)) or np.any(np.isinf(max_bound)):
            raise ValueError("[ICPAligner.__set_align_area] min_bound must be less than max_bound in all dimensions.")
        
        self._min_bound = np.array(min_bound)
        self._max_bound = np.array(max_bound)

    def get_transformation_matrix(self)-> Optional[np.ndarray]:
        return self._transformation_matrix
        
    def align(self, source, target, init_transform)->o3d.geometry.PointCloud:
        """
        Align the source (post scan) to the target (pre scan) using multi-scale ICP.
        """
        import copy
        
        init_transform = np.eye(4) if init_transform is None else init_transform
        if isinstance(init_transform, np.ndarray):
            init_transform = np.array(init_transform, dtype=float)
        
        src = self.__preprocess(source)
        tgt = self.__preprocess(target)

        # Crop the point clouds to a bounding box
        if self._min_bound is None or self._max_bound is None:
            if src is None:
                raise ValueError("[ICPAligner.__preprocess] Source is none type")
            
            src_croped = copy.deepcopy(src)
            tgt_croped = tgt
        else:
            src_croped = crop_pointcloud_by_box(src,box_type="aabb",min_bound=self._min_bound, max_bound=self._max_bound)
            tgt_croped = crop_pointcloud_by_box(tgt,box_type="aabb",min_bound=self._min_bound, max_bound=self._max_bound)

        # Parameters for multi-scale ICP
        
        current_transform_matrix = init_transform.copy()

        for threshold, max_iter in zip(self.config.threshold, self.config.max_iters):
            result_icp = o3d.pipelines.registration.registration_icp(
                src_croped, tgt_croped,
                threshold,
                current_transform_matrix,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
            )
            current_transform_matrix = result_icp.transformation

        self._transformation_matrix = current_transform_matrix
        
        # Apply transformation to original source cloud
        aligned = copy.deepcopy(source)
        aligned.transform(self._transformation_matrix)
        
        return aligned

