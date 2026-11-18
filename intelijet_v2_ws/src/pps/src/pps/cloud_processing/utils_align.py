from pps.cloud_processing.align_manager import PointCloudAlignerManager
from pps.cloud_processing.icp_aligner import ICPConfig
from pps.data_converter import cloudconverter
import numpy as np

# ===== Global instance với config mặc định =====

PRE_CONFIG = ICPConfig(
    threshold=  [1,    0.5, 0.2,   0.05, 0.02],
    voxel_radii=[0.2,  0.1, 0.1,   0.02, 0.01],
    max_iters=  [10, 10, 10, 30,  50],
    align_area=None,
)

DEFAULT_ICP_CONFIG = ICPConfig(
    threshold=  [0.15,   0.05, 0.05,  0.03, 0.02],
    voxel_radii=[0.02,  0.02,  0.01, 0.01, 0.01],
    max_iters=  [20, 20, 20, 100, 100],
    align_area=None,
)

def pre_align_cloud(post_cloud, pre_cloud):
    """
    Wrapper: convert tensor -> legacy, align, trả về ma trận transform
    """

    aligner = PointCloudAlignerManager(strategy="icp", config=PRE_CONFIG)
    post = cloudconverter.tensor_to_o3d_legacy(post_cloud)
    pre = cloudconverter.tensor_to_o3d_legacy(pre_cloud)

    aligner.align(post, pre)
    T = aligner.get_transformation_matrix()

    return T

global_aligner = PointCloudAlignerManager(strategy="icp", config=DEFAULT_ICP_CONFIG)
def align_cloud(post_cloud, pre_cloud, return_transform_only=False):
    """
    Wrapper: convert tensor -> legacy, align, trả về post_cloud đã transform
    source is post_cloud
    target is pre_cloud
    """
    post = cloudconverter.tensor_to_o3d_legacy(post_cloud)
    pre = cloudconverter.tensor_to_o3d_legacy(pre_cloud)
    global_aligner.align(post, pre)
    T = global_aligner.get_transformation_matrix()
    
    if return_transform_only:
        return T
    
    post.transform(T)
    return post
