from pps.cloud_processing.align_manager import PointCloudAlignerManager
from pps.cloud_processing.icp_aligner import ICPConfig
from pps.data_converter import cloudconverter
import numpy as np

# ===== Global instance với config mặc định =====

PRE_CONFIG = ICPConfig(
    threshold=[0.5,0.3],
    max_iters=[1,1],
    align_area=None,
    voxel_radii=[0.25,0.15]
)


DEFAULT_ICP_CONFIG = ICPConfig(
    threshold=[0.5,0.3,0.02],
    max_iters=[20,20,30],
    align_area=None,
    voxel_radii=[0.25,0.15,0.01]
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
