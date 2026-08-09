# pps/cloud_utils/geometry.py
"""Point-cloud geometric operations: crop, smoothing, surface area.

Split out of the old pps/helper.py god-module (see docs/plan/phase_06_pps_cleanup.md).
"""
import numpy as np


def crop_pointcloud_by_box(pcd, box_type='aabb', center=None, extent=None,
                           min_bound=None, max_bound=None, rotation_rpy=(0, 0, 0)):
    """
    Crop a point cloud using a bounding box.

    Parameters:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        box_type (str): 'aabb' (Axis-Aligned Bounding Box) or 'obb' (Oriented Bounding Box).
        center (tuple): The center of the OBB (required if box_type == 'obb').
        extent (tuple): The dimensions of the box (dx, dy, dz) for OBB.
        min_bound (tuple): The minimum (x, y, z) coordinates for AABB.
        max_bound (tuple): The maximum (x, y, z) coordinates for AABB.
        rotation_rpy (tuple): Rotation in radians (roll, pitch, yaw) for OBB.

    Returns:
        cropped_pcd (open3d.geometry.PointCloud): The cropped point cloud.
    """
    import open3d as o3d

    if box_type == 'aabb':
        if min_bound is None or max_bound is None:
            raise ValueError("For AABB, both min_bound and max_bound are required.")
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)

    elif box_type == 'obb':
        if center is None or extent is None:
            raise ValueError("For OBB, both center and extent are required.")
        R = o3d.geometry.get_rotation_matrix_from_xyz(rotation_rpy)
        bbox = o3d.geometry.OrientedBoundingBox(center=center, extent=extent, R=R)

    else:
        raise ValueError("box_type must be either 'aabb' or 'obb'.")

    cropped_pcd = pcd.crop(bbox)
    return cropped_pcd


def smooth_cloud(tcloud, k=8, m=3, threshold=20.0):
    """
    Smooth distances and colors in a tensor PointCloud using KDTreeFlann (legacy).
    Converts t.geometry.PointCloud -> geometry.PointCloud internally.

    Args:
        tcloud: o3d.t.geometry.PointCloud with 'distances' and 'colors'
        k: number of neighbors
        m: min number of neighbors > threshold to trigger smoothing
        threshold: distance threshold for outlier detection (in mm)

    Returns:
        o3d.t.geometry.PointCloud: smoothed cloud (in-place)
    """

    import open3d as o3d

    # Convert to legacy geometry
    legacy_pc = tcloud.to_legacy()
    distances = tcloud.point['distances'].cpu().numpy()
    distances = np.abs(distances)
    colors = tcloud.point['colors'].cpu().numpy()
    new_distances = distances.copy()
    new_colors = colors.copy()
    
    # Build KDTree
    tree = o3d.geometry.KDTreeFlann(legacy_pc)
    
    for i, val in enumerate(distances):
        if val < threshold:
            continue
        
        # Search k nearest neighbors
        [_, idxs, _] = tree.search_knn_vector_3d(legacy_pc.points[i], k)
        neighbor_vals = distances[idxs]
        count_above = np.sum(neighbor_vals > threshold)
        
        if count_above <= m:
            new_distances[i] = neighbor_vals.mean()
            new_colors[i] = colors[idxs].mean(axis=0)
    
    # Update tensor cloud in-place
    tcloud.point['distances'] = o3d.core.Tensor(new_distances.astype(np.float32))
    tcloud.point['colors'] = o3d.core.Tensor(new_colors.astype(np.float32))
        
    return tcloud


def surface_area(
    pcd,
    radii=(0.05, 0.07, 0.1),
    estimate_normals=True
) -> float:
    """
    Tính diện tích bề mặt point cloud bằng Ball Pivoting Algorithm (BPA)

    Parameters
    ----------
    pcd : open3d.geometry.PointCloud
        Point cloud đầu vào
    radii : tuple
        Danh sách bán kính ball (nên tăng dần)
    estimate_normals : bool
        Có tự estimate normals hay không

    Returns
    -------
    area : float
        Diện tích bề mặt (đơn vị theo cloud)
    """
    import open3d as o3d
    import copy

    pcd = copy.deepcopy(pcd)

    if isinstance(pcd, o3d.t.geometry.PointCloud):
        pcd = pcd.to_legacy()
    

    pcd = pcd.voxel_down_sample(voxel_size=min(radii) / 2)


    cl, ind = pcd.remove_radius_outlier(nb_points=8, radius=2*min(radii))
    pcd = pcd.select_by_index(ind)


    if estimate_normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(radii) * 2,
                max_nn=30
            )
        )
        pcd.orient_normals_consistent_tangent_plane(50)

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector(radii)
    )
    # Save mesh for debugging
    # now = datetime.now().strftime("%Y%m%d_%H%M%S")
    # o3d.io.write_triangle_mesh(f"/root/intelijet_v2/data/log/{now}_mesh.ply",mesh)
    
    # Tính diện tích
    area = mesh.get_surface_area()
    return area
