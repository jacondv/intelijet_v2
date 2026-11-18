# pps/cloud_utils/geometry.py
"""Point-cloud geometric operations: crop, smoothing, surface area.

Split out of the old pps/helper.py god-module (see docs/plan/phase_06_pps_cleanup.md).
"""
import time
import numpy as np
import rospy


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

    voxel_size = min(radii) / 2

    t0 = time.perf_counter()
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    t1 = time.perf_counter()
    rospy.loginfo("[surface_area] voxel_down_sample: %.2fs (%d pts)", t1 - t0, len(pcd.points))

    cl, ind = pcd.remove_radius_outlier(nb_points=8, radius=2 * min(radii))
    pcd = pcd.select_by_index(ind)
    t2 = time.perf_counter()
    rospy.loginfo("[surface_area] remove_radius_outlier: %.2fs (%d pts)", t2 - t1, len(pcd.points))

    if len(pcd.points) == 0:
        # estimate_normals()/ball_pivoting() both raise on an empty cloud -
        # remove_radius_outlier can legitimately strip every point on a
        # sparse/noisy enough input. Report 0 area instead of crashing the
        # whole report export, but surface it - a silent 0 would look like
        # a real (if unlikely) measurement instead of "detection failed".
        rospy.logwarn("[surface_area] All points removed by remove_radius_outlier - returning 0.0 area")
        return 0.0

    if estimate_normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(radii) * 2,
                max_nn=30
            )
        )
        t3 = time.perf_counter()
        rospy.loginfo("[surface_area] estimate_normals: %.2fs", t3 - t2)

        pcd.orient_normals_consistent_tangent_plane(50)
        t4 = time.perf_counter()
        rospy.loginfo("[surface_area] orient_normals_consistent_tangent_plane: %.2fs", t4 - t3)
    else:
        t4 = time.perf_counter()

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector(radii)
    )
    t5 = time.perf_counter()
    rospy.loginfo("[surface_area] ball_pivoting (radii=%s): %.2fs (%d triangles)", radii, t5 - t4, len(mesh.triangles))
    # Save mesh for debugging
    # now = datetime.now().strftime("%Y%m%d_%H%M%S")
    # o3d.io.write_triangle_mesh(f"/root/intelijet_v2/data/log/{now}_mesh.ply",mesh)

    # Tính diện tích
    area = mesh.get_surface_area()
    return area


def surface_area_split(
    tpcd,
    distances,
    min_reached_thickness_mm: float,
    radii=(0.1, 0.15),
    estimate_normals=True
):
    """
    Mesh `tpcd` ONCE (BPA) and derive both the full surface area and the
    "reached" (thickness >= min_reached_thickness_mm) sub-area's from that
    single mesh, by classifying triangles after the fact.

    This replaces calling surface_area() twice (once on the full cloud, once
    on a thickness-filtered subset): meshing 2 disjoint point sets
    separately pays the expensive normal-estimation/BPA cost twice AND
    under-counts the true total area, because it loses the triangles that
    would have spanned the boundary between the two sets in a single mesh -
    that boundary strip simply can't be triangulated in either subset once
    the other subset's points are gone. Classifying triangles on ONE mesh
    has no such gap and pays the expensive steps only once.

    Parameters
    ----------
    tpcd : o3d.t.geometry.PointCloud
        The point cloud to mesh (its own "distances" attribute, if any, is
        NOT used - see `distances` below).
    distances : array-like, shape (N,), aligned 1:1 with tpcd's points
        Thickness deviation per point, in the SAME units/convention as
        min_reached_thickness_mm. Passed explicitly (rather than read from
        tpcd's own "distances" attribute) so the caller's own thresholding/
        cleanup of the array (e.g. zeroing noise) is guaranteed to be what
        gets classified here - relying on tpcd's attribute would silently
        diverge if the caller's numpy() copy of it isn't a view.
    min_reached_thickness_mm : float
        Threshold (same units as `distances`) above which a point counts as
        "reached" for the reached_area figure.
    radii, estimate_normals : see surface_area().

    Returns
    -------
    dict with "total_area" and "reached_area" (m²) - "reached_area" is None
    if the assumption that BPA preserves 1 mesh vertex per input point
    (checked below) doesn't hold, since the triangle-classification step
    depends on it; "total_area" is still valid in that case.
    """
    import open3d as o3d
    import copy

    tpcd = copy.deepcopy(tpcd)

    if not isinstance(tpcd, o3d.t.geometry.PointCloud):
        raise TypeError("surface_area_split needs a tensor PointCloud")

    distances = np.asarray(distances, dtype=np.float32).reshape(-1)
    if distances.shape[0] != tpcd.point.positions.shape[0]:
        raise ValueError(
            f"distances length ({distances.shape[0]}) doesn't match tpcd's point count "
            f"({tpcd.point.positions.shape[0]})"
        )
    # Stash into a scratch attribute so the tensor voxel_down_sample below
    # averages it in lockstep with positions - overwrites whatever "distances"
    # tpcd already carried, which is the point (see docstring above).
    tpcd.point["distances"] = o3d.core.Tensor(distances.reshape(-1, 1))

    voxel_size = min(radii) / 2

    t0 = time.perf_counter()
    # Tensor voxel_down_sample averages every point attribute (not just
    # positions) across merged points - "distances" survives downsampling
    # the same way avg_thickness_mm already averages it elsewhere, so the
    # reached/not-reached classification below stays meaningful post-merge.
    tpcd_down = tpcd.voxel_down_sample(voxel_size=voxel_size)
    t1 = time.perf_counter()
    rospy.loginfo("[surface_area_split] voxel_down_sample: %.2fs (%d pts)", t1 - t0, tpcd_down.point.positions.shape[0])

    distances_down = np.abs(tpcd_down.point["distances"].numpy()).reshape(-1)
    pcd = tpcd_down.to_legacy()

    cl, ind = pcd.remove_radius_outlier(nb_points=8, radius=2 * min(radii))
    pcd = pcd.select_by_index(ind)
    distances_down = distances_down[ind]
    t2 = time.perf_counter()
    rospy.loginfo("[surface_area_split] remove_radius_outlier: %.2fs (%d pts)", t2 - t1, len(pcd.points))

    if len(pcd.points) == 0:
        rospy.logwarn("[surface_area_split] All points removed by remove_radius_outlier - returning 0.0 area")
        return {"total_area": 0.0, "reached_area": 0.0}

    if estimate_normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(radii) * 2,
                max_nn=30
            )
        )
        t3 = time.perf_counter()
        rospy.loginfo("[surface_area_split] estimate_normals: %.2fs", t3 - t2)

        pcd.orient_normals_consistent_tangent_plane(50)
        t4 = time.perf_counter()
        rospy.loginfo("[surface_area_split] orient_normals_consistent_tangent_plane: %.2fs", t4 - t3)
    else:
        t4 = time.perf_counter()

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector(radii)
    )
    t5 = time.perf_counter()
    rospy.loginfo("[surface_area_split] ball_pivoting (radii=%s): %.2fs (%d triangles)", radii, t5 - t4, len(mesh.triangles))

    total_area = mesh.get_surface_area()

    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)

    if len(verts) != len(distances_down):
        # BPA is documented/assumed to emit exactly 1 mesh vertex per input
        # point (in the same order), even for points that end up with no
        # triangle attached - if that ever doesn't hold, there's no reliable
        # way to map distances_down onto mesh vertices, so don't silently
        # guess: surface a clear warning and skip reached_area rather than
        # return a number that might be wrong.
        rospy.logwarn(
            "[surface_area_split] mesh vertex count (%d) != input point count (%d) - "
            "can't classify triangles by distance, reached_area unavailable",
            len(verts), len(distances_down)
        )
        return {"total_area": total_area, "reached_area": None}

    reached_vertex = distances_down >= min_reached_thickness_mm
    # Majority vote (>=2 of 3 vertices reached) rather than requiring all 3 -
    # a triangle straddling the threshold boundary is counted on whichever
    # side most of it actually sits on.
    reached_tri_mask = reached_vertex[tris].sum(axis=1) >= 2

    v0, v1, v2 = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    tri_areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    reached_area = float(tri_areas[reached_tri_mask].sum())

    return {"total_area": total_area, "reached_area": reached_area}
