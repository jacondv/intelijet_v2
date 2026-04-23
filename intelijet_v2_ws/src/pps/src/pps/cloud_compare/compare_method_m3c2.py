import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from math import sqrt
from pps.data_converter import cloudconverter
import traceback



def map_distances_to_colors(
    distances, 
    clip_max=150,
    highlight_range=(0.02, 0.04),
    out_of_range_color=(0.678, 0.847, 0.902) #Light blue
):
    """
    Map distances to RGB colors with smooth transitions:
      - dist < highlight_range[0] → red → green gradient
      - dist in highlight_range → pure green
      - dist > highlight_range[1] → green → blue gradient
      - dist > clip_max → out_of_range_color
    """
    distances = np.abs(distances)
    colors = np.zeros((len(distances), 3))

    low, high = highlight_range

    for i, d in enumerate(distances):
        if d > clip_max:
            colors[i] = out_of_range_color

        elif d < low:
            # Gradient red (1,0,0) → green (0,1,0)
            # t = d / low if low > 0 else 0
            # colors[i] = (1 - t, t, 0)
            colors[i] = (0.5, 0, 0)

        elif d <= high:
            # Pure green
            colors[i] = (0, 1, 0)

        else:
            # Gradient green (0,1,0) → blue (0,0,1)
            colors[i] = (0, 0, 1)

    return colors

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

def _to_numpy_points(pcd):
    if isinstance(pcd, o3d.t.geometry.PointCloud):
        return np.asarray(pcd.point.positions.numpy(), dtype=np.float64)
    if isinstance(pcd, o3d.geometry.PointCloud):
        return np.asarray(pcd.points, dtype=np.float64)
    arr = np.asarray(pcd, dtype=np.float64)
    assert arr.ndim == 2 and arr.shape[1] == 3
    return arr

def compute_normals_pca(points, radius, min_neighbors=5):
    """
    Compute normals by PCA for each point in points.
    Returns normals (N,3). If neighborhood too small -> normal = [0,0,1].
    """
    tree = cKDTree(points)
    normals = np.zeros_like(points)
    for i, p in enumerate(points):
        idx = tree.query_ball_point(p, radius)
        if len(idx) < min_neighbors:
            normals[i] = np.array([0.0, 0.0, 1.0])
            continue
        neigh = points[idx]
        cov = np.cov(neigh.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        normal = eigvecs[:, np.argmin(eigvals)]
        normals[i] = normal / (np.linalg.norm(normal) + 1e-12)
    return normals

def orient_normals_towards(normals, points, target_centroid):
    """
    Flip normals so they point roughly toward target_centroid.
    """
    vec = (target_centroid - points)
    dot = np.einsum('ij,ij->i', normals, vec)
    flip_mask = dot < 0
    normals[flip_mask] *= -1
    return normals



import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

# Giả sử các helper sau đã có trong scope:
# _to_numpy_points, compute_normals_pca, orient_normals_towards, map_distances_to_colors

def compute_heatmap_m3c2_ep(source, target,
                            target_thickness=30, tolerance_thickness=10,
                            sigma_inst_mm=12,
                            cyl_radius=0.05, normal_radius=0.2,
                            core_subsample=1, min_neighbors=6,
                            orient_to='target',
                            sigma_reg_mm=None):
    """
    M3C2-like heatmap with LOD95.
    - source, target: o3d or numpy (N,3)
    - sigma_inst_mm: instrument sigma (mm)
    - sigma_reg_mm: registration error (mm). If None, use sigma_inst_mm.
    - cyl_radius, normal_radius: meters
    - core_subsample: take every k-th source point as corepoint
    Returns: (result_cloud, distances_mm)
      result_cloud.point fields:
        - positions, colors, distances (mm), distance_sigma (mm),
        - LOD95 (mm), is_significant (uint8), num_samples (int)
    """
    src_np = _to_numpy_points(source)
    tgt_np = _to_numpy_points(target)

    if sigma_reg_mm is None:
        sigma_reg_mm = float(sigma_inst_mm)

    # choose corepoints
    corepts = src_np[::core_subsample].copy()
    if corepts.shape[0] == 0:
        raise ValueError("No corepoints selected (core_subsample too large).")

    # normals
    normals_all = compute_normals_pca(src_np, normal_radius, min_neighbors=min_neighbors)
    normals = normals_all[::core_subsample]

    if orient_to == 'target':
        tgt_centroid = np.mean(tgt_np, axis=0)
        normals = orient_normals_towards(normals, corepts, tgt_centroid)

    tgt_tree = cKDTree(tgt_np)
    src_tree = cKDTree(src_np)

    M = corepts.shape[0]
    distances_mm = np.full((M, 1), np.nan, dtype=np.float32)
    sigma_mm = np.full((M, 1), np.nan, dtype=np.float32)        # previous combined sigma
    lod95_arr = np.full((M, 1), np.nan, dtype=np.float32)
    significant_arr = np.zeros((M, 1), dtype=np.uint8)
    num_samples = np.zeros((M,), dtype=int)

    z95 = 1.96

    for i, (p, n) in enumerate(zip(corepts, normals)):
        # target candidates: query larger ball then filter by perpendicular distance
        idx_t = tgt_tree.query_ball_point(p, cyl_radius)
        if len(idx_t) == 0:
            continue
        pts_t = tgt_np[idx_t]
        v_t = pts_t - p
        proj_t = np.dot(v_t, n)            # (k_t,)
        perp_t = v_t - np.outer(proj_t, n)
        perp_dist_t = np.linalg.norm(perp_t, axis=1)
        mask_t = perp_dist_t <= cyl_radius + 1e-12
        sel_proj_t = proj_t[mask_t]
        n_t = sel_proj_t.size
        num_samples[i] = n_t
        if n_t == 0:
            continue

        # source patch around corepoint p (dùng normal_radius hoặc cyl_radius tùy ý)
        idx_s = src_tree.query_ball_point(p, normal_radius)
        if len(idx_s) == 0:
            # fallback: dùng một neighbor gần nhất
            _, idx_nn = src_tree.query(p, k=1)
            idx_s = [int(idx_nn)]
        pts_s = src_np[idx_s]
        v_s = pts_s - p
        proj_s = np.dot(v_s, n)
        n_s = proj_s.size

        # median projections
        median_proj_t = np.median(sel_proj_t)
        median_proj_s = np.median(proj_s)

        # MAD -> std approximation
        mad_t = np.median(np.abs(sel_proj_t - median_proj_t))
        std_t = 1.4826 * mad_t
        if std_t == 0 and n_t > 1:
            std_t = np.std(sel_proj_t)

        mad_s = np.median(np.abs(proj_s - median_proj_s))
        std_s = 1.4826 * mad_s
        if std_s == 0 and n_s > 1:
            std_s = np.std(proj_s)

        # convert to mm
        dist_mm = float(median_proj_t * 1000.0)
        std_t_mm = std_t * 1000.0
        std_s_mm = std_s * 1000.0

        # uncertainty of median approx: 1.253 * sigma / sqrt(n)
        u_t = 1.253 * (std_t_mm / np.sqrt(max(1, n_t)))
        u_s = 1.253 * (std_s_mm / np.sqrt(max(1, n_s)))

        # combined measurement sigma (kept for compatibility)
        sigma_measure_mm = u_t
        sigma_total = np.sqrt(sigma_measure_mm**2 + (sigma_inst_mm**2))

        # LOD95
        lod95 = z95 * np.sqrt(u_s**2 + u_t**2 + (sigma_reg_mm**2))

        # significance
        is_sig = 1 if abs(dist_mm) > lod95 else 0

        # store
        distances_mm[i, 0] = dist_mm
        sigma_mm[i, 0] = float(sigma_total)
        lod95_arr[i, 0] = float(lod95)
        significant_arr[i, 0] = np.uint8(is_sig)

    # color mapping (reuse your map_distances_to_colors)
    _min = (target_thickness - tolerance_thickness)
    _max = target_thickness + tolerance_thickness
    colors = map_distances_to_colors(distances_mm, highlight_range=[_min, _max], clip_max=0.15)

    # build result Open3D tensor pointcloud (do not modify source)
    result = o3d.t.geometry.PointCloud()
    result.point["positions"] = o3d.core.Tensor(corepts.astype(np.float32))
    result.point["colors"] = o3d.core.Tensor(colors.astype(np.float32))
    result.point["distances"] = o3d.core.Tensor(distances_mm.astype(np.float32),
                                                dtype=o3d.core.Dtype.Float32)
    # result.point["distance_sigma"] = o3d.core.Tensor(sigma_mm.astype(np.float32),
    #                                                  dtype=o3d.core.Dtype.Float32)
    # result.point["LOD95"] = o3d.core.Tensor(lod95_arr.astype(np.float32),
    #                                         dtype=o3d.core.Dtype.Float32)
    # result.point["is_significant"] = o3d.core.Tensor(significant_arr.astype(np.uint8))
    # result.point["num_samples"] = o3d.core.Tensor(num_samples.reshape(-1,1).astype(np.int32))

    return result, distances_mm