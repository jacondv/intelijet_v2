#!/usr/bin/env python3
"""Deprecated: this module used to be a 1688-line, ~30-function grab-bag.
It has been split into pps/cloud_utils/ by concern - import from there in
new code:

  pps.cloud_utils.geometry     - crop_pointcloud_by_box, smooth_cloud, surface_area
  pps.cloud_utils.comparison   - compute_heatmap_to_plane, run_compare, check_transform,
                                  filter_pcd_by_distance, keep_largest_cluster
  pps.cloud_utils.coloring     - assign_colors, map_distances_to_colors
  pps.cloud_utils.io_utils     - load_ply, notify_one

This shim only re-exports the functions that still have real callers
in the codebase (verified via repo-wide grep, see
docs/plan/phase_06_pps_cleanup.md) so existing `from pps.helper import ...`
call sites keep working unchanged. Everything else that used to live here
(compute_distance_histogram, crop, process_cloud,
remove_points_outside_radius, remove_points_inside_box, run_compare_m3c2,
assign_colors_by_threshold, color_voxel_majority,
convert_pointcloud2_to_o3d, convert_open3d_to_pointcloud2_v2,
convert_open3d_to_pointcloud2_with_diff, convert_msg_to_image,
convert_pointcloud2_to_pointcloud, remove_point, remove_ground_with_pca,
detect_boundary_pca, remove_boundary_region, cloud_downsample,
remove_small_clusters) had zero callers anywhere in the repo and was
deleted rather than moved. convert_open3d_to_pointcloud2 had exactly one
caller and was merged into CloudConverter.legacy_o3d_to_pointcloud2
instead of staying a free function - see data_converter.py.

This shim will be removed once callers are migrated to import from
pps.cloud_utils.* directly.
"""
from pps.cloud_utils.geometry import crop_pointcloud_by_box, smooth_cloud, surface_area
from pps.cloud_utils.comparison import (
    compute_heatmap_to_plane,
    run_compare,
    check_transform,
    filter_pcd_by_distance,
    keep_largest_cluster,
)
from pps.cloud_utils.coloring import assign_colors, map_distances_to_colors
from pps.cloud_utils.io_utils import load_ply, notify_one

__all__ = [
    "crop_pointcloud_by_box",
    "smooth_cloud",
    "surface_area",
    "compute_heatmap_to_plane",
    "run_compare",
    "check_transform",
    "filter_pcd_by_distance",
    "keep_largest_cluster",
    "assign_colors",
    "map_distances_to_colors",
    "load_ply",
    "notify_one",
]
