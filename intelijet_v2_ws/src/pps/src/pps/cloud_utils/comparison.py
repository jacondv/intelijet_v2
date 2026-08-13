# pps/cloud_utils/comparison.py
"""Point-cloud comparison: distance/heatmap computation, alignment sanity
check, and cluster/distance-based filtering of compare results.

Split out of the old pps/helper.py god-module (see docs/plan/phase_06_pps_cleanup.md).
"""
import rospy
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

from pps.data_converter import cloudconverter
from pps.cloud_utils.coloring import map_distances_to_colors


def run_compare(source, target,k=6):
    # Tính trước normal cho target
    # start_time = time.time()
    import open3d as o3d
    rospy.loginfo("Compare prescan vs postscan...")
    source = cloudconverter.tensor_to_o3d_legacy(source)
    target = cloudconverter.tensor_to_o3d_legacy(target)

    # A degenerate cloud here (empty scan, or a target crop with fewer
    # points than the neighbor count normal estimation needs) would
    # otherwise fail deep inside estimate_normals/cKDTree with a cryptic
    # error - surface it clearly and early instead.
    if len(source.points) == 0:
        raise ValueError("run_compare: source cloud is empty")
    if len(target.points) < k:
        raise ValueError(
            f"run_compare: target cloud has {len(target.points)} points, "
            f"fewer than k={k} needed for normal estimation"
        )

    def __orient_normals_inward(pcd, sensor_pos=np.array([0, 0, 0], dtype=np.float32)):
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)

        vec = sensor_pos - points

        dot = np.sum(normals * vec, axis=1)

        normals[dot < 0] *= -1

        pcd.normals = o3d.utility.Vector3dVector(normals)

        return pcd

    target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k)
    )
    # orient_normals_consistent_tangent_plane() used to run here - dropped,
    # ~10s of run_compare's runtime on real scan sizes, and its result was
    # immediately overwritten anyway: __orient_normals_inward() below
    # re-derives every normal's sign per-point from the sensor position,
    # independent of whatever direction the tangent-plane propagation had
    # settled on.
    target = __orient_normals_inward(target)

    target_points = np.asarray(target.points)
    target_normals = np.asarray(target.normals)
    target_tree = cKDTree(target_points)
    source_points = np.asarray(source.points)

    # Với moi diem trong source, tim diem gan nhat trong target.
    # (query's own distance output unused - point-to-plane distance below
    # uses the neighbor's normal instead)
    _, indices = target_tree.query(source_points, k=1)

    centroids = target_points[indices] 
    normals   = target_normals[indices] 
    diff = source_points - centroids 
    distances = np.sum(diff * normals, axis=1)  # (N,)
    distances = distances.astype(np.float32)
    
    source = cloudconverter.o3d_legacy_to_tensor(source)
    distances_mm = np.round(distances * 1000).astype(np.float32)
    distances_mm = distances_mm.reshape(-1, 1)

    n_points = source.point["positions"]
    if len(distances) != len(n_points):
        raise ValueError(f"Number of element distances ({len(distances)}) does not match number of point clouds ({n_points})")

    source.point["distances"] = o3d.core.Tensor(distances_mm, dtype=o3d.core.Dtype.Float32)

    return source, distances


def check_transform(T,
                          max_rot_deg=(10,10,10),
                          max_trans=0.5,
                          verbose=False):

    '''Kiểm tra ma trận transform T có nằm trong ngưỡng cho phép về rotation và translation hay không.
    Args:
    - T: np.ndarray shape (4,4), ma trận transform
    - max_rot_deg: tuple (max_roll, max_pitch, max_yaw) ngưỡng rotation theo độ
    - max_trans: float, ngưỡng translation theo mét
    - verbose: bool, có in chi tiết ra console hay không
    Returns:
    - ok: bool, True nếu T nằm trong ngưỡng, False nếu vượt ngưỡng
    '''
    # --- Rotation ---
    rot = R.from_matrix(T[:3, :3])
    roll, pitch, yaw = rot.as_euler('xyz', degrees=True)

    # --- Translation ---
    t = T[:3, 3]
    trans_norm = np.linalg.norm(t)

    rot_ok = (
        abs(roll)  <= max_rot_deg[0] and
        abs(pitch) <= max_rot_deg[1] and
        abs(yaw)   <= max_rot_deg[2]
    )

    trans_ok = trans_norm <= max_trans

    ok = rot_ok and trans_ok

    # --- Print / Log ---
    if verbose:
        print("---- Alignment Check ----")
        print(f"Rotation [deg]  roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        print(f"Translation [m] x={t[0]:.3f}, y={t[1]:.3f}, z={t[2]:.3f}")
        print(f"Translation norm = {trans_norm:.3f} m")

        if not rot_ok:
            print("❌ Rotation exceeds threshold:", max_rot_deg)
        if not trans_ok:
            print("❌ Translation exceeds threshold:", max_trans)

        print("Translation:", "✅ OK" if ok else "❌ FAILED")
        print("-------------------------")

    return ok


def filter_pcd_by_distance(pcd: 'o3d.t.geometry.PointCloud',
                           d_min: float,
                           d_max: float) -> 'o3d.t.geometry.PointCloud':
    """
    Trích xuất point cloud theo trường 'distances'
    
    Parameters
    ----------
    pcd : o3d.t.geometry.PointCloud
        Point cloud tensor đầu vào (phải có field 'distances')
    d_min : float
        Ngưỡng nhỏ nhất
    d_max : float
        Ngưỡng lớn nhất
    
    Returns
    -------
    pcd_out : o3d.t.geometry.PointCloud
        Cloud đã được lọc, giữ nguyên các field khác
    """
    import open3d as o3d
    import copy
    
    if not isinstance(pcd, o3d.t.geometry.PointCloud):
        raise TypeError("Input must be o3d.t.geometry.PointCloud")

    if "distances" not in pcd.point:
        raise KeyError("PointCloud does not contain 'distances' field")

    # Clone để tránh side-effect
    pcd_out = copy.deepcopy(pcd)

    # mask có shape (N, 1) nhưng TensorMap.__getitem__() CHỈ chấp nhận mask dạng (N,)
    distances = pcd_out.point["distances"][:, 0].abs()
    distances_abs = distances.abs()

    # Boolean mask
    mask = (distances_abs >= d_min) & (distances_abs <= d_max)

    # Áp mask cho toàn bộ point attributes
    pcd_out = pcd_out.select_by_mask(mask)
    return pcd_out


def keep_largest_cluster(
    pcd,
    eps=0.1,
    min_points=30,
    max_cluster_size=100
    ):
    """
    Giữ lại:
    - Các cluster có số điểm <= max_cluster_size
    - Cluster lớn nhất (dù lớn hơn max_cluster_size)

    Hỗ trợ cả:
    - o3d.t.geometry.PointCloud (tensor)
    - o3d.geometry.PointCloud (legacy)
    """

    import numpy as np
    import open3d as o3d

    # --- Detect type ---
    is_tensor = isinstance(pcd, o3d.t.geometry.PointCloud)

    # --- Convert sang legacy nếu cần ---
    legacy = pcd.to_legacy() if is_tensor else pcd

    # --- DBSCAN ---
    labels = np.array(
        legacy.cluster_dbscan(eps=eps, min_points=min_points)
    )

    # --- Lọc cluster hợp lệ ---
    valid_labels = labels[labels >= 0]
    if len(valid_labels) == 0:
        return pcd.select_by_index([])

    counts = np.bincount(valid_labels)

    # --- Chọn cluster lớn nhất ---
    largest_cluster = np.argmax(counts)

    # --- Các cluster nhỏ hơn ngưỡng ---
    small_clusters = np.where(counts <= max_cluster_size)[0]

    # --- Giữ lại cluster nhỏ + cluster lớn nhất ---
    keep_clusters = np.unique(
        np.concatenate([small_clusters, [largest_cluster]])
    )

    keep_indices = np.where(np.isin(labels, keep_clusters))[0]

    # --- Return đúng kiểu input ---
    if is_tensor:
        return pcd.select_by_index(keep_indices.tolist())
    else:
        return pcd.select_by_index(keep_indices)
