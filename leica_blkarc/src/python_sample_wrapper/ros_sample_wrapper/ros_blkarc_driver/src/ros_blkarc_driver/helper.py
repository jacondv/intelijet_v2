#!/usr/bin/env python3
import rospy
import std_msgs.msg
import open3d as o3d
import numpy as np
import ros_numpy
import matplotlib.pyplot as plt
from collections import defaultdict
from datetime import datetime
from cv_bridge import CvBridge
import cv2

import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Không dùng GUI backend
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2


def compute_distance_histogram(
    distances,
    bin_size=0.01,
    filename="histogram.png",
    figsize=(10, 4),
    dpi=150,
    threshold=0.035,
    color="#1f77b4",
    title="Phân bố khoảng cách giữa các điểm",
    xlabel="Khoảng cách (m)",
    ylabel="Số lượng điểm"
):
    """
    Lưu biểu đồ histogram khoảng cách theo định dạng ảnh.

    Parameters:
        distances (list or np.ndarray): Danh sách khoảng cách.
        bin_size (float): Độ rộng mỗi bin (m).
        filename (str): Tên file để lưu (hỗ trợ .png, .pdf, .svg...).
        figsize (tuple): Kích thước hình (inch).
        dpi (int): Độ phân giải ảnh.
        color (str): Màu cột histogram.
        title, xlabel, ylabel: Tiêu đề và nhãn trục.
    """
    distances = np.array(distances)
    distances = distances[distances > threshold]

    if distances.size == 0:
        raise ValueError("Distance list is empty.")

    max_dist = distances.max()
    num_bins = int(np.ceil(max_dist / bin_size))
    bins = np.linspace(0, num_bins * bin_size, num_bins + 1)
    hist, bin_edges = np.histogram(distances, bins=bins)

    # Thiết lập style chuyên nghiệp
    # sns.set_theme(style="whitegrid", palette="muted")
    # plt.style.use('seaborn-darkgrid')
    plt.figure(figsize=figsize, dpi=dpi)
    plt.bar(bin_edges[:-1], hist, width=bin_size, align='edge',
            edgecolor='black', color=color)
    plt.xlim(distances.min(), min(distances.max(), 0.1))
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()

    # Lưu ảnh và giải phóng bộ nhớ
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{filename}_{time_str}.svg"
    plt.savefig(filename, format=filename.split('.')[-1])
    plt.close()

import open3d as o3d
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


def crop(pcd, xlim, ylim, zlim):
    """
    Cắt point cloud theo giới hạn không gian của x, y, z.

    Tham số:
        - pcd: Open3D point cloud
        - xlim: tuple (xmin, xmax)
        - ylim: tuple (ymin, ymax)
        - zlim: tuple (zmin, zmax)
    
    Trả về:
        - point cloud đã được cắt
    """
    points = np.asarray(pcd.points)
    
    mask = (
        (points[:, 0] >= xlim[0]) & (points[:, 0] <= xlim[1]) &
        (points[:, 1] >= ylim[0]) & (points[:, 1] <= ylim[1]) &
        (points[:, 2] >= zlim[0]) & (points[:, 2] <= zlim[1])
    )

    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(points[mask])

    # Giữ lại màu (nếu có)
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        cropped_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    return cropped_pcd

def process_cloud(pcd, voxel_size=0.015):
    pcd_croped = crop(pcd,[-4,4], [0,4],[-0.5,3])
    pcd_croped, _ = pcd_croped.remove_statistical_outlier(nb_neighbors=5, std_ratio=1)
    pcd_croped = pcd_croped.voxel_down_sample(voxel_size=voxel_size)
    return pcd_croped
    

def compute_heatmap_to_plane(source, target, k=10):
    # Tính trước normal cho target
    target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k)
    )

    target_points = np.asarray(target.points)
    target_normals = np.asarray(target.normals)
    target_tree = o3d.geometry.KDTreeFlann(target)

    source_points = np.asarray(source.points)

    distances = []

    for pt in source_points:
        # Tìm điểm gần nhất trong target
        [_, idx, _] = target_tree.search_knn_vector_3d(pt, 1)
        nearest_idx = idx[0]

        centroid = target_points[nearest_idx]
        normal = target_normals[nearest_idx]

        # Khoảng cách point-to-plane
        dist = np.abs(np.dot(pt - centroid, normal))
        distances.append(dist)

    distances = np.array(distances, dtype=np.float32)

    # Scale và tô màu heatmap
    distances_log = np.log1p(distances)
    distances_normalized = (distances_log - distances_log.min()) / (distances_log.ptp() + 1e-9)

    cmap = plt.get_cmap("jet")
    colors = cmap(distances_normalized)[:, :3]

    source.colors = o3d.utility.Vector3dVector(colors)
    return source, distances

# def compute_heatmap_to_plane(source, target, k=10):
#     source_points = np.asarray(source.points)
#     target_points = np.asarray(target.points)

#     source_tree = o3d.geometry.KDTreeFlann(source)
#     target_tree = o3d.geometry.KDTreeFlann(target)

#     errors = []

#     for pt in source_points:
#         # 1. Mặt phẳng cục bộ quanh pt (source)
#         [_, idx_s, _] = source_tree.search_knn_vector_3d(pt, k)
#         neighbors_s = source_points[idx_s]
#         centroid_s = neighbors_s.mean(axis=0)
#         cov_s = np.cov((neighbors_s - centroid_s).T)
#         _, _, vh_s = np.linalg.svd(cov_s)
#         normal_s = vh_s[-1]

#         # 2. Tìm điểm gần nhất trong target và tạo mặt phẳng tương ứng
#         [_, idx_t, _] = target_tree.search_knn_vector_3d(pt, k)
#         neighbors_t = target_points[idx_t]
#         centroid_t = neighbors_t.mean(axis=0)
#         cov_t = np.cov((neighbors_t - centroid_t).T)
#         _, _, vh_t = np.linalg.svd(cov_t)
#         normal_t = vh_t[-1]

#         # 3. Khoảng cách giữa hai mặt phẳng
#         plane_dist = np.abs(np.dot(centroid_s - centroid_t, normal_t))

#         # 4. Góc giữa hai normal
#         # cos_theta = np.clip(np.dot(normal_s, normal_t), -1.0, 1.0)
#         # angle_rad = np.arccos(np.abs(cos_theta))  # Lấy abs để bỏ định hướng
#         # angle_deg = np.degrees(angle_rad)

#         # 5. Tổng hợp lỗi
#         total_error = plane_dist #+ angle_deg / 90.0  # Góc max 90°, chia chuẩn hóa
#         errors.append(total_error)

#     errors = np.array(errors, dtype=np.float32)

#     # Normalize để tạo heatmap
#     errors_log = np.log1p(errors)
#     errors_normalized = (errors_log - errors_log.min()) / (errors_log.ptp() + 1e-9)

#     cmap = plt.get_cmap("jet")
#     colors = cmap(errors_normalized)[:, :3]

#     source.colors = o3d.utility.Vector3dVector(colors)
#     return source, errors


def assign_colors_by_threshold(pcd, distances, threshold=[0.03, 0.04]):
    """
    Gán màu cho point cloud dựa trên khoảng cách và ngưỡng.
    - Xanh: lệch nhiều
    - Vàng: lệch nhẹ
    - Đỏ: Chính xác
    """

    colors = []
    for d in distances:
        color = [1, 0, 0] #Đỏ
        
        if d > threshold[0]:
            color = [0, 1, 0]      # Xanh lá
        
        if d > threshold[1]:
            color = [1, 1, 0]      # vàng     
        
        colors.append(color)     


    colored_pcd = o3d.geometry.PointCloud()
    colored_pcd.points = pcd.points
    colored_pcd.colors = o3d.utility.Vector3dVector(colors)
    return colored_pcd
def color_voxel_majority(pcd, voxel_size=0.1):
    """
    Set màu voxel theo đa số. 
    """
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    red = np.array([1.0, 0.0, 0.0])
    green = np.array([0.0, 1.0, 0.0])  

    # Kiểm tra điểm đỏ (dựa trên khoảng cách màu với red)
    is_red = np.all(np.isclose(colors, red, atol=0.1), axis=1)

    # Lấy chỉ số voxel của từng điểm
    voxel_indices = np.floor(points / voxel_size).astype(int)

    voxel_dict = defaultdict(list)
    for i, voxel_idx in enumerate(map(tuple, voxel_indices)):
        '''
        enumerate(map(tuple, voxel_indices) --> trả về tọa độ voxel dạng dict
        voxel_dict sẽ chứa dah sách index các điểm cùng voxel
        
        {
          (x1, y1, z1): [0, 3, 5, 7],   # các điểm nằm trong voxel (x1, y1, z1)
          (x2, y2, z2): [1, 2],
          ...
        }
        
        '''
        voxel_dict[voxel_idx].append(i)

    new_colors = colors.copy()

    for voxel_key, idx_list in voxel_dict.items():
        red_count = np.sum(is_red[idx_list])
        total = len(idx_list)
        if red_count > total / 2:
            # Đa số đỏ -> gán đỏ toàn bộ điểm voxel
            for idx in idx_list:
                new_colors[idx] = red

        else:
            
            # Ngược lại gán xanh (hoặc giữ nguyên màu hiện tại)
            for idx in idx_list:
                new_colors[idx] = green

    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    return pcd

def convert_pointcloud2_to_o3d(msg):
    """Convert a ROS PointCloud2 message into an Open3D PointCloud."""
    if not isinstance(msg, PointCloud2):
        rospy.logerr("Input message is not of type PointCloud2.")
        return None

    # Convert to structured NumPy array
    cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    # Extract XYZ
    xyz = ros_numpy.point_cloud2.get_xyz_points(cloud_arr, remove_nans=True)

    # Create Open3D PointCloud
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(xyz)

    # Handle RGB if available
    if 'rgb' in cloud_arr.dtype.names:
        # Extract RGB field (float32 packed)
        rgb_packed = cloud_arr['rgb']
        rgb_uint8 = np.zeros((rgb_packed.shape[0], 3), dtype=np.uint8)
        rgb_view = rgb_packed.view(np.uint32)  # Treat float32 as uint32 to extract colors

        rgb_uint8[:, 0] = (rgb_view >> 16) & 255  # R
        rgb_uint8[:, 1] = (rgb_view >> 8) & 255   # G
        rgb_uint8[:, 2] = rgb_view & 255          # B

        # Normalize to [0, 1]
        cloud_o3d.colors = o3d.utility.Vector3dVector(rgb_uint8.astype(np.float32) / 255.0)

    return cloud_o3d


def convert_open3d_to_pointcloud2(o3d_cloud, frame_id="base_link"):
    """
    Chuyển đổi Open3D point cloud sang ROS PointCloud2.
    """

    # o3d.io.write_point_cloud("cloud_output.ply", o3d_cloud)
    if not isinstance(o3d_cloud, o3d.geometry.PointCloud):
        rospy.logerr("Input is not an Open3D PointCloud.")
        return None
    
    if not o3d_cloud.has_points():
        rospy.logerr("Open3D PointCloud has no points.")
        return None
    
    rospy.loginfo("Converting Open3D point cloud to PointCloud2 format ply")
    points = np.asarray(o3d_cloud.points)
    colors = np.asarray(o3d_cloud.colors)

    if colors.shape[0] == points.shape[0] and colors.shape[1] == 3:
        rospy.loginfo("Converting Open3D has colors ")
        colors = (colors * 255).astype(np.uint8)

    else:
        rospy.loginfo("Converting Open3D has no colors ")
        # Adding red color to point cloud
        colors = np.tile(np.array([255, 0, 0], dtype=np.uint8), (points.shape[0], 1))

    rgb_packed = ((colors[:, 0].astype(np.uint32) << 16) |
            (colors[:, 1].astype(np.uint32) << 8) |
            colors[:, 2].astype(np.uint32))
    
    rgb_float = rgb_packed.view(np.float32)

    data = np.zeros(points.shape[0], dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('rgb', np.float32)
    ])
    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    data['rgb'] = rgb_float

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return ros_numpy.point_cloud2.array_to_pointcloud2(data, frame_id=header.frame_id, stamp=header.stamp)

def convert_open3d_to_pointcloud2_with_diff(o3d_cloud, diff_array=None, frame_id="base_link"):
    """
    Chuyển đổi Open3D point cloud sang ROS PointCloud2, thêm field 'diff' nếu có.
    
    Parameters:
        o3d_cloud (open3d.geometry.PointCloud): Cloud đầu vào
        diff_array (np.ndarray or None): (N,) float32, giá trị diff tương ứng với mỗi điểm (tùy chọn)
        frame_id (str): frame_id cho message ROS
    
    Returns:
        sensor_msgs/PointCloud2
    """

    points = np.asarray(o3d_cloud.points)
    num_points = points.shape[0]

    # Kiểm tra và xử lý màu
    has_colors = o3d_cloud.has_colors()
    if has_colors:
        colors = (np.asarray(o3d_cloud.colors) * 255).astype(np.uint8)
        rgb_packed = ((colors[:, 0].astype(np.uint32) << 16) |
                      (colors[:, 1].astype(np.uint32) << 8) |
                      colors[:, 2].astype(np.uint32))
        rgb_float = rgb_packed.view(np.float32)

    # Xác định dtype
    fields = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
    if has_colors:
        fields.append(('rgb', np.float32))
    
    if diff_array is not None:
        assert len(diff_array) == num_points, f"diff_array phải cùng số điểm với point cloud"
        fields.append(('diff', np.float32))

    # Tạo array
    data = np.zeros(num_points, dtype=fields)
    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    if has_colors:
        data['rgb'] = rgb_float
    if diff_array is not None:
        data['diff'] = diff_array.astype(np.float32)

    # Tạo header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Convert sang PointCloud2
    return ros_numpy.point_cloud2.array_to_pointcloud2(data, frame_id=header.frame_id, stamp=header.stamp)

import numpy as np
from scipy.interpolate import UnivariateSpline

def smooth_laser_scan_message(points, window_size=50, overlap=0.5, spline_order=3, multiplier=1.0):
    """
    Làm mượt chuỗi pointcloud 2D bằng cách chia thành nhiều đoạn nhỏ và áp dụng spline từng đoạn.

    Args:
        points (ndarray): Mảng N×2 gồm các điểm (x, y).
        window_size (int): Số điểm trong mỗi đoạn con.
        overlap (float): Phần trăm chồng lắp giữa các đoạn (0.0–0.99).
        spline_order (int): Bậc của spline (thường là 2 hoặc 3).
        multiplier (float): Điều chỉnh độ mượt spline.

    Returns:
        ndarray: Mảng điểm sau khi đã làm mượt từng đoạn và ghép lại.
    """
    points = np.asarray(points)
    N = len(points)
    step = max(1, int(window_size * (1 - overlap)))

    smoothed_points = []
    for start in range(0, N - window_size + 1, step):
        segment = points[start:start + window_size]
        x = segment[:, 0]
        y = segment[:, 1]

        # Arc length
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        # Ước lượng độ nhiễu


        if np.max(distances) > np.mean(distances) + 1 * np.std(distances):
            segment_smooth = np.stack((x, y), axis=1)
            segment_smooth = segment_smooth[int(window_size * overlap):]
        else:
            dy = np.diff(y)
            estimated_std = np.std(dy) / np.sqrt(2)
            arc_lengths = np.concatenate([[0], np.cumsum(distances)])

            estimated_var = estimated_std ** 2
            s = len(segment) * estimated_var * multiplier

            spline_x = UnivariateSpline(arc_lengths, x, k=spline_order, s=s)
            spline_y = UnivariateSpline(arc_lengths, y, k=spline_order, s=s)

            arc_uniform = np.linspace(0, arc_lengths[-1], window_size)
            x_smooth = spline_x(arc_uniform)
            y_smooth = spline_y(arc_uniform)
            segment_smooth = np.stack((x_smooth, y_smooth), axis=1)

            # Tránh trùng điểm ở vùng chồng lắp
            if smoothed_points:
                segment_smooth = segment_smooth[int(window_size * overlap):]
                
        smoothed_points.append(segment_smooth)

    return np.vstack(smoothed_points)


def convert_msg_to_image(msg):
    try:
        bridge = CvBridge()

        if msg._type == "sensor_msgs/Image":
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        elif msg._type == "sensor_msgs/CompressedImage":
            np_arr = np.frombuffer(msg.data, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        elif isinstance(msg, bytes):
            np_arr = np.frombuffer(msg, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        elif isinstance(msg, np.ndarray):
            return msg

        else:
            rospy.logwarn("Unsupported image format.")
            return None

    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return None
