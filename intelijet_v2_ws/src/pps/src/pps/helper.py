#!/usr/bin/env python3
import rospy
import std_msgs.msg
# import open3d as o3d
import numpy as np
import ros_numpy
from collections import defaultdict
from datetime import datetime
from cv_bridge import CvBridge
import cv2
import matplotlib
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
matplotlib.use('Agg')  # Không dùng GUI backend
from sensor_msgs.msg import PointCloud2, PointCloud, PointField

from pps.data_converter import cloudconverter


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

    import open3d as o3d

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
    

def compute_heatmap_to_plane(source, target, k=6,target_thickness=0.03, tolerance_thickness=0.01):
    # Tính trước normal cho target
    # start_time = time.time()
    import open3d as o3d

    source = cloudconverter.tensor_to_o3d_legacy(source)
    target = cloudconverter.tensor_to_o3d_legacy(target)

    target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k)
    )
    target.orient_normals_consistent_tangent_plane(k=3*k)

    target_points = np.asarray(target.points)
    target_normals = np.asarray(target.normals)
    target_tree = cKDTree(target_points)
    source_points = np.asarray(source.points)

    distances = []

    distances_nn, indices = target_tree.query(source_points, k=1) # We don't need distances_nn here because we compute point-to-plane distance

    centroids = target_points[indices] 
    normals   = target_normals[indices] 
    diff = source_points - centroids 
    distances = np.sum(diff * normals, axis=1)  # (N,)
    distances = distances.astype(np.float32)
    
    _min = (target_thickness - tolerance_thickness)    
    _max = target_thickness + tolerance_thickness
    colors = map_distances_to_colors(distances,highlight_range=[_min,_max],clip_max=0.15)

    source.colors = o3d.utility.Vector3dVector(colors)

    source = cloudconverter.o3d_legacy_to_tensor(source)
    distances_mm = np.round(distances * 1000).astype(np.float32)
    distances_mm = distances_mm.reshape(-1, 1)

    n_points = source.point["positions"]
    if len(distances) != len(n_points):
        raise ValueError(f"Number of element distances ({len(distances)}) does not match number of point clouds ({n_points})")

    source.point["distances"] = o3d.core.Tensor(distances_mm, dtype=o3d.core.Dtype.Float32)

    return source, distances




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



# def compute_heatmap_to_plane_old_version(source, target, k=10):

#     import open3d as o3d

#     # Tính trước normal cho target
#     target.estimate_normals(
#         search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k)
#     )

#     target_points = np.asarray(target.points)
#     target_normals = np.asarray(target.normals)
#     target_tree = o3d.geometry.KDTreeFlann(target)

#     source_points = np.asarray(source.points)

#     distances = []

#     for pt in source_points:
#         # Tìm điểm gần nhất trong target
#         [_, idx, _] = target_tree.search_knn_vector_3d(pt, 1)
#         nearest_idx = idx[0]

#         centroid = target_points[nearest_idx]
#         normal = target_normals[nearest_idx]

#         # Khoảng cách point-to-plane
#         dist = np.abs(np.dot(pt - centroid, normal))
#         distances.append(dist)

#     distances = np.array(distances, dtype=np.float32)

#     # Scale và tô màu heatmap
#     distances_log = np.log1p(distances)
#     distances_normalized = (distances_log - distances_log.min()) / (distances_log.ptp() + 1e-9)

#     cmap = plt.get_cmap("jet")
#     colors = cmap(distances_normalized)[:, :3]

#     source.colors = o3d.utility.Vector3dVector(colors)
#     return source, distances


def assign_colors_by_threshold(pcd, distances, threshold=[0.03, 0.04]):
    """
    Gán màu cho point cloud dựa trên khoảng cách và ngưỡng.
    - Xanh: lệch nhiều
    - Vàng: lệch nhẹ
    - Đỏ: Chính xác
    """
    import open3d as o3d

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
    import open3d as o3d

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

    import open3d as o3d

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


def convert_open3d_to_pointcloud2(o3d_cloud, frame_id="base_link",rgb=[255,0,0]):
    """
    Chuyển đổi Open3D point cloud sang ROS PointCloud2.
    """
    import open3d as o3d 
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
        colors = np.tile(np.array(rgb, dtype=np.uint8), (points.shape[0], 1))

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
    msg = ros_numpy.point_cloud2.array_to_pointcloud2(data, frame_id=header.frame_id, stamp=header.stamp)
    msg.is_bigendian = False  # đảm bảo đúng cho ROS chạy trên x86
    return msg 



def convert_open3d_to_pointcloud2_v2(o3d_cloud, frame_id="base_link", default_rgb=(255, 0, 0)):
    """
    Convert Open3D PointCloud -> ROS PointCloud2
    - Giữ nguyên màu nếu có
    - Tự thêm màu mặc định nếu không có
    - Tự động add các field custom như distance_mm, intensity...
    """

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    points = np.asarray(o3d_cloud.points)
    has_color = hasattr(o3d_cloud, "colors") and len(o3d_cloud.colors) > 0
    rgb = np.array(default_rgb, dtype=np.uint8)

    if has_color:
        colors = np.asarray(o3d_cloud.colors)
        if colors.shape[0] == points.shape[0] and colors.shape[1] == 3:
            rospy.loginfo("Converting Open3D cloud: has colors.")
            colors = (colors * 255).astype(np.uint8)
        else:
            rospy.loginfo("Color size mismatch, applying default color.")
            colors = np.tile(rgb, (points.shape[0], 1))
    else:
        rospy.loginfo("Converting Open3D cloud: no colors, applying default color.")
        colors = np.tile(rgb, (points.shape[0], 1))

    rgb_packed = (
        (colors[:, 0].astype(np.uint32) << 16)
        | (colors[:, 1].astype(np.uint32) << 8)
        | (colors[:, 2].astype(np.uint32))
    )
    rgb_float = rgb_packed.view(np.float32)

    # --- Base fields ---
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.FLOAT32, 1),
    ]
    offset = 16  # bytes used so far

    # --- Custom fields (Open3D >= 0.17) ---
    extra_data = {}
    if hasattr(o3d_cloud, "point"):
        for key in o3d_cloud.point.keys():
            if key in ["positions", "points", "normals", "colors"]:
                continue
            arr = np.asarray(o3d_cloud.point[key])
            if arr.ndim == 1:
                arr = arr[:, np.newaxis]

            # chọn kiểu dữ liệu phù hợp
            dtype = arr.dtype
            if dtype == np.float32:
                ros_type = PointField.FLOAT32
                size = 4
            elif np.issubdtype(dtype, np.int16):
                ros_type = PointField.INT16
                size = 2
            elif np.issubdtype(dtype, np.uint16):
                ros_type = PointField.UINT16
                size = 2
            else:
                rospy.logwarn(f"Unsupported field {key} with dtype {dtype}, skipped.")
                continue

            fields.append(PointField(name=key, offset=offset, datatype=ros_type, count=1))
            extra_data[key] = arr[:, 0]
            offset += size

    # --- Tạo structured array ---
    dtype_list = [(f.name, np.float32 if f.datatype == PointField.FLOAT32 else np.int16) for f in fields]
    structured = np.zeros(points.shape[0], dtype=dtype_list)

    structured["x"] = points[:, 0]
    structured["y"] = points[:, 1]
    structured["z"] = points[:, 2]
    structured["rgb"] = rgb_float

    for k, v in extra_data.items():
        structured[k] = v

    # --- Tạo PointCloud2 ROS message ---
    msg = pc2.create_cloud(header, fields, structured)
    return msg

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
        assert len(diff_array) == num_points, f"diff_array must have the same number of points as the point cloud"
        fields.append(('distances', np.float32))

    # Tạo array
    data = np.zeros(num_points, dtype=fields)
    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    if has_colors:
        data['rgb'] = rgb_float
    if diff_array is not None:
        data['distances'] = diff_array.astype(np.float32)

    # Tạo header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Convert sang PointCloud2
    return ros_numpy.point_cloud2.array_to_pointcloud2(data, frame_id=header.frame_id, stamp=header.stamp)


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


from std_msgs.msg import Empty
def notify_one(topic_name: str):
    pub = rospy.Publisher(topic_name, Empty, queue_size=1, latch=True)
    rospy.sleep(0.5)  # Chờ publisher được đăng ký với master
    pub.publish(Empty())
    pub.unregister()  # Giải phóng sau khi publish nếu không cần giữ lại


from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
def convert_pointcloud2_to_pointcloud(pc2_msg):
    pc_msg = PointCloud()
    pc_msg.header = pc2_msg.header
    points = []

    # Đọc từng điểm (x,y,z) từ PointCloud2
    for point in pc2.read_points(pc2_msg, skip_nans=True):
        x, y, z = point[:3]
        points.append(Point32(x, y, z))

    pc_msg.points = points
    return pc_msg


from shared.config_loader import CONFIG as cfg

def map_distances_to_colors(
    distances, 
    clip_max=0.15,
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


def assign_colors(tcloud, clip_max=150, highlight_range=(20, 40)):
    """
    Map the 'distances' field of a tensor PointCloud to 'colors'.
    
    Args:
        tcloud: o3d.t.geometry.PointCloud, must have 'distances' field
        clip_max: maximum distance to clip (values above get out_of_range_color)
        highlight_range: (low, high) range for pure green
    
    Returns:
        tcloud with updated 'colors' field (in-place)
    """
    import open3d as o3d
    if 'distances' not in tcloud.point:
        raise ValueError("PointCloud must have 'distances' field")
    
    distances = tcloud.point['distances'].cpu().numpy()  # CPU numpy array
    colors = map_distances_to_colors(distances, clip_max=clip_max, highlight_range=highlight_range)

    # Update tensor cloud colors
    tcloud.point['colors'] = o3d.core.Tensor(colors.astype(np.float32))
    return tcloud

# def assign_colors(tcloud, clip_max=150, highlight_range=(20, 40)):
#     """
#     Map the 'distances' field of a tensor PointCloud to 'colors'.

#     Args:
#         tcloud: o3d.t.geometry.PointCloud, must have 'distances' field
#         clip_max: maximum distance to clip
#         highlight_range: (low, high) range for pure green

#     Returns:
#         tcloud with updated 'colors' field (in-place)
#     """
#     import open3d as o3d
#     import numpy as np

#     # ---- Validate input type ----
#     if tcloud is None:
#         raise ValueError("tcloud is None")

#     if not isinstance(tcloud, o3d.t.geometry.PointCloud):
#         raise TypeError(
#             f"tcloud must be o3d.t.geometry.PointCloud, got {type(tcloud)}"
#         )

#     # ---- Validate required field ----
#     if 'distances' not in tcloud.point:
#         raise KeyError(
#             "PointCloud is missing required field 'distances'"
#         )

#     # ---- Validate highlight range ----
#     if (
#         not isinstance(highlight_range, (tuple, list))
#         or len(highlight_range) != 2
#         or highlight_range[0] >= highlight_range[1]
#     ):
#         raise ValueError(
#             f"highlight_range must be (low, high), got {highlight_range}"
#         )

#     # ---- Validate clip_max ----
#     if clip_max <= 0:
#         raise ValueError("clip_max must be > 0")

#     # ---- Convert distances safely ----
#     try:
#         distances = tcloud.point['distances']
#         distances_np = distances.cpu().numpy()
#     except Exception as e:
#         raise RuntimeError(
#             "Failed to convert 'distances' tensor to numpy array"
#         ) from e

#     # ---- Map distances to colors ----
#     try:
#         colors = map_distances_to_colors(
#             distances_np,
#             clip_max=clip_max,
#             highlight_range=highlight_range
#         )
#     except Exception as e:
#         raise RuntimeError(
#             "map_distances_to_colors() failed"
#         ) from e

#     # ---- Validate output colors ----
#     if colors.ndim != 2 or colors.shape[1] != 3:
#         raise ValueError(
#             f"colors must have shape (N,3), got {colors.shape}"
#         )

#     # ---- Assign colors back to tensor cloud ----
#     try:
#         tcloud.point['colors'] = o3d.core.Tensor(
#             colors.astype(np.float32),
#             device=tcloud.device
#         )
#     except Exception as e:
#         raise RuntimeError(
#             "Failed to assign colors to tcloud.point['colors']"
#         ) from e

#     return tcloud

def remove_point(pcd, key_points, radius):
    import open3d as o3d

    pts = np.asarray(pcd.points)
    if isinstance(key_points, o3d.geometry.PointCloud):
        query_pts = np.asarray(key_points.points)
    else:
        query_pts = np.asarray(key_points)

    tree = cKDTree(pts)
    idx_list = tree.query_ball_point(query_pts, r=radius)
    remove_idx = np.unique(np.concatenate(idx_list))

    pcd_filtered = pcd.select_by_index(remove_idx, invert=True)
    return pcd_filtered


def remove_ground_with_pca(pcd_origin, z_threshold=0.3, angle_deg=5,voxel_size=0.05,radius_remove=0.05, plane="xy"):
    # Estimate normals bằng PCA trong Open3D
    import open3d as o3d
    
    pcd = pcd_origin.voxel_down_sample(voxel_size=voxel_size)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(30)

    normals = np.asarray(pcd.normals)
    points = np.asarray(pcd.points)

    # vector pháp tuyến chuẩn (Oxy plane -> normal = (0,0,1))
    if plane.lower() in ["xy", "yx"]:
        z_axis = np.array([0, 0, 1])
        axis = 2
    elif plane.lower() in ["yz", "zy"]:
        z_axis = np.array([1, 0, 0])
        axis = 0
    elif plane.lower() in ["xz", "zx"]:
        z_axis = np.array([0, 1, 0])
        axis = 1


    # cos(angle) giữa normal và z_axis
    cos_angle = np.abs(normals @ z_axis)
    angles = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # rad

    # ngưỡng theo độ
    angle_threshold = np.deg2rad(angle_deg)

    # mask: chọn điểm có normal gần song song z, và nằm dưới z_threshold

    mask = (angles < angle_threshold) & (points[:, axis] < z_threshold)

    # giữ lại ground
    ground = pcd.select_by_index(np.where(mask)[0])
    # giữ lại non-ground (cloud đã "cắt bỏ mp")
    # non_ground = pcd.select_by_index(np.where(mask)[0], invert=True)
    non_ground = remove_point(pcd_origin, np.asarray(ground.points), radius=radius_remove)

    return non_ground


def detect_boundary_pca(pcd, k=30, angle_threshold=np.pi):
    """
    Boundary detection dựa trên PCA của lân cận.
    
    Args:
        pcd (o3d.geometry.PointCloud): input cloud
        k (int): số điểm lân cận dùng cho PCA
        angle_threshold (float): góc tối đa (rad) để coi là boundary
    
    Returns:
        mask (np.ndarray): boolean mask các điểm boundary
        boundary_points (np.ndarray): tọa độ các điểm boundary
    """

    import open3d as o3d
    points = np.asarray(pcd.points)
    N = len(points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    mask = np.zeros(N, dtype=bool)

    for i, p in enumerate(points):
        _, idx, _ = kdtree.search_knn_vector_3d(p, k)
        
        if len(idx) < 5:
            # Nếu không đủ điểm, coi luôn là boundary
            mask[i] = True
            continue

        neighbors = points[idx]

        # PCA: tìm 2 vector chính của lân cận
        C = np.cov(neighbors.T)
        eigvals, eigvecs = np.linalg.eigh(C)
        order = np.argsort(eigvals)[::-1]
        plane_axes = eigvecs[:, order[:2]]   # 2 vector chính

        # chiếu lân cận lên mặt phẳng
        proj = (neighbors - p) @ plane_axes
        norms = np.linalg.norm(proj, axis=1)
        valid = norms > 1e-6
        proj = proj[valid] / norms[valid][:, None]

        if len(proj) < 2:
            # quá ít điểm, coi là boundary
            mask[i] = True
            continue

        # tính góc cực
        angles = np.arctan2(proj[:,1], proj[:,0])
        angles = np.sort((angles + 2*np.pi) % (2*np.pi))
        diffs = np.diff(np.r_[angles, angles[0]+2*np.pi])
        max_gap = np.max(diffs)

        if max_gap > angle_threshold:
            mask[i] = True
        
    boundary_points = points[mask]
    return mask, boundary_points


def remove_boundary_region(original_pcd, boundary_points, radius=0.1):
    """
    Remove toàn bộ điểm trong cloud gốc nằm gần boundary points (khoảng cách < radius).

    Parameters
    ----------
    original_pcd : open3d.geometry.PointCloud
        Cloud gốc (full resolution)
    boundary_points : (N,3) np.ndarray
        Tọa độ boundary points (tìm từ cloud downsample)
    radius : float
        Bán kính loại bỏ

    Returns
    -------
    filtered_pcd : open3d.geometry.PointCloud
        Cloud sau khi remove điểm gần biên
    """

    import open3d as o3d

    points = np.asarray(original_pcd.points)

    # KDTree trên cloud gốc để search nhanh
    kdtree = o3d.geometry.KDTreeFlann(original_pcd)
    mask_remove = np.zeros(len(points), dtype=bool)

    for bp in boundary_points:
        [_, idx, _] = kdtree.search_radius_vector_3d(bp, radius)
        mask_remove[idx] = True

    # giữ lại những điểm không bị remove
    keep_idx = np.where(~mask_remove)[0]
    filtered_pcd = original_pcd.select_by_index(keep_idx)

    return filtered_pcd


def load_ply(filepath):
    import open3d as o3d
    try:
        pcd = o3d.io.read_point_cloud(filepath)
        if len(pcd.points) == 0:
            print("⚠️ File have no data:", filepath)
            return None
        return pcd
    except Exception as e:
        print(f"❌ Can't load file {filepath}: {e}")
        return None



def cloud_downsample(pcd, voxel_size: float):
    """
    Downsample point cloud (tensor) và tính trung bình colors + distances theo voxel.

    Args:
        pcd: o3d.t.geometry.PointCloud, phải có fields 'points', 'colors', 'distances'
        voxel_size: kích thước voxel

    Returns:
        down_pcd: o3d.t.geometry.PointCloud đã downsample
    """
    import open3d as o3d

    # Kiểm tra fields
    for field in ["positions", "colors", "distances"]:
        if field not in pcd.point:
            raise ValueError(f"PointCloud thiếu field '{field}'")

    # Downsample và trace
    down_pcd, trace = pcd.voxel_down_sample_and_trace(voxel_size=voxel_size)

    # trace: tensor int32, trace[i] = voxel index của point i
    trace = trace.to(dtype=o3d.core.Dtype.Int32)

    # Lấy colors và distances gốc
    colors = pcd.point["colors"]
    distances = pcd.point["distances"]

    # Tính trung bình distances theo voxel
    voxel_count = o3d.core.Tensor.zeros([len(down_pcd.point["positions"])], dtype=o3d.core.Dtype.Int32)
    voxel_sum = o3d.core.Tensor.zeros([len(down_pcd.point["positions"])], dtype=o3d.core.Dtype.Float32)
    voxel_sum.scatter_add_(trace, distances)
    voxel_count.scatter_add_(trace, o3d.core.Tensor.ones_like(trace, dtype=o3d.core.Dtype.Int32))
    distances_down = voxel_sum / voxel_count.to(o3d.core.Dtype.Float32)
    down_pcd.point["distances"] = distances_down

    # Tính trung bình colors theo voxel
    color_sum = o3d.core.Tensor.zeros([len(down_pcd.point["positions"]), 3], dtype=o3d.core.Dtype.Float32)
    color_sum.scatter_add_(trace, colors)
    voxel_count_float = voxel_count.to(o3d.core.Dtype.Float32).reshape([-1, 1])
    colors_down = color_sum / voxel_count_float
    down_pcd.point["colors"] = colors_down

    return down_pcd



from scipy.spatial.transform import Rotation as R
import numpy as np


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

    # Tính diện tích
    area = mesh.get_surface_area()
    return area





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
