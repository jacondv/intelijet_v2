# import open3d as o3d
import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.path import Path
from scipy.spatial import cKDTree


class TunnelProcessing:
    """
    Class for processing 3D tunnel point clouds:
    - Cropping (box, sphere, custom ROI)
    - Downsampling
    - Ground plane extraction
    - Plane segmentation (floor, ceiling, walls)
    - Alignment & registration
    - Visualization
    """

    def __init__(self, pcd=None):
        """
        Initialize with a point cloud.

        Parameters
        ----------
        pcd : o3d.geometry.PointCloud
            The input point cloud (raw tunnel scan).
        """
        self.pcd = pcd
        self.history = []  # optional: log processing steps

        self.ground_plane_normal = None
        self.left_wall_normal = None
        self.right_wall_normal = None
        self.back_plane_normal = None


    def set_pcd(self, pcd):
        """Gán hoặc thay đổi point cloud sau khi khởi tạo."""
        import open3d as o3d
        if not isinstance(pcd, (o3d.geometry.PointCloud, o3d.t.geometry.PointCloud)):
            raise TypeError("pcd must be an Open3D PointCloud.")
        self.pcd = pcd
        self.history.append("set_pointcloud")

    # ------------------------------
    # Basic operations
    # ------------------------------

    def crop(self, 
            pcd, 
            min_bound: np.ndarray, 
            max_bound: np.ndarray, 
            normal: np.ndarray = None
            ):
        """
        Cắt point cloud bằng hộp giới hạn (bounding box).
        - Nếu normal được truyền vào: xoay cloud sao cho mặt phẳng có normal này song song với mặt XY,
        crop trong hệ toạ độ mới, sau đó xoay lại.
        - Nếu không có normal: crop trực tiếp bằng AABB.

        Args:
            pcd (o3d.geometry.PointCloud): point cloud đầu vào
            min_bound (np.ndarray): biên dưới [x, y, z]
            max_bound (np.ndarray): biên trên [x, y, z]
            normal (np.ndarray, optional): vector pháp tuyến của mặt cần căn chỉnh

        Returns:
            o3d.geometry.PointCloud: cloud sau khi cắt
        """
        import open3d as o3d
        if normal is None:
            # --- Crop trực tiếp bằng AABB ---
            aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            cropped = pcd.crop(aabb)
            return cropped

        # --- Nếu có normal: xoay theo hướng của mặt phẳng ---
        normal = normal / np.linalg.norm(normal)
        z_axis = np.array([0, 0, 1])
        axis = np.cross(normal, z_axis)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-6:
            R_align = np.eye(3)
        else:
            axis /= axis_norm
            angle = np.arccos(np.clip(np.dot(normal, z_axis), -1.0, 1.0))
            R_align = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)

        # --- Xoay cloud tạm thời ---
        pcd_rot = pcd.rotate(R_align, center=(0, 0, 0))

        # --- Crop bằng AABB trong hệ tọa độ đã xoay ---
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        cropped_rot = pcd_rot.crop(aabb)

        # --- Xoay ngược lại về hướng ban đầu ---
        R_inv = R_align.T
        cropped = cropped_rot.rotate(R_inv, center=(0, 0, 0))

        return cropped


    def slice_cloud_vectorized(self,pcd, axis='x', layer_thickness=0.01):
        """
        Chia point cloud thành layer theo một trục, ép Z về center layer, 
        hoàn toàn vectorized, rất nhanh cho clouds lớn.
        
        Args:
            pcd (o3d.t.geometry.PointCloud): point cloud đầu vào
            axis (str): trục để chia ('x','y','z')
            layer_thickness (float): độ dày mỗi layer

        Returns:
            layers (list of o3d.t.geometry.PointCloud)
        """

        import open3d as o3d

        if not isinstance(pcd, o3d.t.geometry.PointCloud):
            pcd = o3d.t.geometry.PointCloud.from_legacy(pcd)
        
        # Lấy points và colors
        points = pcd.point.positions.cpu().numpy()
        has_colors = "colors" in pcd.point
        colors = pcd.point.colors.cpu().numpy() if has_colors else None

        has_distances = "distances" in pcd.point
        distances = pcd.point.distances.cpu().numpy() if has_distances else None

        axis_idx = {"x":0, "y":1, "z":2}[axis.lower()]
        min_val = points[:, axis_idx].min()
        max_val = points[:, axis_idx].max()
        num_layers = int(np.ceil((max_val - min_val) / layer_thickness))

        # Tính layer index cho từng point
        layer_idx = np.floor((points[:, axis_idx] - min_val) / layer_thickness).astype(int)
        layer_idx[layer_idx >= num_layers] = num_layers - 1  # fix điểm max

        # Tính Z center cho từng point
        center_vals = min_val + (layer_idx + 0.5) * layer_thickness
        points[:, axis_idx] = center_vals

        # Sắp xếp points theo layer_idx để gom points cùng layer
        sort_idx = np.argsort(layer_idx)
        points_sorted = points[sort_idx]
        if has_colors:
            colors_sorted = colors[sort_idx]
        
        if has_distances:
             dists_sorted = distances[sort_idx]
        
        layer_idx_sorted = layer_idx[sort_idx]

        # Tìm các điểm bắt đầu layer mới
        unique_layers, first_idx = np.unique(layer_idx_sorted, return_index=True)
        # Thêm chỉ số kết thúc cho slicing
        last_idx = np.append(first_idx[1:], len(points_sorted))

        layers = []
        for start, end in zip(first_idx, last_idx):
            slice_pcd = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
            slice_pcd.point.positions = o3d.core.Tensor(points_sorted[start:end], dtype=o3d.core.Dtype.Float32)
            if has_colors:
                slice_pcd.point.colors = o3d.core.Tensor(colors_sorted[start:end], dtype=o3d.core.Dtype.Float32)

            if has_distances:
                slice_pcd.point.distances = o3d.core.Tensor(dists_sorted[start:end], dtype=o3d.core.Dtype.Float32)

            slice_pcd = slice_pcd.voxel_down_sample(0.01)
            layers.append(slice_pcd)

        print(f"✅ Generated {len(layers)} layers along {axis.upper()} axis")
        return layers
        

    def combine_pointcloud_list(self, clouds):
        """
        Gộp danh sách o3d.t.geometry.PointCloud thành 1 PointCloud CPU duy nhất.
        Hỗ trợ 'positions', 'colors' và 'distances' (nếu có).
        Cực nhanh, không vòng lặp thừa.
        """
        import open3d as o3d
        import numpy as np

        # --- Gộp các mảng numpy ---
        all_pts  = np.vstack([pc.point.positions.cpu().numpy() for pc in clouds])
        all_cols = np.vstack([pc.point.colors.cpu().numpy() for pc in clouds if 'colors' in pc.point]) \
                if any('colors' in pc.point for pc in clouds) else None
        all_dist = np.vstack([pc.point.distances.cpu().numpy() for pc in clouds if 'distances' in pc.point]) \
                if any('distances' in pc.point for pc in clouds) else None

        # --- Tạo cloud kết quả ---
        cloud = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
        cloud.point.positions = o3d.core.Tensor(all_pts.astype(np.float32))

        if all_cols is not None:
            cloud.point.colors = o3d.core.Tensor(all_cols.astype(np.float32))
        if all_dist is not None:
            cloud.point.distances = o3d.core.Tensor(all_dist.astype(np.float32))

        return cloud


    def upsample(self, pcd, min_gap=0.02, step=0.01, axis='x', max_gap=0.5):
        """
        Nội suy thêm điểm còn thiếu vào cloud 2D theo thứ tự góc quanh trục chỉ định.
        Giữ lại tất cả field (colors, distances, ...), 
        và trả về cloud gốc + các điểm nội suy thêm.
        """
        import open3d as o3d
        import numpy as np

        # --- Kiểm tra trục ---
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis not in axis_map:
            raise ValueError("axis phải là 'x', 'y' hoặc 'z'")
        main_axis = axis_map[axis]
        other_axes = [i for i in range(3) if i != main_axis]

        # --- Lấy dữ liệu ---
        pts = pcd.point.positions.cpu().numpy()
        fields = list(pcd.point)
        field_data = {k: pcd.point[k].cpu().numpy() for k in fields if k != "positions"}

        if len(pts) < 2:
            return pcd

        # --- Tính góc quanh trục ---
        center = np.mean(pts[:, other_axes], axis=0)
        vecs = pts[:, other_axes] - center
        angles = np.arctan2(vecs[:, 1], vecs[:, 0])
        sort_idx = np.argsort(angles)

        pts = pts[sort_idx]
        for k in field_data:
            field_data[k] = field_data[k][sort_idx]

        # --- Nội suy ---
        new_pts = []
        new_fields = {k: [] for k in field_data}

        for i in range(1, len(pts)):
            p0, p1 = pts[i - 1], pts[i]
            gap = np.linalg.norm(p1[other_axes] - p0[other_axes])
            if gap > max_gap:
                continue
            n = int(np.ceil(gap / step))
            if gap > min_gap and n > 1:
                interps = p0 + (p1 - p0) * np.linspace(1 / n, (n - 1) / n, n - 1)[:, None]
                interps[:, main_axis] = p0[main_axis]
                new_pts.extend(interps)
                for k in field_data:
                    if k == "distances":
                        fill_val = -1 * np.ones_like(field_data[k][0])
                    else:
                        fill_val = np.zeros_like(field_data[k][0])
                    new_fields[k].extend([fill_val] * len(interps))

        # --- Gộp cloud gốc + điểm mới ---
        all_pts = np.vstack([pts, np.array(new_pts)]) if new_pts else pts
        all_fields = {}
        for k in field_data:
            all_fields[k] = np.vstack([
                field_data[k],
                np.array(new_fields[k]) if len(new_fields[k]) > 0 else np.empty((0, field_data[k].shape[1]))
            ])

        # --- Loại trùng toàn bộ ---
        all_pts_rounded = np.round(all_pts, 6)
        _, unique_idx = np.unique(all_pts_rounded, axis=0, return_index=True)
        all_pts = all_pts[unique_idx]
        for k in all_fields:
            all_fields[k] = all_fields[k][unique_idx]

        # --- Xuất point cloud ---
        pcd_new = o3d.t.geometry.PointCloud()
        pcd_new.point.positions = o3d.core.Tensor(all_pts, o3d.core.Dtype.Float32)
        for k, v in all_fields.items():
            pcd_new.point[k] = o3d.core.Tensor(v, o3d.core.Dtype.Float32)

        return pcd_new


    def fill_null_distances(self,pcd_tensor, k=4, eps=1e-8, null_value=-1):
        """
        Điền các khoảng cách = -1 bằng weighted average của k-lân cận hợp lệ (vectorized version).
        Nhanh hơn bản vòng for rất nhiều.

        Args:
            pcd_tensor: o3d.t.geometry.PointCloud, có trường 'distances'
            k: số lân cận để trung bình
            eps: số nhỏ tránh chia 0
            null_value: giá trị biểu thị khoảng cách bị thiếu (-1)

        Returns:
            o3d.t.geometry.PointCloud mới với distances đã điền
        """
        import open3d as o3d
        import numpy as np
        from scipy.spatial import cKDTree

        if "distances" not in pcd_tensor.point:
            return pcd_tensor.clone()

        device = pcd_tensor.device
        points = pcd_tensor.point.positions.cpu().numpy()
        distances = pcd_tensor.point["distances"].cpu().numpy()

        # mask các điểm cần điền
        if distances.ndim == 1:
            mask_null = distances == null_value
        else:
            mask_null = np.all(distances == null_value, axis=1)
        if not np.any(mask_null):
            return pcd_tensor.clone()

        # Tách hợp lệ / không hợp lệ
        valid_points = points[~mask_null]
        valid_dists = distances[~mask_null]
        null_points = points[mask_null]

        # --- Build tree từ điểm hợp lệ ---
        tree = cKDTree(valid_points)

        # --- Query k-lân cận hợp lệ ---
        dists, idxs = tree.query(null_points, k=min(k, valid_points.shape[0]))

        # --- Weighted average vectorized ---
        w = 1.0 / (dists + eps)
        w = w[:, :, None] 
        if w.ndim == 1:
            w = w[:, :, None] 
            neighbor_vals = valid_dists[idxs][:, None]
        else:
            neighbor_vals = valid_dists[idxs]

        # weighted_sum = np.sum(w * neighbor_vals, axis=1)
        
        # sum_w = np.sum(w, axis=1)
        # filled_vals = weighted_sum / sum_w
        filled_vals = np.mean(neighbor_vals, axis=1)

        # --- Gộp lại ---
        new_distances = distances.copy()
        new_distances[mask_null] = filled_vals

        # --- Tạo PointCloud mới ---
        new_pcd = o3d.t.geometry.PointCloud()
        new_pcd.point.positions = o3d.core.Tensor(points, device=device)
        if "colors" in pcd_tensor.point:
            new_pcd.point.colors = pcd_tensor.point.colors.clone()
        new_pcd.point["distances"] = o3d.core.Tensor(new_distances, device=device)

        return new_pcd


    # ------------------------------
    # Ground & plane operations
    # ------------------------------
    def estimate_normals(self, knn: int = 30):
        """
        Estimate surface normals for the point cloud using PCA.

        Parameters
        ----------
        knn : int, optional (default=30)
            Number of nearest neighbors used in PCA to estimate the normal vector
            at each point.

        Returns
        -------
        pcd_with_normals : o3d.geometry.PointCloud
            Point cloud with estimated normals.
        """

        import open3d as o3d

        # Tính normal cho self.pcd
        self.pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
        )

        # (tùy chọn) chuẩn hóa hướng normal để đồng nhất hơn
        self.pcd.orient_normals_consistent_tangent_plane(k=knn)

        # ghi lại history
        self.history.append(f"estimate_normals(knn={knn})")

        return self.pcd


    def get_plane(self,
        pcd,
        normal_angle_threshold: float = 5.0,
        radius: float = 0.2,
        reference_plane: str = "xy",
        min_bound: np.ndarray = [0.0,-5.0,-1.0],
        max_bound: np.ndarray =  [15.0,5.0,10.0]
    ):
        """
        Remove ground plane from a point cloud using PCA-based normal estimation,
        restricted to points inside a given bounding box.

        Parameters
        ----------
        pcd : o3d.geometry.PointCloud
            Input point cloud.
        normal_angle_threshold : float, optional (default=5.0)
            Angular threshold (in degrees) between normals and reference axis.
        radius : float, optional (default=0.2)
            Radius used to remove ground points from the original point cloud.
        reference_plane : str, optional (default="xy")
            Plane considered as ground. Options:
                - "xy" → ground normal aligned with Z axis
                - "yz" → ground normal aligned with X axis
                - "xz" → ground normal aligned with Y axis
        crop_box : o3d.geometry.AxisAlignedBoundingBox or o3d.geometry.OrientedBoundingBox, optional
            Bounding box to restrict ground detection. If None, the whole cloud is used.

        Returns
        -------
        non_ground : o3d.geometry.PointCloud
            Point cloud with ground removed.
        ground : o3d.geometry.PointCloud
            Extracted ground points.
        ground_center : np.ndarray, shape (3,)
            Centroid of ground points.
        """
        import open3d as o3d
        # --- Step 0: Crop point cloud if crop_box provided ---
        crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

        if isinstance(pcd, o3d.t.geometry.PointCloud):
            pcd = pcd.to_legacy() 

        if crop_box is not None:
            pcd_cropped = pcd.crop(crop_box)
        else:
            pcd_cropped = pcd

        # --- Step 1: Downsample and estimate normals ---
        pcd_down = pcd_cropped.voxel_down_sample(voxel_size=0.05)
        pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        pcd_down.orient_normals_consistent_tangent_plane(30)

        normals = np.asarray(pcd_down.normals)
        points = np.asarray(pcd_down.points)

        # --- Step 2: Choose reference axis based on selected plane ---
        if reference_plane == "xy":
            ref_axis = np.array([0, 0, 1])
            axis_idx = 2
        elif reference_plane == "yz":
            ref_axis = np.array([1, 0, 0])
            axis_idx = 0
        elif reference_plane == "xz":
            ref_axis = np.array([0, 1, 0])
            axis_idx = 1
        else:
            raise ValueError(f"Invalid reference_plane '{reference_plane}'. Use ['xy', 'yz', 'xz'].")

        # --- Step 3: Compute angles between normals and reference axis ---
        cos_angle = np.abs(normals @ ref_axis)
        angles = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # radians
        angle_threshold = np.deg2rad(normal_angle_threshold)

        # --- Step 4: Mask ground points ---
        mask = (angles < angle_threshold)
        ground_idx = np.where(mask)[0]
        if len(ground_idx) < 1000:
            print("[INFO] No ground-like points found.")
            return pcd, None, None, None
               
        ground = pcd_down.select_by_index(np.where(mask)[0])
        tree = cKDTree(self.pcd.points)
        if ground is not None:
            all_idx = tree.query_ball_point(ground.points, r=radius)  # trả về list list
            all_idx = np.unique(np.hstack(all_idx))
            ground_plane = self.pcd.select_by_index(all_idx)

            ground_plane_normal = self.compute_plane_normal(ground_plane)
            non_ground_plane = self.pcd.select_by_index(all_idx, invert=True)
        # --- Step 6: Compute centroid of ground ---
            ground_center = ground.get_center()
        else:
            non_ground_plane = pcd_down
            ground_plane = None
            ground_plane_normal=None
            ground_center=None
        
        return non_ground_plane, ground_plane, ground_center, ground_plane_normal

    # ------------------------------
    # Registration & alignment
    # ------------------------------
    def align(self, target_pcd,
                  init_transformation: np.ndarray = np.eye(4)):
        """Align this cloud with target using ICP."""
        pass

    # ------------------------------
    # Utilities
    # ------------------------------
    def get_center(self) -> np.ndarray:
        """Return centroid of current cloud."""
        pass


    def get_bounding_box_bounds(self):
        """
        Return the min and max bounds (corner points) of the axis-aligned bounding box.

        Returns
        -------
        min_bound : np.ndarray (3,)
            Minimum corner [x_min, y_min, z_min].
        max_bound : np.ndarray (3,)
            Maximum corner [x_max, y_max, z_max].
        """
        aabb = self.pcd.get_axis_aligned_bounding_box()
        min_bound = aabb.get_min_bound()
        max_bound = aabb.get_max_bound()

        self.history.append("get_bounding_box_bounds()")
        return min_bound, max_bound


    def compute_plane_normal(self, cloud) -> np.ndarray:
        """
        Tính vector pháp tuyến đại diện cho một point cloud gần phẳng bằng PCA.

        Parameters
        ----------
        cloud : o3d.geometry.PointCloud
            Point cloud cần tính pháp tuyến.

        Returns
        -------
        normal : np.ndarray shape (3,)
            Vector pháp tuyến đơn vị (normalized).
        """
        points = np.asarray(cloud.points)
        if points.shape[0] < 3:
            raise ValueError("Không đủ điểm để tính pháp tuyến (tối thiểu 3 điểm).")

        # Tính tâm
        centroid = np.mean(points, axis=0)

        # Ma trận hiệp phương sai
        cov = np.cov((points - centroid).T)

        # Eigen decomposition
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Vector ứng với eigenvalue nhỏ nhất là pháp tuyến
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        normal /= np.linalg.norm(normal)

        # Đảm bảo hướng lên trên nếu trục Z âm
        if normal[2] < 0:
            normal = -normal

        return normal


    def run_processing_pipeline(self):

        FRONT_BOX = [(0, -5.0, -0.5), 
                     (3.5, 5.0, 8.0)]
        BACK_BOX = [(4.0, -5.0, 0.0), 
                    (11.0, 5.0, 8.0)]
        LEFT_BOX = [(2.5, 0.0, -0.5), 
                    (11.0, 5.0, 8.0)]
        RIGHT_BOX = [(2.5, -5.0, -0.5), 
                    (11.0, 0.0, 8.0)]
        BOTTOM_BOX = [(2.5, -5.0, -1.5), 
                      (11.0, 5.0, 1.5)]
        TOP_BOX = [(2.5, -5.0, 4.0), 
                    (11.0, 5.0, 8.0)] 

        _, _, ground_center, self.ground_plane_normal = self.get_plane(self.pcd,
                                normal_angle_threshold=5,
                                radius=0.15, 
                                reference_plane="xy",
                                min_bound=BOTTOM_BOX[0],
                                max_bound=BOTTOM_BOX[1]
                                )

        _, _, back_center, self.back_plane_normal = self.get_plane(self.pcd,
                                normal_angle_threshold=5,
                                radius=0.15, 
                                reference_plane="yz",
                                min_bound=BACK_BOX[0],
                                max_bound=BACK_BOX[1]
                                )

        _, _, right_center, self.right_wall_normal = self.get_plane(self.pcd,
                        normal_angle_threshold=5,
                        radius=0.15, 
                        reference_plane="xz",
                        min_bound=RIGHT_BOX[0],
                        max_bound=RIGHT_BOX[1]
                        )
        
        _, _, left_center, self.left_wall_normal = self.get_plane(self.pcd,
                        normal_angle_threshold=5,
                        radius=0.15, 
                        reference_plane="xz",
                        min_bound=LEFT_BOX[0],
                        max_bound=LEFT_BOX[1]
                        )
        
        _, _, front_center, self.front_wall_normal = self.get_plane(self.pcd,
                        normal_angle_threshold=5,
                        radius=0.15, 
                        reference_plane="yz",
                        min_bound=FRONT_BOX[0],
                        max_bound=FRONT_BOX[1]
                        )
        

        def safe_bound_value(center, idx, offset, default):
            if center is None:
                return default
            try:
                return center[idx] + offset
            except:
                return default
        # minbound = [2.1, right_center[1]-0.3, ground_center[2]+0.3]
        # maxbound = [back_center[0]-0.2, left_center[1]+0.3, 6.2]
        minbound = [
            safe_bound_value(front_center, 0, -0.3, 0.5),
            safe_bound_value(right_center, 1, -0.3, -5.0),   # fallback khi right_center None
            safe_bound_value(ground_center, 2, +0.3, 0.0)
        ]

        maxbound = [
            safe_bound_value(back_center, 0, -0.3, 10.0),
            safe_bound_value(left_center, 1, +0.3, 5.0),
            11.0
        ]
        print(minbound, maxbound)
        cloud = self.crop(pcd=self.pcd, min_bound=minbound, max_bound=maxbound, normal=self.ground_plane_normal)
        return cloud


    def run_upsample(self, pcd, axis='x', min_gap=0.02,max_gap=0.5):
        from pps.data_converter import cloudconverter
        slices = self.slice_cloud_vectorized(pcd, axis=axis, layer_thickness=0.01)
        slice_upsample = []
        for s in slices:
            one_slice = self.upsample(s,min_gap=min_gap, step=min_gap, axis=axis, max_gap=max_gap)
            slice_upsample.append(one_slice)

        cloud_combine  = self.combine_pointcloud_list(slice_upsample)
        cloud_combine = self.fill_null_distances(cloud_combine)
        cloud_combine = cloudconverter.voxel_down_sample(cloud_combine,voxel_size=min_gap)
        return cloud_combine


    def remove_point(self, plane, radius=0.2):
        """
        Chọn điểm trong pcd_origin gần mặt phẳng ground và nằm trong polygon footprint của ground.
        """
        # --- 1. Fit mặt phẳng từ ground ---
        pcd_origin = self.pcd
        plane_model, _ = plane.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        a, b, c, d = plane_model
        normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        centroid = np.mean(np.asarray(plane.points), axis=0)

        # --- 2. Định nghĩa hệ toạ độ local trên mặt phẳng ---
        u = np.array([1, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1, 0])
        x_axis = np.cross(normal, u); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis); y_axis /= np.linalg.norm(y_axis)

        # --- 3. Chiếu ground vào 2D ---
        ground_points = np.asarray(plane.points) - centroid
        ground_xy = np.stack([ground_points @ x_axis, ground_points @ y_axis], axis=1)

        # convex hull để tạo polygon
        hull = ConvexHull(ground_xy)
        polygon = Path(ground_xy[hull.vertices])

        # --- 4. Xử lý point cloud gốc ---
        points = np.asarray(pcd_origin.points)
        dist = np.abs(points @ normal + d)  # khoảng cách tới mặt phẳng

        shifted = points - centroid
        proj_xy = np.stack([shifted @ x_axis, shifted @ y_axis], axis=1)

        inside = polygon.contains_points(proj_xy)
        mask = (dist < radius) & inside

        return pcd_origin.select_by_index(np.where(mask)[0]), mask
