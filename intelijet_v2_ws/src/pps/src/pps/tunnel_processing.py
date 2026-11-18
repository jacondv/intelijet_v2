import numpy as np

from shared.config_loader import CONFIG as cfg


def _cfg(*names, default=None):
    """Safe nested getattr on CONFIG - returns `default` if any level is
    missing, so this keeps working on a machine whose last_used.yaml
    predates the `tunnel_processing:` section in runtime.yaml. Mirrors
    pps.cloud_processing.compare_pipeline._cfg() - kept as a separate local
    copy per that file's own precedent (Phase 6), rather than a shared util."""
    obj = cfg
    for name in names:
        obj = getattr(obj, name, None)
        if obj is None:
            return default
    return obj


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
        self.back_plane_normal = None
        self.front_wall_normal = None


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
    # Crop ground (Note only for ground plane)
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


    def _detect_wall_plane(self,
        pcd,
        normal_angle_threshold: float = 8.0,
        distance_threshold: float = 0.15,
        reference_plane: str = "xy",
        min_bound: np.ndarray = [0.0,-5.0,-1.0],
        max_bound: np.ndarray =  [15.0,5.0,10.0],
        num_iterations: int = 1000,
    ):
        """
        Detect a wall-like plane (ground, back or front wall - whichever one
        `reference_plane`/`min_bound`/`max_bound` are set up for) via RANSAC
        plane fitting, restricted to points inside a given bounding box.

        This is the shared primitive behind detect_ground()/detect_back_wall()/
        detect_front_wall() - see run_processing_pipeline(). It only detects
        the plane and reports which points belong to it; it does not decide
        the final crop bounds.

        Replaces the old PCA-classify-by-local-normal-then-radius-expand
        approach: that expanded to full resolution by Euclidean proximity to
        an already-classified point, which pulls in non-coplanar points
        (rubble, a column base) just for standing near a real ground point,
        then averaged them all into one PCA-covariance normal with no
        outlier rejection. RANSAC (`segment_plane`, Open3D's own, no new
        dependency) fits the plane with built-in outlier rejection, and the
        full-resolution expansion below measures actual point-to-plane
        distance instead of point-to-point proximity - the right quantity
        for "does this point belong to this plane".

        Parameters
        ----------
        pcd : o3d.geometry.PointCloud
            Input point cloud.
        normal_angle_threshold : float, optional (default=8.0)
            Angular threshold (in degrees) the fitted plane's normal must be
            within of the reference axis, or the plane is rejected (e.g.
            RANSAC locked onto some other flat surface in the box instead of
            the intended wall).
        distance_threshold : float, optional (default=0.15)
            Max perpendicular distance (metres) from the fitted plane for a
            point to count as belonging to it - used both by RANSAC itself
            (on the downsampled box crop) and by the full-resolution
            expansion step below (on self.pcd). Carried over from the old
            `radius` parameter's value for continuity, but the geometric
            meaning changed (point-to-plane distance, not point-to-point
            proximity) - re-validate against real scans before trusting 0.15
            as tuned for this new meaning.
        reference_plane : str, optional (default="xy")
            Plane considered as ground. Options:
                - "xy" → ground normal aligned with Z axis
                - "yz" → ground normal aligned with X axis
                - "xz" → ground normal aligned with Y axis
        min_bound, max_bound : array-like
            Bounding box restricting where the plane is searched for.
        num_iterations : int, optional (default=1000)
            RANSAC iteration count (`o3d.geometry.PointCloud.segment_plane`).

        Returns
        -------
        non_plane : o3d.geometry.PointCloud
            Point cloud with the detected plane's points removed.
        plane_cloud : o3d.geometry.PointCloud
            Extracted plane points (full resolution).
        center : np.ndarray, shape (3,)
            Centroid of the plane points.
        normal : np.ndarray, shape (3,)
            Unit normal of the plane (refit on the full-resolution inliers).
        """
        import open3d as o3d

        # --- Step 0: Crop point cloud to the search box ---
        crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

        if isinstance(pcd, o3d.t.geometry.PointCloud):
            pcd = pcd.to_legacy()

        pcd_cropped = pcd.crop(crop_box)

        # --- Step 1: Downsample, then fit a plane via RANSAC ---
        pcd_down = pcd_cropped.voxel_down_sample(voxel_size=0.05)

        if len(pcd_down.points) < 3:
            print("[INFO] Not enough points in box to fit a plane.")
            return pcd, None, None, None

        plane_model, inliers = pcd_down.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=num_iterations,
        )
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal /= np.linalg.norm(normal)

        # --- Step 2: Reject the fit if its normal doesn't match the
        # expected reference axis (RANSAC found *a* flat surface in the box,
        # but not the wall we're looking for) ---
        ref_axis = {
            "xy": np.array([0, 0, 1]),
            "yz": np.array([1, 0, 0]),
            "xz": np.array([0, 1, 0]),
        }.get(reference_plane)
        if ref_axis is None:
            raise ValueError(f"Invalid reference_plane '{reference_plane}'. Use ['xy', 'yz', 'xz'].")

        cos_angle = np.abs(np.dot(normal, ref_axis))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        if angle > np.deg2rad(normal_angle_threshold):
            print("[INFO] Fitted plane's normal doesn't match the expected wall orientation.")
            return pcd, None, None, None

        # --- Step 3: Minimum-support sanity check (same threshold/cloud
        # resolution as the old classify-by-normal step, so comparable) ---
        if len(inliers) < 1000:
            print("[INFO] Not enough inlier points for a reliable plane fit.")
            return pcd, None, None, None

        # --- Step 4: Expand to full resolution by point-to-plane distance,
        # not point-to-point proximity - restricted to the same search box
        # so an unrelated coplanar surface elsewhere isn't pulled in ---
        points = np.asarray(self.pcd.points)
        dist_to_plane = np.abs(points @ normal + d)
        in_box = np.all((points >= min_bound) & (points <= max_bound), axis=1)
        full_mask = (dist_to_plane < distance_threshold) & in_box
        full_idx = np.where(full_mask)[0]

        plane_cloud = self.pcd.select_by_index(full_idx)
        non_plane = self.pcd.select_by_index(full_idx, invert=True)

        # --- Step 5: refit the normal on the clean, distance-verified
        # full-resolution inliers for a tighter final estimate ---
        normal = self.compute_plane_normal(plane_cloud)
        center = plane_cloud.get_center()

        return non_plane, plane_cloud, center, normal

    # ------------------------------
    # Named wall-detection wrappers (used by run_processing_pipeline)
    # ------------------------------
    # These only detect a wall plane (center + normal) - they don't remove
    # anything themselves; the actual crop/removal happens once at the end
    # of run_processing_pipeline() via self.crop(). Each wrapper is just
    # _detect_wall_plane() pinned to the box/reference-plane for that one
    # wall. Box/threshold values default to the exact constants
    # this file used before they were parameterized (see runtime.yaml's
    # `tunnel_processing:` section) - so behavior is unchanged.
    def detect_ground(self):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            distance_threshold=_cfg("tunnel_processing", "plane_detection", "distance_threshold", default=0.15),
            num_iterations=_cfg("tunnel_processing", "plane_detection", "num_iterations", default=1000),
            reference_plane="xy",
            min_bound=_cfg("tunnel_processing", "boxes", "bottom", "min", default=(2.5, -5.0, -1.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "bottom", "max", default=(11.0, 5.0, 1.5)),
        )

    def detect_back_wall(self):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            distance_threshold=_cfg("tunnel_processing", "plane_detection", "distance_threshold", default=0.15),
            num_iterations=_cfg("tunnel_processing", "plane_detection", "num_iterations", default=1000),
            reference_plane="yz",
            min_bound=_cfg("tunnel_processing", "boxes", "back", "min", default=(4.0, -5.0, 0.0)),
            max_bound=_cfg("tunnel_processing", "boxes", "back", "max", default=(11.0, 5.0, 8.0)),
        )

    def detect_front_wall(self):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            distance_threshold=_cfg("tunnel_processing", "plane_detection", "distance_threshold", default=0.15),
            num_iterations=_cfg("tunnel_processing", "plane_detection", "num_iterations", default=1000),
            reference_plane="yz",
            min_bound=_cfg("tunnel_processing", "boxes", "front", "min", default=(0, -5.0, -0.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "front", "max", default=(3.5, 5.0, 8.0)),
        )

    # ------------------------------
    # Utilities
    # ------------------------------
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


    def run_processing_pipeline(self, remove_ground: bool = True, remove_back_wall: bool = True):
        """
        remove_ground / remove_back_wall let the caller independently turn off
        tightening that one wall's crop bound - see docs/plan/00_INDEX.md P12
        follow-up. front is not (yet) independently toggleable and always
        gets detected+cropped when this runs, matching prior behavior.
        Right/left wall detection was removed - the Y crop bound is now a
        fixed constant (see minbound/maxbound below), not caller-toggleable.

        Ground is always detected regardless of `remove_ground`: besides its own
        Z bound, self.ground_plane_normal is also what self.crop() uses to
        auto-level the whole cloud before cropping on ANY axis - back/front's
        crops depend on that leveling too, not just ground's. `remove_ground=False`
        only drops the ground_center used to tighten the Z bound (falls back to
        the same default used when ground can't be detected at all), it does
        not skip detecting the normal.
        """
        _, _, ground_center, self.ground_plane_normal = self.detect_ground()
        if not remove_ground:
            ground_center = None

        if remove_back_wall:
            _, _, back_center, self.back_plane_normal = self.detect_back_wall()
        else:
            back_center, self.back_plane_normal = None, None

        _, _, front_center, self.front_wall_normal = self.detect_front_wall()

        # Fallback (plane not detected/toggled off) reuses compare_pipeline's
        # crop_box - that's already the "safe compare area" boundary, no need
        # for a second set of magic numbers meaning the same thing.
        compare_box_min = _cfg("compare_pipeline", "crop_box", "min", default=(0, -10, -0.3))
        compare_box_max = _cfg("compare_pipeline", "crop_box", "max", default=(11, 10, 7))

        front_x = (front_center[0] + _cfg("tunnel_processing", "crop_offsets", "front_x", default=-0.5)
                   if front_center is not None
                   else compare_box_min[0])
        back_x = (back_center[0] + _cfg("tunnel_processing", "crop_offsets", "back_x", default=-0.1)
                  if back_center is not None
                  else compare_box_max[0])
        ground_z = (ground_center[2] + _cfg("tunnel_processing", "crop_offsets", "ground_z", default=0.3)
                    if ground_center is not None
                    else compare_box_min[2])

        minbound = [front_x, -np.inf, ground_z]  # Y unbounded - no left/right wall detection anymore
        maxbound = [back_x, np.inf, 15.0]

        print(minbound, maxbound)
        cloud = self.crop(pcd=self.pcd, min_bound=minbound, max_bound=maxbound, normal=self.ground_plane_normal)

        return cloud
