import numpy as np
import rospy
from scipy.spatial import cKDTree

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
        radius: float = 0.2,
        reference_plane: str = "xy",
        min_bound: np.ndarray = [0.0,-5.0,-1.0],
        max_bound: np.ndarray =  [15.0,5.0,10.0],
        tree: cKDTree = None
    ):
        """
        Detect a wall-like plane (ground, back, left, right or front wall - whichever
        one `reference_plane`/`min_bound`/`max_bound` are set up for) using PCA-based
        normal estimation, restricted to points inside a given bounding box.

        This is the shared primitive behind detect_ground()/detect_back_wall()/
        detect_left_wall()/detect_right_wall()/detect_front_wall() - see
        run_processing_pipeline(). It only detects the plane and reports which
        points belong to it; it does not decide the final crop bounds.

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
        tree : scipy.spatial.cKDTree, optional
            KDTree pre-built over self.pcd.points. run_processing_pipeline() builds
            this once and passes it into every call so it isn't rebuilt identically
            5 times per pipeline run. If None, a tree is built internally here
            (kept for callers that use this method standalone).

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
        # pcd_down.orient_normals_consistent_tangent_plane(30)

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
        if tree is None:
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
    # Named wall-detection wrappers (used by run_processing_pipeline)
    # ------------------------------
    # These only detect a wall plane (center + normal) - they don't remove
    # anything themselves; the actual crop/removal happens once at the end
    # of run_processing_pipeline() via self.crop(). Each wrapper is just
    # _detect_wall_plane() pinned to the box/reference-plane for that one
    # wall. Box/threshold values default to the exact constants
    # this file used before they were parameterized (see runtime.yaml's
    # `tunnel_processing:` section) - so behavior is unchanged.
    def detect_ground(self, tree: cKDTree = None):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            radius=_cfg("tunnel_processing", "plane_detection", "radius", default=0.15),
            reference_plane="xy",
            min_bound=_cfg("tunnel_processing", "boxes", "bottom", "min", default=(2.5, -5.0, -1.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "bottom", "max", default=(11.0, 5.0, 1.5)),
            tree=tree,
        )

    def detect_back_wall(self, tree: cKDTree = None):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            radius=_cfg("tunnel_processing", "plane_detection", "radius", default=0.15),
            reference_plane="yz",
            min_bound=_cfg("tunnel_processing", "boxes", "back", "min", default=(4.0, -5.0, 0.0)),
            max_bound=_cfg("tunnel_processing", "boxes", "back", "max", default=(11.0, 5.0, 8.0)),
            tree=tree,
        )

    def detect_right_wall(self, tree: cKDTree = None):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            radius=_cfg("tunnel_processing", "plane_detection", "radius", default=0.15),
            reference_plane="xz",
            min_bound=_cfg("tunnel_processing", "boxes", "right", "min", default=(2.5, -5.0, -0.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "right", "max", default=(8.0, 0.0, 8.0)),
            tree=tree,
        )

    def detect_left_wall(self, tree: cKDTree = None):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            radius=_cfg("tunnel_processing", "plane_detection", "radius", default=0.15),
            reference_plane="xz",
            min_bound=_cfg("tunnel_processing", "boxes", "left", "min", default=(2.5, 0.0, -0.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "left", "max", default=(8.0, 5.0, 8.0)),
            tree=tree,
        )

    def detect_front_wall(self, tree: cKDTree = None):
        return self._detect_wall_plane(
            self.pcd,
            normal_angle_threshold=_cfg("tunnel_processing", "plane_detection", "normal_angle_threshold", default=5),
            radius=_cfg("tunnel_processing", "plane_detection", "radius", default=0.15),
            reference_plane="yz",
            min_bound=_cfg("tunnel_processing", "boxes", "front", "min", default=(0, -5.0, -0.5)),
            max_bound=_cfg("tunnel_processing", "boxes", "front", "max", default=(3.5, 5.0, 8.0)),
            tree=tree,
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
        follow-up. left/right/front are not (yet) independently toggleable and
        always get detected+cropped when this runs, matching prior behavior.

        Ground is always detected regardless of `remove_ground`: besides its own
        Z bound, self.ground_plane_normal is also what self.crop() uses to
        auto-level the whole cloud before cropping on ANY axis - back/left/
        right/front's crops depend on that leveling too, not just ground's.
        `remove_ground=False` only drops the ground_center used to tighten the
        Z bound (falls back to the same default used when ground can't be
        detected at all), it does not skip detecting the normal.
        """

        # KNOWN ISSUE (not fixed in this pass - see
        # docs/plan/phase_12_tunnel_processing_cleanup.md): a ceiling/top box
        # ("top" in runtime.yaml's tunnel_processing.boxes) is configured but
        # no remove_*_wall() call uses it - maxbound[2] below stays a fixed
        # constant instead of a detected ceiling plane.

        # Build the KDTree over the full cloud once and share it across all
        # wall-plane detections below - they all query the same self.pcd, so
        # rebuilding it once per detection was pure waste.
        tree = cKDTree(self.pcd.points)

        _, _, ground_center, self.ground_plane_normal = self.detect_ground(tree)
        if not remove_ground:
            ground_center = None

        if remove_back_wall:
            _, _, back_center, self.back_plane_normal = self.detect_back_wall(tree)
        else:
            back_center, self.back_plane_normal = None, None

        _, _, right_center, self.right_wall_normal = self.detect_right_wall(tree)
        _, _, left_center, self.left_wall_normal = self.detect_left_wall(tree)
        _, _, front_center, self.front_wall_normal = self.detect_front_wall(tree)
        

        def safe_bound_value(center, idx, offset, default_center):
            if center is None:
                return default_center
            try:
                return center[idx] + offset
            except Exception as e:
                rospy.logwarn(f"[TunnelProcessing] safe_bound_value fallback for idx={idx}: {e}")
                return default_center
            
        # minbound = [2.1, right_center[1]-0.3, ground_center[2]+0.3]
        # maxbound = [back_center[0]-0.2, left_center[1]+0.3, 6.2]
        minbound = [
            safe_bound_value(front_center, 0, -0.5, 0.5),
            safe_bound_value(right_center, 1, -1.0, -5.0),   # fallback khi right_center None
            safe_bound_value(ground_center, 2, +0.3, 0.0)
        ]


        maxbound = [
            safe_bound_value(back_center, 0, -0.1, 10.0),
            safe_bound_value(left_center, 1, +1.0, 5.0),
            15.0
        ]

        print(minbound, maxbound)
        cloud = self.crop(pcd=self.pcd, min_bound=minbound, max_bound=maxbound, normal=self.ground_plane_normal)

        return cloud
