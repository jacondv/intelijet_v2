#KeypointMatcher
import cv2
import numpy as np
import open3d as o3d
import rospy
import random
import cv2

from pps.image_processing.keypoint_project import KeyPointProject

def apply_clahe(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(gray)

def normalize_brightness(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    return cv2.equalizeHist(gray)

import numpy as np
import open3d as o3d
import cv2
import copy


class KeypointCloudExtractor:
    """
    This class is responsible for extracting pointclouds based on keypoints in the image. 
    It helps to create sub-point clouds containing points near the keypoints.
    This facilitates the alignment of point clouds between prescan and postscan.
    """

    def __init__(self, K, T_cl, dist_coeffs=None):
        """
        K: Ma trận nội tại camera (3x3)
        T_cl: Ma trận ngoại tại (4x4) từ LiDAR sang Camera
        dist_coeffs: Hệ số méo (tuỳ chọn)
        """
        self.K = K
        self.T_cl = T_cl
        self.dist_coeffs = dist_coeffs
        self.image_points = None  # Lưu các điểm đã được chiếu

    def extract(self, cloud, image, keypoints_uv, pixel_radius=5,cloud_radius=0.3):
        """
        cloud: open3d.geometry.PointCloud (trong hệ LiDAR)
        image: ảnh đầu vào (chỉ để lấy kích thước)
        keypoints_uv: (M, 2) danh sách tọa độ keypoint trong ảnh (u, v)
        pixel_radius: bán kính xét gần keypoint (pixel)

        Trả về:
            cloud_crop: point cloud con gồm các điểm gần keypoint sau khi chiếu
        """
        cloud_cam = self._transform_cloud_to_camera(cloud)
        # cloud_crop = self._extract_near_keypoints(cloud_cam, keypoints_uv, image.shape[:2], pixel_radius)
        cloud_crop = self._extract_near_keypoints_with_normal_region(cloud, cloud_cam, keypoints_uv, image.shape[:2], pixel_radius, cloud_radius=cloud_radius)
        return cloud_crop

    def _transform_cloud_to_camera(self, cloud):
        """
        Chuyển point cloud từ LiDAR → Camera
        Trả về: numpy array (N, 3) trong hệ Camera
        """
        if cloud is None:
            raise ValueError("_transform_cloud_to_camera: Input cloud is None.")

        if not hasattr(cloud, 'points'):
            raise TypeError("_transform_cloud_to_camera: Input cloud has no 'points' attribute.")
        
        cloud_np = np.asarray(cloud.points)  # (N, 3)

        if cloud_np.ndim != 2 or cloud_np.shape[1] != 3:
            raise ValueError(f"_transform_cloud_to_camera: Invalid shape {cloud_np.shape}. Expected (N, 3).")

        if self.T_cl is None:
            raise ValueError("_transform_cloud_to_camera: Transformation matrix T_cl is not set.")

        if not isinstance(self.T_cl, np.ndarray) or self.T_cl.shape != (4, 4):
            raise ValueError(f"_transform_cloud_to_camera: T_cl must be (4, 4) numpy array, got {type(self.T_cl)} {self.T_cl.shape}")
        
        N = cloud_np.shape[0]
        cloud_homo = np.hstack([cloud_np, np.ones((N, 1))])  # (N, 4)
        cloud_cam = (self.T_cl @ cloud_homo.T).T[:, :3]  # (N, 3)

        return cloud_cam
    
     
    def _extract_near_keypoints_with_normal_region(self, cloud: o3d.geometry.PointCloud, 
                                                cloud_cam: np.ndarray,
                                                keypoints_uv: np.ndarray,
                                                image_shape: tuple,
                                                pixel_radius: int,
                                                cloud_radius: float):
        """
        Tìm các điểm gần keypoints trong ảnh, tra cứu lại trong cloud gốc để:
        - Lấy điểm gần nhất tương ứng mỗi keypoint
        - Lấy normal tại điểm đó
        - Trích các điểm xung quanh bán kính R trong không gian 3D
        
        Trả về: pointcloud tổng hợp các điểm nằm trong bán kính R quanh keypoints
        """

        # ===== VALIDATION tương tự đoạn trước (có thể tái sử dụng) =====

        if not cloud.has_normals():
            cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
      
        # Valid poitn is the point that have z>0, since z is the forward axis of the camera
        idx_valid = np.where(
                    (cloud_cam[:, 2] > 0) &
                    np.isfinite(cloud_cam[:, 0]) &
                    np.isfinite(cloud_cam[:, 1]) &
                    np.isfinite(cloud_cam[:, 2])
                )[0]
        x, y, z = cloud_cam[idx_valid, 0], cloud_cam[idx_valid, 1], cloud_cam[idx_valid, 2]
        
        # u,v are the coordinates of the points in the cloud projected onto the image
        u = (fx * x / z + cx).astype(np.int32)
        v = (fy * y / z + cy).astype(np.int32)

        H, W = image_shape
        inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)

        u_inside = u[inside]
        v_inside = v[inside]
        idx_inside = idx_valid[inside]

        # KDTree for spatial search
        pcd_tree = o3d.geometry.KDTreeFlann(cloud)

        collected_indices = set()
        keypoint3d = []
        for kp in keypoints_uv:
            # Tìm điểm 2D gần nhất với keypoint (u,v)
            du = u_inside - kp[0]
            dv = v_inside - kp[1]
            dist = np.sqrt(du ** 2 + dv ** 2)
            if len(dist) == 0:# or np.min(dist) > pixel_radius:
                continue  # Bỏ qua nếu không có điểm nào gần
            nearest_idx = np.argmin(dist)
            point_idx = idx_inside[nearest_idx]  # Chỉ số trong cloud gốc
            point_xyz = np.asarray(cloud.points)[point_idx]
            keypoint3d.append(point_xyz)

            # Lấy các điểm nằm trong bán kính R quanh điểm đó
            [_, idx_neighbors, _] = pcd_tree.search_radius_vector_3d(point_xyz, cloud_radius)
            collected_indices.update(idx_neighbors)

        collected_indices = list(collected_indices)

        cloud_crop = o3d.geometry.PointCloud()
        cloud_crop.points = o3d.utility.Vector3dVector(np.asarray(cloud.points)[collected_indices])
        cloud_crop.normals = o3d.utility.Vector3dVector(np.asarray(cloud.normals)[collected_indices])

        return cloud_crop, keypoint3d


class KeypointCloudAlignManager:
    def __init__(self, camera_intrinsics, 
                 lidar_to_cam_extrinsic, 
                 dist_coeffs=None,
                 feature_method="SIFT", 
                 pixel_radius=5, 
                 cloud_radius=0.3,
                 match_ratio=0.5):
        
        self.cloud1 = None
        self.image1 = None
        self.kp1 = None
        self.desc1 = None
        self.__pts1 = None

        self.cloud2 = None
        self.image2 = None
        self.kp2 = None
        self.desc2 = None
        self.__pts2 = None

        self.pixel_radius = pixel_radius
        self.cloud_radius = cloud_radius
        self.match_ratio = match_ratio

        self.__camera_intrinsics = camera_intrinsics
        self.__lidar_to_cam_extrinsic = lidar_to_cam_extrinsic
        self.__dist_coeffs = dist_coeffs
        self.__feature_method = feature_method

        self.__croped_cloud1 = None
        self.__croped_cloud2 = None
        self.__good_matched = None

        # self.matcher = KeypointMatcher(method=self.__feature_method)
        from ai_core_pkg.image_matcher_client  import call_matcher_service
        self.matcher = call_matcher_service
        
        self.__process_status = 0 # 0: ready, 1: busy, 2: Done
        self._T = np.eye(4,dtype=np.float64)

        self.result_image = None

    @property
    def process_status(self):
        return self.__process_status
    

    def set_cloud1(self, cloud):
        self.cloud1 = cloud
        # self._try_process()


    def set_image1(self, image):
        self.image1 = image
        # self.kp1, self.desc1 = self.matcher.extract_keypoints_and_descriptors(self.image1)
        # self._try_process()


    def set_cloud2(self, cloud):
        self.cloud2 = cloud
        self._try_process()


    def set_image2(self, image):
        # self.image2 = FisheyeUndistorter(K=self.__camera_intrinsics,D=self.__dist_coeffs).undistort(image)
        self.image2 = image
        # self.kp2, self.desc2 = self.matcher.extract_keypoints_and_descriptors(self.image2)

        self._try_process()


    def get_result(self):
        """
        Trả về các point cloud đã crop theo keypoints và ma trận chuyển đổi
        """
        return self.__croped_cloud1, self.__croped_cloud2, self._T


    def draw_result(self):
        import cv2

        image1 = self.image1
        image2 = self.image2
        kp1=self.__pts1
        kp2=self.__pts2

        assert len(kp1) == len(kp2)
        
        # ---- convert kp array -> cv2.KeyPoint ----
        kp0_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kp1]
        kp1_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kp2]
        # ---- tạo match 1-1 giả ----
        matches = [
            cv2.DMatch(_queryIdx=i, _trainIdx=i, _distance=0)
            for i in range(len(kp1))
        ]

        vis = cv2.drawMatches(
            image1, kp0_cv,
            image2, kp1_cv,
            matches, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        return vis
    

    def is_ready(self):
        """
        Kiểm tra xem đã nhận đủ dữ liệu để xử lý hay chưa.
        """
        return (self.cloud1 is not None and 
                self.cloud2 is not None)
    

    def _try_process(self):
        self.__process_status = 0
        if self.is_ready():
            print("All data is ready")
            self.__process_status = 1
            self.__croped_cloud1, self.__croped_cloud2 = self.__process()
            self.__process_status = 2
            # self.result_image = self.draw_result()

    
    def __process(self):

        # self.__good_matched = self.matcher.match_keypoints_by_proximity(kp1 = self.kp1, des1=self.desc1, 
        #                         kp2=self.kp2, des2=self.desc2, 
        #                         ratio_test=self.match_ratio,
        #                         n_best=1000,
        #                         max_pixel_dist=50
        #                     )
        # good_matches = self.__good_matched  

        # if len(good_matches) == 0:
        #    return self.cloud1, self.cloud2

        keypoint_project1 = KeyPointProject()
        keypoint_project2 = KeyPointProject()

        self.image1, _, _ = keypoint_project1.cloud_to_image(self.cloud1, rot_x=-90, rot_y=90, rot_z=0)
        self.image2, _, _ = keypoint_project2.cloud_to_image(self.cloud2, rot_x=-90, rot_y=90, rot_z=0)

        
        self.__pts1, self.__pts2, self.__good_matched = self.matcher(self.image1, self.image2)
        idx = np.random.choice(len(self.__pts1), 300, replace=True)
        # ---- chọn theo index ----
        print(f"Detect {len(idx)} markers")
        self.__pts1 = self.__pts1[idx]
        self.__pts2 = self.__pts2[idx]

        # cloud1_crop = keypoint_project1.keypoints_to_cloud(self.cloud1, self.__pts1, threshold=5, rot_x=-90, rot_y=90, rot_z=0)
        cloud2_crop = keypoint_project2.keypoints_to_cloud(self.cloud2, self.__pts2, threshold=5, rot_x=-90, rot_y=90, rot_z=0)

        # self._T = self._compute_transform_matrix(source_points=keypoint3d_of_cloud2, target_points=keypoint3d_of_cloud1)
        self._T = None
        cloud1_crop = None
        return cloud1_crop, cloud2_crop
    
    def _compute_transform_matrix(self, source_points, target_points, max_iterations=100, distance_threshold=0.05, ransac_point=5):
        """
        RANSAC-based estimation of transform matrix with rotation clustering to remove outlier transforms.
        """
        source_points = np.array(source_points)
        target_points = np.array(target_points)

        assert source_points.shape == target_points.shape
        n = source_points.shape[0]

        candidates = []  # Danh sách các ứng viên: (T, inlier_count)

        for i in range(max_iterations):
            idx = random.sample(range(n), 5)
            src_sample = source_points[idx]
            tgt_sample = target_points[idx]

            T_candidate = self._estimate_transform(src_sample, tgt_sample)

            src_transformed = (T_candidate[:3, :3] @ source_points.T).T + T_candidate[:3, 3]
            errors = np.linalg.norm(src_transformed - target_points, axis=1)

            inliers = errors < distance_threshold
            inlier_count = np.sum(inliers)

            if inlier_count > 0:
                candidates.append((T_candidate, inlier_count))

        # rospy.logwarn(f"[RANSAC] Total valid candidates: {len(candidates)}")

        if not candidates:
            return None  # Không có transform nào hợp lệ

        # Tính hướng quay (rotation vector) từ ma trận xoay
        def rotation_angle(R):
            angle = np.arccos((np.trace(R) - 1) / 2)
            return angle if not np.isnan(angle) else 0

        # Tính độ lệch giữa các ma trận xoay
        angles = [cv2.Rodrigues(T[0][:3, :3])[0].flatten() for T in candidates]

        # Gom nhóm các ma trận có hướng quay gần nhau
        grouped = []
        used = [False] * len(angles)
        angle_threshold = np.deg2rad(10)  # ngưỡng 10 độ

        for i, a1 in enumerate(angles):
            if used[i]:
                continue
            group = [i]
            used[i] = True
            for j, a2 in enumerate(angles):
                if not used[j]:
                    if np.linalg.norm(a1 - a2) < angle_threshold:
                        group.append(j)
                        used[j] = True
            grouped.append(group)

        # Chọn cụm lớn nhất
        largest_group = max(grouped, key=len)
        # rospy.logwarn(f"[RANSAC] Largest rotation group size: {len(largest_group)}")

        # Chọn transform có inlier cao nhất trong cụm lớn nhất
        best_idx = max(largest_group, key=lambda i: candidates[i][1])
        best_T = candidates[best_idx][0]
        # rospy.logwarn(f"[RANSAC] Best inlier count from group: {candidates[best_idx][1]}")

        return best_T


    def _estimate_transform(self, source_points, target_points):
        """
        Classic SVD-based transform estimation from two Nx3 arrays
        """
        source_points = np.array(source_points)
        target_points = np.array(target_points)

        centroid_src = np.mean(source_points, axis=0)
        centroid_tgt = np.mean(target_points, axis=0)

        src_centered = source_points - centroid_src
        tgt_centered = target_points - centroid_tgt

        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        t = centroid_tgt - R @ centroid_src

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T


