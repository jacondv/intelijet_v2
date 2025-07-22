#KeypointMatcher
import cv2
import numpy as np
import open3d as o3d
import rospy
import random
import cv2

def apply_clahe(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(gray)

def normalize_brightness(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    return cv2.equalizeHist(gray)

class KeypointMatcher:
    
    """
    matcher = KeypointMatcher(method='SIFT')
    kp1, desc1 = matcher.extract_keypoints_and_descriptors(img1)
    kp2, desc2 = matcher.extract_keypoints_and_descriptors(img2)
    matches = matcher.match_keypoints(desc1, desc2, ratio_test=0.4)
    print(matches)
    img_result = matcher.draw_matches(img1, kp1, img2, kp2, matches)

    cv2.namedWindow("Matches", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Matches", 400, 300)

    cv2.imshow("Matches", img_result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """
    
    def __init__(self, method='ORB'):
        self.T = np.eye(4, dtype=np.float64)
        self.method = method.upper()
        self.detector, self.norm_type = self._init_detector(self.method)

    def _init_detector(self, method):
        if method == 'SIFT':
            return cv2.SIFT_create(), cv2.NORM_L2
        elif method == 'ORB':
            return cv2.ORB_create(), cv2.NORM_HAMMING
        elif method == 'AKAZE':
            return cv2.AKAZE_create(), cv2.NORM_HAMMING
        elif method == 'BRISK':
            return cv2.BRISK_create(), cv2.NORM_HAMMING
        else:
            raise ValueError(f"[ERROR] Unsupported method: {method}")
            
    def extract_keypoints_and_descriptors(self, image):
        """
        Trích xuất keypoints và descriptors từ ảnh.
        """
        image = apply_clahe(image)
        image = normalize_brightness(image)
        keypoints, descriptors = self.detector.detectAndCompute(image, None)
        return keypoints, descriptors

    def match_keypoints(self,kp1, desc1, kp2, desc2, ratio_test=0.5, K=np.eye(3,3)):
        """
        Dùng BFMatcher + Lowe's ratio test để khớp các descriptor.
        """

        # ====== VALIDATION ======
        if desc1 is None or desc2 is None:
            raise ValueError("Descriptors must not be None.")

        if not isinstance(desc1, np.ndarray) or not isinstance(desc2, np.ndarray):
            raise TypeError("Descriptors must be numpy arrays.")

        if desc1.ndim != 2 or desc2.ndim != 2:
            raise ValueError(f"Descriptors must be 2D arrays, got {desc1.ndim} and {desc2.ndim} dimensions.")

        if desc1.shape[1] != desc2.shape[1]:
            raise ValueError(f"Descriptor dimensions must match. Got {desc1.shape[1]} vs {desc2.shape[1]}.")

        if len(desc1) < 2 or len(desc2) < 2:
            raise ValueError("Need at least 2 descriptors in each set to perform knnMatch(k=2).")

        if not (0 < ratio_test < 1):
            raise ValueError("ratio_test must be in (0, 1)")

        # Tạo một matcher dùng Brute-Force (BFMatcher) với loại khoảng cách (norm) được chỉ định
        matcher = cv2.BFMatcher(self.norm_type)

        # So khớp các descriptor giữa desc1 và desc2, lấy 2 match gần nhất cho mỗi descriptor trong desc1
        raw_matches = matcher.knnMatch(desc1, desc2, k=2)

        good_matches = []
        for match in raw_matches:
            if len(match) < 2:
                continue
            m,n = match 
            # m,n is best match and second best match
            # Nếu khoảng cách giữa m và n càng lớn chứng tỏ m là tốt vì nó mang đặt trưng khá riêg
            if m.distance < ratio_test * n.distance:
                good_matches.append(m)
                
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        # Tính Essential matrix + mask lọc inliers


        E, mask = cv2.findEssentialMat(src_pts, dst_pts, K[:3, :3], method=cv2.RANSAC, threshold=1.0)

        # Tính pose từ E (R, t) và lọc inliers
        _, R, t, mask_pose = cv2.recoverPose(E, src_pts, dst_pts, K[:3, :3])

        T = np.eye(4, dtype=np.float64)     # Khởi tạo ma trận 4x4 đơn vị
        T[:3, :3] = R                        # Gán phần xoay
        T[:3, 3] = t.ravel()                 # Gán phần tịnh tiến (dạng (3,))
        self.T=np.linalg.inv(T)

        # (Tuỳ chọn) chỉ lấy các điểm inlier
        inlier_matches = [m for i, m in enumerate(good_matches) if mask_pose[i]]
        return inlier_matches
    
    
    def match_keypoints_by_region(self,kp1, des1, kp2, des2, ratio_test=0.75, n_best=2, K=np.eye(3,3),image_shape=None):
        h, w = image_shape[:2]
        
        region_filters = {
            "top_left":     lambda pt: pt[0] < w // 2 and pt[1] < h // 2,
            "top_right":    lambda pt: pt[0] >= w // 2 and pt[1] < h // 2,
            "bottom_left":  lambda pt: pt[0] < w // 2 and pt[1] >= h // 2,
            "bottom_right": lambda pt: pt[0] >= w // 2 and pt[1] >= h // 2,
        }

        good_matches_all = []

        for region_name, condition in region_filters.items():
            idxs = [i for i, kp in enumerate(kp1) if condition(kp.pt)]
            if not idxs:
                continue
            des1_sub = des1[idxs]
            matcher = cv2.BFMatcher(self.norm_type)
            raw_matches = matcher.knnMatch(des1_sub, des2, k=2)

            good = []
            for i, match in enumerate(raw_matches):
                if len(match) < 2:
                    continue

                m, n = match
                if m.distance < ratio_test * n.distance:
                    # print(m.distance, ratio_test*n.distance)

                    m.queryIdx = idxs[m.queryIdx]  # map về index gốc
                    good.append(m)

            good = sorted(good, key=lambda m: m.distance)[:n_best]
            good_matches_all.extend(good)

        # ----- Tính Essential matrix và lọc inliers -----
        if len(good_matches_all) < 5:
            return []  # Không đủ để tính E

        # src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches_all]).reshape(-1, 1, 2)
        # dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches_all]).reshape(-1, 1, 2)

        # E, mask = cv2.findEssentialMat(src_pts, dst_pts, K[:3, :3], method=cv2.RANSAC, threshold=1.0)
        # if E is None or mask is None:
        #     return []

        # _, R, t, mask_pose = cv2.recoverPose(E, src_pts, dst_pts, K[:3, :3])
        # T = np.eye(4)
        # T[:3, :3] = R
        # T[:3, 3] = t.ravel()

        # Lưu T nếu cần dùng lại
        # self.T = np.linalg.inv(T)  # nếu trong class

        # Lọc ra inlier matches
        # inlier_matches = [m for i, m in enumerate(good_matches_all) if mask_pose[i]]
        return good_matches_all


    def draw_good_matches(self,img1, kp1, img2, kp2, good_matches, max_matches=5000):
        """
        Vẽ các good_matches giữa hai ảnh.

        Args:
            img1 (np.ndarray): Ảnh thứ nhất.
            kp1 (list): Danh sách keypoints của ảnh 1.
            img2 (np.ndarray): Ảnh thứ hai.
            kp2 (list): Danh sách keypoints của ảnh 2.
            good_matches (list): Danh sách các cv2.DMatch đã lọc.
            max_matches (int): Số lượng match tối đa để vẽ.

        Returns:
            img_out (np.ndarray): Ảnh đã vẽ match.
        """
        img_out = cv2.drawMatches(
            img1, kp1, img2, kp2,
            good_matches[:max_matches],
            None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        return img_out
    
    def get_matched_keypoint_coords(self, kp1, kp2, matches):
        """
        Trả về tọa độ (x, y) của các keypoint được matched giữa hai ảnh.
        """

        # ===== VALIDATION =====
        if not isinstance(kp1, (list, tuple)) or not all(isinstance(k, cv2.KeyPoint) for k in kp1):
            raise TypeError("kp1 must be a list of cv2.KeyPoint")
        if not isinstance(kp2, (list, tuple)) or not all(isinstance(k, cv2.KeyPoint) for k in kp2):
            raise TypeError("kp2 must be a list of cv2.KeyPoint")
        if not isinstance(matches, (list, tuple)) or not all(hasattr(m, 'queryIdx') and hasattr(m, 'trainIdx') for m in matches):
            raise TypeError("matches must be a list of cv2.DMatch with valid indices")

        if len(matches) == 0:
            raise ValueError("No matches provided.")

        max_qidx = max(m.queryIdx for m in matches)
        max_tidx = max(m.trainIdx for m in matches)
        if max_qidx >= len(kp1) or max_tidx >= len(kp2):
            raise IndexError("Match index out of bounds for provided keypoints.")
        
        # ===== EXTRACT MATCHED COORDINATES =====
        pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float32)
        pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float32)
        return pts1, pts2


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
    
     
    # def _extract_near_keypoints(self, cloud_cam, keypoints_uv, image_shape, pixel_radius):
    #     """
    #     Chiếu point cloud lên ảnh và lấy điểm gần keypoints
    #     Trả về: open3d.geometry.PointCloud gồm các điểm gần keypoints
    #     """
    #     # ====== VALIDATION ======
    #     if cloud_cam is None or not isinstance(cloud_cam, np.ndarray) or cloud_cam.ndim != 2 or cloud_cam.shape[1] != 3:
    #         raise ValueError("cloud_cam must be a numpy array of shape (N, 3)")

    #     if keypoints_uv is None or not isinstance(keypoints_uv, np.ndarray) or keypoints_uv.ndim != 2 or keypoints_uv.shape[1] != 2:
    #         raise ValueError("keypoints_uv must be a numpy array of shape (K, 2)")

    #     if not isinstance(image_shape, (tuple, list)) or len(image_shape) != 2:
    #         raise ValueError("image_shape must be a tuple (H, W)")

    #     if not isinstance(pixel_radius, (int, float)) or pixel_radius <= 0:
    #         raise ValueError("pixel_radius must be a positive number")

    #     if not hasattr(self, 'K') or self.K is None or self.K.shape not in [(3, 3), (4, 4)]:
    #         raise ValueError("Intrinsic matrix self.K must be set and have shape (3, 3)")

    #     fx, fy = self.K[0, 0], self.K[1, 1]
    #     cx, cy = self.K[0, 2], self.K[1, 2]

    #     x = cloud_cam[:, 0]
    #     y = cloud_cam[:, 1]
    #     z = cloud_cam[:, 2]

    #     valid = z > 0
    #     x, y, z = x[valid], y[valid], z[valid]
    #     idx_valid = np.where(valid)[0]

    #     u = (fx * x / z + cx).astype(np.int32)
    #     v = (fy * y / z + cy).astype(np.int32)
    #     self.image_points = np.stack([u, v], axis=1)  # lưu lại vị trí ảnh

    #     H, W = image_shape
    #     inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)

    #     cloud_final = []
    #     for kp in keypoints_uv:
    #         du = u[inside] - kp[0]
    #         dv = v[inside] - kp[1]
    #         dist = np.sqrt(du ** 2 + dv ** 2)
    #         close_mask = dist < pixel_radius
    #         indices = idx_valid[inside][close_mask]
    #         cloud_final.extend(indices)

    #     cloud_final = np.unique(cloud_final)

    #     # Tạo point cloud mới từ chỉ số được chọn
    #     cloud_crop = o3d.geometry.PointCloud()
    #     cloud_crop.points = o3d.utility.Vector3dVector(cloud_cam[cloud_final])

    #     return cloud_crop

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


class FisheyeUndistorter:
    def __init__(self, K: np.ndarray, D: np.ndarray):
        """
        K: Ma trận nội tại camera (3x3 hoặc 4x4)
        D: Distortion coefficients (4x1 hoặc 4,)
        """
        # Tự động cắt K nếu là 4x4
        if K.shape == (4, 4):
            K = K[:3, :3]
        elif K.shape != (3, 3):
            raise ValueError("K phải là ma trận 3x3 hoặc 4x4")

        self.K = K.astype(np.float64)
        self.D = D.astype(np.float64).reshape(-1)

        # Kiểm tra distortion fisheye
        if self.D.size != 4:
            raise ValueError("Distortion fisheye phải có đúng 4 phần tử")

        self.map1 = None
        self.map2 = None
        self.image_size = None

    def undistort(self, image: np.ndarray) -> np.ndarray:
        """
        Undistort ảnh đầu vào. Tự tạo lại map nếu kích thước ảnh thay đổi.
        """
        h, w = image.shape[:2]
        image_size = (w, h)

        # Nếu chưa tạo map hoặc ảnh thay đổi size
        if self.map1 is None or image_size != self.image_size:
            self.image_size = image_size
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3), self.K, image_size, cv2.CV_16SC2
            )

        return cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)


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

        self.matcher = KeypointMatcher(method=self.__feature_method)
        self.extractor = KeypointCloudExtractor(self.__camera_intrinsics, 
                                                self.__lidar_to_cam_extrinsic, 
                                                np.zeros_like(self.__dist_coeffs))
        
        self.__process_status = 0 # 0: ready, 1: busy, 2: Done
        self._T = np.eye(4,dtype=np.float64)

    @property
    def process_status(self):
        return self.__process_status
    

    def set_cloud1(self, cloud):
        self.cloud1 = cloud
        self._try_process()


    def set_image1(self, image):
        fisheye_tool = FisheyeUndistorter(K=self.__camera_intrinsics, D=self.__dist_coeffs)
        self.image1 = fisheye_tool.undistort(image=image)
        self.kp1, self.desc1 = self.matcher.extract_keypoints_and_descriptors(self.image1)
        self._try_process()


    def set_cloud2(self, cloud):
        self.cloud2 = cloud
        self._try_process()


    def set_image2(self, image):
        self.image2 = FisheyeUndistorter(K=self.__camera_intrinsics,D=self.__dist_coeffs).undistort(image)
        self.kp2, self.desc2 = self.matcher.extract_keypoints_and_descriptors(self.image2)

        # if self.image1 is not None:
        #     self.__good_matched = self.matcher.match_keypoints_by_region(kp1 = self.kp1, des1=self.desc1, 
        #                                         kp2=self.kp2, des2=self.desc2, 
        #                                         ratio_test=self.match_ratio,
        #                                         K=self.__camera_intrinsics,
        #                                         image_shape=self.image1.shape)
            
        self._try_process()


    def get_result(self):
        """
        Trả về các point cloud đã crop theo keypoints và ma trận chuyển đổi
        """
        return self.__croped_cloud1, self.__croped_cloud2, self._T


    def draw_result(self):
        """
        Vẽ các good_matches giữa hai ảnh.

        Args:
            img1 (np.ndarray): Ảnh thứ nhất.
            kp1 (list): Danh sách keypoints của ảnh 1.
            img2 (np.ndarray): Ảnh thứ hai.
            kp2 (list): Danh sách keypoints của ảnh 2.
            good_matches (list): Danh sách các cv2.DMatch đã lọc.
            max_matches (int): Số lượng match tối đa để vẽ.

        Returns:
            img_out (np.ndarray): Ảnh đã vẽ match.
        """
        print(f"Draw {len(self.__good_matched)} matched point")
        img1 = self.image1; img2=self.image2; kp1 = self.kp1; kp2 = self.kp2; 
        img_out = cv2.drawMatches(
            img1, 
            kp1, 
            img2, 
            kp2,
            self.__good_matched,
            None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        return img_out
    

    def is_ready(self):
        """
        Kiểm tra xem đã nhận đủ dữ liệu để xử lý hay chưa.
        """
        return (self.image1 is not None and self.kp1 is not None and self.desc1 is not None and
                self.image2 is not None and self.kp2 is not None and self.desc2 is not None and
                self.cloud1 is not None and self.cloud2 is not None)
    

    def _try_process(self):
        self.__process_status = 0
        if self.is_ready():
            print("All data is ready")
            self.__process_status = 1
            self.__croped_cloud1, self.__croped_cloud2 = self.__process()
            self.__process_status = 2

    
    def __process(self):

            # good_matches = self.matcher.match_keypoints(self.kp1, self.desc1, 
            #                                             self.kp2, self.desc2, 
            #                                             ratio_test=self.match_ratio,
            #                                             K=self.__camera_intrinsics)

        self.__good_matched = self.matcher.match_keypoints_by_region(kp1 = self.kp1, des1=self.desc1, 
                                kp2=self.kp2, des2=self.desc2, 
                                ratio_test=self.match_ratio,
                                n_best=5,
                                K=self.__camera_intrinsics,
                                image_shape=self.image1.shape,
                            )
        good_matches = self.__good_matched  

        if len(good_matches) == 0:
           return self.cloud1, self.cloud2

        self.__pts1, self.__pts2 = self.matcher.get_matched_keypoint_coords(self.kp1, self.kp2, good_matches)

        cloud1_crop, keypoint3d_of_cloud1 = self.extractor.extract(self.cloud1, self.image1, self.__pts1, pixel_radius=self.pixel_radius,cloud_radius=self.cloud_radius)
        cloud2_crop, keypoint3d_of_cloud2 = self.extractor.extract(self.cloud2, self.image2, self.__pts2, pixel_radius=self.pixel_radius,cloud_radius=self.cloud_radius)
        self._T = self._compute_transform_matrix(source_points=keypoint3d_of_cloud2, target_points=keypoint3d_of_cloud1)
        return cloud1_crop, cloud2_crop
    
    def _compute_transform_matrix(self, source_points, target_points, max_iterations=100, distance_threshold=0.05):
        """
        source_points, target_points: list or Nx3 array of 3D keypoints (matched)
        Returns: Best transform T (4x4)
        """
        source_points = np.array(source_points)
        target_points = np.array(target_points)

        rospy.logwarn(f"[RANSAC] source_points shape: {source_points.shape}, target_points shape: {target_points.shape}")
        assert source_points.shape == target_points.shape
        n = source_points.shape[0]

        best_inlier_count = 0
        best_T = None

        for i in range(max_iterations):
            # Chọn ngẫu nhiên 3 điểm (3 là tối thiểu để định nghĩa transform 3D)
            idx = random.sample(range(n), 5)
            src_sample = source_points[idx]
            tgt_sample = target_points[idx]

            # Tính transform từ mẫu
            T_candidate = self._estimate_transform(src_sample, tgt_sample)

            # Áp dụng transform
            src_transformed = (T_candidate[:3, :3] @ source_points.T).T + T_candidate[:3, 3]

            # Tính sai số từng điểm
            errors = np.linalg.norm(src_transformed - target_points, axis=1)

            # Đếm số inliers
            inliers = errors < distance_threshold
            inlier_count = np.sum(inliers)

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_T = T_candidate
                rospy.logwarn(f"[RANSAC] Iter {i}: New best inlier count = {inlier_count}")

        rospy.logwarn(f"[RANSAC] Best inlier count: {best_inlier_count}/{n}")
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

class CalibrationVisualizer: 
    """
    This class just for test intrinsic and extrinsic parameters.
    """

    def __init__(self, camera_intrinsics, lidar_to_cam_extrinsic,dist_coeffs=None):
        """
        camera_intrinsics: (3, 3) hoặc (4, 4)
        lidar_to_cam_extrinsic: (4, 4)
        """
        self.K = camera_intrinsics[:3, :3]  # bỏ dòng cuối nếu (4,4)
        self.T_cl = lidar_to_cam_extrinsic
        self.dist_coeffs = dist_coeffs

    def transform_lidar_to_camera(self, cloud: o3d.geometry.PointCloud):
        cloud_np = np.asarray(cloud.points)  # (N, 3)
        N = cloud_np.shape[0]
        cloud_homo = np.hstack([cloud_np, np.ones((N, 1))])  # (N, 4)
        cloud_cam = (self.T_cl @ cloud_homo.T).T[:, :3]  # (N, 3)

        return cloud_cam

    def project_cloud_to_image(self,cloud, image, K, T_cl, dist=None, radius=2,thres=0):
        """
        Chiếu point cloud có màu lên ảnh RGB, có hỗ trợ distortion.

        Args:
            cloud (open3d.geometry.PointCloud): Point cloud đã load từ file.
            image (np.ndarray): Ảnh RGB (HxWx3).
            K (np.ndarray): Ma trận nội tại camera (3x3).
            T_cl (np.ndarray): Ma trận ngoại tại Lidar → Camera (4x4).
            dist (np.ndarray or None): Distortion coefficients (1x5 or 1x8), nếu có.
            radius (int): Bán kính điểm vẽ trên ảnh.

        Returns:
            image_overlay (np.ndarray): Ảnh đã vẽ các điểm từ point cloud.
        """
        if not cloud.has_colors():
            num_points = np.asarray(cloud.points).shape[0]
            c = np.tile([1.0,0,0], (num_points, 1))  # RGB = xám nhạt
            cloud.colors = o3d.utility.Vector3dVector(c)

        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        if colors.max() <= 1.0:
            colors = (colors * 255).astype(np.uint8)

        # Chuyển point cloud sang hệ Camera
        pts_hom = np.hstack([points, np.ones((points.shape[0], 1))])
        pts_cam = (T_cl @ pts_hom.T).T[:, :3]

        # Lọc điểm phía trước camera
        mask = pts_cam[:, 2] >thres
        pts_cam = pts_cam[mask]
        colors = colors[mask]

        # Dùng OpenCV để project (có hỗ trợ distortion)
        rvec = np.zeros((3, 1))  # vì điểm đã ở hệ camera → không cần xoay thêm
        tvec = np.zeros((3, 1))  # tương tự
        img_pts, _ = cv2.projectPoints(pts_cam, rvec, tvec, K, dist)
        # img_pts, _ = cv2.fisheye.projectPoints(pts_cam.reshape(1, -1, 3), rvec, tvec, K, dist)

        # Vẽ điểm lên ảnh
        img_pts = img_pts.reshape(-1, 2).astype(int)
        h, w = image.shape[:2]
        image_overlay = image.copy()
        for i in range(len(img_pts)):
            u, v = img_pts[i]
            if 0 <= u < w and 0 <= v < h:
                color = tuple(map(int, colors[i]))
                cv2.circle(image_overlay, (u, v), radius, color, -1)

        return image_overlay


    def visualize_projection(self, cloud, image, K, T_lc ,dist , radius=1, thres=-1):
        """
        Hiển thị ảnh + chiếu điểm cloud lên ảnh với màu RGB dựa trên (x, y, z)
        """
        
        result = self.project_cloud_to_image(cloud, image, K, T_lc,dist , radius=radius, thres=thres)
        plt.figure(figsize=(10, 10))
        plt.imshow(result)
        plt.axis('off')
        plt.show()
        print(result.shape)

# Example usage:# K = np.array([
#     [2242.45, 0.0, 1579.98],
#     [0.0, 2242.69, 1172.42],
#     [0.0, 0.0, 1.0]
# ])
# T_cl = np.array(...  # Ma trận ngoại tại từ LiDAR sang Camera

# align_manager = KeypointCloudAlignManager(K, T_cl)

# # Khi nhận dữ liệu từ ROS callback:
# align_manager.set_cloud1(cloud_pre)
# align_manager.set_image1(img_pre)
# align_manager.set_cloud2(cloud_post)
# align_manager.set_image2(img_post)

def draw_good_matches(img1, kp1, img2, kp2, good_matches, max_matches=50):
    """
    Vẽ các good_matches giữa hai ảnh.

    Args:
        img1 (np.ndarray): Ảnh thứ nhất.
        kp1 (list): Danh sách keypoints của ảnh 1.
        img2 (np.ndarray): Ảnh thứ hai.
        kp2 (list): Danh sách keypoints của ảnh 2.
        good_matches (list): Danh sách các cv2.DMatch đã lọc.
        max_matches (int): Số lượng match tối đa để vẽ.

    Returns:
        img_out (np.ndarray): Ảnh đã vẽ match.
    """
    img_out = cv2.drawMatches(
        img1, kp1, img2, kp2,
        good_matches[:max_matches],
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )
    return img_out
def match_and_draw_sift_keypoints(img1, img2, ratio_test=0.75, max_matches=50):
    """
    So sánh và vẽ các keypoint match giữa hai ảnh bằng SIFT + BFMatcher.

    Args:
        img1 (ndarray): Ảnh thứ nhất (grayscale hoặc BGR).
        img2 (ndarray): Ảnh thứ hai (grayscale hoặc BGR).
        ratio_test (float): Ngưỡng cho Lowe's ratio test.
        max_matches (int): Số match tối đa để vẽ.

    Returns:
        img_match (ndarray): Ảnh đã vẽ các match tốt.
        good_matches (list): Danh sách match tốt.
    """
    # Nếu ảnh là BGR, chuyển sang grayscale
    if len(img1.shape) == 3:
        img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    else:
        img1_gray = img1.copy()

    if len(img2.shape) == 3:
        img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    else:
        img2_gray = img2.copy()

    # Tạo SIFT detector
    sift = cv2.SIFT_create()
    kp1, desc1 = sift.detectAndCompute(img1_gray, None)
    kp2, desc2 = sift.detectAndCompute(img2_gray, None)

    # Tạo BFMatcher với L2 norm
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    raw_matches = matcher.knnMatch(desc1, desc2, k=2)

    # Lowe's ratio test + lọc match lỗi chỉ số
    good_matches = []
    for match in raw_matches:
        if len(match) < 2:
            continue
        m, n = match
        if m.distance < ratio_test * n.distance:
            if 0 <= m.queryIdx < len(kp1) and 0 <= m.trainIdx < len(kp2):
                good_matches.append(m)

    # Vẽ match
    img_match = cv2.drawMatches(
        img1, kp1, img2, kp2,
        good_matches[:max_matches],
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )

    return img_match, good_matches

def rotation_matrix_x(angle_deg):
    """
    Tạo ma trận 4x4 xoay quanh trục X với góc angle_deg (độ).
    """
    angle_rad = np.deg2rad(angle_deg)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)

    R = np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1]
    ], dtype=np.float64)

    return R

def rotation_matrix_z(angle_deg):
    """
    Tạo ma trận 4x4 xoay quanh trục X với góc angle_deg (độ).
    """
    angle_rad = np.deg2rad(angle_deg)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)

    R = np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,         0, 1, 0],
        [0,         0, 0, 1]
    ], dtype=np.float64)
    return R

def rotation_matrix_y(degree):
    theta = np.deg2rad(degree)  # Chuyển độ sang radian
    cos_a = np.cos(theta)
    sin_a = np.sin(theta)

    Ry = np.array([
        [ cos_a, 0, sin_a, 0],
        [     0, 1,     0, 0],
        [-sin_a, 0, cos_a, 0],
        [     0, 0,     0, 1]
    ], dtype=np.float64)

    return Ry


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


if __name__ == "__main__":
    import json
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation as R

    #IPC Param
    ICP_THRESHOLDS = [1.0, 0.1, 0.05]      # coarse → fine
    ICP_MAX_ITERS = [30, 40, 50]           # coarse → fine
    ICP_ALIGN_AREA = None                  # hoặc [[xmin, xmax], [ymin, ymax], [zmin, zmax]]

    # === Keypoint Matching Parameters ===
    CAMERA_INTRINSICS = np.array([
                            [653.76474515,     0.0,          756.55566752],
                            [0.0,              655.82709085, 541.23742774],
                            [0.0,              0.0,          1.0]
                        ], dtype=np.float64)
    
    # LIDAR_TO_CAM_EXTRINSIC = np.array([
    #     [1,  0,  0, 0],
    #     [0,  0, -1, 0],
    #     [0,  1,  0, 0],
    #     [0,  0,  0, 1]
    # ], dtype=np.float64)
    
    # từ base → camera front
    LIDAR_TO_CAM_EXTRINSIC =  np.array([
        [-0.98383629,  0.00314181, -0.17904252, -0.00723122],
        [ 0.17903614, -0.0022086,  -0.98383999, -0.04821113],
        [-0.00348647, -0.99999261,  0.00161041, -0.10187813],
        [ 0.        ,  0.        ,  0.        ,  1.        ]
    ])

    DIST_COEFFS = np.array([-0.06065661 , 0.04854407 ,-0.0606453  , 0.02428142])
    FEATURE_METHOD = "SIFT"
    PIXEL_RADIUS = 5
    MATCH_RATIO = 0.5

    image = cv2.imread(r'/mnt/c/work/projects/intelijet_v2/pano_rest_1_20250721_132024.png', cv2.IMREAD_COLOR_RGB)
    image2 = cv2.imread(r'/mnt/c/work/projects/intelijet_v2/pano_rest_1_20250721_132117.png', cv2.IMREAD_COLOR_RGB)

    cloud = o3d.io.read_point_cloud(r"/mnt/c/work/projects/intelijet_v2/data/cloud_20250721_132036.ply")
    cloud2 = o3d.io.read_point_cloud(r"/mnt/c/work/projects/intelijet_v2/data/cloud_20250721_132129.ply")

 
    keypoint_matcher = KeypointCloudAlignManager(camera_intrinsics=CAMERA_INTRINSICS,
                                                 dist_coeffs=DIST_COEFFS,
                                                 lidar_to_cam_extrinsic=LIDAR_TO_CAM_EXTRINSIC,
                                                 feature_method="SIFT",
                                                 pixel_radius=5,
                                                 cloud_radius=0.3,
                                                 match_ratio=0.5)
   
    # undistorted_img1 = FisheyeUndistorter(K=CAMERA_INTRINSICS,
    #                                      D=DIST_COEFFS).undistort(image=image)
    # undistorted_img2 = FisheyeUndistorter(K=CAMERA_INTRINSICS,
    #                                     D=DIST_COEFFS).undistort(image=image2)
    
    keypoint_matcher.set_image1(image)
    keypoint_matcher.set_image2(image2)
    keypoint_matcher.set_cloud1(cloud)
    keypoint_matcher.set_cloud2(cloud2)

    pcd1, pcd2, img_result = keypoint_matcher.get_result()

    # o3d.io.write_point_cloud("pcd1.ply", pcd1)
    # o3d.io.write_point_cloud("pcd2.ply", pcd2)

    plt.imshow(keypoint_matcher.draw_result())
    plt.axis("off")
    plt.show()

    # # # Undistort ảnh bằng mô hình fisheye
    # # map3, map4 = cv2.fisheye.initUndistortRectifyMap(
    # # CAMERA_INTRINSICS, DIST_COEFFS, np.eye(3), CAMERA_INTRINSICS, image_size, cv2.CV_16SC2)
    # # undistorted_img2 = cv2.remap(image2, map3, map4, interpolation=cv2.INTER_LINEAR)

    # # image_result, matches = match_and_draw_sift_keypoints(undistorted_img, undistorted_img2,ratio_test=0.1)

    # # # print(f"Found {len(matches)} good matches")

    # # plt.imshow(cv2.cvtColor(undistorted_img2, cv2.COLOR_BGR2RGB))
    # # plt.axis("off")
    # # plt.show()

    # # R_x_90 = np.array([
    # #         [1,  0,  0, 0],
    # #         [0,  0, -1, 0],
    # #         [0,  1,  0, 0],
    # #         [0,  0,  0, 1]
    # #     ], dtype=np.float64)
    
    # x = rotation_matrix_x(-0.5)
    # z = rotation_matrix_z(0)
    # y = rotation_matrix_y(-2)

    # p = crop(cloud, (-5,5), (-5,10), (0,10))
    # vis = CalibrationVisualizer(camera_intrinsics=CAMERA_INTRINSICS, lidar_to_cam_extrinsic=LIDAR_TO_CAM_EXTRINSIC,dist_coeffs=np.zeros((4, 1), dtype=np.float64))
    # vis.visualize_projection(cloud=p, 
    #                          image=keypoint_matcher.image1, 
    #                          K=CAMERA_INTRINSICS,
    #                          dist=np.zeros((4, 1),dtype=np.float64), 
    #                          T_lc=LIDAR_TO_CAM_EXTRINSIC@x@z@y,
    #                          radius=1, thres=0)

    
    # plt.imshow(cv2.cvtColor(keypoint_matcher.image1, cv2.COLOR_BGR2RGB))
    # plt.axis("off")
    # plt.show()

    # plt.imshow(cv2.cvtColor(img_result[0], cv2.COLOR_BGR2RGB))
    # plt.axis("off")
    # plt.show()    
    # plt.imshow(cv2.cvtColor(img_result[1], cv2.COLOR_BGR2RGB))
    # plt.axis("off")
    # plt.show()

