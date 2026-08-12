import numpy as np
import open3d as o3d
import cv2
import copy

class KeyPointProject:
    def __init__(self, width=800, height=800):
        self.width = width
        self.height = height
        self.intrinsic = None
        self.extrinsic = None
        self.img = None

 

    def rotate_cloud(self, pcd, rot_x=0, rot_y=0, rot_z=0, invert=False):
        """
        Xoay point cloud quanh các trục X,Y,Z theo góc độ (degree).
        Trả về một bản copy đã xoay, dữ liệu gốc không thay đổi.
        """
        pcd_copy = copy.deepcopy(pcd)

        Rx = pcd_copy.get_rotation_matrix_from_xyz(np.radians([rot_x, 0, 0]))
        Ry = pcd_copy.get_rotation_matrix_from_xyz(np.radians([0, rot_y, 0]))
        Rz = pcd_copy.get_rotation_matrix_from_xyz(np.radians([0, 0, rot_z]))

        R = Rz @ Ry @ Rx
        if invert:
            R = R.T
        pcd_copy.rotate(R, center=pcd_copy.get_center())

        return pcd_copy

    def cloud_to_image(self, pcd, rot_x=0, rot_y=0, rot_z=0):
        """
        Render point cloud thành ảnh 2D.
        Trả về (img, intrinsic, extrinsic).
        """
        pcd_copy = copy.deepcopy(pcd)
        pcd_copy = self.rotate_cloud(pcd_copy, rot_x=rot_x, rot_y=rot_y, rot_z=rot_z)
        
        if isinstance(pcd_copy, o3d.t.geometry.PointCloud):
            from pps.data_converter import cloudconverter
            pcd_copy = cloudconverter.tensor_to_o3d_legacy(pcd_copy)

        if not pcd_copy.has_normals():
            pcd_copy.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

        pcd_copy.paint_uniform_color([0.7, 0.7, 0.7])

        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=self.width, height=self.height)
        vis.add_geometry(pcd_copy)

        ctr = vis.get_view_control()
        param = ctr.convert_to_pinhole_camera_parameters()
        self.intrinsic = param.intrinsic.intrinsic_matrix.copy()
        self.extrinsic = np.asarray(param.extrinsic).copy()

        vis.poll_events()
        vis.update_renderer()

        img = np.asarray(vis.capture_screen_float_buffer(do_render=True))
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        vis.destroy_window()
        self.img = img
        return img, self.intrinsic, self.extrinsic


    def keypoints_to_cloud(self, pcd, keypoints,
                        threshold=5,
                        radius=0.2,
                        rot_x=0, rot_y=0, rot_z=0):

        if self.intrinsic is None or self.extrinsic is None or self.img is None:
            raise ValueError("Bạn cần gọi cloud_to_image trước")

        # ---- rotate cloud CHỈ để project ----
        pcd_rot = self.rotate_cloud(
            pcd, rot_x=rot_x, rot_y=rot_y, rot_z=rot_z
        )

        points_rot = np.asarray(pcd_rot.points)     # dùng cho project
        points_orig = np.asarray(pcd.points)        # dùng để trích kết quả

        # ---- project rotated cloud to image ----
        pts_homo = np.hstack((points_rot, np.ones((points_rot.shape[0], 1))))  # Nx4
        pts_cam = (self.extrinsic @ pts_homo.T)[:3, :]                          # 3xN

        pts_img = self.intrinsic @ pts_cam
        pts_img /= pts_img[2, :]                                                # normalize

        # ---- KDTree xây trên cloud GỐC ----
        kdtree = o3d.geometry.KDTreeFlann(pcd)

        selected_indices = set()   # tránh trùng

        for (kx, ky) in keypoints:
            dists = np.sqrt(
                (pts_img[0, :] - kx) ** 2 +
                (pts_img[1, :] - ky) ** 2
            )

            min_idx = np.argmin(dists)

            if dists[min_idx] <= threshold:
                # seed point lấy từ CLOUD GỐC
                seed_pt = points_orig[min_idx]

                # lấy toàn bộ điểm trong bán kính r (cloud gốc)
                _, idxs, _ = kdtree.search_radius_vector_3d(seed_pt, radius)

                for i in idxs:
                    selected_indices.add(i)

        if len(selected_indices) == 0:
            return o3d.geometry.PointCloud()

        # ---- tạo sub cloud từ CLOUD GỐC ----
        sub_cloud = o3d.geometry.PointCloud()
        sub_cloud.points = o3d.utility.Vector3dVector(
            points_orig[list(selected_indices)]
        )

        return sub_cloud