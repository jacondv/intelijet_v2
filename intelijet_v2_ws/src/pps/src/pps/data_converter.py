#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_converter.py
Module hỗ trợ chuyển đổi giữa ROS PointCloud2 và Open3D (legacy & tensor).
"""

import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2


class CloudConverter:

    @staticmethod
    def pointcloud2_to_o3d_tensor(msg: PointCloud2):
        """
        Convert ROS PointCloud2 message -> Open3D Tensor PointCloud (o3d.t.geometry.PointCloud).
        Giữ tất cả các field có trong PointCloud2 (x, y, z, rgb, intensity, distance, ...).
        """
        import open3d as o3d

        if not isinstance(msg, PointCloud2):
            print("[convert_pointcloud2_to_o3d_tensor] Input message is not of type PointCloud2.")
            return None

        # Convert to structured NumPy array
        cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        field_names = cloud_arr.dtype.names

        if field_names is None:
            print("[pointcloud2_to_o3d_tensor] PointCloud2 message has no fields.")
            return None

        # Extract XYZ
        xyz = ros_numpy.point_cloud2.get_xyz_points(cloud_arr, remove_nans=True)
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3d.core.Tensor(xyz, dtype=o3d.core.Dtype.Float32)

        # Loop over all fields except x,y,z
        for field in field_names:
            if field in ["x", "y", "z"]:
                continue

            data = np.asarray(cloud_arr[field])

            # Handle RGB (float32 packed)
            if field == "rgb" and data.dtype == np.float32:
                rgb_view = data.view(np.uint32)
                r = ((rgb_view >> 16) & 255).astype(np.uint8)
                g = ((rgb_view >> 8) & 255).astype(np.uint8)
                b = (rgb_view & 255).astype(np.uint8)
                colors = np.stack([r, g, b], axis=-1).astype(np.float32) / 255.0
                pcd_t.point["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)
                continue

            # Convert to 2D array if needed
            if data.ndim == 1:
                data = data.reshape(-1, 1)

            # Map NumPy dtype -> Open3D dtype
            dtype_map = {
                np.dtype('float32'): o3d.core.Dtype.Float32,
                np.dtype('float64'): o3d.core.Dtype.Float64,
                np.dtype('int8'): o3d.core.Dtype.Int8,
                np.dtype('int16'): o3d.core.Dtype.Int16,
                np.dtype('int32'): o3d.core.Dtype.Int32,
                np.dtype('uint8'): o3d.core.Dtype.UInt8,
                np.dtype('uint16'): o3d.core.Dtype.UInt16,
                np.dtype('uint32'): o3d.core.Dtype.UInt32,
            }
            dtype = dtype_map.get(data.dtype, o3d.core.Dtype.Float32)

            # Add field
            pcd_t.point[field] = o3d.core.Tensor(data, dtype=dtype)

        print(f"Converted PointCloud2 to Open3D Tensor Cloud with fields: {list(pcd_t.point)}")
        return pcd_t

    # --------------------------------------------------------------------------

    @staticmethod
    def o3d_tensor_to_pointcloud2(pcd_t, frame_id="base_link"):
        """
        Convert Open3D Tensor PointCloud (o3d.t.geometry.PointCloud) -> ROS PointCloud2.
        Giữ tất cả các field có trong pcd_t.point (x, y, z, colors, distance, intensity, ...).
        """
        import open3d as o3d
        import numpy as np
        import rospy
        from sensor_msgs.msg import PointCloud2, PointField
        import std_msgs.msg

        if not isinstance(pcd_t, o3d.t.geometry.PointCloud):
            print("[o3d_tensor_to_pointcloud2] Input must be o3d.t.geometry.PointCloud")
            return None

        # Lấy danh sách field
        point_attrs = list(pcd_t.point)

        # Luôn cần có "positions"
        if "positions" not in point_attrs:
            print("[o3d_tensor_to_pointcloud2] Missing 'positions' field in Open3D tensor cloud.")
            return None

        points = np.asarray(pcd_t.point["positions"].numpy(), dtype=np.float32)
        n_points = points.shape[0]

        # Khởi tạo dict dữ liệu cho ROS
        data_dict = {
            "x": points[:, 0],
            "y": points[:, 1],
            "z": points[:, 2],
        }

        # Các field khác
        for key in point_attrs:
            if key == "positions":
                continue

            arr = np.asarray(pcd_t.point[key].numpy())

            # Màu đặc biệt xử lý
            if key == "colors" and arr.shape[1] == 3:
                rgb_uint8 = (arr * 255).astype(np.uint8)
                rgb_packed = (
                    (rgb_uint8[:, 0].astype(np.uint32) << 16)
                    | (rgb_uint8[:, 1].astype(np.uint32) << 8)
                    | rgb_uint8[:, 2].astype(np.uint32)
                )
                rgb_float = rgb_packed.view(np.float32)
                data_dict["rgb"] = rgb_float
                continue

            # Thêm các field thông thường khác
            if arr.ndim == 1:
                data_dict[key] = arr
            elif arr.ndim == 2 and arr.shape[1] == 1:
                data_dict[key] = arr[:, 0]
            else:
                print(f"[o3d_tensor_to_pointcloud2] Skip field '{key}' - unsupported shape {arr.shape}")

        # Tạo structured array
        dtype_list = [(name, np.float32) for name in data_dict.keys()]
        cloud_arr = np.zeros(n_points, dtype=dtype_list)
        for name in data_dict.keys():
            cloud_arr[name] = data_dict[name]

        # Tạo PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now() if rospy.core.is_initialized() else rospy.Time(0)
        header.frame_id = frame_id

        fields = [PointField(name, i * 4, PointField.FLOAT32, 1)
                for i, name in enumerate(cloud_arr.dtype.names)]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 4 * len(cloud_arr.dtype.names)
        msg.row_step = msg.point_step * n_points
        msg.is_dense = True
        msg.data = cloud_arr.tobytes()

        print(f"Converted Open3D Tensor PointCloud to PointCloud2 with fields: {list(cloud_arr.dtype.names)}")
        return msg

    # --------------------------------------------------------------------------
    @staticmethod
    def o3d_to_ply(pcd, filepath, write_ascii=False):
        """
        Save Open3D point cloud using Open3D writer only.
        - If pcd is o3d.t.geometry.PointCloud: use o3d.t.io.write_point_cloud (preserve attributes).
        - If pcd is o3d.geometry.PointCloud: try to convert to tensor and use tensor writer.
        If conversion or tensor-writer fails, fallback to o3d.io.write_point_cloud (legacy writer).
        Args:
            pcd: o3d.geometry.PointCloud or o3d.t.geometry.PointCloud
            filepath: output .ply path
            write_ascii: True -> ASCII PLY, False -> Binary PLY (default)
        Returns:
            filepath if success, None on failure
        Note:
            - Requires Open3D >= ~0.17 for tensor writer to preserve custom attributes.
            - Use lazy import to reduce chance of import-time conflicts with VTK.
        """
        import os
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)

        try:
            import open3d as o3d
        except Exception as e:
            print(f"[o3d_to_ply] Failed to import open3d: {e}")
            return None

        # If tensor pointcloud, use tensor writer directly

        pcd = CloudConverter.o3d_legacy_to_tensor(pcd)

        try:
            if isinstance(pcd, o3d.t.geometry.PointCloud):
                try:
                    # o3d.t.io.write_point_cloud returns True on success in newer versions
                    success = o3d.t.io.write_point_cloud(filepath, pcd, write_ascii=write_ascii)
                    if success is True or success is None:
                        # Some versions return None; consider it success if no exception
                        print(f"[o3d_to_ply] Saved tensor pointcloud to {filepath}")
                        return filepath
                    else:
                        # explicit False
                        print(f"[o3d_to_ply] o3d.t.io.write_point_cloud returned {success}")
                except Exception as e:
                    print(f"[o3d_to_ply] Error writing tensor pointcloud via o3d.t.io: {e}")
                    # fallthrough to try conversion/fallback
        except Exception:
            # pcd might not be tensor type
            pass

        return None

    # --------------------------------------------------------------------------

    @staticmethod
    def o3d_legacy_to_tensor(pcd_legacy):
        """Convert legacy -> tensor"""
        import open3d as o3d

        if isinstance(pcd_legacy, o3d.geometry.PointCloud):
            return o3d.t.geometry.PointCloud.from_legacy(pcd_legacy)
        elif isinstance(pcd_legacy, o3d.t.geometry.PointCloud):
            return pcd_legacy
        else:
            return None

    # --------------------------------------------------------------------------

    @staticmethod
    def tensor_to_o3d_legacy(pcd_t):
        """Convert tensor -> legacy"""
        import open3d as o3d

        if isinstance(pcd_t, o3d.t.geometry.PointCloud):
            return pcd_t.to_legacy()
        elif isinstance(pcd_t, o3d.geometry.PointCloud):
            return pcd_t
        else:
            return None

    # ------------------------------------------------------------------------------

    @staticmethod    
    def o3d_to_vtk_polydata(pcd, voxel_size=0.0):
        import vtk
        import numpy as np

        pcd = CloudConverter.tensor_to_o3d_legacy(pcd)

        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        points = np.asarray(pcd.points)
        has_colors = pcd.has_colors()
        colors = np.asarray(pcd.colors) if has_colors else None

        vtk_points = vtk.vtkPoints()
        vtk_colors = vtk.vtkUnsignedCharArray()
        vtk_colors.SetNumberOfComponents(3)
        vtk_colors.SetName("Colors")

        for i in range(points.shape[0]):
            vtk_points.InsertNextPoint(points[i])

            if has_colors:
                r, g, b = (colors[i] * 255).astype(np.uint8)
            else:
                # Default to red (255, 0, 0)
                r, g, b = 255, 0, 0

            vtk_colors.InsertNextTuple3(r, g, b)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.GetPointData().SetScalars(vtk_colors)

        return polydata

    # ------------------------------------------------------------------------------

    @staticmethod
    def load_ply(filepath):
        """
        Load PLY file dưới dạng o3d.t.geometry.PointCloud (tensor)
        """
        import open3d as o3d
        pcd = o3d.t.io.read_point_cloud(filepath)  # trả về tensor PointCloud
        return pcd

    # ------------------------------------------------------------------------------

    @staticmethod
    def crop(pcd, min_bound, max_bound):
        """
        Crop point cloud theo giới hạn min/max.
        Hỗ trợ cả legacy và tensor PointCloud.
        """

        import open3d as o3d
        # --- Nếu là legacy (numpy) ---

        min_bound = np.asarray(min_bound, dtype=np.float32)
        max_bound = np.asarray(max_bound, dtype=np.float32)

        if isinstance(pcd, o3d.geometry.PointCloud):
            points = np.asarray(pcd.points)
            mask = np.all((points >= min_bound) & (points <= max_bound), axis=1)

            cropped = o3d.geometry.PointCloud()
            cropped.points = o3d.utility.Vector3dVector(points[mask])

            if pcd.has_colors():
                cropped.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
            if pcd.has_normals():
                cropped.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[mask])
            return cropped

        # --- Nếu là tensor (GPU/CPU Tensor) ---
        elif isinstance(pcd, o3d.t.geometry.PointCloud):
            points = pcd.point["positions"]
            mask = ((points >= min_bound) & (points <= max_bound)).all(dim=1)
            return pcd.select_by_mask(mask)

        else:
            raise TypeError(f"Error {type(pcd)}")

    # ------------------------------------------------------------------------------

    @staticmethod
    def voxel_down_sample(pcd_tensor, voxel_size):
        """
        Downsample Open3D Tensor PointCloud bằng voxel grid.
        Giữ colors và tính lại distances trung bình, vectorized (không loop Python).
        
        Args:
            pcd_tensor: o3d.t.geometry.PointCloud
            voxel_size: float, kích thước voxel
        
        Returns:
            o3d.t.geometry.PointCloud: point cloud đã downsample
        """

        import open3d as o3d
        device = pcd_tensor.device

        # --- 1. Chuyển tensor sang numpy ---
        points = pcd_tensor.point.positions.cpu().numpy()
        if points.ndim != 2 or points.shape[1] != 3:
            points = points.reshape(-1,3)
        points = points.astype(np.float64)

        colors = None
        if "colors" in pcd_tensor.point:
            colors = pcd_tensor.point.colors.cpu().numpy()
            if colors.ndim != 2 or colors.shape[1] != 3:
                colors = colors.reshape(-1,3)
            colors = colors.astype(np.float64)

        distances = None
        if "distances" in pcd_tensor.point:
            distances = pcd_tensor.point["distances"].cpu().numpy()
            if distances.ndim == 1:
                distances = distances.reshape(-1,1)

        # --- 2. Tạo voxel index ---
        voxel_idx = np.floor(points / voxel_size).astype(np.int64)

        # --- 3. Lấy unique voxel và mapping point->voxel ---
        keys, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
        num_voxels = keys.shape[0]

        # --- 4. Tính trung bình positions vectorized ---
        sum_points = np.zeros((num_voxels,3), dtype=np.float64)
        count = np.zeros((num_voxels,1), dtype=np.int64)
        np.add.at(sum_points, inverse, points)
        np.add.at(count, inverse, 1)
        down_points = sum_points / count

        # --- 5. Tính trung bình colors vectorized ---
        down_colors = None
        if colors is not None:
            sum_colors = np.zeros((num_voxels,3), dtype=np.float64)
            np.add.at(sum_colors, inverse, colors)
            down_colors = sum_colors / count

        # --- 6. Tính trung bình distances vectorized ---
        down_distances = None
        if distances is not None:
            dist_dim = distances.shape[1]
            sum_dist = np.zeros((num_voxels, dist_dim), dtype=np.float64)
            np.add.at(sum_dist, inverse, distances)
            down_distances = sum_dist / count

        # --- 7. Tạo lại Tensor point cloud ---
        new_pcd = o3d.t.geometry.PointCloud()
        new_pcd.point.positions = o3d.core.Tensor(down_points, device=device)
        if down_colors is not None:
            new_pcd.point.colors = o3d.core.Tensor(down_colors, device=device)
        if down_distances is not None:
            new_pcd.point["distances"] = o3d.core.Tensor(down_distances, device=device)

        return new_pcd
        
                
    @staticmethod
    def voxel_down_sample_spatial(pcd_tensor, voxel_size):
        """
        Downsample Open3D Tensor PointCloud bằng voxel grid, sau đó loại bỏ các điểm quá gần nhau theo min_dist.
        Giữ colors và distances trung bình, kết hợp KDTree lọc khoảng cách tối thiểu.
        
        Args:
            pcd_tensor: o3d.t.geometry.PointCloud
            voxel_size: float, kích thước voxel
            min_dist: float, khoảng cách tối thiểu giữa các điểm trong cloud
        
        Returns:
            o3d.t.geometry.PointCloud: point cloud đã downsample
        """
        import open3d as o3d
        from scipy.spatial import cKDTree

        device = pcd_tensor.device
        min_dist = 0.9*voxel_size
        # --- 1. Chuyển tensor sang numpy ---
        points = pcd_tensor.point.positions.cpu().numpy().reshape(-1,3).astype(np.float64)

        colors = None
        if "colors" in pcd_tensor.point:
            colors = pcd_tensor.point.colors.cpu().numpy().reshape(-1,3).astype(np.float64)

        distances = None
        if "distances" in pcd_tensor.point:
            distances = pcd_tensor.point["distances"].cpu().numpy()
            if distances.ndim == 1:
                distances = distances.reshape(-1,1)

        # --- 2. Tạo voxel index ---
        voxel_idx = np.floor(points / voxel_size).astype(np.int64)

        # --- 3. Lấy unique voxel và mapping point->voxel ---
        keys, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
        num_voxels = keys.shape[0]

        # --- 4. Tính trung bình positions vectorized ---
        sum_points = np.zeros((num_voxels,3), dtype=np.float64)
        count = np.zeros((num_voxels,1), dtype=np.int64)
        np.add.at(sum_points, inverse, points)
        np.add.at(count, inverse, 1)
        down_points = sum_points / count

        # --- 5. Tính trung bình colors vectorized ---
        down_colors = None
        if colors is not None:
            sum_colors = np.zeros((num_voxels,3), dtype=np.float64)
            np.add.at(sum_colors, inverse, colors)
            down_colors = sum_colors / count

        # --- 6. Tính trung bình distances vectorized ---
        down_distances = None
        if distances is not None:
            dist_dim = distances.shape[1]
            sum_dist = np.zeros((num_voxels, dist_dim), dtype=np.float64)
            np.add.at(sum_dist, inverse, distances)
            down_distances = sum_dist / count

        # --- 7. Lọc các điểm gần nhau bằng KDTree ---
        tree = cKDTree(down_points)
        mask = np.ones(len(down_points), dtype=bool)
        selected = []

        for i, p in enumerate(down_points):
            if mask[i]:
                selected.append(i)
                idxs = tree.query_ball_point(p, min_dist)
                mask[idxs] = False

        selected = np.array(selected, dtype=np.int64)
        final_points = down_points[selected]
        final_colors = down_colors[selected] if down_colors is not None else None
        final_distances = down_distances[selected] if down_distances is not None else None

        # --- 8. Tạo lại Tensor point cloud ---
        new_pcd = o3d.t.geometry.PointCloud()
        new_pcd.point.positions = o3d.core.Tensor(final_points, device=device)
        if final_colors is not None:
            new_pcd.point.colors = o3d.core.Tensor(final_colors, device=device)
        if final_distances is not None:
            new_pcd.point["distances"] = o3d.core.Tensor(final_distances, device=device)

        return new_pcd

# ------------------------------------------------------------------------------

cloudconverter = CloudConverter()

# ------------------------------------------------------------------------------
# if __name__ == "__main__":
#     rospy.init_node("cloud_converter_test")
#     msg = rospy.wait_for_message("/velodyne_points", PointCloud2)

#     converter = CloudConverter()
#     pcd_t = converter.pointcloud2_to_o3d_tensor(msg)
#     print("Converted to Open3D Tensor Cloud with fields:", pcd_t.point.keys())

#     msg2 = converter.o3d_tensor_to_pointcloud2(pcd_t, "map")
#     print("Back to PointCloud2:", type(msg2))
