import os
import vtk
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import ros_numpy

from vtk.util import numpy_support

import numpy as np


def ros_pointcloud2_to_vtk_polydata(msg):
    points = []
    colors = []

    for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z, rgb_float = p

        points.append([x, y, z])

        # Giải mã giá trị màu float32 thành RGB uint8
        rgb_uint32 = struct.unpack('I', struct.pack('f', rgb_float))[0]
        r = (rgb_uint32 >> 16) & 0xFF
        g = (rgb_uint32 >> 8) & 0xFF
        b = rgb_uint32 & 0xFF
        colors.append([r, g, b])

    np_points = np.array(points, dtype=np.float32)
    np_colors = np.array(colors, dtype=np.uint8)

    # Tạo vtkPoints từ numpy array
    vtk_points = vtk.vtkPoints()
    vtk_points.SetData(numpy_support.numpy_to_vtk(np_points, deep=True))

    # Tạo vtkUnsignedCharArray cho màu
    vtk_colors = numpy_support.numpy_to_vtk(np_colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
    vtk_colors.SetName("Colors")
    vtk_colors.SetNumberOfComponents(3)

    # Gán dữ liệu vào vtkPolyData
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(vtk_points)
    polydata.GetPointData().SetScalars(vtk_colors)

    return polydata

def ros_pointcloud2_to_o3d_to_vtk_polydata_voxel(msg, voxel_size=0.0):
    # B1: ROS PointCloud2 → Open3D
    o3d_cloud = convert_pointcloud2_to_o3d(msg)

    # B2: Downsample bằng voxel filter
    if voxel_size > 0:
        o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size)

    # B3: Open3D → VTK
    polydata = o3d_to_vtk_polydata(o3d_cloud)
    return polydata

    
def o3d_to_vtk_polydata(pcd, voxel_size=0.0):
    import vtk
    import numpy as np

    if voxel_size > 0:
        o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size)

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

def convert_pointcloud2_to_o3d(msg):
    import rospy
    import numpy as np
    import open3d as o3d
    import ros_numpy
    from sensor_msgs.msg import PointCloud2

    """Convert a ROS PointCloud2 message into an Open3D PointCloud."""
    if not isinstance(msg, PointCloud2):
        rospy.logerr("Input message is not of type PointCloud2.")
        return None

    try:
        # Convert ROS PointCloud2 to structured NumPy array
        cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

        # Extract XYZ points
        xyz = ros_numpy.point_cloud2.get_xyz_points(cloud_arr, remove_nans=True)

        # Create Open3D point cloud
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(xyz)

        # Check for RGB field
        if 'rgb' in cloud_arr.dtype.names:
            # Extract RGB field (float32 packed as uint32)
            rgb_packed = cloud_arr['rgb']
            rgb_view = rgb_packed.view(np.uint32)

            # Decode RGB to 8-bit values
            r = (rgb_view >> 16) & 255
            g = (rgb_view >> 8) & 255
            b = rgb_view & 255

            rgb = np.stack([r, g, b], axis=-1).astype(np.float32) / 255.0

            # Assign colors (only for valid XYZ points)
            if len(rgb) == len(xyz):
                cloud_o3d.colors = o3d.utility.Vector3dVector(rgb)
            else:
                rospy.logwarn("Mismatch between XYZ and RGB point count. Skipping color assignment.")
        else:
            rospy.loginfo("No RGB field in PointCloud2 message. Creating point cloud without color.")

        return cloud_o3d

    except Exception as e:
        rospy.logerr(f"Failed to convert PointCloud2 to Open3D format: {e}")
        return None

def convert_pointcloud2_to_o3d_v2(msg):
    import ros_numpy
    import open3d as o3d


    """Convert a ROS PointCloud2 message into an Open3D PointCloud, preserving extra fields."""
    # if not isinstance(msg, PointCloud2):
    #     rospy.logerr("Input message is not of type PointCloud2.")
    #     return None

    # Convert to structured NumPy array
    cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    field_names = cloud_arr.dtype.names

    # Extract XYZ
    if not all(k in field_names for k in ('x', 'y', 'z')):
        return None

    xyz = np.vstack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z'])).T
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(xyz)

    # === Handle RGB ===
    if 'rgb' in field_names:
        rgb_packed = cloud_arr['rgb']
        rgb_uint8 = np.zeros((rgb_packed.shape[0], 3), dtype=np.uint8)
        rgb_view = rgb_packed.view(np.uint32)
        rgb_uint8[:, 0] = (rgb_view >> 16) & 255
        rgb_uint8[:, 1] = (rgb_view >> 8) & 255
        rgb_uint8[:, 2] = rgb_view & 255
        o3d_cloud.colors = o3d.utility.Vector3dVector(rgb_uint8.astype(np.float32) / 255.0)

    # === Handle any other extra fields (e.g., distances, intensity, normals) ===
    skip_fields = {'x', 'y', 'z', 'rgb'}
    for field in field_names:
        if field in skip_fields:
            continue
        data = cloud_arr[field].astype(np.float32).reshape(-1)
        print(f"Adding extra field to Open3D: {field} (len={len(data)})")
        o3d_cloud.point[field] = o3d.utility.Vector3dVector(np.expand_dims(data, axis=1)) if data.ndim == 1 else o3d.utility.Vector3dVector(data)

    return o3d_cloud


def convert_pointcloud2_to_o3d_tensor(msg: PointCloud2):
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
        print("[convert_pointcloud2_to_o3d_tensor] PointCloud2 message has no fields.")
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

    print(f"Converted PointCloud2 to Open3D Tensor Cloud with fields: {list(pcd_t.point.keys())}")
    return pcd_t



def load_ply_as_polydata(filepath, voxel_size=0.01):
    """
    Load a PLY file using Open3D, downsample by voxel, and convert to vtkPolyData.

    Args:
        filepath (str): path to .ply file
        voxel_size (float): voxel size for downsampling

    Returns:
        vtk.vtkPolyData: downsampled polydata, or None if failed
    """
    import open3d as o3d

    if not os.path.isfile(filepath):
        print(f"Error: File does not exist: {filepath}")
        return None

    # Load PLY bằng Open3D
    pcd = o3d.io.read_point_cloud(filepath)
    if pcd.is_empty():
        print(f"Warning: PLY is empty: {filepath}")
        return None

    # Downsample bằng voxel
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    polydata = o3d_to_vtk_polydata(pcd)

    return polydata



