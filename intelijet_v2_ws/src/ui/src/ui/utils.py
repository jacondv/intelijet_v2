import vtk
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
from vtk.util import numpy_support
from sensor_msgs.msg import PointCloud2

import rospy
import std_msgs.msg
import numpy as np
import ros_numpy
from collections import defaultdict
from datetime import datetime
from cv_bridge import CvBridge
import cv2
import matplotlib
import matplotlib.pyplot as plt

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

def ros_pointcloud2_to_o3d_to_vtk_polydata_voxel(msg, voxel_size=0.02):
    # B1: ROS PointCloud2 → Open3D
    o3d_cloud = convert_pointcloud2_to_o3d(msg)

    # B2: Downsample bằng voxel filter
    if voxel_size > 0:
        o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size)

    # B3: Open3D → VTK
    polydata = o3d_to_vtk_polydata(o3d_cloud)
    return polydata

def o3d_to_vtk_polydata(pcd):
    import open3d as o3d
    import vtk
    import numpy as np
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    vtk_points = vtk.vtkPoints()
    vtk_colors = vtk.vtkUnsignedCharArray()
    vtk_colors.SetNumberOfComponents(3)
    vtk_colors.SetName("Colors")

    for i in range(points.shape[0]):
        vtk_points.InsertNextPoint(points[i])
        # Open3D color [0.0, 1.0] → [0, 255]
        r, g, b = (colors[i] * 255).astype(np.uint8)
        vtk_colors.InsertNextTuple3(r, g, b)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(vtk_points)
    polydata.GetPointData().SetScalars(vtk_colors)

    return polydata


def convert_pointcloud2_to_o3d(msg):
    import open3d as o3d
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
