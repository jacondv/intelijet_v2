import vtk
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
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

    
def o3d_to_vtk_polydata(pcd):
    import vtk
    import numpy as np

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





