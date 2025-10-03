#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# --- Class InterlacedAssembler ---
class InterlacedAssembler:
    def __init__(self, n_frames: int):
        self.n_frames = n_frames
        self.frames = []

    def add_cloud_msg(self, cloud_msg):
        points = np.array(list(pc2.read_points(cloud_msg, field_names=("x","y","z"), skip_nans=True)))
        self.frames.append(points)

    def is_full(self) -> bool:
        return len(self.frames) >= self.n_frames

    def assemble_cloud(self) -> o3d.geometry.PointCloud:
        if not self.frames:
            return None
        all_points = np.vstack(self.frames)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        self.reset()
        return pcd

    def reset(self):
        self.frames.clear()

# --- Node ROS1 ---
class InterlacerNode:
    def __init__(self):
        rospy.init_node("interlaced_assembler_node")
        
        # Number of frames to assemble
        self.n_frames = 4
        self.assembler = InterlacedAssembler(self.n_frames)

        # Subscriber
        rospy.Subscriber("/cloud", PointCloud2, self.cloud_callback)

        # Publisher
        self.pub = rospy.Publisher("/cloud_interlaced", PointCloud2, queue_size=1)

        rospy.loginfo("Sickscan Interlaced Assembler Node started")
        rospy.spin()

    def cloud_callback(self, msg):
        self.assembler.add_cloud_msg(msg)
        if self.assembler.is_full():
            assembled_pcd = self.assembler.assemble_cloud()
            if assembled_pcd is not None:
                # Convert Open3D point cloud back to PointCloud2
                points = np.asarray(assembled_pcd.points)
                header = msg.header
                assembled_msg = pc2.create_cloud_xyz32(header, points)
                self.pub.publish(assembled_msg)
                # rospy.loginfo("Published assembled cloud with {} points.".format(points.shape[0]))

if __name__ == "__main__":
    try:
        node = InterlacerNode()
    except rospy.ROSInterruptException:
        pass
