#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import numpy as np
import math
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

class ScanGroupedCloudLP:
    def __init__(self):
        rospy.init_node("sickscan_interlate_process_node2")

        # LaserProjection để convert scan → cloud
        self.lp = LaserProjection()

        self.groups = {}  # {angle_min_key: [scan_msg,...]}

        self.pub = rospy.Publisher("/cloud_interlaced", PointCloud2, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("sickscan_interlate_process_node2 started")
        rospy.spin()

    def scan_callback(self, scan_msg):
        # Gom frame theo angle_min, làm tròn 6 chữ số
        key = round(scan_msg.angle_min, 6)
        if key not in self.groups:
            self.groups[key] = []
            rospy.loginfo("New interlaced group: angle_min=%.6f" % scan_msg.angle_min)

        self.groups[key].append(scan_msg)

        # Assemble và xoay toàn bộ frame trong nhóm
        assembled_points = []
        angle_min_ref = self.groups[key][0].angle_min

        for frame in self.groups[key]:
            # 1️⃣ Convert LaserScan → PointCloud2 bằng LaserProjection
            cloud_msg = self.lp.projectLaser(frame)
            # 2️⃣ Chuyển sang numpy
            points = self.pointcloud2_to_xyz(cloud_msg)
            # 3️⃣ Xoay theo delta_angle
            delta_theta = frame.angle_min - angle_min_ref
            rotated = self.rotate_points(points, delta_theta)
            assembled_points.append(rotated)

        # Merge tất cả frame
        if len(assembled_points) > 0:
            assembled_points = np.vstack(assembled_points)
        else:
            assembled_points = np.zeros((0,3))

        # Convert lại sang PointCloud2 và publish
        cloud_assembled = self.xyz_to_pointcloud2(assembled_points, scan_msg.header)
        self.pub.publish(cloud_assembled)

    def pointcloud2_to_xyz(self, cloud_msg):
        pts = []
        for p in point_cloud2.read_points(cloud_msg, skip_nans=True):
            pts.append([p[0], p[1], p[2]])
        if len(pts) == 0:
            return np.zeros((0,3))
        return np.array(pts)

    def rotate_points(self, points, delta_theta):
        c = math.cos(delta_theta)
        s = math.sin(delta_theta)
        R = np.array([[c, -s, 0],
                      [s,  c, 0],
                      [0,  0, 1]])
        return points.dot(R.T)

    def xyz_to_pointcloud2(self, points, header):
        fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
        ]
        return point_cloud2.create_cloud(header, fields, points)


if __name__ == "__main__":
    try:
        ScanGroupedCloudLP()
    except rospy.ROSInterruptException:
        pass
