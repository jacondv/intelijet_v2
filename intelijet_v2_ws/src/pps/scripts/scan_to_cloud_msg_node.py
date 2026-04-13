#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class ScanToCloudNode:
    """
    Docstring for ScanToCloudNode
    This node subscribes to /scan (LaserScan), converts it to PointCloud2,
    and publishes to /cloud_msg (PointCloud2). 
    It used for interfacing with other nodes that require point cloud data.
    Note: We should forward the topic in launch file (for Assemblecloud serive to work).
    """
    def __init__(self):
        rospy.init_node("scan_to_cloud_msg_node", anonymous=True)

        # Publisher cloud
        self.pub = rospy.Publisher("/cloud_msg", PointCloud2, queue_size=1)

        # LaserProjection object
        self.lp = LaserProjection()

        # Subscriber scan
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("[scan_to_cloud_msg_node] Node started, converting /scan -> /cloud_msg")
        rospy.spin()

    def scan_callback(self, scan_msg):
        """
        Chuyển LaserScan -> PointCloud2 và publish
        """
        # Convert scan -> cloud
        cloud_msg = self.lp.projectLaser(scan_msg)

        # Gắn frame_id là base_link
        # cloud_msg.header.frame_id = "base_link"

        # Publish
        self.pub.publish(cloud_msg)

if __name__ == "__main__":
    try:
        ScanToCloudNode()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())
