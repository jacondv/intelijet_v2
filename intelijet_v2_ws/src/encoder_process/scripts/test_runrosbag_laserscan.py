#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointCloud
import ros_numpy
# from pps.helper import convert_pointcloud2_to_pointcloud

def callback(msg):
    # msg_pt = convert_pointcloud2_to_pointcloud(msg)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('test_runrosbag_laserscan')

    pub = rospy.Publisher('/cloud1', PointCloud2, queue_size=100)
    rospy.Subscriber('/cloud', PointCloud2, callback)
    

    rospy.spin()
