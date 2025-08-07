#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('test_runrosbag_laserscan')

    pub = rospy.Publisher('/cloud1', PointCloud2, queue_size=10)
    rospy.Subscriber('/cloud', PointCloud2, callback)

    rospy.spin()
