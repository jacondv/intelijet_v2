#!/usr/bin/env python

import rospy
from system_monitor.msg import SystemStatus

class SystemMonitor:
    def __init__(self):
        self.pub = rospy.Publisher("/system/status", SystemStatus, queue_size=10)
        rospy.Timer(rospy.Duration(2.0), self.publish_status)

    def publish_status(self, event):
        msg = SystemStatus()
        msg.hardware_name = "LiDAR"
        msg.status = "READY"
        msg.message = "LiDAR is online"
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("system_monitor")
    monitor = SystemMonitor()
    rospy.spin()
