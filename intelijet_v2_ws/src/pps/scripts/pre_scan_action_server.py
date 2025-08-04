#!/usr/bin/env python3
import rospy
import yaml
import os

import actionlib
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from pps.msg import PreScanAction, PreScanResult, PreScanFeedback
from pps.msg import StartScanAction, StartScanResult, StartScanFeedback
from pps.utils import load_config
from shared.config_loader import CONFIG as cfg


class PreScanActionServer:
    def __init__(self,scan_service_name, pointcloud_topic):
        # self.lidar_name = lidar_name
        self.scan_service_name = scan_service_name
        self.pointcloud_topic = pointcloud_topic
        self.server = actionlib.SimpleActionServer('pre_scan',
            PreScanAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.cloud_received = None
        self.server.start()

    def cloud_callback(self, msg):
        self.cloud_received = msg
        rospy.loginfo("Received pointcloud for pre-scan")

    def execute_cb(self, goal):
        rospy.loginfo("Pre-scan started...")

        # Step 1: Call /lidar1/start_scan
        try:
            rospy.wait_for_service(self.scan_service_name, timeout=5.0)
            trigger = rospy.ServiceProxy(self.scan_service_name, Trigger)
            result = trigger()
            if not result.success:
                self.abort("start_scan failed: " + result.message)
                return
        except Exception as e:
            self.abort("Exception calling start_scan: " + str(e))
            return

        rospy.loginfo("Scan started, waiting for pointcloud...")

        # Step 2: Subscribe to pointcloud and wait
        self.cloud_received = None
        sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.cloud_callback)

        timeout = rospy.Time.now() + rospy.Duration(10)
        rate = rospy.Rate(10)
        while not self.cloud_received and rospy.Time.now() < timeout:
            self.server.publish_feedback(PreScanFeedback(status="Waiting for pointcloud..."))
            rate.sleep()

        sub.unregister()

        if not self.cloud_received:
            self.abort("Timeout waiting for pointcloud.")
            return

        # Step 3: Do pre-processing (placeholder)
        rospy.loginfo("Processing pointcloud...")
        # TODO: filter, save, etc.

        result = PreScanResult()
        result.success = True
        result.message = "Pre-scan completed successfully"
        self.server.set_succeeded(result)

    def execute(self, goal):
        rospy.loginfo("Executing pre-scan action...")
        start_scan_action = StartScanAction()
        start_scan_action.output_topic = self.pointcloud_topic
        pass

    def abort(self, msg):
        rospy.logerr(msg)
        result = PreScanResult(success=False, message=msg)
        self.server.set_aborted(result)

def main():
    rospy.init_node('pre_scan_action_server')

    # cfg = load_config(default_filename="config/lidar.yaml")
    active_lidar = cfg.active_lidar
    rospy.loginfo(f"Active lidar: {active_lidar}")

    PreScanActionServer(scan_service_name=cfg[active_lidar].start_scan_service,
                        pointcloud_topic=cfg[active_lidar].pointcloud_topic)
    
    rospy.loginfo("Pre-scan Action Server is ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())