#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from align_service_client import AlignServiceClient

from pps.msg import StartScanAction, StartScanGoal
from ros_blkarc_msgs.msg import TimedScanAction, TimedScanGoal
from pps.utils import load_config


SCAN_TIME_SECONDS = 10  # Default scan time in seconds, can be overridden by config
PRE_SCAN_TOPIC = "/pre_scan_0"
POST_SCAN_TOPIC = "/post_scan_0"

class ScanManagerNode:
    def __init__(self, action_server_name= "/start_scan"):
        # Action client
        rospy.loginfo("Starting HMI")
        self.__scan_action_client = actionlib.SimpleActionClient(action_server_name, TimedScanAction)
        rospy.loginfo(f"Waiting for scan action server...{action_server_name}")
        self.__scan_action_client.wait_for_server(rospy.Duration(10.0))

        self.__align_service_client = AlignServiceClient()

        # Lắng nghe lệnh từ HMI
        rospy.Subscriber("/hmi/cmd", String, self.cmd_cb)
        rospy.loginfo("ScanManager ready. Listening on /hmi/cmd")

    def cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        rospy.logwarn("Received HMI command: %s", cmd)

        if cmd == "start_prescan":
            self.__send_scan_cmd(PRE_SCAN_TOPIC)

        elif cmd == "start_postscan":
            if self.__send_scan_cmd(POST_SCAN_TOPIC):
                # post_scan_cloud_trigger được gọi bằng hàm notify sẽ sinh ra 1 message rỗng sau khi cloud duoc publish
                # rospy.wait_for_message("/post_scan_cloud_trigger", Empty, timeout=30)
                rospy.loginfo("Post scan complete")

        elif cmd == "start_compare":
            rospy.loginfo("Start compare command received")
            success, message = self.__align_service_client.call()
            if success:
                rospy.loginfo("Alignment successful: %s", message)
            else:
                rospy.logerr("Alignment failed: %s", message)

        else:
            rospy.logwarn("Unknown command from HMI: %s", cmd)

    def __send_scan_cmd(self, output_topic):
        
        goal = TimedScanGoal(output_topic=output_topic,
                             scan_time_seconds=SCAN_TIME_SECONDS)

        rospy.loginfo("Sending scan goal: %s", output_topic)
        self.__scan_action_client.send_goal(goal)
        self.__scan_action_client.wait_for_result()
        result = self.__scan_action_client.get_result()
        if result.success:
            rospy.loginfo("Scan succeeded:")
            return True
        else:
            rospy.logerr("Scan failed:")
            return False

if __name__ == '__main__':
    rospy.init_node('hmi_scan_command_handler')
    config = load_config(default_filename="config/lidar.yaml")
    __lidar_type = config.get("active_lidar", "lms511")
    __action_server_name = config.get(__lidar_type).get("action_server_name", "/start_scan")
    SCAN_TIME_SECONDS = config.get(__lidar_type).get("scan_time_seconds", 10)
    
    ScanManagerNode(action_server_name=__action_server_name)
    rospy.spin()

