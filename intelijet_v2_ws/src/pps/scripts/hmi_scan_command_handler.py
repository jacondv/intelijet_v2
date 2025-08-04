#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from align_service_client import AlignServiceClient

from pps.msg import StartScanAction, StartScanGoal
from ros_blkarc_msgs.msg import TimedScanAction, TimedScanGoal
from shared.pps_command import PPSCommand

from shared.config_loader import CONFIG as cfg

PRE_SCAN_TOPIC = "/pre_scan_0"
POST_SCAN_TOPIC = "/post_scan_0"

class ScanManagerNode:
    def __init__(self, action_server_name,
                 scan_time_seconds=10):
        # Action client
        rospy.loginfo("Starting HMI")
        self.scan_time_seconds = scan_time_seconds
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

        if cmd == PPSCommand.START_PRESCAN.value:
            self.__send_scan_cmd(output_topic=cfg.PRE_SCAN_TOPIC)

        elif cmd == PPSCommand.START_POSTSCAN.value:
            if self.__send_scan_cmd(output_topic=cfg.POST_SCAN_TOPIC):
                # post_scan_cloud_trigger được gọi bằng hàm notify sẽ sinh ra 1 message rỗng sau khi cloud duoc publish
                # rospy.wait_for_message("/post_scan_cloud_trigger", Empty, timeout=30)
                rospy.loginfo("Post scan complete")

        elif cmd == PPSCommand.START_COMPARE.value:
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
                             scan_time_seconds=self.scan_time_seconds)

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

def main():
    rospy.init_node('hmi_scan_command_handler', anonymous=True)
    active_lidar = cfg.active_lidar
    ScanManagerNode(action_server_name=getattr(cfg, active_lidar).action_server_name,
                    scan_time_seconds=getattr(cfg, active_lidar).scan_time_seconds)
    rospy.spin()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())

    

