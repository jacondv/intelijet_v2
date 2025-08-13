#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String, Empty, Int32
from std_srvs.srv import Trigger
from align_service_client import AlignServiceClient

from pps.msg import StartScanAction, StartScanGoal
from ros_blkarc_msgs.msg import TimedScanAction, TimedScanGoal
from shared.pps_command import PPSCommand

from pps.sick_scan_controller import SickScanController


from shared.config_loader import CONFIG as cfg

def get_scanner_controller(active_lidar=cfg.active_lidar):
    if active_lidar == "lms511":
        controller = SickScanController()
        return controller

class ScanManagerNode:
    def __init__(self, action_server_name,
                 scan_time_seconds=10):
        # Action client
        # action_server_name like "/blk360g2/start_scan" send request scan to device
        rospy.loginfo("Starting HMI")
        # self.scan_time_seconds = scan_time_seconds
        # self.__scan_action_client = actionlib.SimpleActionClient(action_server_name, TimedScanAction)
        # rospy.loginfo(f"Waiting for scan action server...{action_server_name}")
        # self.__scan_action_client.wait_for_server(rospy.Duration(10.0))

        # Lắng nghe lệnh từ HMI
        rospy.Subscriber(cfg.HMI_CMD_TOPIC, Int32, self.cmd_cb)
        rospy.loginfo("ScanManager ready. Listening on %s", cfg.HMI_CMD_TOPIC)

        self.__align_service_client = AlignServiceClient()
        self.scanner_controller = get_scanner_controller()


    def cmd_cb(self, msg):
        cmd = msg.data
        rospy.logwarn("Received HMI command: %d", cmd)

        if cmd == PPSCommand.START_PRESCAN.value:
            self.scanner_controller.run_prescan()

            # self.__send_scan_cmd(output_topic=cfg.PRE_SCAN_TOPIC)

        elif cmd == PPSCommand.START_POSTSCAN.value:
            self.scanner_controller.run_postscan()

            # if self.__send_scan_cmd(output_topic=cfg.POST_SCAN_TOPIC):
            #     # post_scan_cloud_trigger được gọi bằng hàm notify sẽ sinh ra 1 message rỗng sau khi cloud duoc publish
            #     # rospy.wait_for_message("/post_scan_cloud_trigger", Empty, timeout=30)
            #     rospy.loginfo("Post scan complete")

        elif cmd == PPSCommand.CANCEL_JOB.value:
            self.scanner_controller.on_cancel()

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
    #  action_server_name: "/lms511/start_scan is set in config file lidar.yaml"
    ScanManagerNode(action_server_name=getattr(cfg, active_lidar).action_server_name, 
                    scan_time_seconds=getattr(cfg, active_lidar).scan_time_seconds)
    rospy.spin()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())

    

