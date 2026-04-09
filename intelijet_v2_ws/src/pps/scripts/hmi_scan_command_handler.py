#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String, Empty, Int32
# from std_srvs.srv import Trigger
# from align_service_client import AlignServiceClient
# from pps.msg import StartScanAction, StartScanGoal
# from pps.msg import CompareCloudAction, CompareCloudGoal
from pps.cloud_compare.compare_cloud_base_client import CompareBaseClient
# from pps.sick_scan_controller import SickScanController
from pps.sick_scan_eRob_controller import SickScanErobController

# from ros_blkarc_msgs.msg import TimedScanAction, TimedScanGoal

from shared.pps_command import PPSCommand
# from shared.log_status import log_status
from shared.msg import DeviceStatus

from shared.config_loader import CONFIG as cfg

def get_scanner_controller(status_callback=None, active_lidar=cfg.active_lidar):
    if active_lidar == "lms511":
        # controller = SickScanController(status_callback=status_callback)
        controller = SickScanErobController(status_callback=status_callback)
        return controller
    

class ScanManagerNode:
    def __init__(self, action_server_name,
                 scan_time_seconds=10):
        # Action client
        # action_server_name like "/blk360g2/start_scan" send request scan to device
        rospy.loginfo("Starting HMI")
        self.state_pub = rospy.Publisher("/pps/state", String, queue_size=10, latch=True)
        self.current_state = DeviceStatus.IDLE

        # self.scan_time_seconds = scan_time_seconds
        # self.__scan_action_client = actionlib.SimpleActionClient(action_server_name, TimedScanAction)
        # rospy.loginfo(f"Waiting for scan action server...{action_server_name}")
        # self.__scan_action_client.wait_for_server(rospy.Duration(10.0))

        # Lắng nghe lệnh từ HMI
        rospy.Subscriber(cfg.HMI_CMD_TOPIC, Int32, self.cmd_cb)
        rospy.loginfo("ScanManager ready. Listening on %s", cfg.HMI_CMD_TOPIC)

        # self.__align_service_client = AlignServiceClient()
        # rospy.logwarn("Starting AlignServiceClient")
        self.scanner_controller = get_scanner_controller(status_callback=self.set_state)
        # ---- compare client ----

        do_align        = rospy.get_param("/runtime/do_align", True)
        do_pre_process  = rospy.get_param("/runtime/do_pre_process", True)
        do_2d_keypoint  = rospy.get_param("/runtime/do_2d_keypoint", False)
        do_upsample     = rospy.get_param("/runtime/do_upsample", False)
        do_post_process     = rospy.get_param("/runtime/do_post_process", False)
        
        self.compare_client = CompareBaseClient(
            prescan_path="",
            postscan_path="",
            do_pre_process=do_pre_process,
            do_2d_keypoint=do_2d_keypoint,
            do_post_process=do_post_process,
            do_align=do_align,
            do_upsample=do_upsample,
            timeout=150.0
        )

        # self.client = actionlib.SimpleActionClient(
        #     '/compare_cloud',
        #     CompareCloudAction
        # )
        # self.client.wait_for_server()
        # rospy.loginfo("Connected to /compare_cloud")


    def is_state(self, state):
        return self.current_state == state
    

    def set_state(self, state):
        self.current_state = state
        self.state_pub.publish(String(data=state))

    def cmd_cb(self, msg):
        cmd = msg.data
        rospy.loginfo("[ScanManagerNode] Received HMI command: %d", cmd)

        if cmd == PPSCommand.START_PRESCAN.value:
            if self.is_state(DeviceStatus.PRESCAN):
                rospy.logwarn("[ScanManagerNode] Already in PRESCAN state, ignoring command")
                return
            
            self.scanner_controller.run_prescan()
                   
        elif cmd == PPSCommand.START_POSTSCAN.value:    
            if self.is_state(DeviceStatus.POSTSCAN):
                rospy.logwarn("Already in POSTSCAN state, ignoring command")
                return
            
            self.scanner_controller.run_postscan()

        elif cmd == PPSCommand.CANCEL_JOB.value:
            self.scanner_controller.on_cancel()
            self.set_state(DeviceStatus.IDLE)
            self.compare_client.cancel()

        elif cmd == PPSCommand.START_COMPARE.value:

            rospy.loginfo("Start compare command received")
            # avoid double call start()
            state = self.compare_client.client.get_state()
            if state in [actionlib.GoalStatus.ACTIVE,
                         actionlib.GoalStatus.PENDING]:
                rospy.logwarn("Compare already running")
                return
            
            # Get parameter from ros server
            self.compare_client.set_pre_process(rospy.get_param("/runtime/do_pre_process", True))
            self.compare_client.set_2d_keypoint(rospy.get_param("/runtime/do_2d_keypoint", True))
            self.compare_client.set_align(rospy.get_param("/runtime/do_align", True))
            self.compare_client.set_upsample(rospy.get_param("/runtime/do_upsample", True))
            self.compare_client.send_goal()

            # success, message = self.__align_service_client.call()

            # goal = CompareCloudGoal()
            # goal.do_pre_process = True
            # goal.do_post_process = True
            # goal.do_align = True
            # self.client.send_goal(goal)
            
            # self.client.wait_for_result()
            # success = self.client.wait_for_result(rospy.Duration(150.0))
            # if not success:
            #     rospy.logerr("Compare timeout")
            #     self.client.cancel_goal()
            #     return   
            
            # result = self.client.get_result()
            # state = self.client.get_state()
            # if state == actionlib.GoalStatus.SUCCEEDED and result.success:
            #     rospy.loginfo("Compare SUCCESS job_id=%s", result.job_id)
            # else:
            #     rospy.logerr("Compare FAILED state=%d", state)
                
            # if success:
            #     rospy.loginfo("Alignment successful: %s", message)
            # else:
            #     rospy.logerr("Alignment failed: %s", message)

        elif cmd == PPSCommand.OPEN_HOUSING.value:
            if self.is_state(DeviceStatus.OPEN_HOUSING):
                rospy.logwarn("Already in OPEN_HOUSING state, ignoring command")
                return
            self.scanner_controller.open_housing_auto()

        elif cmd == PPSCommand.CLOSE_HOUSING.value:
            if self.is_state(DeviceStatus.CLOSE_HOUSING):
                rospy.logwarn("Already in CLOSE_HOUSING state, ignoring command")
                return  
            self.scanner_controller.close_housing_auto()

        else:
            pass


    # def __send_scan_cmd(self, output_topic):
        
    #     goal = TimedScanGoal(output_topic=output_topic,
    #                          scan_time_seconds=self.scan_time_seconds)

    #     rospy.loginfo("Sending scan goal: %s", output_topic)
    #     self.__scan_action_client.send_goal(goal)
    #     self.__scan_action_client.wait_for_result()
    #     result = self.__scan_action_client.get_result()
    #     if result.success:
    #         rospy.loginfo("Scan succeeded:")
    #         return True
    #     else:
    #         rospy.logerr("Scan failed:")
    #         return False



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

    

