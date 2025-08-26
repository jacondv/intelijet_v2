#!/usr/bin/env python3
# scripts/generic_scan_controller.py
import rospy
from abc import ABC, abstractmethod
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud2

from shared.config_loader import CONFIG as cfg
from shared.log_status import log_status
from shared.pps_command import PPSCommand
from shared.msg import PPSCommand as PPSCommandMsg

import threading

PI = 3.141592

class HousingControl():
    def __init__(self):

        self.cmd_pub = rospy.Publisher(cfg.PLC_CMD_TOPIC, PPSCommandMsg, queue_size=1)

        self.open_housing_cmd = PPSCommandMsg()
        self.open_housing_cmd.code = PPSCommand.PLC_OPEN_HOUSING.value

        self.close_housing_cmd = PPSCommandMsg()
        self.close_housing_cmd.code = PPSCommand.PLC_CLOSE_HOUSING.value

        self.stop_housing_cmd = PPSCommandMsg()
        self.stop_housing_cmd.code = PPSCommand.PLC_PAUSE_HOUSING.value     


        self.set_retract_speed_cmd = PPSCommandMsg()
        self.set_retract_speed_cmd.code = PPSCommand.PLC_SET_RETRACT_SPEED.value

        self.set_extend_speed_cmd = PPSCommandMsg()
        self.set_extend_speed_cmd.code = PPSCommand.PLC_SET_EXTEND_SPEED.value


    def open(self, speed='fast'):
        """ speed: int or 'fast', 'medium' or 'slow' 
        """
        if speed == 'fast':
            speed = cfg.housing_open_fast_speed
        elif speed == 'medium':
            speed = cfg.housing_open_medium_speed
        elif speed == 'slow':
            speed = cfg.housing_open_slow_speed
        elif isinstance(speed, int):
            speed = speed
        else:
            speed = 0

        self.cmd_pub.publish(self.open_housing_cmd)
        self.open_housing_cmd.uint16_value = speed

    def close(self,speed='fast'):

        if speed == 'fast':
            speed = cfg.housing_close_fast_speed
        elif speed == 'medium':
            speed = cfg.housing_close_medium_speed
        elif speed == 'slow':
            speed = cfg.housing_close_slow_speed
        elif isinstance(speed, int):
            speed = speed
        else:
            speed = 0

        self.cmd_pub.publish(self.close_housing_cmd)
        self.close_housing_cmd.uint16_value = speed

        # if speed is not None:
        #     self.cmd_pub.publish((self.set_retract_speed_cmd,speed))

    def stop(self):
        self.cmd_pub.publish(self.stop_housing_cmd)



class GenericScanController(ABC):
    def __init__(self):

        self.current_encoder_value = None # indarian
        self.current_encoder_value_in_degree = None
        
        self.prescan_pub = rospy.Publisher(cfg.PRE_SCAN_TOPIC, PointCloud2, queue_size=1)
        self.postscan_pub = rospy.Publisher(cfg.POST_SCAN_TOPIC, PointCloud2, queue_size=1)

        self.cancel_job = False
        self._thread = None

        self.housing = HousingControl()

        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

    @abstractmethod
    def run_workflow(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    def joint_state_cb(self, msg):
        try:
            idx = msg.name.index(cfg.ENCODER_JOINT_NAME)
            self.current_encoder_value = msg.position[idx]
            self.current_encoder_value_in_degree = self.current_encoder_value * 180 / PI

        except ValueError:
            rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")

    def wait_until_target(self, target_position_in_degree: float,timeout: float=60.0, direction = True):
        rospy.loginfo("Waiting until encoder reaches %.2f..." % target_position_in_degree)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.cancel_job:
                rospy.loginfo("[GenericScanController] Cancel job, exiting wait.")
                break

            if direction:
                if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree >= target_position_in_degree:
                    rospy.loginfo("Target reached: %.2f" % self.current_encoder_value_in_degree)
                    return True
            else:
                if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree <= target_position_in_degree:
                    rospy.loginfo("Target reached: %.2f" % self.current_encoder_value_in_degree)
                    return True

        # TODO add condition to break this function due to encoder error or PLC not responding...    
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn(f"Timeout waiting for target position {target_position_in_degree}° after {timeout} seconds")
                break
            rate.sleep()
        
        
        return False

    def run_prescan(self):
        self.__run_scan(self.prescan_pub)


    def run_postscan(self):    
        self.__run_scan(self.postscan_pub)
    

    def on_cancel(self):       
        if self._thread is not None and self._thread.is_alive():
            log_status(
                    name=cfg.NOTIFICATION, 
                    status=None, 
                    value=None, 
                    message="[INFO] Canceling job", 
                    node=None
                )
            self.cancel_job = True
            rospy.sleep(1)
            if self._thread is not None and not self._thread.is_alive():
                log_status(
                        name=cfg.NOTIFICATION, 
                        status=None, 
                        value=None, 
                        message="Job canceled", 
                        node=None
                    )
                
    def open_housing_auto(self):
        self.__move_housing(direction=True)

    def close_housing_auto(self):
        self.__move_housing(direction=False)


    def __run_scan(self, publisher):
       
        if self._thread is not None and self._thread.is_alive():
            rospy.logwarn("Scan already running, cannot start another.")
            return False
        
        self.cancel_job = False

        def run_thread():
            try:
                cloud = self.run_workflow(publisher)
                self.reset()
                # if cloud is not None:
                #     publisher.publish(cloud)
            except Exception as e:
                rospy.logerr(f"Error during run_workflow: {e}")

        self._thread = threading.Thread(target=run_thread, daemon=True)
        self._thread.start()

        return True
    
    def __move_housing(self, direction=True):

        if self._thread is not None and self._thread.is_alive():
            rospy.logwarn("Scan already running, cannot start another.")
            return False
        
        self.cancel_job = False

        def run_thread():

            try:
                if direction:
                    TARGET=cfg.housing_end_position
                    self.housing.open('fast') # Open fast speed
                else:
                    TARGET=cfg.housing_start_position
                    self.housing.close('fast') # Close fast speed
                    
                self.wait_until_target(target_position_in_degree=TARGET,direction=direction)
                if not direction:
                    self.housing.close('medium') # Close slow speed
                    rospy.sleep(3)

                self.housing.stop()
            except Exception as e:
                rospy.logerr(f"Error during run_workflow: {e}")

        self._thread = threading.Thread(target=run_thread, daemon=True)
        self._thread.start()

        return True

                
