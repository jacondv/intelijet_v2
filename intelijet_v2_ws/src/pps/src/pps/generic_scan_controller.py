#!/usr/bin/env python3
# scripts/generic_scan_controller.py
import rospy
from abc import ABC, abstractmethod
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud2

from shared.config_loader import CONFIG as cfg
from shared.log_status import log_status

import threading

PI = 3.141592

class GenericScanController(ABC):
    def __init__(self):

        self.current_encoder_value = None # indarian
        self.current_encoder_value_in_degree = None

        self.cmd_pub = rospy.Publisher(cfg.HMI_CMD_TOPIC, Int32, queue_size=1)
        self.prescan_pub = rospy.Publisher(cfg.PRE_SCAN_TOPIC, PointCloud2, queue_size=1)
        self.postscan_pub = rospy.Publisher(cfg.POST_SCAN_TOPIC, PointCloud2, queue_size=1)

        self.cancel_job = False
        self._thread = None

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

    def wait_until_target(self, target_position_in_degree: float,timeout: float=10.0, direction = True):
        rospy.loginfo("Waiting until encoder reaches %.2f..." % target_position_in_degree)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.cancel_job:
                rospy.loginfo("[GenericScanController] Cancel job, exiting wait.")
                break

            if direction:
                if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree >= target_position_in_degree:
                    rospy.loginfo("Target reached: %.2f" % self.current_position)
                    return True
            else:
                if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree < target_position_in_degree:
                    rospy.loginfo("Target reached: %.2f" % self.current_position)
                    return True

            
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn(f"Timeout waiting for target position {target_position_in_degree}° after {timeout} seconds")
                break
            rate.sleep()
        
        # TODO add condition to break this function due to encoder error or PLC not responding...
        return False

    def __run(self, publisher):
        
        if self._thread is not None and self._thread.is_alive():
            rospy.logwarn("Scan already running, cannot start another.")
            return False
        
        self.cancel_job = False

        def run_thread():
            try:
                cloud = self.run_workflow()
                self.reset()
                if cloud is not None:
                    publisher.publish(cloud)
            except Exception as e:
                rospy.logerr(f"Error during run_workflow: {e}")

        self._thread = threading.Thread(target=run_thread, daemon=True)
        self._thread.start()

        return True


    def run_prescan(self):
        log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[INFO] Pre-Scan is scanning", 
                node=None
            )
        self.__run(self.prescan_pub)


    def run_postscan(self):
        log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[INFO] Post-Scan is scanning", 
                node=None
            )        
        self.__run(self.postscan_pub)
    

    def on_cancel(self):       
        if self._thread is not None and self._thread.is_alive():
            log_status(
                    name=cfg.NOTIFICATION, 
                    status=None, 
                    value=None, 
                    message="[INFO] Cancel job", 
                    node=None
                )
            self.cancel_job = True
            rospy.sleep(2)
            if self._thread is not None and not self._thread.is_alive():
                log_status(
                        name=cfg.NOTIFICATION, 
                        status=None, 
                        value=None, 
                        message="JACON EQUIPMENT", 
                        node=None
                    )


