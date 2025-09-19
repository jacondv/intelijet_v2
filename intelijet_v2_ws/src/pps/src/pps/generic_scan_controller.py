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
from shared.msg import DeviceStatus
import threading

PI = 3.141592
# ------------------- HousingControl -------------------
class HousingControl():
    def __init__(self):
        self.cmd_pub = rospy.Publisher(cfg.PLC_CMD_TOPIC, PPSCommandMsg, queue_size=1)
        self.cmd_map = {
            'open': PPSCommand.PLC_OPEN_HOUSING.value,
            'close': PPSCommand.PLC_CLOSE_HOUSING.value,
            'stop': PPSCommand.PLC_PAUSE_HOUSING.value,
            'set_retract_speed': PPSCommand.PLC_SET_RETRACT_SPEED.value,
            'set_extend_speed': PPSCommand.PLC_SET_EXTEND_SPEED.value,
        }

    def _send_cmd(self, cmd_name, speed=0):
        msg = PPSCommandMsg()
        msg.code = self.cmd_map[cmd_name]
        msg.uint16_value = speed
        self.cmd_pub.publish(msg)


    def _parse_speed(self, speed, action):
        """Convert 'fast', 'medium', 'slow' to actual values"""
        speed_map = {
            'open': {
                'fast': cfg.housing_open_speed_fast,
                'medium': cfg.housing_open_speed_medium,
                'slow': cfg.housing_open_speed_slow
            },
            'close': {
                'fast': cfg.housing_close_speed_fast,
                'medium': cfg.housing_close_speed_medium,
                'slow': cfg.housing_close_speed_slow
            }
        }
        if isinstance(speed, int):
            return speed
        return speed_map[action].get(speed, 0)
    
    
    def open(self, speed='fast'):
        self._send_cmd('open', self._parse_speed(speed, 'open'))


    def close(self, speed='fast'):
        self._send_cmd('close', self._parse_speed(speed, 'close'))


    def stop(self):
        self._send_cmd('stop')


# ------------------- GenericScanController -------------------
class GenericScanController(ABC):
    def __init__(self, status_callback=None):

        self.status_callback = status_callback  # callback update state
        self.current_encoder_value = None
        self.current_encoder_value_in_degree = None

        self.prescan_pub = rospy.Publisher(cfg.PRE_SCAN_TOPIC, PointCloud2, queue_size=1)
        self.postscan_pub = rospy.Publisher(cfg.POST_SCAN_TOPIC, PointCloud2, queue_size=1)

        self.cancel_job = False
        self._thread = None
        self.housing = HousingControl()

        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

    # ----- Abstract methods -----
    @abstractmethod
    def run_workflow(self, publisher):
        """Scan workflow, implement ở lớp con"""
        pass

    @abstractmethod
    def reset(self):
        """Reset sau scan, implement ở lớp con"""
        pass

    # ----- Encoder callback -----
    def joint_state_cb(self, msg):
        try:
            idx = msg.name.index(cfg.ENCODER_JOINT_NAME)
            self.current_encoder_value = msg.position[idx]
            self.current_encoder_value_in_degree = self.current_encoder_value * 180 / PI
        except ValueError:
            rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")

    # ----- Wait until housing reaches target -----
    def wait_until_target(self, target: float, timeout: float=60.0, direction=True, tolerance: float=0.01):
        rospy.loginfo(f"[GenericScanController] Waiting until encoder reaches {target:.2f}°...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.cancel_job:
                rospy.loginfo("[GenericScanController] Job canceled, exiting wait.")
                return False
            current = self.current_encoder_value_in_degree
            if current is not None:
                if (direction and current >= target - tolerance) or \
                   (not direction and current <= target + tolerance):
                    rospy.loginfo(f"Target reached: {current:.2f}°")
                    return True
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"Timeout waiting for target {target:.2f}° after {timeout}s")
                return False
            rate.sleep()
        return False

    # ----- Scan API -----
    def run_prescan(self):
        return self._run_in_thread(lambda: self._scan_thread(self.prescan_pub), name=DeviceStatus.PRESCAN)

    def run_postscan(self):
        return self._run_in_thread(lambda: self._scan_thread(self.postscan_pub), name=DeviceStatus.POSTSCAN)

    def _scan_thread(self, publisher):
        try:
            cloud = self.run_workflow(publisher)
            self.reset()
            if cloud is not None and publisher:
                publisher.publish(cloud)
        except Exception as e:
            rospy.logerr(f"Error during scan workflow: {e}")
        finally:
            if self.status_callback:
                self.status_callback(DeviceStatus.STANDBY)
            self.reset()

    # ----- Housing auto -----
    def open_housing_auto(self):
        return self._run_in_thread(lambda: self._move_housing_thread(direction=True), name=DeviceStatus.OPEN_HOUSING)

    def close_housing_auto(self):
        return self._run_in_thread(lambda: self._move_housing_thread(direction=False), name=DeviceStatus.CLOSE_HOUSING)

    def _move_housing_thread(self, direction):
        #direction = True is open housing direction, False is Close housing direction
        try:
            target = cfg.housing_end_position if direction else cfg.housing_start_position
            if direction:
                log_status(name=cfg.NOTIFICATION, message="[INFO] Opening Housing")
                self.housing.open('fast')
            else:
                log_status(name=cfg.NOTIFICATION, message="[INFO] Closing Housing")
                self.housing.close('fast')

            if not self.wait_until_target(target, direction=direction):
                return

            if not direction:
                # Wait another 2 seconds to ensure the housing closing process is complete
                self.housing.close('medium')
                self.wait_until_target(0.0, direction=direction, timeout=2)

        except Exception as e:
            
            rospy.logerr(f"Error during housing move: {e}")
        
        finally:
            if self.status_callback:
                self.status_callback(DeviceStatus.STANDBY)
            self.housing.stop()

    # ----- Thread helper -----
    def _run_in_thread(self, func, name="Thread"):
        if self._thread is not None and self._thread.is_alive():
            rospy.logwarn(f"{self._thread.name} already running, cannot create thread {name}")
            return False
        
        if self.status_callback:
                # we create thread name = DeviceStatus.PRESCAN.. so we can use name like DeviceStatus 
                self.status_callback(name)

        self.cancel_job = False
        self._thread = threading.Thread(target=func, name=name, daemon=True)
        self._thread.start()
        return True

    # ----- Cancel job -----
    def on_cancel(self):
        log_status(name=cfg.NOTIFICATION, message="[INFO] Job canceling...")
        self.cancel_job = True
        rospy.sleep(1)
        if self._thread is None or not self._thread.is_alive():
            log_status(name=cfg.NOTIFICATION, message="Job canceled")
            return True
        else:
            log_status(name=cfg.NOTIFICATION, message="Job canceling failed")
            return False

#===========================
# class HousingControl():
#     def __init__(self):

#         self.cmd_pub = rospy.Publisher(cfg.PLC_CMD_TOPIC, PPSCommandMsg, queue_size=1)

#         self.open_housing_cmd = PPSCommandMsg()
#         self.open_housing_cmd.code = PPSCommand.PLC_OPEN_HOUSING.value

#         self.close_housing_cmd = PPSCommandMsg()
#         self.close_housing_cmd.code = PPSCommand.PLC_CLOSE_HOUSING.value

#         self.stop_housing_cmd = PPSCommandMsg()
#         self.stop_housing_cmd.code = PPSCommand.PLC_PAUSE_HOUSING.value     


#         self.set_retract_speed_cmd = PPSCommandMsg()
#         self.set_retract_speed_cmd.code = PPSCommand.PLC_SET_RETRACT_SPEED.value

#         self.set_extend_speed_cmd = PPSCommandMsg()
#         self.set_extend_speed_cmd.code = PPSCommand.PLC_SET_EXTEND_SPEED.value


#     def open(self, speed='fast'):
#         """ speed: int or 'fast', 'medium' or 'slow' 
#         """
#         if speed == 'fast':
#             speed = cfg.housing_open_speed_fast
#         elif speed == 'medium':
#             speed = cfg.housing_open_speed_medium
#         elif speed == 'slow':
#             speed = cfg.housing_open_speed_slow
#         elif isinstance(speed, int):
#             speed = speed
#         else:
#             speed = 0


#         self.open_housing_cmd.uint16_value = speed
#         self.cmd_pub.publish(self.open_housing_cmd)

#     def close(self,speed='fast'):

#         if speed == 'fast':
#             speed = cfg.housing_close_speed_fast
#         elif speed == 'medium':
#             speed = cfg.housing_close_speed_medium
#         elif speed == 'slow':
#             speed = cfg.housing_close_speed_slow
#         elif isinstance(speed, int):
#             speed = speed
#         else:
#             speed = 0

#         self.close_housing_cmd.uint16_value = speed
#         self.cmd_pub.publish(self.close_housing_cmd)

#         # if speed is not None:
#         #     self.cmd_pub.publish((self.set_retract_speed_cmd,speed))

#     def stop(self):
#         self.cmd_pub.publish(self.stop_housing_cmd)


# class GenericScanController(ABC):
#     def __init__(self):

#         self.current_encoder_value = None # indarian
#         self.current_encoder_value_in_degree = None
        
#         self.prescan_pub = rospy.Publisher(cfg.PRE_SCAN_TOPIC, PointCloud2, queue_size=1)
#         self.postscan_pub = rospy.Publisher(cfg.POST_SCAN_TOPIC, PointCloud2, queue_size=1)

#         self.cancel_job = False
#         self._thread = None

#         self.housing = HousingControl()

#         rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

#     @abstractmethod
#     def run_workflow(self):
#         pass

#     @abstractmethod
#     def reset(self):
#         pass

#     def joint_state_cb(self, msg):
#         try:
#             idx = msg.name.index(cfg.ENCODER_JOINT_NAME)
#             self.current_encoder_value = msg.position[idx]
#             self.current_encoder_value_in_degree = self.current_encoder_value * 180 / PI

#         except ValueError:
#             rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")

#     def wait_until_target(self, target_position_in_degree: float,timeout: float=60.0, direction = True):
#         rospy.loginfo("Waiting until encoder reaches %.2f..." % target_position_in_degree)
#         rate = rospy.Rate(10)
#         start_time = rospy.Time.now()
        
#         while not rospy.is_shutdown():
#             if self.cancel_job:
#                 rospy.loginfo("[GenericScanController] Cancel job, exiting wait.")
#                 break

#             if direction:
#                 if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree >= target_position_in_degree:
#                     rospy.loginfo("Target reached: %.2f" % self.current_encoder_value_in_degree)
#                     return True
#             else:
#                 if self.current_encoder_value_in_degree is not None and self.current_encoder_value_in_degree <= target_position_in_degree:
#                     rospy.loginfo("Target reached: %.2f" % self.current_encoder_value_in_degree)
#                     return True

#         # TODO add condition to break this function due to encoder error or PLC not responding...    
#             elapsed = (rospy.Time.now() - start_time).to_sec()
#             if elapsed > timeout:
#                 rospy.logwarn(f"Timeout waiting for target position {target_position_in_degree}° after {timeout} seconds")
#                 break
#             rate.sleep()
        
        
#         return False

#     def run_prescan(self):
#         self.__run_scan(self.prescan_pub)


#     def run_postscan(self):    
#         self.__run_scan(self.postscan_pub)
    

#     def on_cancel(self):       
#         # if self._thread is not None and self._thread.is_alive():
#         log_status(name=cfg.NOTIFICATION,message="[INFO] Job canceling...")
#         self.cancel_job = True
#         rospy.sleep(1)
#         if self._thread is not None and not self._thread.is_alive():
#             log_status(name=cfg.NOTIFICATION, message="Job canceled")
#             return True
#         else:
#             log_status(name=cfg.NOTIFICATION, message="Job canceling failed")
#             return False
                
#     def open_housing_auto(self):
#         self.__move_housing(direction=True)

#     def close_housing_auto(self):
#         self.__move_housing(direction=False)


#     def __run_scan(self, publisher):
       
#         if self._thread is not None and self._thread.is_alive():
#             rospy.logwarn("Scan already running, cannot start another.")
#             return False
        
#         self.cancel_job = False

#         def run_thread():
#             try:
#                 cloud = self.run_workflow(publisher)
#                 self.reset()
#                 # if cloud is not None:
#                 #     publisher.publish(cloud)
#             except Exception as e:
#                 rospy.logerr(f"Error during run_workflow: {e}")

#         self._thread = threading.Thread(target=run_thread, daemon=True)
#         self._thread.start()

#         return True
    
#     def __move_housing(self, direction=True):

#         if self._thread is not None and self._thread.is_alive():
#             rospy.logwarn("Scan already running, cannot start another.")
#             return False
        
#         self.cancel_job = False

#         def run_thread():

#             try:
#                 if direction:
#                     TARGET=cfg.housing_end_position
#                     self.housing.open('fast') # Open fast speed
#                     log_status(name=cfg.NOTIFICATION,message="[INFO] Opening Housing")

#                 else:
#                     log_status(name=cfg.NOTIFICATION,message="[INFO] Closing Housing")
#                     TARGET=cfg.housing_start_position
#                     self.housing.close('fast') # Close fast speed
                    
#                 self.wait_until_target(target_position_in_degree=TARGET,direction=direction)
#                 if not direction:
#                     self.housing.close('medium') # Close slow speed
#                     rospy.sleep(3)

#                 self.housing.stop()
#             except Exception as e:
#                 rospy.logerr(f"Error during run_workflow: {e}")

#         self._thread = threading.Thread(target=run_thread, daemon=True)
#         self._thread.start()

#         return True

                
