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
            'set_home_position': PPSCommand.PLC_SET_HOME_POSITION.value
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

    def set_home_position(self):
        self._send_cmd('set_home_position')


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
    def wait_until_target(self, target: float, timeout: float=60.0, direction=True, tolerance: float=0.5):
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
            # rospy.logwarn(f"target {target:.2f}° CURRENT {current}°")
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
            # if cloud is not None and publisher:
            #     publisher.publish(cloud)
        except Exception as e:
            rospy.logerr(f"Error during scan workflow: {e}")
        finally:
            # if self.status_callback:
            #     self.status_callback(DeviceStatus.IDLE)
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
            self.status_callback(DeviceStatus.IDLE) if self.status_callback else None
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
            rospy.sleep(2)
            log_status(name=cfg.NOTIFICATION, message="...")
            return True
        else:
            log_status(name=cfg.NOTIFICATION, message="Job canceling failed")
            return False
        

    # ----- Set home position -----
    def set_home_position(self):
        try:
            log_status(name=cfg.NOTIFICATION, message="[INFO] Setting home position...")
            self.housing.set_home_position()
            rospy.sleep(1)
            log_status(name=cfg.NOTIFICATION, message="[INFO] Home position set.")
        except Exception as e:
            rospy.logerr(f"Error setting home position: {e}")