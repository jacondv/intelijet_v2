#!/usr/bin/env python3
import rospy
import importlib
from shared.msg import DeviceStatus
from genpy.message import Message
from shared.config_loader import load_config


from genpy.message import Message
import rospy

def ros_msg_to_dict(msg):
    """
    Convert any ROS message to a dictionary, recursively for nested messages.
    - rospy.Time and rospy.Duration -> float (seconds)
    - Arrays -> list
    """
    result = {}
    for field in msg.__slots__:
        value = getattr(msg, field)

        if value is None:
            result[field] = None
        elif isinstance(value, rospy.Time):
            result[field] = value.to_sec()
        elif isinstance(value, rospy.Duration):
            result[field] = value.to_sec()
        elif isinstance(value, Message):
            result[field] = ros_msg_to_dict(value)
        elif isinstance(value, (list, tuple)):
            result[field] = [
                v.to_sec() if isinstance(v, (rospy.Time, rospy.Duration)) else
                ros_msg_to_dict(v) if isinstance(v, Message) else
                v for v in value
            ]
        else:
            # Nếu object có .tolist() (numpy array), convert
            if hasattr(value, "tolist"):
                result[field] = value.tolist()
            else:
                result[field] = value
    return result


class Monitor:
    def __init__(self, cfg):
        # Khởi tạo DeviceStatus từ config
        self.status = DeviceStatus()
        self.status.name = cfg.name
        self.status.detail = "Init"
        self.status.device_state = DeviceStatus.DISCONNECTED
        self.status.process_state = DeviceStatus.STANDBY
        self.status.mode = DeviceStatus.CONTINUOUS if cfg.mode.lower() == "continuous" else DeviceStatus.ON_DEMAND

        self.topic = cfg.topic
        self.timeout = cfg.timeout
        self.last_msg_time = None

        # dynamic import msg type
        pkg, msg = cfg.msg_type.split("/")
        module = importlib.import_module(pkg + ".msg")
        msg_class = getattr(module, msg)

        rospy.Subscriber(self.topic, msg_class, self.handle_message)
        rospy.Timer(rospy.Duration(self.timeout / 2.0), self.check_status)

    def handle_message(self, msg):
        raise NotImplementedError("Override me in subclass")
    
    def check_status(self, event):
        raise NotImplementedError("Override me in subclass")
    
    def get_status(self):
        return self.status
    
    def update_status(self, dev_state='', proc_state='', detail=''):
        self.status.device_state = dev_state
        self.status.process_state = proc_state
        self.status.detail = detail
        # giữ timestamp local
        self.status.last_update = rospy.Time.now()

class LidarMonitor(Monitor):
    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)
        else:
            self.update_status(DeviceStatus.CONNECTED)


class EncoderMonitor(Monitor):
    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)
        else:
            self.update_status(DeviceStatus.CONNECTED)

class PCANMonitor(Monitor):
    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)
        else:
            self.update_status(DeviceStatus.CONNECTED)

class PLCMonitor(Monitor):
    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)
        else:
            self.update_status(DeviceStatus.CONNECTED)

class PPSMonitor(Monitor):
    def handle_message(self, msg):
        self.update_status(dev_state=msg.data)

    def check_status(self, event):
        pass        


# Config with devices will be monitor
# DEVICE_CLASSES = {
#     "lidar": LidarMonitor,
#     "encoder": EncoderMonitor,
#     "pcan": PCANMonitor,
#     "pps": PPSMonitor,
#     "plc":PLCMonitor
# }

def get_monitor_class(class_name):
    try:
        return eval(class_name)  # vì các class đã được định nghĩa trong file
    except NameError:
        rospy.logwarn(f"Monitor class [{class_name}] not found, fallback to Monitor")
        return Monitor

class StatusReader:
    _instance = None  # biến lưu instance duy nhất

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(StatusReader, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, config_file="devices.yaml"):

        if hasattr(self, "_initialized") and self._initialized:
            return  # đã khởi tạo rồi, không làm gì nữa

        self._initialized = True
        # tạo danh sách monitors từ file yaml
        cfg = load_config(config_file)
        self.monitors = []
        devices = cfg.devices if hasattr(cfg, "devices") else []
        # self.monitors = [DeviceMonitor(dev) for dev in devices]

        # for dev in devices:
        #     cls = DEVICE_CLASSES.get(dev.name.lower())
        #     if cls is None:
        #         rospy.logwarn(f"No monitor class for device [{dev.name}], skipping")
        #         continue
        #     rospy.loginfo(f"Start monitor [{dev.name}]")
        #     self.monitors.append(cls(dev))

        for dev in devices:
            cls = get_monitor_class(dev.type)
            rospy.loginfo(f"Start monitor [{dev.name}]")
            self.monitors.append(cls(dev))

    def get_status(self):
        return {m.status.name: ros_msg_to_dict(m.get_status()) for m in self.monitors}
    
    def get_device(self, name):
        for m in self.monitors:
            if m.status.name == name:
                return m
        return None




# class DeviceMonitor:
#     def __init__(self, cfg):
#         # Khởi tạo DeviceStatus từ config
#         self.status = DeviceStatus()
#         self.status.name = cfg.name
#         self.status.detail = "Init"
#         self.status.device_state = DeviceStatus.DISCONNECTED
#         self.status.process_state = DeviceStatus.STANDBY
#         self.status.mode = DeviceStatus.CONTINUOUS if cfg.mode.lower() == "continuous" else DeviceStatus.ON_DEMAND

#         self.topic = cfg.topic
#         self.timeout = cfg.timeout
#         self.last_msg_time = None

#         # dynamic import msg type
#         pkg, msg = cfg.msg_type.split("/")
#         module = importlib.import_module(pkg + ".msg")
#         msg_class = getattr(module, msg)

#         rospy.Subscriber(self.topic, msg_class, self.cb)
#         rospy.Timer(rospy.Duration(self.timeout / 2.0), self.check_status)

#     def cb(self, msg):
#         self.last_msg_time = rospy.Time.now()


#     def check_status(self, event):
#         now = rospy.Time.now()
#         if self.status.mode == DeviceStatus.CONTINUOUS:
#             if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
#                 self.update_status(DeviceStatus.DISCONNECTED)
#             else:
#                 self.update_status(DeviceStatus.CONNECTED)
                   
#         elif self.status.mode == DeviceStatus.ON_DEMAND:
#             if self.last_msg_time is None:
#                 self.update_status(DeviceStatus.CONNECTED)
#             else:
#                 self.update_status(DeviceStatus.CONNECTED)
        


#     def update_status(self, dev_state='', detail=''):
#         if dev_state != '':
#             self.status.device_state = dev_state
#         self.status.detail = detail
#         # giữ timestamp local
#         self.status.last_update = rospy.Time.now()

#     def get_status(self):
#         return self.status


# class DeviceStatusReader:
#     def __init__(self):
#         # tạo danh sách monitors từ file yaml
#         cfg = load_config("devices.yaml")
#         devices = cfg.devices if hasattr(cfg, "devices") else []
#         self.monitors = [DeviceMonitor(dev) for dev in devices]

#     def get_status(self):
#         return {m.status.name: ros_msg_to_dict(m.get_status()) for m in self.monitors}



# This is device status message definition
# File: shared/msg/DeviceStatus.msg
# string name
# string device_state
# string process_state
# string mode
# string detail
# time   last_update

# # ---- Constants for device_state ----
# string CONNECTED    = Connected
# string DISCONNECTED = Disconnected
# string ERROR        = Error

# # ---- Constants for process_state ----
# string IDLE         = Idle
# string PROCESSING   = Processing
# string FINISHED     = Finished
# string STANDBY      = Standby
# string PRESCAN     = Prescanning
# string POSTSCAN    = Postscanning
# string OPEN_HOUSING    = HousingOpening
# string CLOSE_HOUSING   = HousingClosing
# # ---- Constants for mode ----
# string CONTINUOUS   = Continuous
# string ON_DEMAND    = On-demand