#!/usr/bin/env python3
import rospy
import importlib
from shared.msg import DeviceStatus
from genpy.message import Message
from shared.config_loader import load_config


def ros_msg_to_dict(msg):
    """
    Convert any ROS message to a dictionary, recursively for nested messages.
    - Converts rospy.Time and rospy.Duration to float (seconds)
    - Converts arrays of messages or primitive types
    """
    result = {}
    # ROS message slots
    for field in msg.__slots__:
        value = getattr(msg, field)

        # ROS Time → float
        if isinstance(value, rospy.Time):
            result[field] = value.to_sec()
        # ROS Duration → float
        elif isinstance(value, rospy.Duration):
            result[field] = value.to_sec()
        # Nested message → recursive
        elif isinstance(value, Message):
            result[field] = ros_msg_to_dict(value)
        # List/array → convert each element
        elif isinstance(value, (list, tuple)):
            new_list = []
            for v in value:
                if isinstance(v, rospy.Time):
                    new_list.append(v.to_sec())
                elif isinstance(v, rospy.Duration):
                    new_list.append(v.to_sec())
                elif isinstance(v, Message):
                    new_list.append(ros_msg_to_dict(v))
                else:
                    new_list.append(v)
            result[field] = new_list
        # Primitive type → keep
        else:
            result[field] = value
    return result

class DeviceMonitor:
    def __init__(self, cfg):
        # Khởi tạo DeviceStatus từ config
        self.status = DeviceStatus()
        self.status.name = cfg.name
        self.status.detail = "Init"
        self.status.device_state = DeviceStatus.DISCONNECTED
        self.status.process_state = DeviceStatus.STANDBY
        self.status.mode = DeviceStatus.CONTINUOUS if cfg.mode == "continuous" else DeviceStatus.ON_DEMAND

        self.topic = cfg.topic
        self.timeout = cfg.timeout
        self.last_msg_time = None

        # dynamic import msg type
        pkg, msg = cfg.msg_type.split("/")
        module = importlib.import_module(pkg + ".msg")
        msg_class = getattr(module, msg)

        rospy.Subscriber(self.topic, msg_class, self.cb)
        rospy.Timer(rospy.Duration(self.timeout / 2.0), self.check_status)

    def cb(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.update_status(DeviceStatus.CONNECTED,
                           DeviceStatus.IDLE,
                           "Message received")

    def check_status(self, event):
        now = rospy.Time.now()
        if self.status.mode == DeviceStatus.CONTINUOUS:
            if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
                self.update_status(DeviceStatus.DISCONNECTED,
                                   DeviceStatus.IDLE,
                                   "No messages received")
            else:
                self.update_status(DeviceStatus.CONNECTED,
                                   DeviceStatus.IDLE,
                                   "OK")
        elif self.status.mode == DeviceStatus.ON_DEMAND:
            if self.last_msg_time is None:
                self.update_status(DeviceStatus.CONNECTED,
                                   DeviceStatus.STANDBY,
                                   "Waiting for data")
            else:
                self.update_status(DeviceStatus.CONNECTED,
                                   DeviceStatus.IDLE,
                                   f"Last msg at {self.last_msg_time.to_sec():.1f}")

    def update_status(self, dev_state, proc_state, detail=""):
        self.status.device_state = dev_state
        self.status.process_state = proc_state
        self.status.detail = detail
        # giữ timestamp local
        self.status.last_update = rospy.Time.now()

    def get_status(self):
        return self.status


class DeviceStatusReader:
    def __init__(self):
        # tạo danh sách monitors từ file yaml
        cfg = load_config("devices.yaml")
        devices = cfg.devices if hasattr(cfg, "devices") else []
        self.monitors = [DeviceMonitor(dev) for dev in devices]

    def get_status(self):
        return {m.status.name: ros_msg_to_dict(m.get_status()) for m in self.monitors}
