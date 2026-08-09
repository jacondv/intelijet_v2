#!/usr/bin/env python3
"""Device connectivity monitoring.

Three monitor types, chosen per device via the `type:` field in devices.yaml:
  - TopicAliveMonitor: CONNECTED while a message keeps arriving on `topic`
    within `timeout` seconds. Used for CAN-based devices (encoder, pcan, plc).
  - PingMonitor: CONNECTED while periodic ICMP ping to `ip` succeeds. Runs
    ping in a background thread so it never blocks the ROS timer/spin loop.
    Used for network devices (lidar) where "connected" should reflect actual
    reachability rather than whether a scan happens to be running.
  - StateEchoMonitor: device_state mirrors the last message payload (e.g.
    /pps/state), but still goes DISCONNECTED if no message arrives within
    `timeout` seconds (prevents getting stuck on a stale state forever).

To add a new device: add an entry to devices.yaml with a `type:` matching one
of the classes registered in MONITOR_CLASSES below, then add/extend a monitor
class here if none of the three existing types fit.
"""
import rospy
import importlib
import subprocess
import threading
from shared.msg import DeviceStatus
from genpy.message import Message
from shared.config_loader import load_config


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

        self.timeout = cfg.timeout
        self.check_interval = max(self.timeout / 2.0, 0.5)

        self._setup(cfg)
        rospy.Timer(rospy.Duration(self.check_interval), self.check_status)

    def _setup(self, cfg):
        """Override in subclass to wire up subscribers/threads before the
        periodic check_status timer starts."""
        pass

    def check_status(self, event):
        raise NotImplementedError("Override me in subclass")

    def get_status(self):
        return self.status

    def update_status(self, dev_state, proc_state=None, detail=None):
        self.status.device_state = dev_state
        if proc_state is not None:
            self.status.process_state = proc_state
        if detail is not None:
            self.status.detail = detail
        # giữ timestamp local
        self.status.last_update = rospy.Time.now()


class TopicAliveMonitor(Monitor):
    """CONNECTED while messages keep arriving on `topic` within `timeout`s.
    Used for CAN-based devices: encoder, pcan, plc."""

    def _setup(self, cfg):
        self.topic = cfg.topic
        self.last_msg_time = None

        # dynamic import msg type
        pkg, msg = cfg.msg_type.split("/")
        module = importlib.import_module(pkg + ".msg")
        msg_class = getattr(module, msg)

        rospy.Subscriber(self.topic, msg_class, self.handle_message)

    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)
        else:
            self.update_status(DeviceStatus.CONNECTED)


class PingMonitor(Monitor):
    """CONNECTED while periodic ICMP ping to `ip` succeeds. Ping runs in a
    dedicated background thread so subprocess calls never block the ROS
    timer/spin loop. Requires 2 consecutive failed pings before flipping to
    DISCONNECTED, to avoid flapping on a single dropped packet."""

    FAIL_THRESHOLD = 2

    def _setup(self, cfg):
        self.ip = cfg.ip
        self._connected = False
        self._consecutive_failures = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._ping_thread = threading.Thread(
            target=self._ping_loop, daemon=True,
            name=f"PingMonitor-{cfg.name}"
        )
        self._ping_thread.start()

    def _ping_once(self):
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", self.ip],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            return result.returncode == 0
        except Exception as e:
            rospy.logwarn(f"[PingMonitor] ping to {self.ip} failed to run: {e}")
            return False

    def _ping_loop(self):
        while not self._stop_event.is_set():
            ok = self._ping_once()
            with self._lock:
                if ok:
                    self._consecutive_failures = 0
                    self._connected = True
                else:
                    self._consecutive_failures += 1
                    if self._consecutive_failures >= self.FAIL_THRESHOLD:
                        self._connected = False
            self._stop_event.wait(self.check_interval)

    def check_status(self, event):
        with self._lock:
            connected = self._connected
        if connected:
            self.update_status(DeviceStatus.CONNECTED)
        else:
            self.update_status(DeviceStatus.DISCONNECTED)


class StateEchoMonitor(Monitor):
    """device_state mirrors the last message payload (e.g. /pps/state), but
    still goes DISCONNECTED if no message arrives within `timeout`s -
    prevents getting stuck showing a stale state forever."""

    def _setup(self, cfg):
        self.topic = cfg.topic
        self.last_msg_time = None

        pkg, msg = cfg.msg_type.split("/")
        module = importlib.import_module(pkg + ".msg")
        msg_class = getattr(module, msg)

        rospy.Subscriber(self.topic, msg_class, self.handle_message)

    def handle_message(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.update_status(dev_state=msg.data)

    def check_status(self, event):
        now = rospy.Time.now()
        if self.last_msg_time is None or (now - self.last_msg_time).to_sec() > self.timeout:
            self.update_status(DeviceStatus.DISCONNECTED)


MONITOR_CLASSES = {
    "TopicAliveMonitor": TopicAliveMonitor,
    "PingMonitor": PingMonitor,
    "StateEchoMonitor": StateEchoMonitor,
}


def get_monitor_class(class_name):
    cls = MONITOR_CLASSES.get(class_name)
    if cls is None:
        rospy.logerr(f"[device_monitor] Unknown monitor type [{class_name}], "
                      f"expected one of {list(MONITOR_CLASSES.keys())}")
    return cls


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

        for dev in devices:
            cls = get_monitor_class(dev.type)
            if cls is None:
                rospy.logerr(f"[device_monitor] Skipping device [{dev.name}]: "
                              f"invalid type [{dev.type}]")
                continue
            try:
                monitor = cls(dev)
            except Exception as e:
                rospy.logerr(f"[device_monitor] Failed to start monitor "
                              f"[{dev.name}]: {e}")
                continue
            rospy.loginfo(f"Start monitor [{dev.name}]")
            self.monitors.append(monitor)

    def get_status(self):
        return {m.status.name: ros_msg_to_dict(m.get_status()) for m in self.monitors}

    def get_device(self, name):
        for m in self.monitors:
            if m.status.name == name:
                return m
        return None
