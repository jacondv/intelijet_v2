# ros_thread.py

import rospy
from sensor_msgs.msg import PointCloud2, JointState

from std_msgs.msg import Int32

import threading
from shared.config_loader import CONFIG as cfg, load_config

from shared.device_monitor import  StatusReader
from shared.pps_command import PPSCommand
from shared.msg import Notification
from ui.system_status import build_system_status
from ui.services import project_repository as repo

STORAGE_SCAN_INTERVAL_SEC = 600  # 10 minutes - see project_dir_size_bytes()

HMI_CMD_TOPIC = cfg.HMI_CMD_TOPIC
PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC
CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
CLOUD_COMPARED_UPSAMPLE_TOPIC = f"{CLOUD_COMPARED_TOPIC}/upsample"
CLOUD_COMPARED_TOPIC_MANUAL = CLOUD_COMPARED_TOPIC + "_manual"
CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL = f"{CLOUD_COMPARED_TOPIC_MANUAL}/upsample"

ENCODER_DATA_TOPIC =  cfg.ENCODER01_DATA

class RosThread(threading.Thread):
    def __init__(self, cloud_received_signal, ui_send_cmd_signal, ui_data_update,
                 notification_received_signal=None):
        super(RosThread, self).__init__()
        self.daemon = True
        self.cloud_received_signal = cloud_received_signal
        self.ui_send_cmd_signal = ui_send_cmd_signal
        self.ui_data_update = ui_data_update # Data update to UI
        self.notification_received_signal = notification_received_signal
        self.encoder_deg = None
        self.encoder_raw = None
        # Storage & Data card (SYSTEM tab) - filled in by
        # _update_storage_stats, None until the first scan completes.
        self.storage_used_gb = None
        self.storage_max_gb = None
        self.project_count = None
        self.project_max = None

    def run(self):
        # Run when thread .start() called
        rospy.init_node("gui_node", anonymous=True, disable_signals=True)
        self.cmd_pub = rospy.Publisher(HMI_CMD_TOPIC, Int32, queue_size=1)
        # Autoload device config from devices.yaml. Device connect/disconnect
        # is surfaced only via the SYSTEM tab's live status badges - it's
        # deliberately not pushed to the notification/status-bar stream
        # (that's reserved for scan-process errors), so no on_transition
        # callback is wired here.
        self.device_status_reader = StatusReader()

        rospy.Subscriber(PRE_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback,callback_args=PRE_SCAN_CLOUD_TOPIC,queue_size=1)
        rospy.Subscriber(POST_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback,callback_args=POST_SCAN_CLOUD_TOPIC,queue_size=1)
        rospy.Subscriber(CLOUD_COMPARED_TOPIC, PointCloud2, self.cloud_received_signal_callback,callback_args=CLOUD_COMPARED_TOPIC,queue_size=1)
        rospy.Subscriber(CLOUD_COMPARED_UPSAMPLE_TOPIC, PointCloud2, self.cloud_received_signal_callback,callback_args=CLOUD_COMPARED_UPSAMPLE_TOPIC,queue_size=1)
        rospy.Subscriber(CLOUD_COMPARED_TOPIC_MANUAL, PointCloud2, self.cloud_received_signal_callback,callback_args=CLOUD_COMPARED_TOPIC_MANUAL,queue_size=1)
        rospy.Subscriber(CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL, PointCloud2, self.cloud_received_signal_callback,callback_args=CLOUD_COMPARED_UPSAMPLE_TOPIC_MANUAL,queue_size=1)

        # listennig Encoder value
        rospy.Subscriber(ENCODER_DATA_TOPIC, Int32, self.update_encoder_raw_value)
        
        # listening topic update infomation for UI.
        rospy.Subscriber("/joint_states", JointState, self.update_joint_states_status)

        # Typed notification channel - see shared/notify.py. Replaces the
        # old /rosout-JSON "notification" hack (log_status(name=cfg.NOTIFICATION,
        # ...) parsed back out of the global debug-log topic).
        rospy.Subscriber(cfg.NOTIFICATION_TOPIC, Notification, self.notification_callback)

        rospy.Timer(rospy.Duration(1.0), self.emit_ui_data_update) # Update data 1Hz
        # rospy.Subscriber(HMI_CMD_TOPIC,Int32, self.update_hmi_cmd)

        # Storage & Data card (SYSTEM tab) - os.walk over the whole
        # Projects folder is too slow to do every 1Hz tick, so it gets its
        # own low-frequency timer. rospy.Timer runs each registered timer's
        # callback on its own thread, so this doesn't stall the 1Hz timer
        # above even while a scan is in progress. Run once immediately
        # (don't make the operator wait 10 minutes for the first number).
        self._update_storage_stats(None)
        rospy.Timer(rospy.Duration(STORAGE_SCAN_INTERVAL_SEC), self._update_storage_stats)

        rospy.spin()

    def cloud_received_signal_callback(self, msg, topic_name):
        # Đẩy msg về Qt bằng signal
        self.cloud_received_signal.emit(msg, topic_name)


    def send_command(self, cmd: PPSCommand):
        if hasattr(self, 'cmd_pub'):
            self.cmd_pub.publish(Int32(data=cmd))    
    

    def update_joint_states_status(self, msg):
        try:
            idx = msg.name.index(cfg.ENCODER_JOINT_NAME)
            self.encoder_deg = msg.position[idx] * 180 / 3.14
        except ValueError:
            rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")


    def update_encoder_raw_value(self,msg):
        if msg is None or not hasattr(msg, "data"):
            return
        self.encoder_raw = msg.data


    def notification_callback(self, msg):
        if self.notification_received_signal is not None:
            self.notification_received_signal.emit(msg.source, msg.message, msg.level, msg.code)


    def _update_storage_stats(self, event):
        # storage_cleanup.yaml is read-only here - shared config also read
        # (unmodified) by pps/scripts/storage_cleanup_node.py, which owns
        # actually enforcing these caps. This is purely a display read.
        try:
            limits = load_config("storage_cleanup.yaml")
            self.storage_max_gb = limits.max_data_gb
            self.project_max = limits.max_projects
            self.storage_used_gb = repo.project_dir_size_bytes() / (1024 ** 3)
            self.project_count = len(repo.list_projects())
        except Exception as e:
            rospy.logwarn(f"[RosThread] Storage stats scan failed: {e}")


    def emit_ui_data_update(self, event):
        status = build_system_status(
            self.device_status_reader.get_status(),
            encoder_deg=self.encoder_deg,
            encoder_raw=self.encoder_raw,
            storage_used_gb=self.storage_used_gb,
            storage_max_gb=self.storage_max_gb,
            project_count=self.project_count,
            project_max=self.project_max,
        )
        self.ui_data_update.emit(status)


