# ros_thread.py

from ui.handlers import *
import rospy
from sensor_msgs.msg import PointCloud2, JointState

from std_msgs.msg import Int32

import threading
from shared.config_loader import CONFIG as cfg

from shared.device_monitor import  StatusReader
from shared.pps_command import PPSCommand
from shared.msg import Notification

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
        self.data_store = {}

    def run(self):
        # Run when thread .start() called
        rospy.init_node("gui_node", anonymous=True, disable_signals=True)
        self.cmd_pub = rospy.Publisher(HMI_CMD_TOPIC, Int32, queue_size=1)
        self.device_status_reader = StatusReader() # Autoload device config from devices.yaml

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
            current_encoder_value_in_degree = msg.position[idx] * 180 / 3.14

            self.data_store["encoder_value_in_deg"] = current_encoder_value_in_degree

        except ValueError:

            rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")


    def update_encoder_raw_value(self,msg):
        if msg is None or not hasattr(msg, "data"):
            return
        self.data_store["encoder_value_raw"] = msg.data


    def notification_callback(self, msg):
        if self.notification_received_signal is not None:
            self.notification_received_signal.emit(msg.source, msg.message, msg.level)


    def emit_ui_data_update(self, msg):
        # print(self.data_store["devices"])
        self.data_store["devices"] = self.device_status_reader.get_status()
        # print("Emitting ui_data_update", self.data_store["devices"])
        self.ui_data_update.emit(self.data_store)


