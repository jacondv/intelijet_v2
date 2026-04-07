#!/usr/bin/env python3

# 2026-03-24 Change to eRob Motor, now we can read the Housing angle value from the PLC



import rospy
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from std_msgs.msg import Int32, Float32

from encoder_process import encoder_utils as solver
from encoder_process.base_encoder import BaseEncoder, EncoderEROB, Encoder58x8

from shared.config_loader import CONFIG
cfg = CONFIG
ENCODER_DATA_TOPIC =  cfg.ENCODER01_DATA

class EncoderProcessNode:
    def __init__(self, encoder: BaseEncoder):
        rospy.init_node("encoder_process_node")
        self.encoder = encoder

        # ROS Pub/Sub
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.Subscriber(cfg.ENCODER01_CAN_MSG, Frame, self.can_callback)
        self.pub_encoder_data = rospy.Publisher(ENCODER_DATA_TOPIC, Float32, queue_size=10)

    def can_callback(self, msg: Frame):
        data_bytes = bytes(msg.data)

        # decode theo encoder hiện tại
        angle = self.encoder.decode_angle(data_bytes)
        raw_val = self.encoder.decode_raw(data_bytes)

        # publish JointState
        joint_msg = JointState()
        joint_msg.header.frame_id = "base_link"
        joint_msg.header.stamp = rospy.Time.now()
        # Gắn encoder vào trục quay của robot, khai báo trong scanner_housing.urdf
        joint_msg.name = ["scanner_deploy_retract"]
        joint_msg.position = [angle]
        self.pub_joint_states.publish(joint_msg)

        # publish raw encoder
        msg_int = Int32()
        msg_int.data = int(raw_val)
        self.pub_encoder_data.publish(msg_int)


if __name__ == "__main__":
    # encoder_type = EncoderEROB()
    encoder_type = Encoder58x8()
    node = EncoderProcessNode(encoder=encoder_type)
    rospy.spin()
