#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from enum import Enum

from encoder_process.data_models import EncoderConfig, KinematicParams, SolverType
from encoder_process import encoder_utils as solver


class EncoderProcessNode:
    def __init__(self):
        rospy.init_node("encoder_process_node")

        self.solver_type = SolverType.POLYNOMIAL

        # ROS Pub/Sub
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.Subscriber("/encoder01/can_msg", Frame, self.can_callback)

    def can_callback(self, msg: Frame):
        draw_wire_length = solver.convert_draw_wire_length(msg.data)
 
        if self.solver_type == SolverType.POLYNOMIAL:
            angle = solver.length_to_angle_polynomial(draw_wire_length)

        else:
            rospy.logwarn(f"Solver type '{self.solver_type}' is invalid, using raw length as angle.")
            angle = draw_wire_length

        rospy.logwarn(f"[EncoderProcessNode] Length {draw_wire_length} to angle {angle} rad, {angle*180/3.14} deg.")

        # Publish JointState
        joint_msg = JointState()
        joint_msg.header.frame_id = "base_link"
        joint_msg.header.stamp = rospy.Time.now()
        # Gắn encoder vào trục quay của robot, khai báo trong scanner_housing.urdf
        joint_msg.name = ["scanner_deploy_retract"] 
        joint_msg.position = [angle]
        self.pub_joint_states.publish(joint_msg)


if __name__ == "__main__":
    node = EncoderProcessNode()
    rospy.spin()