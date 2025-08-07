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

        # Gán sẵn config thay vì lấy từ ROS param
        self.config = EncoderConfig(
            solver_type="KINEMATIC",
            coeffs=[0, 0, 1],
            model_angle_when_closed=0.2292,
            model_correction_gain=1.8,
            theoretical_draw_wire_length_when_closed=15,
            estimated_p1_p2_length=300,
            solution_tolerance=1e-8,
            kinematics=KinematicParams(
                r1=25.8,
                r2=45.3,
                b=42.0,
                c=393.6,
                d=309.5,
                e=93.1,
                f=231.6,
            )
        )

        self.solver_type = SolverType[self.config.solver_type.upper()]

        # ROS Pub/Sub
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.Subscriber("/encoder01/can_msg", Frame, self.can_callback)

    def can_callback(self, msg: Frame):
        draw_wire_length = solver.convert_draw_wire_length(msg.data)

        if self.solver_type == SolverType.POLYNOMIAL:
            angle = solver.length_to_angle_polynomial(
                draw_wire_length,
                self.config.coeffs,
                self.config.model_angle_when_closed,
                self.config.model_correction_gain
            )

        elif self.solver_type == SolverType.KINEMATIC:
            kin = self.config.kinematics
            F_func = solver.make_F(draw_wire_length, kin.r1, kin.r2, kin.b, kin.c, kin.d, kin.e, kin.f)
            angle = solver.length_to_angle_kinematic(
                draw_wire_length,
                F_func,
                self.config.estimated_p1_p2_length,
                self.config.solution_tolerance,
                kin.c,
                kin.d,
                self.config.model_angle_when_closed,
                self.config.model_correction_gain,
                self.config.theoretical_draw_wire_length_when_closed,
                kin.r1,
                kin.r2
            )


            if angle is None:
                return
            # rospy.logwarn(f"Length {draw_wire_length} to angle {angle} rad, {angle*180/3.14} deg.")
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