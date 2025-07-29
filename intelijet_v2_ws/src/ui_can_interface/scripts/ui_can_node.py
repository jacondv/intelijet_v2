#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from can_msgs.msg import Frame
from ui_can_interfaces import load_pdos_from_yaml
from ui_can_interfaces.command_handler import CommandHandler

class UICANInterface:
    def __init__(self):
        
        rospy.Subscriber("/hmi/cmd", String, self.command_callback)
        rospy.Subscriber("received_messages", Frame, self.recv_callback)

        #Configure the CAN PDOs from a YAML file
        _pdos = load_pdos_from_yaml("can_pdo_config.yaml")
        self.cmd_handler = CommandHandler(pdos=_pdos, can_topic_to_send="sent_messages")
        rospy.loginfo("ui_can_interface node started.")


    def command_callback(self, msg):
        command = msg.data.strip().lower()
        rospy.loginfo("Received control command: %s", command)
        result = self.cmd_handler.execute_command(command)
    
        if not result:
            rospy.logwarn("[WARN UICANInterface]Sent command [%s] failed %s", command)

    def recv_callback(self, frame):
        rospy.loginfo("Received CAN Frame: ID=0x%X, DLC=%d, DATA=%s", frame.id, frame.dlc, list(frame.data[:frame.dlc]))

if __name__ == "__main__":
    rospy.init_node("ui_can_node")
    UICANInterface()
    rospy.spin()
