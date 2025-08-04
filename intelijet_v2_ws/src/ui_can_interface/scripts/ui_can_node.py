#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from can_msgs.msg import Frame
from ui_can_interface.command_handler import CommandHandler
from ui_can_interface.pdo import load_pdos_from_yaml

# We will send and receive CAN messages via PCAN Ethernet Gateway, so we need to define the topics.
# pcan_ehternet_gateway node will reposibility to communicate with real CAN bus.
PCAN_GATEWAY_RECV_TOPIC = "/pcan_received_messanges"  # Topic to publish CAN frames
PCAN_GATEWAY_SEND_TOPIC = "/pcan_sent_messanges"  # Topic to publish CAN frames

class UICANInterface:
    def __init__(self):
        
        rospy.Subscriber("/hmi/cmd", String, self.command_callback)
        rospy.Subscriber(PCAN_GATEWAY_RECV_TOPIC, Frame, self.recv_callback)

        #Configure the CAN PDOs from a YAML file
        _pdos = load_pdos_from_yaml("src/ui_can_interface/can_pdo_config.yaml")
        self.cmd_handler = CommandHandler(pdos=_pdos, can_topic_to_send=PCAN_GATEWAY_SEND_TOPIC)
        rospy.loginfo("ui_can_interface node started.")


    def command_callback(self, msg:String):
        command = msg.data.strip().lower()
        rospy.loginfo("Received control command: %s", command)
        result = self.cmd_handler.execute_command(command)
    
        if not result:
            rospy.logwarn("[WARN UICANInterface]Sent command [%s] failed %s", command)

    def recv_callback(self, frame:Frame):
        rospy.loginfo("Received CAN Frame: ID=0x%X, DLC=%d, DATA=%s", frame.id, frame.dlc, list(frame.data[:frame.dlc]))

if __name__ == "__main__":
    rospy.init_node("ui_can_node")
    UICANInterface()
    rospy.spin()
