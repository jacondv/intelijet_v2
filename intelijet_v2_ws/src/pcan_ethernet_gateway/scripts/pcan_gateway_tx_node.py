#!/usr/bin/env python3
import rospy
from pcan_ethernet_gateway.frame_builder import build_udp_frame
from pcan_ethernet_gateway.pcan_udp_driver import PcanUdpSender
from can_msgs.msg import Frame

PCAN_GATEWAY_IP = "192.168.1.10" # Replace with your actual gateway IP
PCAN_GATEWAY_SEND_PORT  = 56000 # This is the port for sending messages of PC
PCAN_GATEWAY_SEND_TOPIC = "/pcan_sent_messanges"  # Topic to publish CAN frames

class PcanGatewayTxNode:

    def __init__(self, gateway_ip, gateway_port):
        self.sender = PcanUdpSender(gateway_ip=gateway_ip, gateway_port=gateway_port)
        rospy.Subscriber(PCAN_GATEWAY_SEND_TOPIC, Frame, self.callback)  # hoặc String nếu topic là String

    def callback(self, msg: Frame):
        can_id = msg.id
        data = list(msg.data[:msg.dlc])
        frame = build_udp_frame(can_id, data, is_extended=msg.is_extended)
        self.sender.send_frame(frame)
        rospy.loginfo(f"Sent CAN frame: ID=0x{can_id:X}, Data={data}")


def main():
    rospy.init_node("pcan_gateway_tx_node", anonymous=True)
   
    PcanGatewayTxNode(gateway_ip=PCAN_GATEWAY_IP , 
                    gateway_port=PCAN_GATEWAY_SEND_PORT )
    

    rospy.loginfo("pcan_gateway_tx_node started")
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())
    
    
