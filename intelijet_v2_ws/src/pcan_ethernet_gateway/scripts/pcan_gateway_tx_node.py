#!/usr/bin/env python3

# This script receives CAN msgs from HMI or ROS topic, then converts to TCP and sends to PCAN Ethernet Gateway via TCP.

import rospy
# from pcan_ethernet_gateway.pcan_udp_driver import PcanUdpSender as PcanSender
from pcan_ethernet_gateway.pcan_tcp_driver import PcanTcpSender as PcanSender
from can_msgs.msg import Frame

PCAN_GATEWAY_IP         = "192.168.82.10"   # Replace with your actual gateway IP
PCAN_GATEWAY_SEND_PORT  = 56000             # This is the port for sending messages of PC
PCAN_GATEWAY_SEND_TOPIC = "/pcan_sent_messanges"  # Topic to subscribe CAN frames

class PcanGatewayTxNode:

    def __init__(self, gateway_ip, gateway_port):
        self.sender = PcanSender(gateway_ip=gateway_ip, gateway_port=gateway_port)
        rospy.Subscriber(PCAN_GATEWAY_SEND_TOPIC, Frame, self.callback)
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg: Frame):
        # Gửi frame qua TCP (can_msgs/Frame will be converted to TCP frame inside PcanTcpSender)
        self.sender.send_frame(msg)

    def shutdown(self):
        self.sender.close()
        rospy.loginfo("pcan_gateway_tx_node shutdown, TCP connection closed.")


def main():
    rospy.init_node("pcan_gateway_tx_node", anonymous=True)

    PcanGatewayTxNode(gateway_ip=PCAN_GATEWAY_IP,
                      gateway_port=PCAN_GATEWAY_SEND_PORT)

    rospy.loginfo("pcan_gateway_tx_node started")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())

