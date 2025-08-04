import socket
from can_msgs.msg import Frame
from pcan_ethernet_gateway.frame_builder import build_udp_frame
import struct
import binascii
import rospy

# This class reponsible for sending CAN frames over UDP to the PCAN Gateway
# |Can device| <-CAN frame-> |PCAN Gateway| <-UDP frame-> |PCAN Gateway Node| <-ROS can_msgs-> ROS Topic

def log_udp_frame(frame: bytes):
    header = frame[0:20]
    dlc = frame[20]
    reserved = frame[21]
    flags = frame[22]
    can_id = int.from_bytes(frame[23:27], "big")
    data = frame[27:27 + dlc]
    rospy.loginfo(f"Header: {header.hex()}")
    rospy.loginfo(f"DLC: {dlc}, Reserved: {reserved}, Flags: {flags}")
    rospy.loginfo(f"CAN ID: 0x{can_id:X}")
    rospy.loginfo(f"Data: {[hex(b) for b in data]}")

class PcanUdpSender:
    def __init__(self, gateway_ip, gateway_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gateway_ip = gateway_ip
        self.gateway_port = gateway_port

    def send_frame(self, msg:Frame):
        can_id = msg.id
        data = list(msg.data[:msg.dlc])
        frame = self._decode_can_to_udp(can_id, data, is_extended=msg.is_extended)

        log_udp_frame(frame)

        # Gửi frame UDP
        self.sock.sendto(frame, (self.gateway_ip, self.gateway_port))

    def _decode_can_to_udp(self,can_id, data, is_extended=False):
        dlc = len(data)
        flags = 0x02 if is_extended else 0x00

        # Tạo mảng 36 bytes ban đầu (giống msg_list mẫu)
        msg_list = [0x00, 0x24, 0x00, 0x80, 0x9d, 0x7c, 0x3c,0xd5,0xf0,0x73,0xae,0x00,0x35,0xba,0x5e,0xf3,0x00,0x05,0x40,0x50,0x00,0x08,0x00,0x02,0x80,0x0c,0x12,0x34,0x8f,0x8e,0xb8,0x0c,0x00,0x00,0x00,0x00]


        # Pos 21 = DLC
        msg_list[21] = dlc

        # Pos 22/23 = Reserved + Flags
        msg_list[22] = 0x00
        msg_list[23] = flags

        # Pos 24/27 = CAN ID (11-bit trong 4 byte big-endian)
        msg_list[24:28] = list(struct.pack(">I", can_id))

        # Pos 28/35 = Dữ liệu (pad 0 nếu thiếu)
        data_bytes = list(data) + [0] * (8 - dlc)
        msg_list[28:36] = data_bytes

        return bytes(msg_list)


# This class reponsible for receiving CAN frames over UDP from the PCAN Gateway
# |Can device| <-CAN frame-> |PCAN Gateway| <-UDP frame-> |PCAN Gateway Node| <-ROS can_msgs-> ROS Topic
class PcanUdpReceiver:
    def __init__(self, listen_ip: str, listen_port: int):
        """UDP Receiver cho PCAN Gateway"""

        # Bảng chuyển đổi DLC → số byte
        self.DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((listen_ip, listen_port))
        self.sock.setblocking(False)  # Non-blocking để không treo loop

    def receive_frame(self)-> Frame:
        """Nhận 1 frame UDP và trả về can_msgs/Frame hoặc None nếu không có dữ liệu"""
        try:
            data, _ = self.sock.recvfrom(1024)
            return self._decode_udp_to_can(data)
        except BlockingIOError:
            return None

    def _decode_udp_to_can(self, message: bytes):
        """Chuyển đổi gói UDP từ PCAN Gateway thành can_msgs/Frame"""
        if len(message) < 32:  # Gói tin không hợp lệ
            return None

        frame = Frame()

        # Flags (Byte 22-23)
        can_msg_flag = int.from_bytes(message[22:24], byteorder="big", signed=False)
        frame.is_extended = bool(can_msg_flag & 0x80)

        # CAN ID (Byte 24-27)
        can_id_raw = int.from_bytes(message[24:28], byteorder="big", signed=False)
        frame.id = can_id_raw & 0x1FFFFFFF  # Mask 29 bit

        # DLC (Byte 21)
        dlc = message[21]
        length = self.DLC_TO_LEN[dlc] if dlc < len(self.DLC_TO_LEN) else 8
        frame.dlc = min(length, 8)  # ROS Frame chỉ hỗ trợ 8 bytes (CAN 2.0)

        # Data bytes
        frame.data = message[28:28 + frame.dlc]

        return frame