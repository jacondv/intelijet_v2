import socket
from can_msgs.msg import Frame
from pcan_ethernet_gateway.frame_builder import build_udp_frame
import struct

# This class reponsible for sending CAN frames over UDP to the PCAN Gateway
# |Can device| <-CAN frame-> |PCAN Gateway| <-UDP frame-> |PCAN Gateway Node| <-ROS can_msgs-> ROS Topic
class PcanUdpSender:
    def __init__(self, gateway_ip, gateway_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gateway_ip = gateway_ip
        self.gateway_port = gateway_port

    def send_frame(self, msg:Frame):
        can_id = msg.id
        data = list(msg.data[:msg.dlc])
        frame = self._decode_can_to_udp(can_id, data, is_extended=msg.is_extended)

        # Gửi frame UDP
        self.sock.sendto(frame, (self.gateway_ip, self.gateway_port))

    def _decode_can_to_udp(self,can_id, data, is_extended=False):
        dlc = len(data)
        flags = 0x02 if is_extended else 0x00

        # Header cố định (20 bytes) theo Developer Doku
        header = b'\x00\x24\x00\x80\x9d\x7c\x3c\xd5\xf0\x73\xae\x00\x35\xba\x5e\xf3\x00\x05\x40\x50'

        # DLC (1B) + Reserved (1B) + Flags (1B)
        frame = header + struct.pack('B', dlc) + struct.pack('BB', 0x00, flags)

        # CAN ID (4B big-endian)
        frame += struct.pack('>I', can_id)

        # Data 8 bytes (pad 0 nếu thiếu)
        frame += bytes(data) + bytes(8 - dlc)

        return frame


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
        for i in range(frame.dlc):
            frame.data[i] = message[28 + i]

        return frame