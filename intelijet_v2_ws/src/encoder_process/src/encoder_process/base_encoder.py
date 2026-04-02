import struct
import encoder_utils as solver

class BaseEncoder:
    """Interface chung cho tất cả encoder"""
    def decode_angle(self, data: bytes) -> float:
        """Decode giá trị góc từ CAN frame"""
        raise NotImplementedError

    def decode_raw(self, data: bytes) -> int:
        """Decode giá trị raw từ CAN frame"""
        raise NotImplementedError
    

class EncoderEROB(BaseEncoder):
    """Encoder kiểu float32 + int32"""
    def decode_angle(self, data: bytes) -> float:
        return struct.unpack('<f', data[0:4])[0]

    def decode_raw(self, data: bytes) -> int:
        return struct.unpack('<i', data[4:8])[0]
    

class Encoder58x8(BaseEncoder):
    """
    Encoder EROB: decode CAN frame theo solver của bạn
    - CAN data -> raw encoder value
    - CAN data -> draw_wire_length -> angle (rad)
    """
    

    def decode_angle(self, data: bytes) -> float:
        """
        Trả về góc (rad)
        """
        # convert bytes sang list of int nếu cần
        message_data = list(data)
        draw_wire_length = solver.convert_draw_wire_length(message_data)
        angle = solver.length_to_angle_polynomial(draw_wire_length)
        return angle

    def decode_raw(self, data: bytes) -> int:
        """
        Trả về raw encoder value (int)
        """
        message_data = list(data)
        raw_val = solver.onvert_can_to_encoder_value(message_data)  # nếu hàm cũ trả 1 giá trị
        return int(raw_val)