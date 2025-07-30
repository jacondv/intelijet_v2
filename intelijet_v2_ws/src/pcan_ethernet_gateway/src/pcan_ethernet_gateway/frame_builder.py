import struct

def build_udp_frame(can_id, data, is_extended=False):
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
