import struct
import yaml
import yaml
import os
import rospkg



class PDO:
   
    def __init__(self, cob_id: int, size: int = 8):
        self.cob_id = cob_id
        self.data = bytearray(size)
        self.fields = {}  # Tùy chọn: ánh xạ tên -> (byte index, bit index or full byte)

    def set_byte(self, index: int, value: int):
        self.data[index] = value & 0xFF

    def get_byte(self, index: int) -> int:
        return self.data[index]

    def set_bit(self, byte_index: int, bit_index: int, value: bool):
        if value:
            self.data[byte_index] |= (1 << bit_index)
        else:
            self.data[byte_index] &= ~(1 << bit_index)

    def get_bit(self, byte_index: int, bit_index: int) -> bool:
        return (self.data[byte_index] >> bit_index) & 1

    def define_field(self, field_name: str, byte_index: int, bit_index: int = None, dtype: str = 'byte'):
        self.fields[field_name] = (byte_index, bit_index, dtype)

    def set_field(self, field_name: str, value):
        if field_name not in self.fields:
            raise KeyError(f"Field '{field_name}' not defined in this PDO.")
        self[field_name] = value

    def __setitem__(self, name, value):
        # Hàm này được ọi khi gán giá trị cho field
        byte_idx, length, dtype = self.fields[name]
        if dtype == 'byte':
            self.data[byte_idx] = value
        elif dtype == 'bool':
            # Lưu 1 bit trong byte
            bit_offset = length  # 0..7
            if value:
                self.data[byte_idx] |= (1 << bit_offset)
            else:
                self.data[byte_idx] &= ~(1 << bit_offset)
        elif dtype == 'int16':
            self.data[byte_idx:byte_idx+2] = struct.pack("<h", value)
        elif dtype == 'uint16' or dtype == 'word':
            self.data[byte_idx:byte_idx+2] = struct.pack("<H", value)
        elif dtype == 'uint32' or dtype == 'dword':
            self.data[byte_idx:byte_idx+4] = struct.pack("<I", value)
        elif dtype == 'float':
            self.data[byte_idx:byte_idx+4] = struct.pack("<f", value)
        else:
            raise ValueError(f"Unknown dtype: {dtype}")

    def __getitem__(self, name):
        byte_idx, length, dtype = self.fields[name]
        if dtype == 'byte':
            return self.data[byte_idx]
        elif dtype == 'bool':
            bit_offset = length
            return bool((self.data[byte_idx] >> bit_offset) & 1)
        elif dtype == 'int16':
            return struct.unpack("<h", self.data[byte_idx:byte_idx+2])[0]
        elif dtype == 'uint16' or dtype == 'word':
            return struct.unpack("<H", self.data[byte_idx:byte_idx+2])[0]
        elif dtype == 'uint32' or dtype == 'dword':
            return struct.unpack("<I", self.data[byte_idx:byte_idx+4])[0]
        elif dtype == 'float':
            return struct.unpack("<f", self.data[byte_idx:byte_idx+4])[0]
        else:
            raise ValueError(f"Unknown dtype: {dtype}")

    def to_frame(self):
        from can_msgs.msg import Frame
        data = list(self.data)
        data += [0] * (8 - len(data))
        return Frame(
            id=self.cob_id,
            is_extended=False,
            is_rtr=False,
            is_error=False,
            dlc=len(self.data),
            data = data
        )
    
    
    def from_frame(self, frame):
        """Load CAN frame data vào PDO"""
        if frame.id != self.cob_id:
            raise ValueError(f"Frame ID 0x{frame.id:X} không khớp với PDO 0x{self.cob_id:X}")
        self.data = bytearray(frame.data[:frame.dlc])  # copy dữ liệu từ CAN frame
        # Nếu frame nhỏ hơn size thì pad thêm
        if len(self.data) < len(self.fields):
            self.data += b'\x00' * (8 - len(self.data))
        return self

    def decode(self):
        """Trả về dict chứa tất cả các field đã giải mã"""
        decoded = {}
        for field_name in self.fields.keys():
            decoded[field_name] = self[field_name]
        return decoded


def load_pdos_from_yaml(yaml_path):
    caller_package = _guess_calling_package()

    if caller_package:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(caller_package)

    file_path = os.path.join(pkg_path, yaml_path) if caller_package else yaml_path
    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)

    pdos = {}
    for name, pdo_cfg in config.get('pdos', {}).items():
        cob_id = pdo_cfg['cob_id']
        pdo = PDO(cob_id)  # chỉ truyền cob_id vì class PDO không cần name
        for field in pdo_cfg['fields']:
            pdo.define_field(
                field_name=field['name'],
                byte_index=field['byte'], 
                bit_index=field.get('bit',0),  # optional bit or len
                dtype=field['type']

            )
        pdos[name] = pdo  # key là tên PDO trong YAML, ví dụ 'scanner_command'
    return pdos


def _guess_calling_package():
    """
    Tries to guess the calling package by inspecting __file__ variable
    of the caller. Only works if caller is in a ROS package.
    """
    import inspect
    try:
        # Lấy đường dẫn file gọi hàm này
        frame = inspect.stack()[2]
        module_path = os.path.abspath(frame.filename)

        # Dò ngược từ module_path để tìm gói ROS
        while module_path != "/" and module_path:
            if os.path.exists(os.path.join(module_path, "package.xml")):
                return os.path.basename(module_path)
            module_path = os.path.dirname(module_path)

    except Exception:
        pass

    return None


# from can_msgs.msg import Frame

# pdos = load_pdos_from_yaml("pdos.yaml")
# scanner_pdo = pdos["RxScannerCommandRos"]

# # Giả lập frame CAN
# frame = Frame(
#     id=801,
#     is_extended=False,
#     is_rtr=False,
#     is_error=False,
#     dlc=2,
#     data=[0b00000011, 10, 0, 0, 0, 0, 0, 0]  # bit0 & bit1 = True, byte1=10
# )

# # Giải mã frame
# scanner_pdo.from_frame(frame)
# decoded = scanner_pdo.decode()

# print(decoded)
# # Output:
# # {
# #   'bServiceOpenScanner': True,
# #   'bServiceCloseScanner': True,
# #   'gRx_byZeroRequest': 10
# # }
