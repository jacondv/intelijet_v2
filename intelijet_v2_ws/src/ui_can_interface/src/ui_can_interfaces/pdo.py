import struct
import yaml

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

    def define_field(self, field_name: str, byte_index: int, bit_index: int = None):
        self.fields[field_name] = (byte_index, bit_index)

    def set_field(self, field_name: str, value):
        if field_name not in self.fields:
            raise KeyError(f"Field '{field_name}' not defined in this PDO.")
        self[field_name] = value

    def __setitem__(self, name, value):
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
        return Frame(
            id=self.cob_id,
            is_extended=False,
            is_rtr=False,
            is_error=False,
            data=self.data
        )


def load_pdos_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    pdos = {}
    for name, pdo_cfg in config.get('pdos', {}).items():
        cob_id = pdo_cfg['cob_id']
        pdo = PDO(cob_id)  # chỉ truyền cob_id vì class PDO không cần name
        for field in pdo_cfg['fields']:
            pdo.define_field(
                name=field['name'],
                byte_index=field['byte'],
                size=field.get('size', 1),
                dtype=field['type'],
                bit_index=field.get('bit')  # optional
            )
        pdos[name] = pdo  # key là tên PDO trong YAML, ví dụ 'scanner_command'
    return pdos

