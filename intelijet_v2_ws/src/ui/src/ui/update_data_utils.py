from shared.config_loader import CONFIG as cfg
from PyQt5 import QtWidgets
from shared.msg import DeviceStatus

import rospy
# ===== Hàm load dữ liệu vào UI =====

config_mapping = {
    "txtHousingOpenFast": {
        "get": lambda: cfg.housing_open_speed_fast,
        "set": lambda v: setattr(cfg, "housing_open_speed_fast", v)
    },
    "txtHousingOpenMedium": {
        "get": lambda: cfg.housing_open_speed_medium,
        "set": lambda v: setattr(cfg, "housing_open_speed_medium", v)
    },
    "txtHousingOpenSlow": {
        "get": lambda: cfg.housing_open_speed_slow,
        "set": lambda v: setattr(cfg, "housing_open_speed_slow", v)
    },

    "txtHousingCloseFast": {
        "get": lambda: cfg.housing_close_speed_fast,
        "set": lambda v: setattr(cfg, "housing_close_speed_fast", v)
    },
    "txtHousingCloseMedium": {
        "get": lambda: cfg.housing_close_speed_medium,
        "set": lambda v: setattr(cfg, "housing_close_speed_medium", v)
    },
    "txtHousingCloseSlow": {
        "get": lambda: cfg.housing_close_speed_slow,
        "set": lambda v: setattr(cfg, "housing_close_speed_slow", v)
    },

    "txtHousingStartPosition": {
        "get": lambda: cfg.housing_start_position,
        "set": lambda v: setattr(cfg, "housing_start_position", v)
    },
    "txtHousingEndPosition": {
        "get": lambda: cfg.housing_end_position,
        "set": lambda v: setattr(cfg, "housing_end_position", v)
    },
    "txtHousingClosedPosition": {
        "get": lambda: cfg.encoder_length_at_zero_possition,
        "set": lambda v: setattr(cfg, "encoder_length_at_zero_possition", v)
    },

    "txtTunnelMinBoundX": {
        "get": lambda: cfg.crop_box.min.x,
        "set": lambda v: setattr(cfg.crop_box.min, "x", v)
    },
    "txtTunnelMinBoundY": {
        "get": lambda: cfg.crop_box.min.y,
        "set": lambda v: setattr(cfg.crop_box.min, "y", v)
    },
    "txtTunnelMinBoundZ": {
        "get": lambda: cfg.crop_box.min.z,
        "set": lambda v: setattr(cfg.crop_box.min, "z", v)
    },
    "txtTunnelMaxBoundX": {
        "get": lambda: cfg.crop_box.max.x,
        "set": lambda v: setattr(cfg.crop_box.max, "x", v)
    },
    "txtTunnelMaxBoundY": {
        "get": lambda: cfg.crop_box.max.y,
        "set": lambda v: setattr(cfg.crop_box.max, "y", v)
    },
    "txtTunnelMaxBoundZ": {
        "get": lambda: cfg.crop_box.max.z,
        "set": lambda v: setattr(cfg.crop_box.max, "z", v)
    },
}

def load_config_to_ui(widget, tube_dict = config_mapping):

    """
    widget: container widget
    tubes: list of tuples {objectid, value,...}
    """

    # Tạo dict để tra cứu nhanh
    # tube_dict = {obj: val for obj, val in tubes}

    for child in widget.findChildren(QtWidgets.QWidget):
        obj_name = child.objectName()
        if obj_name in tube_dict:
            value = tube_dict[obj_name]["get"]()  # Lấy giá trị từ hàm get
            if isinstance(child, QtWidgets.QLineEdit):
                child.setText(str(value))
            elif isinstance(child, QtWidgets.QTextEdit):
                child.setPlainText(str(value))
            elif isinstance(child, QtWidgets.QLabel):
                child.setText(str(value))

def load_ui_to_config(widget, data_maping=config_mapping):
    """
    Duyệt tất cả widget con, lấy giá trị hiện tại và cập nhật vào data_maping.
    Nếu giá trị có thể chuyển thành số (int/float) thì parse, nếu không giữ string.
    """
    for child in widget.findChildren(QtWidgets.QWidget):
        obj_name = child.objectName()
        
        if not obj_name:
            continue
              
        if obj_name in data_maping:
            # Lấy giá trị hiện tại của widget
            if isinstance(child, QtWidgets.QLineEdit):
                value = child.text()

            elif isinstance(child, QtWidgets.QTextEdit):
                value = child.toPlainText()
                   
            elif isinstance(child, QtWidgets.QLabel):
                value = child.text()
            else:
                continue

            # Thử parse thành số
            try:
                if '.' in value:
                    value = float(value)
                else:
                    value = int(value)
            except ValueError:
                pass  # nếu không parse được thì giữ nguyên string

            data_maping[obj_name]["set"](value)
            
            # Cập nhật lại CONFIG tương ứng
    return cfg


# ===== Hàm set style cho button =====
status_ui_mapping = {
    "lidar": "lblLidarStatus",  
    "encoder": "lblEncoderStatus",
    "pcan": "lblPCANStatus",
    "camera": "lblCameraStatus",
}


BUTTON_COLORS = {
    "active":   "#00FF00",  # Xanh lá - đang hoạt động
    "inactive": "#CCCCCC",  # Xám - không hoạt động
    "default":  "#FFA500",  # Trắng - trạng thái mặc định
    "warning":  "#FFA500",  # Cam - cảnh báo
    "error":    "#FF0000",  # Đỏ - lỗi nghiêm trọng
    "ready":    "#0000FF",  # Xanh dương - sẵn sàng
}

button_state = {

    "btnPreScan": {
        "border-left": f"8px solid {BUTTON_COLORS['default']}"
    },
    "btnPostScan": {
        "border-left": f"8px solid  {BUTTON_COLORS['default']}"    
    },
    "btnCompare": {
        "border-left": f"8px solid  {BUTTON_COLORS['default']}"   
    },
    "btnCancel": {
        "border-left": f"8px solid  {BUTTON_COLORS['default']}"   
    },
    "btnOpenScanner": {
        "border-left": f"8px solid  {BUTTON_COLORS['default']}"    
    },
    "btnCloseScanner": {
        "border-left": f"8px solid  {BUTTON_COLORS['default']}"    
    }

}

def set_button_stage(button_name, state="default"):

    global button_state

    hex_color = BUTTON_COLORS.get(state, BUTTON_COLORS['default'])
    # đảm bảo hợp lệ
    if not (isinstance(hex_color, str) and hex_color.startswith("#") and len(hex_color) == 7):
        raise ValueError("The color must be in hex #RRGGBB, for example: #FF0000")

    # Cập nhật vào state_map
    button_state[button_name] = {
        "border-left": f"2px solid {hex_color}"
    }


from PyQt5 import QtWidgets

from PyQt5 import QtWidgets



class DataBinder:
    def __init__(self,root_widget: QtWidgets.QWidget, mapping = status_ui_mapping):
        """
        mapping: dict ánh xạ key trong status -> objectName của widget
        root_widget: QMainWindow/QWidget gốc
        """
        self.mapping = mapping
        self.root_widget = root_widget
        self._widget_cache = {}
        self._build_cache()

    def _build_cache(self):
        """Tìm và lưu tất cả widget vào cache"""
        for key, widget_name in self.mapping.items():
            child = self.root_widget.findChild(QtWidgets.QWidget, widget_name)
            if child:
                self._widget_cache[key] = child

    def update_ui_from_status(self, status: dict):
        """Update UI từ dict status, dùng cache để tránh findChild nhiều lần"""
        for key, values in status.items():
            value = values['device_state']
            child = self._widget_cache.get(key)
            if not child:
                continue
            if isinstance(child, (QtWidgets.QLineEdit, QtWidgets.QLabel)):
                child.setText(str(value))
            elif isinstance(child, QtWidgets.QCheckBox):
                child.setChecked(bool(value))
            elif isinstance(child, QtWidgets.QSpinBox):
                child.setValue(int(value))
            elif isinstance(child, QtWidgets.QComboBox):
                idx = child.findText(str(value))
                if idx >= 0:
                    child.setCurrentIndex(idx)
