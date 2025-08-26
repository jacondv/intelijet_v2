from shared.config_loader import CONFIG as cfg
from PyQt5 import QtWidgets
import rospy
# ===== Hàm load dữ liệu vào UI =====

data_mapping = {
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


def upload_data_to_ui(widget, tube_dict = data_mapping):

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

def download_data_from_ui(widget, data_maping=data_mapping):
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
                print(obj_name, value)
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