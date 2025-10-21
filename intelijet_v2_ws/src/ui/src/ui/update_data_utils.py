from enum import Enum
from shared.config_loader import CONFIG as cfg, save_config, reload_config 
from PyQt5 import QtWidgets
from PyQt5.QtCore import QObject

from PyQt5.QtWidgets import QWidget, QLineEdit, QLabel, QPushButton, QCheckBox
from shared.msg import DeviceStatus
from pps.helper import notify_one

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
    "txtTargetThickness":{
        "get": lambda: cfg.thickness.target,
        "set": lambda v: setattr(cfg.thickness,"target", v)
    },
        "txtThicknessTolerance":{
        "get": lambda: cfg.thickness.tolerance,
        "set": lambda v: setattr(cfg.thickness,"tolerance", v)
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
    save_config(cfg)

    notify_one("/system/config/update")
    return cfg


class Status(Enum):
    ACTIVE   = ("Active", "#00FF00")
    INACTIVE = ("Inactive", "#CCCCCC")
    DEFAULT  = ("Default", "#FFFFFF")
    WARNING  = ("Warning", "#FFA500")
    ERROR    = ("Error", "#FF0000")
    READY    = ("Ready", "#0000FF")

    def label(self):
        return self.value[0]

    def color(self):
        return self.value[1]

def set_control_button_stage(widget, state=Status.DEFAULT):
    hex_color = state.color()
    style_dict = {
        "border-left": f"8px solid {hex_color}",
        "padding": "40px 0px 40px 0px"
    }
    style_str = "; ".join([f"{k}: {v}" for k, v in style_dict.items()])
    if widget:
        widget.setStyleSheet(style_str)



class DataBinder:
    def __init__(self,root_widget: QtWidgets.QWidget):
        """
        mapping: dict ánh xạ key trong status -> objectName của widget
        root_widget: QMainWindow/QWidget gốc
        """

        self.root_widget = root_widget
        self._widget_cache = {}
        self._build_cache()

    def _build_cache(self):
        """Quét toàn bộ widget con và lưu theo objectName"""
        for child in self.root_widget.findChildren(QtWidgets.QWidget):
            name = child.objectName()
            if name:  # chỉ cache widget có đặt tên
                self._widget_cache[name] = child


    def _update_control_button_style(self, status: dict):

        if "devices" not in status:
            return
        for key, values in status["devices"].items():

            if key.lower() in ['pps']:
                value = values['device_state']
                if value == DeviceStatus.PRESCAN:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if not widget:
                            continue  
                        if btn_name == 'btnPreScan':
                            set_control_button_stage(widget, Status.ACTIVE)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)
                elif value == DeviceStatus.POSTSCAN:

                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if btn_name == 'btnPostScan':               
                            set_control_button_stage(widget, Status.ACTIVE)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)
                
                elif value == DeviceStatus.OPEN_HOUSING:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if btn_name == 'btnOpenScanner':               
                            set_control_button_stage(widget, Status.ACTIVE)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)
                
                elif value == DeviceStatus.CLOSE_HOUSING:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)                        
                        if btn_name == 'btnCloseScanner':               
                            set_control_button_stage(widget, Status.ACTIVE)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)

                elif value == DeviceStatus.IDLE:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if not widget:
                            continue  
                        set_control_button_stage(widget, Status.INACTIVE)

                elif value == DeviceStatus.PRESCAN_ERROR:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if btn_name == 'btnPreScan':               
                            set_control_button_stage(widget, Status.ERROR)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)

                elif value == DeviceStatus.POSTSCAN_ERROR:
                    for btn_name in ['btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel', 'btnOpenScanner', 'btnCloseScanner']:
                        widget = self._widget_cache.get(btn_name)
                        if btn_name == 'btnPostScan':               
                            set_control_button_stage(widget, Status.ERROR)
                        else:
                            set_control_button_stage(widget, Status.INACTIVE)

                return

                
    def update_ui_from_status(self, status: dict):
        self._update_control_button_style(status)
       


class UiObjectManager(QObject):
    def __init__(self, parent_widget: QWidget):
        super().__init__(parent_widget)
        self.widgets = {}

        # Lưu tất cả widget con có objectName
        for w in parent_widget.findChildren(QWidget):
            if w.objectName():
                self.widgets[w.objectName()] = w

    def object_set_value(self, name: str, value):
        if name not in self.widgets:
            print(f"[WARN] Không tìm thấy widget: {name}")
            return

        w = self.widgets[name]

        if isinstance(w, QLineEdit):
            w.setText(str(value))
        elif isinstance(w, QLabel):
            w.setText(str(value))
        elif isinstance(w, QPushButton):
            w.setText(str(value))
        elif isinstance(w, QCheckBox):
            w.setChecked(bool(value))
        else:
            print(f"[INFO] Widget {name} ({type(w)}) chưa hỗ trợ set_value")

    def object_set_style(self, name: str, style: str):
        if name not in self.widgets:
            print(f"[WARN] Không tìm thấy widget: {name}")
            return

        self.widgets[name].setStyleSheet(style)
