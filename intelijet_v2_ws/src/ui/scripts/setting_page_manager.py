# setting_page_manager.py
from PyQt5 import QtWidgets
from PyQt5.QtGui import QDoubleValidator, QIntValidator
from PyQt5.QtWidgets import QMessageBox
from ui.setting_page_ui import Ui_setting_page  # file ui bạn vừa đưa
from ui.update_data_utils import *

class SettingPageManager(QtWidgets.QDialog, Ui_setting_page):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
 
        # --- Tạo validator ---
        # Target thickness: số thực -100 -> 100, tối đa 3 chữ số thập phân
        self.thickness_validator = QDoubleValidator(-100.0, 100.0, 3, self)
        self.thickness_validator.setNotation(QDoubleValidator.StandardNotation)
        self.txtTargetThickness.setValidator(self.thickness_validator)

        # Thickness tolerance: số thực 0 -> 50 ví dụ
        self.tolerance_validator = QDoubleValidator(0.0, 50.0, 2, self)
        self.tolerance_validator.setNotation(QDoubleValidator.StandardNotation)
        self.txtThicknessTolerance.setValidator(self.tolerance_validator)

        # Tunnel bounds: ví dụ cho ±100
        self.bound_validator = QDoubleValidator(-100.0, 100.0, 3, self)
        self.bound_validator.setNotation(QDoubleValidator.StandardNotation)
        self.txtTunnelMinBoundX.setValidator(self.bound_validator)
        self.txtTunnelMaxBoundX.setValidator(self.bound_validator)
        self.txtTunnelMinBoundY.setValidator(self.bound_validator)
        self.txtTunnelMaxBoundY.setValidator(self.bound_validator)
        self.txtTunnelMinBoundZ.setValidator(self.bound_validator)
        self.txtTunnelMaxBoundZ.setValidator(self.bound_validator)

        # Housing positions: số thực ±360 deg ví dụ
        self.angle_validator = QDoubleValidator(-360.0, 360.0, 2, self)
        self.angle_validator.setNotation(QDoubleValidator.StandardNotation)
        self.txtHousingStartPosition.setValidator(self.angle_validator)
        self.txtHousingEndPosition.setValidator(self.angle_validator)
        # Encoder value
        self.encoder_validator = QIntValidator(-2147483648, 2147483647, self)
        self.txtHousingClosedPosition.setValidator(self.encoder_validator) 

        # Housing speeds: số nguyên 0->5000
        self.speed_validator = QIntValidator(0, 5000, self)
        self.txtHousingOpenFast.setValidator(self.speed_validator)
        self.txtHousingOpenMedium.setValidator(self.speed_validator)
        self.txtHousingOpenSlow.setValidator(self.speed_validator)
        self.txtHousingCloseFast.setValidator(self.speed_validator)
        self.txtHousingCloseMedium.setValidator(self.speed_validator)
        self.txtHousingCloseSlow.setValidator(self.speed_validator)

        # --- Connect signal ---
        self.txtTargetThickness.editingFinished.connect(lambda: self.check_value(self.txtTargetThickness, self.thickness_validator, 0, 500))
        self.txtThicknessTolerance.editingFinished.connect(lambda: self.check_value(self.txtThicknessTolerance, self.tolerance_validator, 0, 50))

        # Tunnel bounds
        self.txtTunnelMinBoundX.editingFinished.connect(lambda: self.check_value(self.txtTunnelMinBoundX, self.bound_validator, -100, 100))
        self.txtTunnelMaxBoundX.editingFinished.connect(lambda: self.check_value(self.txtTunnelMaxBoundX, self.bound_validator, -100, 100))
        self.txtTunnelMinBoundY.editingFinished.connect(lambda: self.check_value(self.txtTunnelMinBoundY, self.bound_validator, -100, 100))
        self.txtTunnelMaxBoundY.editingFinished.connect(lambda: self.check_value(self.txtTunnelMaxBoundY, self.bound_validator, -100, 100))
        self.txtTunnelMinBoundZ.editingFinished.connect(lambda: self.check_value(self.txtTunnelMinBoundZ, self.bound_validator, -100, 100))
        self.txtTunnelMaxBoundZ.editingFinished.connect(lambda: self.check_value(self.txtTunnelMaxBoundZ, self.bound_validator, -100, 100))

        # Housing angles
        self.txtHousingStartPosition.editingFinished.connect(lambda: self.check_value(self.txtHousingStartPosition, self.angle_validator, -180, 180))
        self.txtHousingEndPosition.editingFinished.connect(lambda: self.check_value(self.txtHousingEndPosition, self.angle_validator, -180, 180))
        self.txtHousingClosedPosition.editingFinished.connect(lambda: self.check_value(self.txtHousingClosedPosition, self.encoder_validator, -2147483648, 2147483647))

        # Housing speeds
        self.txtHousingOpenFast.editingFinished.connect(lambda: self.check_value(self.txtHousingOpenFast, self.speed_validator, 0, 5000))
        self.txtHousingOpenMedium.editingFinished.connect(lambda: self.check_value(self.txtHousingOpenMedium, self.speed_validator, 0, 5000))
        self.txtHousingOpenSlow.editingFinished.connect(lambda: self.check_value(self.txtHousingOpenSlow, self.speed_validator, 0, 5000))
        self.txtHousingCloseFast.editingFinished.connect(lambda: self.check_value(self.txtHousingCloseFast, self.speed_validator, 0, 5000))
        self.txtHousingCloseMedium.editingFinished.connect(lambda: self.check_value(self.txtHousingCloseMedium, self.speed_validator, 0, 5000))
        self.txtHousingCloseSlow.editingFinished.connect(lambda: self.check_value(self.txtHousingCloseSlow, self.speed_validator, 0, 5000))

        # Buttons
        self.btnUpdateHousingParam.clicked.connect(self.on_update)
        self.btnCancelHousingParam.clicked.connect(self.on_cancel)

        self.btnSetClosedPosition.clicked.connect(self.set_encoder_at_zero_default) # Set current value of encoder

        load_config_to_ui(self)

    def check_value(self, widget, validator, min_val, max_val):
        text = widget.text()
        state, _, _ = validator.validate(text, 0)
        if state != validator.Acceptable:
            QtWidgets.QMessageBox.warning(self, "Error", f"Value from {min_val} to {max_val}")
            widget.setText(str(min_val if float(text or 0) < min_val else max_val))


    def on_update(self):
        reply = QMessageBox.question(
            self,
            "Confirm Update",
            "Are you sure you want to save these changes?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            # --- Get values from textboxes and save ---
            load_ui_to_config(self)

    
    def on_cancel(self):
        load_config_to_ui(self)

    def set_encoder_at_zero_default(self):
        text = self.txtEncodeValueRaw.text().strip()
        try:
            int(text)
            reply = QMessageBox.question(
                self,
                "Confirm Change",
                f"Are you sure you want to set the encoder value to {text}?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.txtHousingClosedPosition.setText(text)
                return True
            else:
                return False
        except ValueError:
            return False

        
        
        

# --- Test chạy ---
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    dlg = SettingPageManager()
    dlg.show()
    sys.exit(app.exec_())
