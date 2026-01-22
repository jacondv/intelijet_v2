from PyQt5.QtCore import QObject, QEvent, QTimer
from PyQt5.QtWidgets import (
    QLineEdit, QTextEdit,
    QSpinBox, QDoubleSpinBox,
    QApplication
)
import subprocess
import shutil
import warnings


class TouchKeyboard(QObject):
    def __init__(self):
        super().__init__()
        self.proc = None

    def show_keyboard(self):
        # if shutil.which("onboard") is None:
        #     return
        
        if self.proc is None or self.proc.poll() is not None:
            try:
                self.proc = subprocess.Popen(
                    ["onboard"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except FileNotFoundError:
                warnings.warn("On-screen keyboard (onboard) is not installed.")
            except Exception as e:
                warnings.warn(f"Failed to launch onboard keyboard: {e}")

    def hide_keyboard(self):
        if self.proc:
            self.proc.terminate()
            self.proc = None

    def has_input_focus(self):
        w = QApplication.focusWidget()
        return isinstance(
            w,
            (QLineEdit, QTextEdit, QSpinBox, QDoubleSpinBox)
        )

    def update_keyboard(self):
        if self.has_input_focus():
            self.show_keyboard()
        else:
            self.hide_keyboard()

    def eventFilter(self, obj, event):
        if event.type() in (
            QEvent.FocusIn,
            QEvent.FocusOut,
            QEvent.WindowActivate,
            QEvent.WindowDeactivate,
        ):
            # Delay nhẹ để Qt xử lý xong focus (fix dialog case)
            QTimer.singleShot(50, self.update_keyboard)

        return False
