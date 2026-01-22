from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLineEdit, QPushButton

class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Login")

        layout = QVBoxLayout(self)

        self.edit = QLineEdit()
        self.edit.setEchoMode(QLineEdit.Password)
        self.edit.setPlaceholderText("Enter password")
        layout.addWidget(self.edit)

        btn = QPushButton("Login")
        btn.clicked.connect(self.accept)
        layout.addWidget(btn)

    def password(self):
        return self.edit.text()
