from PyQt5.QtCore import QObject, pyqtSignal

class SecurityManager(QObject):
    auth_changed = pyqtSignal(int)

    VIEWER = 0
    OPERATOR = 1
    ENGINEER = 2
    ADMIN = 3

    def __init__(self):
        super().__init__()
        self._level = self.VIEWER

    def level(self):
        return self._level

    def login(self, password: str) -> bool:
        if password == "operator":
            self._level = self.OPERATOR
        elif password == "engineer":
            self._level = self.ENGINEER
        elif password == "admin":
            self._level = self.ADMIN
        else:
            return False

        self.auth_changed.emit(self._level)
        return self._level

    def logout(self):
        self._level = self.VIEWER
        self.auth_changed.emit(self._level)


security = SecurityManager()
