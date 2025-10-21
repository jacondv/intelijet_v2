# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'folder_item.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_FolderItem(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(506, 79)
        Form.setStyleSheet("    QLineEdit {\n"
"        border: none;\n"
"        background: transparent;\n"
"    }\n"
"\n"
"    QPushButton {\n"
"        border: none;           /* bỏ viền */\n"
"        background: transparent;/* nền trong suốt */\n"
"        padding: 0px;           /* không thừa margin */\n"
"    }\n"
"    QPushButton:hover {\n"
"        background: #e0e0e0;    /* hover highlight nhẹ */\n"
"        border-radius: 6px;\n"
"    }\n"
"")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout_2.setContentsMargins(0, 5, 0, 5)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setMinimumSize(QtCore.QSize(0, 45))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(-1, 0, 0, 0)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.txtName = QtWidgets.QLineEdit(self.widget)
        self.txtName.setMinimumSize(QtCore.QSize(0, 40))
        self.txtName.setMaximumSize(QtCore.QSize(16777215, 45))
        self.txtName.setObjectName("txtName")
        self.horizontalLayout.addWidget(self.txtName)
        self.btnSelect = QtWidgets.QPushButton(self.widget)
        self.btnSelect.setMinimumSize(QtCore.QSize(40, 40))
        self.btnSelect.setMaximumSize(QtCore.QSize(40, 40))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/check.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnSelect.setIcon(icon)
        self.btnSelect.setIconSize(QtCore.QSize(20, 20))
        self.btnSelect.setObjectName("btnSelect")
        self.horizontalLayout.addWidget(self.btnSelect)
        self.btnEdit = QtWidgets.QPushButton(self.widget)
        self.btnEdit.setMinimumSize(QtCore.QSize(40, 40))
        self.btnEdit.setMaximumSize(QtCore.QSize(40, 40))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/icon/note.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnEdit.setIcon(icon1)
        self.btnEdit.setIconSize(QtCore.QSize(25, 25))
        self.btnEdit.setObjectName("btnEdit")
        self.horizontalLayout.addWidget(self.btnEdit)
        self.btnSave = QtWidgets.QPushButton(self.widget)
        self.btnSave.setMinimumSize(QtCore.QSize(40, 40))
        self.btnSave.setMaximumSize(QtCore.QSize(40, 40))
        self.btnSave.setIcon(icon)
        self.btnSave.setIconSize(QtCore.QSize(20, 20))
        self.btnSave.setObjectName("btnSave")
        self.horizontalLayout.addWidget(self.btnSave)
        self.btnCancel = QtWidgets.QPushButton(self.widget)
        self.btnCancel.setMinimumSize(QtCore.QSize(40, 40))
        self.btnCancel.setMaximumSize(QtCore.QSize(40, 40))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/icon/icon/multiplication.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnCancel.setIcon(icon2)
        self.btnCancel.setIconSize(QtCore.QSize(20, 20))
        self.btnCancel.setObjectName("btnCancel")
        self.horizontalLayout.addWidget(self.btnCancel)
        self.btnDelete = QtWidgets.QPushButton(self.widget)
        self.btnDelete.setMinimumSize(QtCore.QSize(40, 40))
        self.btnDelete.setMaximumSize(QtCore.QSize(40, 40))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/icon/icon/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnDelete.setIcon(icon3)
        self.btnDelete.setIconSize(QtCore.QSize(20, 20))
        self.btnDelete.setObjectName("btnDelete")
        self.horizontalLayout.addWidget(self.btnDelete)
        self.horizontalLayout_2.addWidget(self.widget)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_FolderItem()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
