# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'jobselect_dlg.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1061, 683)
        Dialog.setStyleSheet("QLineEdit#txtSearch{\n"
"    border-radius: 20px;\n"
"    padding-left:10px;\n"
"}")
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(870, 640, 166, 24))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setGeometry(QtCore.QRect(20, 10, 561, 56))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(6, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.txtSearch = QtWidgets.QLineEdit(self.widget)
        self.txtSearch.setMinimumSize(QtCore.QSize(0, 40))
        self.txtSearch.setObjectName("txtSearch")
        self.horizontalLayout.addWidget(self.txtSearch)
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.cbShow = QtWidgets.QComboBox(self.widget)
        self.cbShow.setMinimumSize(QtCore.QSize(0, 40))
        self.cbShow.setObjectName("cbShow")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.horizontalLayout.addWidget(self.cbShow)
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.horizontalLayout.setStretch(0, 5)
        self.horizontalLayout.setStretch(3, 1)
        self.widget_2 = QtWidgets.QWidget(Dialog)
        self.widget_2.setGeometry(QtCore.QRect(20, 80, 1011, 551))
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.lstJobs = QtWidgets.QListWidget(self.widget_2)
        self.lstJobs.setObjectName("lstJobs")
        self.verticalLayout_2.addWidget(self.lstJobs)
        self.widget_3 = QtWidgets.QWidget(self.widget_2)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setContentsMargins(0, -1, 0, -1)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lstJobItems = QtWidgets.QListWidget(self.widget_3)
        self.lstJobItems.setObjectName("lstJobItems")
        self.horizontalLayout_2.addWidget(self.lstJobItems)
        self.widget_4 = QtWidgets.QWidget(self.widget_3)
        self.widget_4.setMaximumSize(QtCore.QSize(80, 16777215))
        self.widget_4.setObjectName("widget_4")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_4)
        self.verticalLayout.setContentsMargins(-1, 30, -1, -1)
        self.verticalLayout.setObjectName("verticalLayout")
        self.btnAdd = QtWidgets.QPushButton(self.widget_4)
        self.btnAdd.setMinimumSize(QtCore.QSize(0, 40))
        self.btnAdd.setObjectName("btnAdd")
        self.verticalLayout.addWidget(self.btnAdd)
        self.btnRemove = QtWidgets.QPushButton(self.widget_4)
        self.btnRemove.setMinimumSize(QtCore.QSize(0, 40))
        self.btnRemove.setObjectName("btnRemove")
        self.verticalLayout.addWidget(self.btnRemove)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout_2.addWidget(self.widget_4)
        self.lstJobCompare = QtWidgets.QListWidget(self.widget_3)
        self.lstJobCompare.setObjectName("lstJobCompare")
        self.horizontalLayout_2.addWidget(self.lstJobCompare)
        self.verticalLayout_2.addWidget(self.widget_3)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.txtSearch.setPlaceholderText(_translate("Dialog", "Search by name or keyword…"))
        self.label.setText(_translate("Dialog", "Show"))
        self.cbShow.setItemText(0, _translate("Dialog", "10"))
        self.cbShow.setItemText(1, _translate("Dialog", "20"))
        self.cbShow.setItemText(2, _translate("Dialog", "50"))
        self.cbShow.setItemText(3, _translate("Dialog", "All"))
        self.label_2.setText(_translate("Dialog", "items"))
        self.btnAdd.setText(_translate("Dialog", ">>"))
        self.btnRemove.setText(_translate("Dialog", "<<"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
