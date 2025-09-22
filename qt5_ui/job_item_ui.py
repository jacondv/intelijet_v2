# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'job_item.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frm_JobItem(object):
    def setupUi(self, frm_JobItem):
        frm_JobItem.setObjectName("frm_JobItem")
        frm_JobItem.resize(453, 23)
        self.gridLayout_2 = QtWidgets.QGridLayout(frm_JobItem)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setSpacing(0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.widget = QtWidgets.QWidget(frm_JobItem)
        self.widget.setObjectName("widget")
        self.gridLayout = QtWidgets.QGridLayout(self.widget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setHorizontalSpacing(6)
        self.gridLayout.setVerticalSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.txtJobItem = QtWidgets.QLineEdit(self.widget)
        self.txtJobItem.setObjectName("txtJobItem")
        self.gridLayout.addWidget(self.txtJobItem, 0, 0, 1, 1)
        self.btnJobEdit = QtWidgets.QPushButton(self.widget)
        self.btnJobEdit.setObjectName("btnJobEdit")
        self.gridLayout.addWidget(self.btnJobEdit, 0, 1, 1, 1)
        self.btnSave = QtWidgets.QPushButton(self.widget)
        self.btnSave.setObjectName("btnSave")
        self.gridLayout.addWidget(self.btnSave, 0, 2, 1, 1)
        self.btnDelete = QtWidgets.QPushButton(self.widget)
        self.btnDelete.setObjectName("btnDelete")
        self.gridLayout.addWidget(self.btnDelete, 0, 3, 1, 1)
        self.gridLayout_2.addWidget(self.widget, 0, 0, 1, 1)

        self.retranslateUi(frm_JobItem)
        QtCore.QMetaObject.connectSlotsByName(frm_JobItem)

    def retranslateUi(self, frm_JobItem):
        _translate = QtCore.QCoreApplication.translate
        frm_JobItem.setWindowTitle(_translate("frm_JobItem", "Form"))
        self.btnJobEdit.setText(_translate("frm_JobItem", "Edit"))
        self.btnSave.setText(_translate("frm_JobItem", "Save"))
        self.btnDelete.setText(_translate("frm_JobItem", "Del"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frm_JobItem = QtWidgets.QWidget()
    ui = Ui_frm_JobItem()
    ui.setupUi(frm_JobItem)
    frm_JobItem.show()
    sys.exit(app.exec_())
