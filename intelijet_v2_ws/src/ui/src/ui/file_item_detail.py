# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'file_item_detail.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frmFileItemDetail(object):
    def setupUi(self, frmFileItemDetail):
        frmFileItemDetail.setObjectName("frmFileItemDetail")
        frmFileItemDetail.resize(577, 98)
        frmFileItemDetail.setStyleSheet("#lblRow1 {\n"
"    font-size: 13pt;\n"
"}\n"
"#lblRow2 {\n"
"    font-size: 15pt;\n"
"}\n"
"#lblRow3 {\n"
"    font-size: 12pt;\n"
"}\n"
"#chkChooseCloud::indicator {\n"
"        width: 30px;\n"
"        height: 30px;\n"
"    }")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(frmFileItemDetail)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.widget = QtWidgets.QWidget(frmFileItemDetail)
        self.widget.setMinimumSize(QtCore.QSize(0, 80))
        self.widget.setObjectName("widget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_3.setContentsMargins(9, 0, 50, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout.setObjectName("verticalLayout")
        self.lblRow1 = QtWidgets.QLabel(self.widget_2)
        self.lblRow1.setObjectName("lblRow1")
        self.verticalLayout.addWidget(self.lblRow1)
        self.lblRow2 = QtWidgets.QLabel(self.widget_2)
        self.lblRow2.setObjectName("lblRow2")
        self.verticalLayout.addWidget(self.lblRow2)
        self.lblRow3 = QtWidgets.QLabel(self.widget_2)
        self.lblRow3.setObjectName("lblRow3")
        self.verticalLayout.addWidget(self.lblRow3)
        self.horizontalLayout_3.addWidget(self.widget_2)
        self.widget_4 = QtWidgets.QWidget(self.widget)
        self.widget_4.setMaximumSize(QtCore.QSize(50, 16777215))
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.chkChooseCloud = QtWidgets.QCheckBox(self.widget_4)
        self.chkChooseCloud.setText("")
        self.chkChooseCloud.setObjectName("chkChooseCloud")
        self.horizontalLayout.addWidget(self.chkChooseCloud)
        self.horizontalLayout_3.addWidget(self.widget_4)
        self.widget_3 = QtWidgets.QWidget(self.widget)
        self.widget_3.setMaximumSize(QtCore.QSize(50, 16777215))
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.btnOpen = QtWidgets.QPushButton(self.widget_3)
        self.btnOpen.setMinimumSize(QtCore.QSize(40, 40))
        self.btnOpen.setMaximumSize(QtCore.QSize(40, 40))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/view.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnOpen.setIcon(icon)
        self.btnOpen.setIconSize(QtCore.QSize(30, 30))
        self.btnOpen.setObjectName("btnOpen")
        self.horizontalLayout_2.addWidget(self.btnOpen)
        self.horizontalLayout_3.addWidget(self.widget_3)
        self.horizontalLayout_4.addWidget(self.widget)

        self.retranslateUi(frmFileItemDetail)
        QtCore.QMetaObject.connectSlotsByName(frmFileItemDetail)

    def retranslateUi(self, frmFileItemDetail):
        _translate = QtCore.QCoreApplication.translate
        frmFileItemDetail.setWindowTitle(_translate("frmFileItemDetail", "Form"))
        self.lblRow1.setText(_translate("frmFileItemDetail", "#Job Name"))
        self.lblRow2.setText(_translate("frmFileItemDetail", "#Cloud name"))
        self.lblRow3.setText(_translate("frmFileItemDetail", "#Time Create"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frmFileItemDetail = QtWidgets.QWidget()
    ui = Ui_frmFileItemDetail()
    ui.setupUi(frmFileItemDetail)
    frmFileItemDetail.show()
    sys.exit(app.exec_())
