# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'job_item_detail.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frmFileItemDetail(object):
    def setupUi(self, frmFileItemDetail):
        frmFileItemDetail.setObjectName("frmFileItemDetail")
        frmFileItemDetail.resize(386, 111)
        self.gridLayout_2 = QtWidgets.QGridLayout(frmFileItemDetail)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.widget = QtWidgets.QWidget(frmFileItemDetail)
        self.widget.setMinimumSize(QtCore.QSize(0, 80))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.widget_2)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.label_2 = QtWidgets.QLabel(self.widget_2)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.label_3 = QtWidgets.QLabel(self.widget_2)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.horizontalLayout.addWidget(self.widget_2)
        self.widget_3 = QtWidgets.QWidget(self.widget)
        self.widget_3.setObjectName("widget_3")
        self.gridLayout = QtWidgets.QGridLayout(self.widget_3)
        self.gridLayout.setObjectName("gridLayout")
        self.btnOpen = QtWidgets.QPushButton(self.widget_3)
        self.btnOpen.setMinimumSize(QtCore.QSize(32, 32))
        self.btnOpen.setMaximumSize(QtCore.QSize(32, 32))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/visual.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnOpen.setIcon(icon)
        self.btnOpen.setIconSize(QtCore.QSize(30, 30))
        self.btnOpen.setObjectName("btnOpen")
        self.gridLayout.addWidget(self.btnOpen, 0, 0, 1, 1)
        self.horizontalLayout.addWidget(self.widget_3)
        self.horizontalLayout.setStretch(0, 5)
        self.horizontalLayout.setStretch(1, 1)
        self.gridLayout_2.addWidget(self.widget, 0, 0, 1, 1)

        self.retranslateUi(frmFileItemDetail)
        QtCore.QMetaObject.connectSlotsByName(frmFileItemDetail)

    def retranslateUi(self, frmFileItemDetail):
        _translate = QtCore.QCoreApplication.translate
        frmFileItemDetail.setWindowTitle(_translate("frmFileItemDetail", "Form"))
        self.label.setText(_translate("frmFileItemDetail", "#Job Name"))
        self.label_2.setText(_translate("frmFileItemDetail", "#Cloud name"))
        self.label_3.setText(_translate("frmFileItemDetail", "#Time Create"))
import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frmFileItemDetail = QtWidgets.QWidget()
    ui = Ui_frmFileItemDetail()
    ui.setupUi(frmFileItemDetail)
    frmFileItemDetail.show()
    sys.exit(app.exec_())
