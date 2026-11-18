# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pps.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Frame(object):
    def setupUi(self, Frame):
        Frame.setObjectName("Frame")
        Frame.resize(1147, 1038)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Frame.sizePolicy().hasHeightForWidth())
        Frame.setSizePolicy(sizePolicy)
        Frame.setBaseSize(QtCore.QSize(960, 600))
        Frame.setStyleSheet("QTabWidget {\n"
"    font-family: \"Arial\";\n"
"    font-size: 16pt;\n"
"    font-weight: normal; /* optional: normal, bold */\n"
"\n"
"}\n"
"")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(Frame)
        self.verticalLayout_4.setContentsMargins(10, 0, 10, 5)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.lblNotification = QtWidgets.QLabel(Frame)
        self.lblNotification.setStyleSheet("QLabel {\n"
"    font-family: \"Arial\";\n"
"    font-size: 13pt;\n"
"    font-weight: normal; /* optional: normal, bold */\n"
"\n"
"}\n"
"")
        self.lblNotification.setObjectName("lblNotification")
        self.verticalLayout_4.addWidget(self.lblNotification)
        self.tabWidget = QtWidgets.QTabWidget(Frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setMinimumSize(QtCore.QSize(0, 0))
        self.tabWidget.setBaseSize(QtCore.QSize(0, 0))
        self.tabWidget.setStyleSheet("\n"
"#tab_main {\n"
"    border-radius: 10px; /* bo góc frame nếu muốn */\n"
"    border: none; \n"
"}\n"
"\n"
"*{\n"
"    font-size: 14px;\n"
"}\n"
"\n"
"\n"
"")
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setIconSize(QtCore.QSize(32, 32))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_main = QtWidgets.QWidget()
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tab_main.sizePolicy().hasHeightForWidth())
        self.tab_main.setSizePolicy(sizePolicy)
        self.tab_main.setObjectName("tab_main")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.tab_main)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.cloudFrame = QtWidgets.QFrame(self.tab_main)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cloudFrame.sizePolicy().hasHeightForWidth())
        self.cloudFrame.setSizePolicy(sizePolicy)
        self.cloudFrame.setToolTip("")
        self.cloudFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.cloudFrame.setObjectName("cloudFrame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.cloudFrame)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2.addWidget(self.cloudFrame)
        self.frame_3 = QtWidgets.QFrame(self.tab_main)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_3.sizePolicy().hasHeightForWidth())
        self.frame_3.setSizePolicy(sizePolicy)
        self.frame_3.setStyleSheet("QPushButton {\n"
"    font-family: \"Arial\";\n"
"    font-size: 16pt;\n"
"    font-weight: bold; /* optional: normal, bold */\n"
"\n"
"}\n"
"")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setObjectName("frame_3")
        self.formLayout = QtWidgets.QFormLayout(self.frame_3)
        self.formLayout.setContentsMargins(5, 5, 5, 5)
        self.formLayout.setVerticalSpacing(20)
        self.formLayout.setObjectName("formLayout")
        self.logoLabel = QtWidgets.QLabel(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.logoLabel.sizePolicy().hasHeightForWidth())
        self.logoLabel.setSizePolicy(sizePolicy)
        self.logoLabel.setMaximumSize(QtCore.QSize(300, 130))
        self.logoLabel.setBaseSize(QtCore.QSize(0, 200))
        self.logoLabel.setText("")
        self.logoLabel.setPixmap(QtGui.QPixmap("intelijet_v2/jacon_logo.png"))
        self.logoLabel.setScaledContents(True)
        self.logoLabel.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.logoLabel.setObjectName("logoLabel")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.logoLabel)
        self.btnPreScan = QtWidgets.QPushButton(self.frame_3)
        self.btnPreScan.setMinimumSize(QtCore.QSize(0, 100))
        self.btnPreScan.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnPreScan.setObjectName("btnPreScan")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.btnPreScan)
        self.btnPostScan = QtWidgets.QPushButton(self.frame_3)
        self.btnPostScan.setMinimumSize(QtCore.QSize(0, 100))
        self.btnPostScan.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnPostScan.setObjectName("btnPostScan")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.btnPostScan)
        self.btnCompare = QtWidgets.QPushButton(self.frame_3)
        self.btnCompare.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCompare.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #63cdda, stop:1 #63cdda);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnCompare.setObjectName("btnCompare")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.btnCompare)
        self.btnCancel = QtWidgets.QPushButton(self.frame_3)
        self.btnCancel.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCancel.setAutoFillBackground(False)
        self.btnCancel.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e77f67, stop:1 #e15f41); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e77f67, stop:1 #e15f41); \n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e15f41, stop:1 #e15f41); \n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnCancel.setObjectName("btnCancel")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.btnCancel)
        self.btnOpenScanner = QtWidgets.QPushButton(self.frame_3)
        self.btnOpenScanner.setMinimumSize(QtCore.QSize(0, 100))
        self.btnOpenScanner.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #596275, stop:1 #596275); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #596275, stop:1 #596275);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #303952, stop:1 #303952);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnOpenScanner.setObjectName("btnOpenScanner")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.btnOpenScanner)
        self.btnCloseScanner = QtWidgets.QPushButton(self.frame_3)
        self.btnCloseScanner.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCloseScanner.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #596275, stop:1 #596275); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #596275, stop:1 #596275);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #303952, stop:1 #303952);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnCloseScanner.setObjectName("btnCloseScanner")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.btnCloseScanner)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.formLayout.setItem(7, QtWidgets.QFormLayout.FieldRole, spacerItem)
        self.btnShutdown = QtWidgets.QPushButton(self.frame_3)
        self.btnShutdown.setStyleSheet("QPushButton {\n"
"    border-radius: 10px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e77f67, stop:1 #e15f41); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e77f67, stop:1 #e15f41); \n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #e77f67, stop:1 #e15f41); \n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnShutdown.setObjectName("btnShutdown")
        self.formLayout.setWidget(8, QtWidgets.QFormLayout.FieldRole, self.btnShutdown)
        self.horizontalLayout_2.addWidget(self.frame_3)
        self.horizontalLayout_2.setStretch(0, 5)
        self.horizontalLayout_2.setStretch(1, 1)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.tabWidget.addTab(self.tab_main, "")
        self.tab_setting = QtWidgets.QWidget()
        self.tab_setting.setObjectName("tab_setting")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.tab_setting)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget.addTab(self.tab_setting, "")
        self.tab_system = QtWidgets.QWidget()
        self.tab_system.setObjectName("tab_system")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.tab_system)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.widget = QtWidgets.QWidget(self.tab_system)
        self.widget.setMaximumSize(QtCore.QSize(300, 250))
        self.widget.setObjectName("widget")
        self.gridLayout = QtWidgets.QGridLayout(self.widget)
        self.gridLayout.setObjectName("gridLayout")
        self.label_5 = QtWidgets.QLabel(self.widget)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 4, 0, 1, 3)
        self.lblCameraStatus = QtWidgets.QLabel(self.widget)
        self.lblCameraStatus.setObjectName("lblCameraStatus")
        self.gridLayout.addWidget(self.lblCameraStatus, 5, 3, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.widget)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 5, 0, 1, 2)
        self.label_3 = QtWidgets.QLabel(self.widget)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 2)
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.lblPLCStatus = QtWidgets.QLabel(self.widget)
        self.lblPLCStatus.setObjectName("lblPLCStatus")
        self.gridLayout.addWidget(self.lblPLCStatus, 0, 2, 1, 2)
        self.lblEncoder = QtWidgets.QLabel(self.widget)
        self.lblEncoder.setObjectName("lblEncoder")
        self.gridLayout.addWidget(self.lblEncoder, 3, 4, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.widget)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 0, 1, 1)
        self.lblPCANStatus = QtWidgets.QLabel(self.widget)
        self.lblPCANStatus.setObjectName("lblPCANStatus")
        self.gridLayout.addWidget(self.lblPCANStatus, 4, 3, 1, 1)
        self.lblEncoderStatus = QtWidgets.QLabel(self.widget)
        self.lblEncoderStatus.setObjectName("lblEncoderStatus")
        self.gridLayout.addWidget(self.lblEncoderStatus, 2, 2, 1, 2)
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 3, 1, 1, 3)
        self.lblLidarStatus = QtWidgets.QLabel(self.widget)
        self.lblLidarStatus.setObjectName("lblLidarStatus")
        self.gridLayout.addWidget(self.lblLidarStatus, 1, 2, 1, 2)
        self.gridLayout_2.addWidget(self.widget, 0, 0, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(20, 806, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem1, 1, 0, 1, 1)
        self.tabWidget.addTab(self.tab_system, "")
        self.verticalLayout_4.addWidget(self.tabWidget)

        self.retranslateUi(Frame)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Frame)
        Frame.setTabOrder(self.btnOpenScanner, self.btnPreScan)
        Frame.setTabOrder(self.btnPreScan, self.btnCompare)
        Frame.setTabOrder(self.btnCompare, self.btnCancel)
        Frame.setTabOrder(self.btnCancel, self.btnPostScan)
        Frame.setTabOrder(self.btnPostScan, self.btnShutdown)
        Frame.setTabOrder(self.btnShutdown, self.btnCloseScanner)

    def retranslateUi(self, Frame):
        _translate = QtCore.QCoreApplication.translate
        Frame.setWindowTitle(_translate("Frame", "Jacon Intelijet"))
        self.lblNotification.setText(_translate("Frame", "Notification..."))
        self.btnPreScan.setText(_translate("Frame", "Pre-Scan"))
        self.btnPostScan.setText(_translate("Frame", "Post-Scan"))
        self.btnCompare.setText(_translate("Frame", "Compare"))
        self.btnCancel.setText(_translate("Frame", "Cancel job"))
        self.btnOpenScanner.setText(_translate("Frame", "Open Housing"))
        self.btnCloseScanner.setText(_translate("Frame", "Close Housing"))
        self.btnShutdown.setText(_translate("Frame", "Close App"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_main), _translate("Frame", "Home"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_setting), _translate("Frame", "Setting"))
        self.label_5.setText(_translate("Frame", "PCAN Gatway:"))
        self.lblCameraStatus.setText(_translate("Frame", "--"))
        self.label_6.setText(_translate("Frame", "Camera:"))
        self.label_3.setText(_translate("Frame", "Encoder:"))
        self.label_2.setText(_translate("Frame", "Lidar:"))
        self.lblPLCStatus.setText(_translate("Frame", "--"))
        self.lblEncoder.setText(_translate("Frame", "--"))
        self.label_4.setText(_translate("Frame", "PLC:"))
        self.lblPCANStatus.setText(_translate("Frame", "--"))
        self.lblEncoderStatus.setText(_translate("Frame", "--"))
        self.label.setText(_translate("Frame", "Encoder value (rad):"))
        self.lblLidarStatus.setText(_translate("Frame", "--"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_system), _translate("Frame", "System"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Frame = QtWidgets.QFrame()
    ui = Ui_Frame()
    ui.setupUi(Frame)
    Frame.show()
    sys.exit(app.exec_())
