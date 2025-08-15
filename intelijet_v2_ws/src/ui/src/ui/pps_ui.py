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
        Frame.setWindowModality(QtCore.Qt.NonModal)
        Frame.resize(1147, 1100)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Frame.sizePolicy().hasHeightForWidth())
        Frame.setSizePolicy(sizePolicy)
        Frame.setMinimumSize(QtCore.QSize(0, 0))
        Frame.setMaximumSize(QtCore.QSize(16777215, 16777215))
        Frame.setBaseSize(QtCore.QSize(960, 600))
        Frame.setStyleSheet("")
        self.verticalLayout = QtWidgets.QVBoxLayout(Frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(Frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setMinimumSize(QtCore.QSize(0, 0))
        self.tabWidget.setBaseSize(QtCore.QSize(0, 0))
        self.tabWidget.setStyleSheet("")
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Triangular)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_main = QtWidgets.QWidget()
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tab_main.sizePolicy().hasHeightForWidth())
        self.tab_main.setSizePolicy(sizePolicy)
        self.tab_main.setObjectName("tab_main")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.tab_main)
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
        self.cloudFrame.setMinimumSize(QtCore.QSize(0, 0))
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
        self.frame_3.setStyleSheet("QFrame {\n"
"    border-radius: 10px; /* bo góc frame nếu muốn */\n"
"    border: 1px solid #E67E22; /* viền frame */\n"
"    \n"
"    background-color: qlineargradient(spread:reflect, x1:0, y1:1, x2:0, y2:0, stop:0 rgba(245, 205, 121,1.0), stop:1 rgba(247, 255, 255,1.0));\n"
"\n"
"\n"
"}")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setObjectName("frame_3")
        self.formLayout = QtWidgets.QFormLayout(self.frame_3)
        self.formLayout.setLabelAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.formLayout.setFormAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.formLayout.setVerticalSpacing(20)
        self.formLayout.setObjectName("formLayout")
        self.logoLabel = QtWidgets.QLabel(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.logoLabel.sizePolicy().hasHeightForWidth())
        self.logoLabel.setSizePolicy(sizePolicy)
        self.logoLabel.setMaximumSize(QtCore.QSize(300, 150))
        self.logoLabel.setBaseSize(QtCore.QSize(0, 200))
        self.logoLabel.setText("")
        self.logoLabel.setPixmap(QtGui.QPixmap("/root/intelijet_v2/intelijet_v2_ws/src/ui/src/ui/jacon_logo.png"))
        self.logoLabel.setScaledContents(True)
        self.logoLabel.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.logoLabel.setObjectName("logoLabel")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.logoLabel)
        self.btnPreScan = QtWidgets.QPushButton(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnPreScan.sizePolicy().hasHeightForWidth())
        self.btnPreScan.setSizePolicy(sizePolicy)
        self.btnPreScan.setMinimumSize(QtCore.QSize(0, 100))
        self.btnPreScan.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.btnPreScan.setBaseSize(QtCore.QSize(0, 200))
        self.btnPreScan.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnPreScan.setObjectName("btnPreScan")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.btnPreScan)
        self.btnPostScan = QtWidgets.QPushButton(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnPostScan.sizePolicy().hasHeightForWidth())
        self.btnPostScan.setSizePolicy(sizePolicy)
        self.btnPostScan.setMinimumSize(QtCore.QSize(0, 100))
        self.btnPostScan.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.btnPostScan.setBaseSize(QtCore.QSize(0, 200))
        self.btnPostScan.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnPostScan.setObjectName("btnPostScan")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.btnPostScan)
        self.btnCompare = QtWidgets.QPushButton(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCompare.sizePolicy().hasHeightForWidth())
        self.btnCompare.setSizePolicy(sizePolicy)
        self.btnCompare.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCompare.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.btnCompare.setBaseSize(QtCore.QSize(0, 200))
        self.btnCompare.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnCompare.setObjectName("btnCompare")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.btnCompare)
        self.btnCancel = QtWidgets.QPushButton(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCancel.sizePolicy().hasHeightForWidth())
        self.btnCancel.setSizePolicy(sizePolicy)
        self.btnCancel.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCancel.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.btnCancel.setBaseSize(QtCore.QSize(0, 200))
        self.btnCancel.setAutoFillBackground(False)
        self.btnCancel.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #e15f41;\n"
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
        self.btnCancel.setObjectName("btnCancel")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.btnCancel)
        self.btnOpenScanner = QtWidgets.QPushButton(self.frame_3)
        self.btnOpenScanner.setMinimumSize(QtCore.QSize(0, 100))
        self.btnOpenScanner.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
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
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnCloseScanner.setObjectName("btnCloseScanner")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.btnCloseScanner)
        self.btnStop = QtWidgets.QPushButton(self.frame_3)
        self.btnStop.setMinimumSize(QtCore.QSize(0, 100))
        self.btnStop.setStyleSheet("QPushButton {\n"
"    border-radius: 40px; /* bo góc */\n"
"    color: white;\n"
"    padding: 6px 12px;\n"
"    border: 1px solid #63cdda;\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #63cdda); /* gradient xanh */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #3dc1d3, stop:1 #3dc1d3);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,\n"
"                                stop:0 #1E8449, stop:1 #145A32);\n"
"    padding-left: 4px;  /* tạo cảm giác nhấn sâu */\n"
"    padding-top: 4px;\n"
"}\n"
"")
        self.btnStop.setObjectName("btnStop")
        self.formLayout.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.btnStop)
        self.horizontalLayout_2.addWidget(self.frame_3)
        self.horizontalLayout_2.setStretch(0, 5)
        self.horizontalLayout_2.setStretch(1, 1)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.tabWidget.addTab(self.tab_main, "")
        self.tab_setting = QtWidgets.QWidget()
        self.tab_setting.setObjectName("tab_setting")
        self.tabWidget.addTab(self.tab_setting, "")
        self.tab_system = QtWidgets.QWidget()
        self.tab_system.setObjectName("tab_system")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.tab_system)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.infoFrame = QtWidgets.QFrame(self.tab_system)
        self.infoFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.infoFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.infoFrame.setObjectName("infoFrame")
        self.label = QtWidgets.QLabel(self.infoFrame)
        self.label.setGeometry(QtCore.QRect(80, 200, 141, 16))
        self.label.setObjectName("label")
        self.lblEncoder = QtWidgets.QLabel(self.infoFrame)
        self.lblEncoder.setGeometry(QtCore.QRect(210, 200, 59, 15))
        self.lblEncoder.setObjectName("lblEncoder")
        self.label_2 = QtWidgets.QLabel(self.infoFrame)
        self.label_2.setGeometry(QtCore.QRect(30, 150, 41, 16))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.infoFrame)
        self.label_3.setGeometry(QtCore.QRect(30, 180, 61, 16))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.infoFrame)
        self.label_4.setGeometry(QtCore.QRect(30, 120, 41, 16))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.infoFrame)
        self.label_5.setGeometry(QtCore.QRect(30, 230, 91, 16))
        self.label_5.setObjectName("label_5")
        self.lblPLCStatus = QtWidgets.QLabel(self.infoFrame)
        self.lblPLCStatus.setGeometry(QtCore.QRect(90, 120, 59, 15))
        self.lblPLCStatus.setObjectName("lblPLCStatus")
        self.lblLidarStatus = QtWidgets.QLabel(self.infoFrame)
        self.lblLidarStatus.setGeometry(QtCore.QRect(90, 150, 59, 15))
        self.lblLidarStatus.setObjectName("lblLidarStatus")
        self.lblPCANStatus = QtWidgets.QLabel(self.infoFrame)
        self.lblPCANStatus.setGeometry(QtCore.QRect(120, 230, 59, 15))
        self.lblPCANStatus.setObjectName("lblPCANStatus")
        self.lblEncoderStatus = QtWidgets.QLabel(self.infoFrame)
        self.lblEncoderStatus.setGeometry(QtCore.QRect(90, 180, 59, 15))
        self.lblEncoderStatus.setObjectName("lblEncoderStatus")
        self.label_6 = QtWidgets.QLabel(self.infoFrame)
        self.label_6.setGeometry(QtCore.QRect(30, 260, 61, 16))
        self.label_6.setObjectName("label_6")
        self.lblCameraStatus = QtWidgets.QLabel(self.infoFrame)
        self.lblCameraStatus.setGeometry(QtCore.QRect(120, 260, 59, 15))
        self.lblCameraStatus.setObjectName("lblCameraStatus")
        self.horizontalLayout.addWidget(self.infoFrame)
        self.frmLeftSite = QtWidgets.QFrame(self.tab_system)
        self.frmLeftSite.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frmLeftSite.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frmLeftSite.setObjectName("frmLeftSite")
        self.horizontalLayout.addWidget(self.frmLeftSite)
        self.horizontalLayout.setStretch(0, 5)
        self.horizontalLayout.setStretch(1, 1)
        self.horizontalLayout_3.addLayout(self.horizontalLayout)
        self.tabWidget.addTab(self.tab_system, "")
        self.verticalLayout.addWidget(self.tabWidget)
        self.lblNotification = QtWidgets.QLabel(Frame)
        self.lblNotification.setObjectName("lblNotification")
        self.verticalLayout.addWidget(self.lblNotification)
        self.verticalLayout.setStretch(0, 5)

        self.retranslateUi(Frame)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        _translate = QtCore.QCoreApplication.translate
        Frame.setWindowTitle(_translate("Frame", "Frame"))
        self.btnPreScan.setText(_translate("Frame", "Pre-Scan"))
        self.btnPostScan.setText(_translate("Frame", "Post-Scan"))
        self.btnCompare.setText(_translate("Frame", "Compare"))
        self.btnCancel.setText(_translate("Frame", "Cancel job"))
        self.btnOpenScanner.setText(_translate("Frame", "Open Housing"))
        self.btnCloseScanner.setText(_translate("Frame", "Close Housing"))
        self.btnStop.setText(_translate("Frame", "Stop Housing"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_main), _translate("Frame", "Home"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_setting), _translate("Frame", "Setting"))
        self.label.setText(_translate("Frame", "Encoder value (rad):"))
        self.lblEncoder.setText(_translate("Frame", "--"))
        self.label_2.setText(_translate("Frame", "Lidar:"))
        self.label_3.setText(_translate("Frame", "Encoder:"))
        self.label_4.setText(_translate("Frame", "PLC:"))
        self.label_5.setText(_translate("Frame", "PCAN Gatway:"))
        self.lblPLCStatus.setText(_translate("Frame", "--"))
        self.lblLidarStatus.setText(_translate("Frame", "--"))
        self.lblPCANStatus.setText(_translate("Frame", "--"))
        self.lblEncoderStatus.setText(_translate("Frame", "--"))
        self.label_6.setText(_translate("Frame", "Camera:"))
        self.lblCameraStatus.setText(_translate("Frame", "--"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_system), _translate("Frame", "System"))
        self.lblNotification.setText(_translate("Frame", "Notification..."))
