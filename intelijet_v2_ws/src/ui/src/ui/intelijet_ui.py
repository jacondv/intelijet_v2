# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'intelijet.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1235, 889)
        MainWindow.setStyleSheet("\n"
"/* --- Button mặc định --- */\n"
"QPushButton {\n"
"    background-color: #2196F3;     /* xanh dương */\n"
"    color: white;\n"
"    border-radius: 6px;\n"
"    padding: 8px 16px;\n"
"    font-size: 14px;\n"
"    font-weight: bold;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #1976D2;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #0D47A1;\n"
"}\n"
"\n"
"/* --- Text editor (QLineEdit & QTextEdit) --- */\n"
"QLineEdit, QTextEdit {\n"
"    border: 1px solid #BDBDBD;\n"
"    border-radius: 4px;\n"
"    padding: 6px;\n"
"    font-size: 14px;\n"
"    background: #FAFAFA;\n"
"}\n"
"QLineEdit:focus, QTextEdit:focus {\n"
"    border: 1px solid #2196F3;\n"
"    background: #FFFFFF;\n"
"}\n"
"\n"
"QListWidget {\n"
"    border: 1px solid #BDBDBD;\n"
"    border-radius: 6px;\n"
"    padding: 4px;\n"
"    background: #FAFAFA;\n"
"}\n"
"\n"
"QListWidget::item {\n"
"    height: 40px;\n"
"    padding-left: 10px;\n"
"    border-bottom: 1px solid #E0E0E0;\n"
"    font-size: 14px;\n"
"}\n"
"\n"
"QListWidget::item:selected {\n"
"    background-color: #2196F3;\n"
"    color: white;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QListWidget::item:hover {\n"
"    background-color: #BBDEFB;\n"
"}\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"*{\n"
"border: none;\n"
"color: back;\n"
"font-size: 17px;\n"
"}\n"
"#statusbar {\n"
"    background-color: #185F63;\n"
"    \n"
"}\n"
"\n"
"#centralwidget {\n"
"    background-color: black;\n"
"\n"
"}\n"
"\n"
"#centralFrame {\n"
"    background-color: #185F63;\n"
"    border-radius: 30px;\n"
"}\n"
"\n"
"#control_panel{\n"
"    /*background-color: #00796B;*/\n"
"\n"
"    \n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0.3 #185F63, stop:1 #185F63);\n"
"    border-left: 8px solid #F8D40F;     /* viền trái */\n"
"    border-top-right-radius: 25px;\n"
"    border-top-left-radius: 0px;\n"
"    border-bottom-right-radius: 25px;\n"
"    border-bottom-left-radius: 0px;\n"
"\n"
"}\n"
"\n"
"/* Định dạng cho tất cả QPushButton bên trong control_panel */\n"
"#control_panel QPushButton {\n"
"    color: #153E42;\n"
"    border: none;\n"
"    padding: 40px 0px 40px 0px;\n"
"    \n"
"    font-size: 17px;\n"
"    font-weight: bold;\n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 #F8D40F, stop:1 #FFF9C4);\n"
"\n"
"    border-top-right-radius: 40px;\n"
"    border-top-left-radius: 0px;\n"
"    border-bottom-right-radius: 40px;\n"
"    border-bottom-left-radius: 0px;\n"
"\n"
"    border-left: 8px solid #95a5a6; /*#55ff00;  #F8D40F;*/\n"
"}\n"
"\n"
"/* Hiệu ứng hover / nhấn */\n"
"#control_panel QPushButton:hover {\n"
"    font-size: 17px;\n"
"    color: #00aa7f;\n"
"    padding: 36px 5px 36px 0px;\n"
"\n"
"\n"
"}\n"
"\n"
"#control_panel QPushButton:pressed {\n"
"    background-color: #FBC02D;\n"
"}\n"
"\n"
"#control_panel QPushButton:disabled {\n"
"    background-color: #95a5a6;\n"
"    color: #dcdcdc;\n"
"}\n"
"\n"
"#toolBox:tab {\n"
"    color:white;\n"
"    border-bottom: 1px solid white;\n"
"}\n"
"\n"
"#toolBox QWidget{\n"
"    background-color: #185F63;\n"
"}\n"
"\n"
"#btnShutdown {\n"
"    background-color: #185F63;\n"
"\n"
"}\n"
"#lblCurrentJob, #lblCurrentJob_title {\n"
"    color: white;\n"
"}\n"
"#btnSelectJob {\n"
"    border: 1px solid white;\n"
"}\n"
"\n"
"#menu_small {\n"
"\n"
"}\n"
"\n"
"#menu_main QPushButton {\n"
"    /*border-right: 1px solid #FFEB3B;     /* viền trái */\n"
"\n"
"    color: white;\n"
"    font-weight: normal;\n"
"\n"
"\n"
"}\n"
"\n"
"\n"
"#menu_main QPushButton:hover {\n"
"    /*border-right: 1px solid #FFEB3B;     /* viền trái */\n"
"\n"
"    color: white;\n"
"    font-weight: bold;\n"
"\n"
"\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"    height: 110px;     \n"
"    width: 20px;\n"
"    margin: 0;\n"
"\n"
"    border: none;\n"
"    font-weight: normal;\n"
"    padding-top: 30px;\n"
"    padding-left: 10px;\n"
"    min-height: 100px;\n"
"\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    font-size: 18px;\n"
"    font-weight: bold;\n"
"    color: #fff;\n"
"    border-left: 4px solid #FFEB3B;\n"
"\n"
"}\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.centralFrame = QtWidgets.QWidget(self.centralwidget)
        self.centralFrame.setObjectName("centralFrame")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.centralFrame)
        self.horizontalLayout_2.setContentsMargins(-1, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.menu_main = QtWidgets.QWidget(self.centralFrame)
        self.menu_main.setMinimumSize(QtCore.QSize(200, 0))
        font = QtGui.QFont()
        font.setPointSize(-1)
        self.menu_main.setFont(font)
        self.menu_main.setObjectName("menu_main")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.menu_main)
        self.verticalLayout.setObjectName("verticalLayout")
        self.toolBox = QtWidgets.QToolBox(self.menu_main)
        font = QtGui.QFont()
        font.setPointSize(-1)
        self.toolBox.setFont(font)
        self.toolBox.setObjectName("toolBox")
        self.tboxPage1 = QtWidgets.QWidget()
        self.tboxPage1.setGeometry(QtCore.QRect(0, 0, 332, 676))
        self.tboxPage1.setObjectName("tboxPage1")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.tboxPage1)
        self.verticalLayout_2.setContentsMargins(0, 0, 20, 0)
        self.verticalLayout_2.setSpacing(4)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget_3 = QtWidgets.QWidget(self.tboxPage1)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.lblCurrentJob_title = QtWidgets.QLabel(self.widget_3)
        self.lblCurrentJob_title.setObjectName("lblCurrentJob_title")
        self.horizontalLayout_4.addWidget(self.lblCurrentJob_title)
        self.lblCurrentJob = QtWidgets.QLabel(self.widget_3)
        self.lblCurrentJob.setObjectName("lblCurrentJob")
        self.horizontalLayout_4.addWidget(self.lblCurrentJob)
        self.btnSelectJob = QtWidgets.QPushButton(self.widget_3)
        self.btnSelectJob.setObjectName("btnSelectJob")
        self.horizontalLayout_4.addWidget(self.btnSelectJob)
        self.horizontalLayout_4.setStretch(1, 3)
        self.horizontalLayout_4.setStretch(2, 2)
        self.verticalLayout_2.addWidget(self.widget_3)
        spacerItem = QtWidgets.QSpacerItem(20, 437, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/home-4-48 (2).ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolBox.addItem(self.tboxPage1, icon, "")
        self.tboxPage2 = QtWidgets.QWidget()
        self.tboxPage2.setGeometry(QtCore.QRect(0, 0, 332, 676))
        self.tboxPage2.setObjectName("tboxPage2")
        self.toolBox.addItem(self.tboxPage2, "")
        self.verticalLayout.addWidget(self.toolBox)
        self.exit_widget = QtWidgets.QWidget(self.menu_main)
        self.exit_widget.setObjectName("exit_widget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.exit_widget)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btnShutdown = QtWidgets.QPushButton(self.exit_widget)
        self.btnShutdown.setMinimumSize(QtCore.QSize(0, 0))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/icon/power-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnShutdown.setIcon(icon1)
        self.btnShutdown.setIconSize(QtCore.QSize(32, 32))
        self.btnShutdown.setCheckable(False)
        self.btnShutdown.setObjectName("btnShutdown")
        self.horizontalLayout_3.addWidget(self.btnShutdown)
        self.verticalLayout.addWidget(self.exit_widget)
        self.horizontalLayout_2.addWidget(self.menu_main)
        self.menu_small = QtWidgets.QWidget(self.centralFrame)
        self.menu_small.setObjectName("menu_small")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.menu_small)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.pushButton_4 = QtWidgets.QPushButton(self.menu_small)
        self.pushButton_4.setMinimumSize(QtCore.QSize(25, 100))
        self.pushButton_4.setMaximumSize(QtCore.QSize(25, 16777215))
        self.pushButton_4.setStyleSheet("background-color: #FFF9C4;\n"
"border-radius:12px;")
        self.pushButton_4.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/icon/icon/arrow-96-48.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon2.addPixmap(QtGui.QPixmap(":/icon/icon/arrow-31-48.ico"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.pushButton_4.setIcon(icon2)
        self.pushButton_4.setCheckable(True)
        self.pushButton_4.setObjectName("pushButton_4")
        self.verticalLayout_3.addWidget(self.pushButton_4)
        spacerItem2 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.MinimumExpanding)
        self.verticalLayout_3.addItem(spacerItem2)
        self.horizontalLayout_2.addWidget(self.menu_small)
        self.tab_mainview = QtWidgets.QTabWidget(self.centralFrame)
        self.tab_mainview.setEnabled(True)
        self.tab_mainview.setTabPosition(QtWidgets.QTabWidget.West)
        self.tab_mainview.setUsesScrollButtons(False)
        self.tab_mainview.setObjectName("tab_mainview")
        self.tab_operator = QtWidgets.QWidget()
        self.tab_operator.setObjectName("tab_operator")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.tab_operator)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(4)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.tab_cloud_view = QtWidgets.QWidget(self.tab_operator)
        self.tab_cloud_view.setObjectName("tab_cloud_view")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.tab_cloud_view)
        self.verticalLayout_7.setContentsMargins(4, 4, 4, 4)
        self.verticalLayout_7.setSpacing(4)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.cloudFrame = QtWidgets.QWidget(self.tab_cloud_view)
        self.cloudFrame.setObjectName("cloudFrame")
        self.verticalLayout_7.addWidget(self.cloudFrame)
        self.horizontalLayout.addWidget(self.tab_cloud_view)
        self.horizontalLayout.setStretch(0, 10)
        self.tab_mainview.addTab(self.tab_operator, "")
        self.tab_setting = QtWidgets.QWidget()
        self.tab_setting.setObjectName("tab_setting")
        self.gridLayout = QtWidgets.QGridLayout(self.tab_setting)
        self.gridLayout.setObjectName("gridLayout")
        self.tab_mainview.addTab(self.tab_setting, "")
        self.tab_jobnumber = QtWidgets.QWidget()
        self.tab_jobnumber.setObjectName("tab_jobnumber")
        self.tab_mainview.addTab(self.tab_jobnumber, "")
        self.tab_system = QtWidgets.QWidget()
        self.tab_system.setObjectName("tab_system")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.tab_system)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.widget = QtWidgets.QWidget(self.tab_system)
        self.widget.setObjectName("widget")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.widget)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setMaximumSize(QtCore.QSize(300, 250))
        self.widget_2.setObjectName("widget_2")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.widget_2)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label = QtWidgets.QLabel(self.widget_2)
        self.label.setObjectName("label")
        self.gridLayout_3.addWidget(self.label, 3, 1, 1, 3)
        self.label_2 = QtWidgets.QLabel(self.widget_2)
        self.label_2.setObjectName("label_2")
        self.gridLayout_3.addWidget(self.label_2, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.widget_2)
        self.label_3.setObjectName("label_3")
        self.gridLayout_3.addWidget(self.label_3, 2, 0, 1, 2)
        self.lblEncoder = QtWidgets.QLabel(self.widget_2)
        self.lblEncoder.setObjectName("lblEncoder")
        self.gridLayout_3.addWidget(self.lblEncoder, 3, 4, 1, 1)
        self.lblPCANStatus = QtWidgets.QLabel(self.widget_2)
        self.lblPCANStatus.setObjectName("lblPCANStatus")
        self.gridLayout_3.addWidget(self.lblPCANStatus, 4, 3, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.widget_2)
        self.label_6.setObjectName("label_6")
        self.gridLayout_3.addWidget(self.label_6, 5, 0, 1, 2)
        self.label_5 = QtWidgets.QLabel(self.widget_2)
        self.label_5.setObjectName("label_5")
        self.gridLayout_3.addWidget(self.label_5, 4, 0, 1, 3)
        self.lblPLCStatus = QtWidgets.QLabel(self.widget_2)
        self.lblPLCStatus.setObjectName("lblPLCStatus")
        self.gridLayout_3.addWidget(self.lblPLCStatus, 0, 2, 1, 2)
        self.lblCameraStatus = QtWidgets.QLabel(self.widget_2)
        self.lblCameraStatus.setObjectName("lblCameraStatus")
        self.gridLayout_3.addWidget(self.lblCameraStatus, 5, 3, 1, 1)
        self.lblLidarStatus = QtWidgets.QLabel(self.widget_2)
        self.lblLidarStatus.setObjectName("lblLidarStatus")
        self.gridLayout_3.addWidget(self.lblLidarStatus, 1, 2, 1, 2)
        self.label_7 = QtWidgets.QLabel(self.widget_2)
        self.label_7.setObjectName("label_7")
        self.gridLayout_3.addWidget(self.label_7, 0, 0, 1, 1)
        self.lblEncoderStatus = QtWidgets.QLabel(self.widget_2)
        self.lblEncoderStatus.setObjectName("lblEncoderStatus")
        self.gridLayout_3.addWidget(self.lblEncoderStatus, 2, 2, 1, 2)
        self.gridLayout_4.addWidget(self.widget_2, 0, 0, 1, 1)
        self.verticalLayout_5.addWidget(self.widget)
        self.tab_mainview.addTab(self.tab_system, "")
        self.horizontalLayout_2.addWidget(self.tab_mainview)
        self.control_panel = QtWidgets.QWidget(self.centralFrame)
        self.control_panel.setMinimumSize(QtCore.QSize(100, 0))
        self.control_panel.setMaximumSize(QtCore.QSize(250, 16777215))
        self.control_panel.setObjectName("control_panel")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.control_panel)
        self.verticalLayout_4.setContentsMargins(18, 0, 15, 10)
        self.verticalLayout_4.setSpacing(10)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        spacerItem3 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem3)
        self.lblLogo = QtWidgets.QLabel(self.control_panel)
        self.lblLogo.setMaximumSize(QtCore.QSize(180, 65))
        self.lblLogo.setText("")
        self.lblLogo.setPixmap(QtGui.QPixmap(":/icon/icon/Jacon Equipment Logo PNG.png"))
        self.lblLogo.setScaledContents(True)
        self.lblLogo.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.lblLogo.setObjectName("lblLogo")
        self.verticalLayout_4.addWidget(self.lblLogo)
        spacerItem4 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem4)
        self.btnPreScan = QtWidgets.QPushButton(self.control_panel)
        self.btnPreScan.setStyleSheet("")
        self.btnPreScan.setObjectName("btnPreScan")
        self.verticalLayout_4.addWidget(self.btnPreScan)
        self.btnPostScan = QtWidgets.QPushButton(self.control_panel)
        self.btnPostScan.setObjectName("btnPostScan")
        self.verticalLayout_4.addWidget(self.btnPostScan)
        self.btnCompare = QtWidgets.QPushButton(self.control_panel)
        self.btnCompare.setObjectName("btnCompare")
        self.verticalLayout_4.addWidget(self.btnCompare)
        self.btnCancel = QtWidgets.QPushButton(self.control_panel)
        self.btnCancel.setIconSize(QtCore.QSize(64, 64))
        self.btnCancel.setFlat(True)
        self.btnCancel.setObjectName("btnCancel")
        self.verticalLayout_4.addWidget(self.btnCancel)
        self.btnOpenScanner = QtWidgets.QPushButton(self.control_panel)
        self.btnOpenScanner.setObjectName("btnOpenScanner")
        self.verticalLayout_4.addWidget(self.btnOpenScanner)
        self.btnCloseScanner = QtWidgets.QPushButton(self.control_panel)
        self.btnCloseScanner.setObjectName("btnCloseScanner")
        self.verticalLayout_4.addWidget(self.btnCloseScanner)
        spacerItem5 = QtWidgets.QSpacerItem(20, 119, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem5)
        self.horizontalLayout_2.addWidget(self.control_panel)
        self.horizontalLayout_2.setStretch(0, 3)
        self.horizontalLayout_2.setStretch(2, 5)
        self.horizontalLayout_2.setStretch(3, 2)
        self.gridLayout_2.addWidget(self.centralFrame, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setMinimumSize(QtCore.QSize(0, 35))
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tab_mainview.setCurrentIndex(0)
        self.pushButton_4.clicked['bool'].connect(self.menu_main.setHidden)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Jacon Intelijet"))
        self.lblCurrentJob_title.setText(_translate("MainWindow", "Job: "))
        self.lblCurrentJob.setText(_translate("MainWindow", "#Jobtitle"))
        self.btnSelectJob.setText(_translate("MainWindow", "Select Job"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.tboxPage1), _translate("MainWindow", "Jobs view"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.tboxPage2), _translate("MainWindow", "History"))
        self.btnShutdown.setText(_translate("MainWindow", "Exit"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_operator), _translate("MainWindow", "3D Viewer"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_setting), _translate("MainWindow", "Setting"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_jobnumber), _translate("MainWindow", "Job No."))
        self.label.setText(_translate("MainWindow", "Encoder value (rad):"))
        self.label_2.setText(_translate("MainWindow", "Lidar:"))
        self.label_3.setText(_translate("MainWindow", "Encoder:"))
        self.lblEncoder.setText(_translate("MainWindow", "--"))
        self.lblPCANStatus.setText(_translate("MainWindow", "--"))
        self.label_6.setText(_translate("MainWindow", "Camera:"))
        self.label_5.setText(_translate("MainWindow", "PCAN Gatway:"))
        self.lblPLCStatus.setText(_translate("MainWindow", "--"))
        self.lblCameraStatus.setText(_translate("MainWindow", "--"))
        self.lblLidarStatus.setText(_translate("MainWindow", "--"))
        self.label_7.setText(_translate("MainWindow", "PLC:"))
        self.lblEncoderStatus.setText(_translate("MainWindow", "--"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_system), _translate("MainWindow", "System"))
        self.btnPreScan.setText(_translate("MainWindow", "PRE-SCAN"))
        self.btnPostScan.setText(_translate("MainWindow", "POST-SCAN"))
        self.btnCompare.setText(_translate("MainWindow", "Compare"))
        self.btnCancel.setText(_translate("MainWindow", "CANCEL JOB"))
        self.btnOpenScanner.setText(_translate("MainWindow", "OPEN HOUSING"))
        self.btnCloseScanner.setText(_translate("MainWindow", "CLOSE HOUSING"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
