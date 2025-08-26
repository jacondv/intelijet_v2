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
        MainWindow.resize(1080, 761)
        MainWindow.setStyleSheet("*{\n"
"border: 0px solid green;\n"
"color: back;\n"
"}\n"
"\n"
"#centralwidget {\n"
"    background-color: black;\n"
"}\n"
"\n"
"#centralFrame {\n"
"    background-color: #153E42;\n"
"    border-radius: 30px;\n"
"}\n"
"\n"
"#control_panel{\n"
"    /*background-color: #00796B;*/\n"
"\n"
"    \n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0.3 #153E42, stop:1 #153E42);\n"
"    border-left: 8px solid #F8D40F;     /* viền trái */\n"
"\n"
"    border-top-right-radius: 25px;\n"
"    border-top-left-radius: 0px;\n"
"    border-bottom-right-radius: 25px;\n"
"    border-bottom-left-radius: 0px;\n"
"\n"
"}\n"
"\n"
"/* Định dạng cho tất cả QPushButton bên trong control_panel */\n"
"#control_panel QPushButton {\n"
"    color: #19232D;\n"
"    border: none;\n"
"    padding: 36px;\n"
"    font-size: 14px;\n"
"    font-weight: bold;\n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 #F8D40F, stop:1 #FFF9C4);\n"
"\n"
"    border-top-right-radius: 0px;\n"
"    border-top-left-radius: 0px;\n"
"    border-bottom-right-radius: 0px;\n"
"    border-bottom-left-radius: 0px;\n"
"\n"
"    border-left: 6px solid #F8D40F;\n"
"}\n"
"\n"
"/* Hiệu ứng hover / nhấn */\n"
"#control_panel QPushButton:hover {\n"
"    font-size: 15px;\n"
"    color: #212121;\n"
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
"    background-color: #153E42;\n"
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
"    font-size: 14px;\n"
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
"    font-size: 16px;\n"
"    font-weight: bold;\n"
"\n"
"\n"
"}\n"
"\n"
"\n"
"\n"
"QTabBar::tab {\n"
"    height: 110px;     \n"
"    width: 20px;\n"
"    margin: 0;\n"
"\n"
"    border: none;\n"
"    font-weight: normal;\n"
"    padding-top: 30px;\n"
"    min-height: 100px;\n"
"\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    font-size: 15px;\n"
"    font-weight: bold;\n"
"    color: #fff;\n"
"    border-left: 4px solid #FFEB3B;\n"
"\n"
"}\n"
"\n"
"QPushButton#btnPreScan {\n"
"    border-left: 6px solid #00FF00;\n"
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
        font.setPointSize(10)
        self.menu_main.setFont(font)
        self.menu_main.setObjectName("menu_main")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.menu_main)
        self.verticalLayout.setObjectName("verticalLayout")
        self.toolBox = QtWidgets.QToolBox(self.menu_main)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.toolBox.setFont(font)
        self.toolBox.setObjectName("toolBox")
        self.page_4 = QtWidgets.QWidget()
        self.page_4.setGeometry(QtCore.QRect(0, 0, 235, 659))
        self.page_4.setObjectName("page_4")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.page_4)
        self.verticalLayout_2.setContentsMargins(0, 0, 20, 0)
        self.verticalLayout_2.setSpacing(4)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.pushButton = QtWidgets.QPushButton(self.page_4)
        self.pushButton.setMinimumSize(QtCore.QSize(0, 0))
        self.pushButton.setTabletTracking(False)
        self.pushButton.setCheckable(True)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout_2.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.page_4)
        self.pushButton_2.setMinimumSize(QtCore.QSize(0, 0))
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout_2.addWidget(self.pushButton_2)
        spacerItem = QtWidgets.QSpacerItem(20, 437, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.btnShutdown = QtWidgets.QPushButton(self.page_4)
        self.btnShutdown.setMinimumSize(QtCore.QSize(0, 0))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/power-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnShutdown.setIcon(icon)
        self.btnShutdown.setIconSize(QtCore.QSize(32, 32))
        self.btnShutdown.setCheckable(False)
        self.btnShutdown.setObjectName("btnShutdown")
        self.verticalLayout_2.addWidget(self.btnShutdown)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/icon/home-4-48 (2).ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolBox.addItem(self.page_4, icon1, "")
        self.verticalLayout.addWidget(self.toolBox)
        self.horizontalLayout_2.addWidget(self.menu_main)
        self.menu_small = QtWidgets.QWidget(self.centralFrame)
        self.menu_small.setObjectName("menu_small")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.menu_small)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.pushButton_4 = QtWidgets.QPushButton(self.menu_small)
        self.pushButton_4.setMinimumSize(QtCore.QSize(25, 100))
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
        self.tab_system = QtWidgets.QWidget()
        self.tab_system.setObjectName("tab_system")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.tab_system)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.tab_mainview.addTab(self.tab_system, "")
        self.horizontalLayout_2.addWidget(self.tab_mainview)
        self.control_panel = QtWidgets.QWidget(self.centralFrame)
        self.control_panel.setMinimumSize(QtCore.QSize(100, 0))
        self.control_panel.setMaximumSize(QtCore.QSize(250, 16777215))
        self.control_panel.setObjectName("control_panel")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.control_panel)
        self.verticalLayout_4.setContentsMargins(8, 0, 5, 0)
        self.verticalLayout_4.setSpacing(1)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        spacerItem3 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem3)
        self.label_4 = QtWidgets.QLabel(self.control_panel)
        self.label_4.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_4.addWidget(self.label_4)
        spacerItem4 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem4)
        self.btnPreScan = QtWidgets.QPushButton(self.control_panel)
        self.btnPreScan.setObjectName("btnPreScan")
        self.verticalLayout_4.addWidget(self.btnPreScan)
        self.btnPostScan = QtWidgets.QPushButton(self.control_panel)
        self.btnPostScan.setObjectName("btnPostScan")
        self.verticalLayout_4.addWidget(self.btnPostScan)
        self.btnCompare = QtWidgets.QPushButton(self.control_panel)
        self.btnCompare.setObjectName("btnCompare")
        self.verticalLayout_4.addWidget(self.btnCompare)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem5)
        self.btnCancel = QtWidgets.QPushButton(self.control_panel)
        self.btnCancel.setIconSize(QtCore.QSize(64, 64))
        self.btnCancel.setFlat(True)
        self.btnCancel.setObjectName("btnCancel")
        self.verticalLayout_4.addWidget(self.btnCancel)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem6)
        self.btnOpenScanner = QtWidgets.QPushButton(self.control_panel)
        self.btnOpenScanner.setObjectName("btnOpenScanner")
        self.verticalLayout_4.addWidget(self.btnOpenScanner)
        self.btnCloseScanner = QtWidgets.QPushButton(self.control_panel)
        self.btnCloseScanner.setObjectName("btnCloseScanner")
        self.verticalLayout_4.addWidget(self.btnCloseScanner)
        spacerItem7 = QtWidgets.QSpacerItem(20, 119, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem7)
        self.horizontalLayout_2.addWidget(self.control_panel)
        self.horizontalLayout_2.setStretch(0, 2)
        self.horizontalLayout_2.setStretch(2, 4)
        self.horizontalLayout_2.setStretch(3, 2)
        self.gridLayout_2.addWidget(self.centralFrame, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setMinimumSize(QtCore.QSize(0, 35))
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.toolBox.setCurrentIndex(0)
        self.tab_mainview.setCurrentIndex(2)
        self.pushButton_4.clicked['bool'].connect(self.menu_main.setHidden)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.pushButton, self.pushButton_2)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Operator"))
        self.pushButton_2.setText(_translate("MainWindow", "Job Setting"))
        self.btnShutdown.setText(_translate("MainWindow", "Exit"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page_4), _translate("MainWindow", "General"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_operator), _translate("MainWindow", "3D Viewer"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_setting), _translate("MainWindow", "Report"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_system), _translate("MainWindow", "System"))
        self.label_4.setText(_translate("MainWindow", "LOGO"))
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
