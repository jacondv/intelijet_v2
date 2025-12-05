# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './intelijet.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1311, 953)
        MainWindow.setStyleSheet("\n"
"/* ==================== PUSHBUTTON ==================== */\n"
"\n"
"QPushButton {\n"
"    background-color: #5C87C9;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 0px solid #3c6382;\n"
"    border-radius: 15px;\n"
"    padding: 6px 10px;\n"
"    font-size: 20px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #3c6382;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #3c6382;\n"
"    border: 1px solid #355A8A;\n"
"}\n"
"\n"
"\n"
"/* --- Text editor (QLineEdit & QTextEdit) --- */\n"
"\n"
"/* Text box chung */\n"
"QLineEdit, QTextEdit {\n"
"    background-color: #FFFFFF;        /* nền trắng */\n"
"    color: #2F4F6E;                  /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;       /* viền nhẹ */\n"
"    padding: 4px 8px;                /* khoảng cách chữ và viền */\n"
"    font-size: 24px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"/* Khi focus */\n"
"QLineEdit:focus, QTextEdit:focus {\n"
"    border: 1px solid #5C87C9;       /* viền xanh khi focus */\n"
"    background-color: #F9FBFF;        /* nền sáng hơn khi focus */\n"
"}\n"
"\n"
"/* Khi hover (chuột vào) */\n"
"QLineEdit:hover, QTextEdit:hover {\n"
"    border: 1px solid #74A9D8;       /* viền hover nhẹ */\n"
"}\n"
"\n"
"\n"
"/* --- Checkbox chung --- */\n"
"QCheckBox {\n"
"    spacing: 8px;             /* khoảng cách giữa ô và text */\n"
"    color: #2F4F6E;           /* chữ giống TextEdit */\n"
"    font-size: 24px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"/* --- Indicator (ô vuông) --- */\n"
"QCheckBox::indicator {\n"
"    width: 18px;\n"
"    height: 18px;\n"
"    border: 1px solid #B0C4DE; /* viền nhẹ như TextEdit */\n"
"    border-radius: 4px;         /* bo góc nhẹ */\n"
"    background: #FFFFFF;        /* nền trắng */\n"
"    margin: 0;\n"
"    transition: all 0.2s;\n"
"}\n"
"\n"
"/* Hover: viền xanh nhạt */\n"
"QCheckBox::indicator:hover {\n"
"    border: 1px solid #74A9D8; /* giống hover TextEdit */\n"
"}\n"
"\n"
"/* Focus: viền xanh đậm, nền sáng */\n"
"QCheckBox::indicator:focus {\n"
"    border: 1px solid #5C87C9; /* giống focus TextEdit */\n"
"    background-color: #F9FBFF;\n"
"}\n"
"\n"
"/* Checked: tick hiện mặc định, nền vẫn trắng, viền xanh đậm */\n"
"QCheckBox::indicator:checked {\n"
"    border: 1px solid #5C87C9;   /* giống focus */\n"
"    background-color: #2F4F6E;    /* nền trắng, tick hiện */\n"
"}\n"
"\n"
"/* Disabled: mờ */\n"
"QCheckBox::indicator:disabled {\n"
"    border: 1px solid #D3D3D3;\n"
"    background: #F5F5F5;\n"
"    color: #A0A0A0;\n"
"}\n"
"\n"
"\n"
"QComboBox {\n"
"    background-color: #FFFFFF;       /* nền sáng */\n"
"    color: #2F4F6E;                 /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;      /* viền nhẹ */\n"
"    padding: 4px 8px;\n"
"    font-size: 24px;\n"
"    font-weight: bold;\n"
"    selection-background-color: #D0E4F5; /* khi chọn item */\n"
"}\n"
"\n"
"/* Drop-down list */\n"
"QComboBox QAbstractItemView {\n"
"    background-color: #FFFFFF;\n"
"    color: #2F4F6E;\n"
"    border: 1px solid #B0C4DE;\n"
"    selection-background-color: #D0E4F5;\n"
"}\n"
"QComboBox QAbstractItemView::item {\n"
"    min-height: 60px;\n"
"    padding-top: 6px;\n"
"    padding-bottom: 6px;\n"
"}\n"
"\n"
"\n"
"/* Hover khi trỏ vào QComboBox */\n"
"QComboBox:hover {\n"
"    border: 1px solid #5C87C9;\n"
"}\n"
"\n"
"/* Khi mở drop-down */\n"
"QComboBox:focus {\n"
"    border: 1px solid #5C87C9;\n"
"}\n"
"\n"
"QGroupBox {\n"
"    border: 1px solid #B0C4DE;   /* viền nhẹ */\n"
"    background-color: #FFFFFF;    /* nền trắng */\n"
"    margin-top: 10px;\n"
"    padding: 8px;\n"
"    font-weight: bold;\n"
"    font-size: 24px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;                   /* khoảng cách từ viền trái */\n"
"    padding: 0 4px;\n"
"    color: #2F4F6E;              /* chữ xanh đậm */\n"
"}\n"
"\n"
"\n"
"\n"
"/* ==================== SCROLLBAR ==================== */\n"
"QScrollBar:vertical {\n"
"    width: 30px;\n"
"    background: #f0f0f0;\n"
"    margin: 0;\n"
"    border-radius: 0px;\n"
"}\n"
"QScrollBar::handle:vertical {\n"
"    background: #b0b0b0;\n"
"    min-height: 20px;\n"
"    border-radius: 0px;\n"
"}\n"
"QScrollBar::handle:vertical:hover {\n"
"    background: #909090;\n"
"}\n"
"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {\n"
"    height: 0;\n"
"}\n"
"\n"
"*{\n"
"border: none;\n"
"font-size: 24px;\n"
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
"    font-size:24px;\n"
"    font-weight: bold;\n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 #F8D40F, stop:1 #FFF9C4);\n"
"\n"
"    border-top-right-radius: 40px;\n"
"    border-top-left-radius: 0px;\n"
"    border-bottom-right-radius: 40px;\n"
"    border-bottom-left-radius: 0px;\n"
"\n"
"    border-left: 24px solid #95a5a6; /*#55ff00;  #F8D40F;*/\n"
"}\n"
"\n"
"/* Hiệu ứng hover / nhấn */\n"
"#control_panel QPushButton:hover {\n"
"    font-size: 25px;\n"
"    color: #00aa7f;\n"
"    padding: 36px 5px 36px 0px;\n"
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
"#control_panel QLabel {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"    border: none;\n"
"}\n"
"\n"
"QLabel {\n"
"    background-color: #EAF2F8;   /* nền nhạt */\n"
"    color: #2F4F6E;              /* chữ xanh công nghiệp */\n"
"    border: 1px solid #B0C4DE;   /* viền nhẹ */\n"
"    padding: 4px 8px;\n"
"    font-size: 24px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"#toolBox{\n"
"    padding-top:30px;\n"
"    color:white;\n"
"}\n"
"\n"
"#toolBox:tab {\n"
"    border-bottom: 2px solid white;\n"
"    padding-left: 2px;\n"
"    padding-bottom: 2px;\n"
"}\n"
"\n"
"#toolBox QWidget{\n"
"    /*background-color: #185F63;*/\n"
"    background: transparent;\n"
"    color: white;\n"
"\n"
"}\n"
"\n"
"\n"
"#btnShutdown {\n"
"    background-color: #185F63;\n"
"    color: white;\n"
"}\n"
"\n"
"\n"
"#menu_small {\n"
"\n"
"}\n"
"\n"
"#menu_main QPushButton {\n"
"    /*border-right: 1px solid #FFEB3B;     /* viền trái */\n"
"\n"
"    font-weight: normal;\n"
"\n"
"}\n"
"\n"
"\n"
"#menu_main QPushButton:hover {\n"
"    /*border-right: 1px solid #FFEB3B;     /* viền trái */\n"
"\n"
"    font-weight: bold;\n"
"\n"
"}\n"
"#menu_main {\n"
"    color: none;\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"    height: 110px;     \n"
"    width: 30px;\n"
"    margin: 0;\n"
"    color: #ddd;\n"
"\n"
"    border: none;\n"
"    font-weight: normal;\n"
"    padding-top: 30px;\n"
"    padding-left: 10px;\n"
"    padding-right: 8px;\n"
"    min-height: 150px;\n"
"    max-height: 200px;\n"
"\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    font-size: 25x;\n"
"    font-weight: bold;\n"
"    color: #ee0;\n"
"    border-left: 4px solid #FFEB3B;\n"
"\n"
"}\n"
"\n"
"QWidget#widget_6 QPushButton {\n"
"    background-color: #aaa;\n"
"    color: white;\n"
"    border-radius: 0px;\n"
"    padding: 6px 12px;\n"
"}\n"
"QWidget#widget_6 QPushButton:hover {\n"
"    background-color: #2A8C91;\n"
"}\n"
"\n"
"\n"
"")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setContentsMargins(2, 2, 2, 2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.centralFrame = QtWidgets.QWidget(self.centralwidget)
        self.centralFrame.setObjectName("centralFrame")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.centralFrame)
        self.horizontalLayout_2.setContentsMargins(-1, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.menu_main = QtWidgets.QWidget(self.centralFrame)
        self.menu_main.setMinimumSize(QtCore.QSize(400, 0))
        self.menu_main.setMaximumSize(QtCore.QSize(400, 16777215))
        font = QtGui.QFont()
        font.setPointSize(-1)
        self.menu_main.setFont(font)
        self.menu_main.setObjectName("menu_main")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.menu_main)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_3 = QtWidgets.QWidget(self.menu_main)
        self.widget_3.setObjectName("widget_3")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.widget_3)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.lblCurrentJob = QtWidgets.QLabel(self.widget_3)
        self.lblCurrentJob.setMinimumSize(QtCore.QSize(0, 60))
        self.lblCurrentJob.setObjectName("lblCurrentJob")
        self.verticalLayout_8.addWidget(self.lblCurrentJob)
        self.cbbJobSelect = QtWidgets.QComboBox(self.widget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cbbJobSelect.sizePolicy().hasHeightForWidth())
        self.cbbJobSelect.setSizePolicy(sizePolicy)
        self.cbbJobSelect.setMinimumSize(QtCore.QSize(0, 0))
        self.cbbJobSelect.setStyleSheet("QComboBox {\n"
"    border: 1px solid #60a3bc;\n"
"    padding: 6px 10px;\n"
"    font-size: 24px;\n"
"}\n"
"QComboBox QAbstractItemView::item {\n"
"    min-height: 60px;\n"
"    padding-top: 6px;\n"
"    padding-bottom: 6px;\n"
"}\n"
"")
        self.cbbJobSelect.setEditable(False)
        self.cbbJobSelect.setObjectName("cbbJobSelect")
        self.cbbJobSelect.addItem("")
        self.cbbJobSelect.addItem("")
        self.cbbJobSelect.addItem("")
        self.verticalLayout_8.addWidget(self.cbbJobSelect)
        self.verticalLayout.addWidget(self.widget_3)
        self.toolBox = QtWidgets.QToolBox(self.menu_main)
        self.toolBox.setObjectName("toolBox")
        self.tboxPage1 = QtWidgets.QWidget()
        self.tboxPage1.setGeometry(QtCore.QRect(0, 0, 382, 579))
        self.tboxPage1.setObjectName("tboxPage1")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.tboxPage1)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.widget_4 = QtWidgets.QWidget(self.tboxPage1)
        self.widget_4.setObjectName("widget_4")
        self.verticalLayout_12 = QtWidgets.QVBoxLayout(self.widget_4)
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.btnViewReport = QtWidgets.QPushButton(self.widget_4)
        self.btnViewReport.setMinimumSize(QtCore.QSize(120, 100))
        self.btnViewReport.setStyleSheet("QPushButton {\n"
"    background-color: #60a3bc;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 1px solid #60a3bc;\n"
"    padding: 6px 10px;\n"
"    font-size: 30px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #74b5cd;     /* #60a3bc + sáng */\n"
"    border: 1px solid #74b5cd;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #4d8ea3;     /* #60a3bc - đậm */\n"
"    border: 1px solid #4d8ea3;\n"
"}")
        self.btnViewReport.setObjectName("btnViewReport")
        self.verticalLayout_12.addWidget(self.btnViewReport)
        self.btnCompare2 = QtWidgets.QPushButton(self.widget_4)
        self.btnCompare2.setMinimumSize(QtCore.QSize(0, 100))
        self.btnCompare2.setStyleSheet("QPushButton {\n"
"    background-color: #60a3bc;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 1px solid #60a3bc;\n"
"    padding: 6px 10px;\n"
"    font-size: 30px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #74b5cd;     /* #60a3bc + sáng */\n"
"    border: 1px solid #74b5cd;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #4d8ea3;     /* #60a3bc - đậm */\n"
"    border: 1px solid #4d8ea3;\n"
"}")
        self.btnCompare2.setObjectName("btnCompare2")
        self.verticalLayout_12.addWidget(self.btnCompare2)
        self.verticalLayout_6.addWidget(self.widget_4)
        spacerItem = QtWidgets.QSpacerItem(20, 437, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/home-4-48 (2).ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolBox.addItem(self.tboxPage1, icon, "")
        self.tboxPage2 = QtWidgets.QWidget()
        self.tboxPage2.setGeometry(QtCore.QRect(0, 0, 382, 579))
        self.tboxPage2.setObjectName("tboxPage2")
        self.toolBox.addItem(self.tboxPage2, "")
        self.verticalLayout.addWidget(self.toolBox)
        self.exit_widget = QtWidgets.QWidget(self.menu_main)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.exit_widget.sizePolicy().hasHeightForWidth())
        self.exit_widget.setSizePolicy(sizePolicy)
        self.exit_widget.setObjectName("exit_widget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.exit_widget)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btnShutdown = QtWidgets.QPushButton(self.exit_widget)
        self.btnShutdown.setMinimumSize(QtCore.QSize(0, 0))
        self.btnShutdown.setMaximumSize(QtCore.QSize(16777215, 60))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/icon/power-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnShutdown.setIcon(icon1)
        self.btnShutdown.setIconSize(QtCore.QSize(60, 60))
        self.btnShutdown.setCheckable(False)
        self.btnShutdown.setObjectName("btnShutdown")
        self.horizontalLayout_3.addWidget(self.btnShutdown)
        self.btnFullScreen = QtWidgets.QPushButton(self.exit_widget)
        self.btnFullScreen.setStyleSheet("QPushButton {\n"
"    background-color: #60a3bc;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 1px solid #60a3bc;\n"
"    padding: 6px 10px;\n"
"    font-size: 20px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #74b5cd;     /* #60a3bc + sáng */\n"
"    border: 1px solid #74b5cd;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #4d8ea3;     /* #60a3bc - đậm */\n"
"    border: 1px solid #4d8ea3;\n"
"}")
        self.btnFullScreen.setObjectName("btnFullScreen")
        self.horizontalLayout_3.addWidget(self.btnFullScreen)
        self.verticalLayout.addWidget(self.exit_widget)
        self.horizontalLayout_2.addWidget(self.menu_main)
        self.menu_small = QtWidgets.QWidget(self.centralFrame)
        self.menu_small.setMaximumSize(QtCore.QSize(50, 16777215))
        self.menu_small.setObjectName("menu_small")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.menu_small)
        self.verticalLayout_3.setContentsMargins(0, -1, -1, -1)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.pushButton_4 = QtWidgets.QPushButton(self.menu_small)
        self.pushButton_4.setMinimumSize(QtCore.QSize(35, 120))
        self.pushButton_4.setMaximumSize(QtCore.QSize(35, 120))
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
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tab_mainview.sizePolicy().hasHeightForWidth())
        self.tab_mainview.setSizePolicy(sizePolicy)
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
        self.verticalLayout_7.setContentsMargins(2, 2, 2, 2)
        self.verticalLayout_7.setSpacing(2)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.cloudFrame = QtWidgets.QWidget(self.tab_cloud_view)
        self.cloudFrame.setObjectName("cloudFrame")
        self.verticalLayout_7.addWidget(self.cloudFrame)
        self.horizontalLayout.addWidget(self.tab_cloud_view)
        self.horizontalLayout.setStretch(0, 10)
        self.tab_mainview.addTab(self.tab_operator, "")
        self.tab_jobnumber = QtWidgets.QWidget()
        self.tab_jobnumber.setObjectName("tab_jobnumber")
        self.tab_mainview.addTab(self.tab_jobnumber, "")
        self.tab_setting = QtWidgets.QWidget()
        self.tab_setting.setObjectName("tab_setting")
        self.gridLayout = QtWidgets.QGridLayout(self.tab_setting)
        self.gridLayout.setObjectName("gridLayout")
        self.tab_mainview.addTab(self.tab_setting, "")
        self.tab_system = QtWidgets.QWidget()
        self.tab_system.setObjectName("tab_system")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.tab_system)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.widget = QtWidgets.QWidget(self.tab_system)
        self.widget.setObjectName("widget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.groupBox = QtWidgets.QGroupBox(self.widget)
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout(self.groupBox)
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.widget_2 = QtWidgets.QWidget(self.groupBox)
        self.widget_2.setMinimumSize(QtCore.QSize(500, 0))
        self.widget_2.setMaximumSize(QtCore.QSize(500, 500))
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_11 = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.widget_7 = QtWidgets.QWidget(self.widget_2)
        self.widget_7.setObjectName("widget_7")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_7)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_7 = QtWidgets.QLabel(self.widget_7)
        self.label_7.setMinimumSize(QtCore.QSize(0, 60))
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_5.addWidget(self.label_7)
        self.lblPLCStatus = QtWidgets.QLabel(self.widget_7)
        self.lblPLCStatus.setMinimumSize(QtCore.QSize(0, 60))
        self.lblPLCStatus.setObjectName("lblPLCStatus")
        self.horizontalLayout_5.addWidget(self.lblPLCStatus)
        self.verticalLayout_11.addWidget(self.widget_7)
        self.widget_12 = QtWidgets.QWidget(self.widget_2)
        self.widget_12.setObjectName("widget_12")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_12)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_5 = QtWidgets.QLabel(self.widget_12)
        self.label_5.setMinimumSize(QtCore.QSize(0, 60))
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_10.addWidget(self.label_5)
        self.lblPCANStatus = QtWidgets.QLabel(self.widget_12)
        self.lblPCANStatus.setMinimumSize(QtCore.QSize(0, 60))
        self.lblPCANStatus.setObjectName("lblPCANStatus")
        self.horizontalLayout_10.addWidget(self.lblPCANStatus)
        self.verticalLayout_11.addWidget(self.widget_12)
        self.widget_8 = QtWidgets.QWidget(self.widget_2)
        self.widget_8.setObjectName("widget_8")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_8)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_2 = QtWidgets.QLabel(self.widget_8)
        self.label_2.setMinimumSize(QtCore.QSize(0, 60))
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_6.addWidget(self.label_2)
        self.lblLidarStatus = QtWidgets.QLabel(self.widget_8)
        self.lblLidarStatus.setMinimumSize(QtCore.QSize(0, 60))
        self.lblLidarStatus.setObjectName("lblLidarStatus")
        self.horizontalLayout_6.addWidget(self.lblLidarStatus)
        self.verticalLayout_11.addWidget(self.widget_8)
        self.widget_9 = QtWidgets.QWidget(self.widget_2)
        self.widget_9.setObjectName("widget_9")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget_9)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_3 = QtWidgets.QLabel(self.widget_9)
        self.label_3.setMinimumSize(QtCore.QSize(0, 60))
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_7.addWidget(self.label_3)
        self.lblEncoderStatus = QtWidgets.QLabel(self.widget_9)
        self.lblEncoderStatus.setMinimumSize(QtCore.QSize(0, 60))
        self.lblEncoderStatus.setObjectName("lblEncoderStatus")
        self.horizontalLayout_7.addWidget(self.lblEncoderStatus)
        self.verticalLayout_11.addWidget(self.widget_9)
        self.widget_10 = QtWidgets.QWidget(self.widget_2)
        self.widget_10.setObjectName("widget_10")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_10)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label = QtWidgets.QLabel(self.widget_10)
        self.label.setMinimumSize(QtCore.QSize(0, 60))
        self.label.setObjectName("label")
        self.horizontalLayout_8.addWidget(self.label)
        self.lblEncoder = QtWidgets.QLabel(self.widget_10)
        self.lblEncoder.setMinimumSize(QtCore.QSize(0, 60))
        self.lblEncoder.setObjectName("lblEncoder")
        self.horizontalLayout_8.addWidget(self.lblEncoder)
        self.verticalLayout_11.addWidget(self.widget_10)
        self.widget_11 = QtWidgets.QWidget(self.widget_2)
        self.widget_11.setObjectName("widget_11")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.widget_11)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_4 = QtWidgets.QLabel(self.widget_11)
        self.label_4.setMinimumSize(QtCore.QSize(0, 60))
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_9.addWidget(self.label_4)
        self.lblEncoderRawValue = QtWidgets.QLabel(self.widget_11)
        self.lblEncoderRawValue.setMinimumSize(QtCore.QSize(0, 60))
        self.lblEncoderRawValue.setObjectName("lblEncoderRawValue")
        self.horizontalLayout_9.addWidget(self.lblEncoderRawValue)
        self.verticalLayout_11.addWidget(self.widget_11)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_11.addItem(spacerItem3)
        self.horizontalLayout_15.addWidget(self.widget_2)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_15.addItem(spacerItem4)
        self.verticalLayout_2.addWidget(self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(self.widget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.groupBox_2)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.widget_5 = QtWidgets.QWidget(self.groupBox_2)
        self.widget_5.setMinimumSize(QtCore.QSize(500, 0))
        self.widget_5.setObjectName("widget_5")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_5)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_14 = QtWidgets.QWidget(self.widget_5)
        self.widget_14.setObjectName("widget_14")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout(self.widget_14)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_8 = QtWidgets.QLabel(self.widget_14)
        self.label_8.setMinimumSize(QtCore.QSize(0, 60))
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_11.addWidget(self.label_8)
        self.cbbAutoAlign = QtWidgets.QComboBox(self.widget_14)
        self.cbbAutoAlign.setMinimumSize(QtCore.QSize(0, 60))
        self.cbbAutoAlign.setMaxVisibleItems(2)
        self.cbbAutoAlign.setObjectName("cbbAutoAlign")
        self.cbbAutoAlign.addItem("")
        self.cbbAutoAlign.addItem("")
        self.horizontalLayout_11.addWidget(self.cbbAutoAlign)
        self.verticalLayout_4.addWidget(self.widget_14)
        self.widget_13 = QtWidgets.QWidget(self.widget_5)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.widget_13)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_6 = QtWidgets.QLabel(self.widget_13)
        self.label_6.setMinimumSize(QtCore.QSize(0, 60))
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_12.addWidget(self.label_6)
        self.cbbAutoCompare = QtWidgets.QComboBox(self.widget_13)
        self.cbbAutoCompare.setMinimumSize(QtCore.QSize(0, 60))
        self.cbbAutoCompare.setMaxVisibleItems(2)
        self.cbbAutoCompare.setObjectName("cbbAutoCompare")
        self.cbbAutoCompare.addItem("")
        self.cbbAutoCompare.addItem("")
        self.horizontalLayout_12.addWidget(self.cbbAutoCompare)
        self.verticalLayout_4.addWidget(self.widget_13)
        self.widget_15 = QtWidgets.QWidget(self.widget_5)
        self.widget_15.setObjectName("widget_15")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.widget_15)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_9 = QtWidgets.QLabel(self.widget_15)
        self.label_9.setMinimumSize(QtCore.QSize(0, 60))
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_13.addWidget(self.label_9)
        self.cbbAutoReport = QtWidgets.QComboBox(self.widget_15)
        self.cbbAutoReport.setMinimumSize(QtCore.QSize(0, 60))
        self.cbbAutoReport.setMaxVisibleItems(2)
        self.cbbAutoReport.setObjectName("cbbAutoReport")
        self.cbbAutoReport.addItem("")
        self.cbbAutoReport.addItem("")
        self.horizontalLayout_13.addWidget(self.cbbAutoReport)
        self.verticalLayout_4.addWidget(self.widget_15)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem5)
        self.horizontalLayout_14.addWidget(self.widget_5)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_14.addItem(spacerItem6)
        self.verticalLayout_2.addWidget(self.groupBox_2)
        spacerItem7 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem7)
        self.verticalLayout_5.addWidget(self.widget)
        self.tab_mainview.addTab(self.tab_system, "")
        self.horizontalLayout_2.addWidget(self.tab_mainview)
        self.control_panel = QtWidgets.QWidget(self.centralFrame)
        self.control_panel.setMinimumSize(QtCore.QSize(220, 0))
        self.control_panel.setMaximumSize(QtCore.QSize(220, 16777215))
        self.control_panel.setObjectName("control_panel")
        self.verticalLayout_14 = QtWidgets.QVBoxLayout(self.control_panel)
        self.verticalLayout_14.setObjectName("verticalLayout_14")
        spacerItem8 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_14.addItem(spacerItem8)
        self.lblLogo = QtWidgets.QLabel(self.control_panel)
        self.lblLogo.setMaximumSize(QtCore.QSize(180, 65))
        self.lblLogo.setText("")
        self.lblLogo.setPixmap(QtGui.QPixmap(":/icon/icon/Jacon Equipment Logo PNG.png"))
        self.lblLogo.setScaledContents(True)
        self.lblLogo.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.lblLogo.setObjectName("lblLogo")
        self.verticalLayout_14.addWidget(self.lblLogo)
        spacerItem9 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_14.addItem(spacerItem9)
        self.btnPreScan = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnPreScan.sizePolicy().hasHeightForWidth())
        self.btnPreScan.setSizePolicy(sizePolicy)
        self.btnPreScan.setStyleSheet("")
        self.btnPreScan.setObjectName("btnPreScan")
        self.verticalLayout_14.addWidget(self.btnPreScan)
        self.btnPostScan = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnPostScan.sizePolicy().hasHeightForWidth())
        self.btnPostScan.setSizePolicy(sizePolicy)
        self.btnPostScan.setObjectName("btnPostScan")
        self.verticalLayout_14.addWidget(self.btnPostScan)
        self.btnCompare = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCompare.sizePolicy().hasHeightForWidth())
        self.btnCompare.setSizePolicy(sizePolicy)
        self.btnCompare.setObjectName("btnCompare")
        self.verticalLayout_14.addWidget(self.btnCompare)
        self.btnCancel = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCancel.sizePolicy().hasHeightForWidth())
        self.btnCancel.setSizePolicy(sizePolicy)
        self.btnCancel.setIconSize(QtCore.QSize(64, 64))
        self.btnCancel.setFlat(True)
        self.btnCancel.setObjectName("btnCancel")
        self.verticalLayout_14.addWidget(self.btnCancel)
        self.btnOpenScanner = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnOpenScanner.sizePolicy().hasHeightForWidth())
        self.btnOpenScanner.setSizePolicy(sizePolicy)
        self.btnOpenScanner.setObjectName("btnOpenScanner")
        self.verticalLayout_14.addWidget(self.btnOpenScanner)
        self.btnCloseScanner = QtWidgets.QPushButton(self.control_panel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCloseScanner.sizePolicy().hasHeightForWidth())
        self.btnCloseScanner.setSizePolicy(sizePolicy)
        self.btnCloseScanner.setObjectName("btnCloseScanner")
        self.verticalLayout_14.addWidget(self.btnCloseScanner)
        spacerItem10 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_14.addItem(spacerItem10)
        self.horizontalLayout_2.addWidget(self.control_panel)
        self.horizontalLayout_2.setStretch(0, 3)
        self.horizontalLayout_2.setStretch(1, 1)
        self.horizontalLayout_2.setStretch(2, 4)
        self.horizontalLayout_2.setStretch(3, 3)
        self.gridLayout_2.addWidget(self.centralFrame, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setMinimumSize(QtCore.QSize(0, 35))
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.toolBox.layout().setSpacing(6)
        self.tab_mainview.setCurrentIndex(3)
        self.cbbAutoAlign.setCurrentIndex(0)
        self.cbbAutoCompare.setCurrentIndex(0)
        self.cbbAutoReport.setCurrentIndex(0)
        self.pushButton_4.clicked['bool'].connect(self.menu_main.setHidden)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Jacon Intelijet"))
        self.lblCurrentJob.setText(_translate("MainWindow", "Current Job"))
        self.cbbJobSelect.setItemText(0, _translate("MainWindow", "New Item"))
        self.cbbJobSelect.setItemText(1, _translate("MainWindow", "New Item"))
        self.cbbJobSelect.setItemText(2, _translate("MainWindow", "New Item"))
        self.btnViewReport.setText(_translate("MainWindow", "View Report"))
        self.btnCompare2.setText(_translate("MainWindow", "Compare"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.tboxPage1), _translate("MainWindow", "Compare && Reports"))
        self.toolBox.setItemText(self.toolBox.indexOf(self.tboxPage2), _translate("MainWindow", "History"))
        self.btnShutdown.setText(_translate("MainWindow", "Exit"))
        self.btnFullScreen.setText(_translate("MainWindow", "Full Screen"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_operator), _translate("MainWindow", "3D VIEW"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_jobnumber), _translate("MainWindow", "JOB No."))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_setting), _translate("MainWindow", "SETTING"))
        self.groupBox.setTitle(_translate("MainWindow", "Device Status"))
        self.label_7.setText(_translate("MainWindow", "PLC:"))
        self.lblPLCStatus.setText(_translate("MainWindow", "--"))
        self.label_5.setText(_translate("MainWindow", "PCAN Gatway:"))
        self.lblPCANStatus.setText(_translate("MainWindow", "--"))
        self.label_2.setText(_translate("MainWindow", "Lidar:"))
        self.lblLidarStatus.setText(_translate("MainWindow", "--"))
        self.label_3.setText(_translate("MainWindow", "Encoder:"))
        self.lblEncoderStatus.setText(_translate("MainWindow", "--"))
        self.label.setText(_translate("MainWindow", "Encoder value (rad):"))
        self.lblEncoder.setText(_translate("MainWindow", "--"))
        self.label_4.setText(_translate("MainWindow", "Encoder value (raw):"))
        self.lblEncoderRawValue.setText(_translate("MainWindow", "--"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Runtime Option"))
        self.label_8.setText(_translate("MainWindow", "Auto Align Cloud"))
        self.cbbAutoAlign.setItemText(0, _translate("MainWindow", "On"))
        self.cbbAutoAlign.setItemText(1, _translate("MainWindow", "Off"))
        self.label_6.setText(_translate("MainWindow", "Auto Compare"))
        self.cbbAutoCompare.setItemText(0, _translate("MainWindow", "On"))
        self.cbbAutoCompare.setItemText(1, _translate("MainWindow", "Off"))
        self.label_9.setText(_translate("MainWindow", "Auto Export Report"))
        self.cbbAutoReport.setItemText(0, _translate("MainWindow", "On"))
        self.cbbAutoReport.setItemText(1, _translate("MainWindow", "Off"))
        self.tab_mainview.setTabText(self.tab_mainview.indexOf(self.tab_system), _translate("MainWindow", "SYSTEM"))
        self.btnPreScan.setText(_translate("MainWindow", "PRE-SCAN"))
        self.btnPostScan.setText(_translate("MainWindow", "POST-SCAN"))
        self.btnCompare.setText(_translate("MainWindow", "COMPARE"))
        self.btnCancel.setText(_translate("MainWindow", "CANCEL JOB"))
        self.btnOpenScanner.setText(_translate("MainWindow", "OPEN\n"
"HOUSING"))
        self.btnCloseScanner.setText(_translate("MainWindow", "CLOSE\n"
"HOUSING"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
