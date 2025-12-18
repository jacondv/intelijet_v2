# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './setting_page.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_setting_page(object):
    def setupUi(self, setting_page):
        setting_page.setObjectName("setting_page")
        setting_page.resize(982, 944)
        setting_page.setStyleSheet("* {\n"
"    font-size: 24px;\n"
"}\n"
"QPushButton {\n"
"    background-color: #5C87C9;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 0px solid #3c6382;\n"
"    border-radius: 15px;\n"
"    padding: 6px 10px;\n"
"    min-height: 50px;\n"
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
"/* Text box chung */\n"
"QLineEdit, QTextEdit {\n"
"    background-color: #FFFFFF;        /* nền trắng */\n"
"    color: #2F4F6E;                  /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;       /* viền nhẹ */\n"
"    padding: 4px 8px;                /* khoảng cách chữ và viền */\n"
"    font-weight: bold;\n"
"    \n"
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
"QComboBox {\n"
"    background-color: #FFFFFF;       /* nền sáng */\n"
"    color: #2F4F6E;                 /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;      /* viền nhẹ */\n"
"    padding: 4px 8px;\n"
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
"QGroupBox {\n"
"    border: 1px solid #B0C4DE;   /* viền nhẹ */\n"
"    background-color: #FFFFFF;    /* nền trắng */\n"
"    margin-top: 10px;\n"
"    padding: 8px;\n"
"    font-weight: bold;\n"
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
"/* --- Checkbox chung --- */\n"
"QCheckBox {\n"
"    spacing: 8px;             /* khoảng cách giữa ô và text */\n"
"    color: #2F4F6E;           /* chữ giống TextEdit */\n"
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
"\n"
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
"QLabel {\n"
"    color: #2F4F6E;           /* chữ giống TextEdit */\n"
"    font-weight: bold;\n"
"}\n"
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
"")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(setting_page)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.widget = QtWidgets.QWidget(setting_page)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)
        self.widget.setObjectName("widget")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.widget_20 = QtWidgets.QWidget(self.widget)
        self.widget_20.setObjectName("widget_20")
        self.verticalLayout_12 = QtWidgets.QVBoxLayout(self.widget_20)
        self.verticalLayout_12.setContentsMargins(-1, -1, 0, -1)
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.groupBox_4 = QtWidgets.QGroupBox(self.widget_20)
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayout_15 = QtWidgets.QVBoxLayout(self.groupBox_4)
        self.verticalLayout_15.setObjectName("verticalLayout_15")
        self.widget_16 = QtWidgets.QWidget(self.groupBox_4)
        self.widget_16.setObjectName("widget_16")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.widget_16)
        self.verticalLayout_9.setContentsMargins(-1, 20, -1, -1)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.widget_3 = QtWidgets.QWidget(self.widget_16)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_14 = QtWidgets.QLabel(self.widget_3)
        self.label_14.setMinimumSize(QtCore.QSize(0, 60))
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_9.addWidget(self.label_14)
        self.txtHousingClosedPosition = QtWidgets.QLineEdit(self.widget_3)
        self.txtHousingClosedPosition.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingClosedPosition.setObjectName("txtHousingClosedPosition")
        self.horizontalLayout_9.addWidget(self.txtHousingClosedPosition)
        self.verticalLayout_9.addWidget(self.widget_3)
        self.widget_17 = QtWidgets.QWidget(self.widget_16)
        self.widget_17.setObjectName("widget_17")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.widget_17)
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_17 = QtWidgets.QLabel(self.widget_17)
        self.label_17.setMinimumSize(QtCore.QSize(0, 60))
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_12.addWidget(self.label_17)
        self.txtEncodeValueRaw = QtWidgets.QLineEdit(self.widget_17)
        self.txtEncodeValueRaw.setEnabled(False)
        self.txtEncodeValueRaw.setMinimumSize(QtCore.QSize(0, 60))
        self.txtEncodeValueRaw.setObjectName("txtEncodeValueRaw")
        self.horizontalLayout_12.addWidget(self.txtEncodeValueRaw)
        self.verticalLayout_9.addWidget(self.widget_17)
        self.widget_15 = QtWidgets.QWidget(self.widget_16)
        self.widget_15.setObjectName("widget_15")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_15)
        self.horizontalLayout_10.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.btnSetClosedPosition = QtWidgets.QPushButton(self.widget_15)
        self.btnSetClosedPosition.setMinimumSize(QtCore.QSize(0, 62))
        self.btnSetClosedPosition.setMaximumSize(QtCore.QSize(150, 16777215))
        self.btnSetClosedPosition.setObjectName("btnSetClosedPosition")
        self.horizontalLayout_10.addWidget(self.btnSetClosedPosition)
        self.verticalLayout_9.addWidget(self.widget_15)
        self.verticalLayout_15.addWidget(self.widget_16)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_15.addItem(spacerItem)
        self.verticalLayout_12.addWidget(self.groupBox_4)
        self.groupBox_3 = QtWidgets.QGroupBox(self.widget_20)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_19 = QtWidgets.QWidget(self.groupBox_3)
        self.widget_19.setObjectName("widget_19")
        self.verticalLayout_10 = QtWidgets.QVBoxLayout(self.widget_19)
        self.verticalLayout_10.setContentsMargins(-1, 20, -1, 0)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.widget_8 = QtWidgets.QWidget(self.widget_19)
        self.widget_8.setObjectName("widget_8")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_8)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label = QtWidgets.QLabel(self.widget_8)
        self.label.setMinimumSize(QtCore.QSize(0, 60))
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.txtTargetThickness = QtWidgets.QLineEdit(self.widget_8)
        self.txtTargetThickness.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTargetThickness.setInputMethodHints(QtCore.Qt.ImhSensitiveData)
        self.txtTargetThickness.setObjectName("txtTargetThickness")
        self.horizontalLayout_2.addWidget(self.txtTargetThickness)
        self.verticalLayout_10.addWidget(self.widget_8)
        self.widget_7 = QtWidgets.QWidget(self.widget_19)
        self.widget_7.setObjectName("widget_7")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_7)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.widget_7)
        self.label_2.setMinimumSize(QtCore.QSize(0, 60))
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_3.addWidget(self.label_2)
        self.txtThicknessTolerance = QtWidgets.QLineEdit(self.widget_7)
        self.txtThicknessTolerance.setMinimumSize(QtCore.QSize(0, 60))
        self.txtThicknessTolerance.setObjectName("txtThicknessTolerance")
        self.horizontalLayout_3.addWidget(self.txtThicknessTolerance)
        self.verticalLayout_10.addWidget(self.widget_7)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_10.addItem(spacerItem1)
        self.verticalLayout.addWidget(self.widget_19)
        self.verticalLayout_12.addWidget(self.groupBox_3)
        self.horizontalLayout_4.addWidget(self.widget_20)
        self.widget_21 = QtWidgets.QWidget(self.widget)
        self.widget_21.setObjectName("widget_21")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout(self.widget_21)
        self.verticalLayout_13.setContentsMargins(0, -1, -1, -1)
        self.verticalLayout_13.setSpacing(9)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.groupBox = QtWidgets.QGroupBox(self.widget_21)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.widget_13 = QtWidgets.QWidget(self.groupBox)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_13)
        self.horizontalLayout_5.setContentsMargins(-1, 20, -1, -1)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.widget_2 = QtWidgets.QWidget(self.widget_13)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout_8.setContentsMargins(-1, -1, 0, -1)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_4 = QtWidgets.QLabel(self.widget_2)
        self.label_4.setText("")
        self.label_4.setObjectName("label_4")
        self.verticalLayout_8.addWidget(self.label_4)
        self.label_11 = QtWidgets.QLabel(self.widget_2)
        self.label_11.setMinimumSize(QtCore.QSize(0, 60))
        self.label_11.setObjectName("label_11")
        self.verticalLayout_8.addWidget(self.label_11)
        self.label_10 = QtWidgets.QLabel(self.widget_2)
        self.label_10.setMinimumSize(QtCore.QSize(0, 60))
        self.label_10.setObjectName("label_10")
        self.verticalLayout_8.addWidget(self.label_10)
        self.label_9 = QtWidgets.QLabel(self.widget_2)
        self.label_9.setMinimumSize(QtCore.QSize(0, 60))
        self.label_9.setObjectName("label_9")
        self.verticalLayout_8.addWidget(self.label_9)
        self.horizontalLayout_5.addWidget(self.widget_2)
        self.widget_11 = QtWidgets.QWidget(self.widget_13)
        self.widget_11.setObjectName("widget_11")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_11)
        self.verticalLayout_4.setContentsMargins(-1, -1, 0, -1)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_7 = QtWidgets.QLabel(self.widget_11)
        self.label_7.setMinimumSize(QtCore.QSize(0, 60))
        self.label_7.setObjectName("label_7")
        self.verticalLayout_4.addWidget(self.label_7)
        self.txtHousingOpenSlow = QtWidgets.QLineEdit(self.widget_11)
        self.txtHousingOpenSlow.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingOpenSlow.setObjectName("txtHousingOpenSlow")
        self.verticalLayout_4.addWidget(self.txtHousingOpenSlow)
        self.txtHousingOpenMedium = QtWidgets.QLineEdit(self.widget_11)
        self.txtHousingOpenMedium.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingOpenMedium.setObjectName("txtHousingOpenMedium")
        self.verticalLayout_4.addWidget(self.txtHousingOpenMedium)
        self.txtHousingOpenFast = QtWidgets.QLineEdit(self.widget_11)
        self.txtHousingOpenFast.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingOpenFast.setObjectName("txtHousingOpenFast")
        self.verticalLayout_4.addWidget(self.txtHousingOpenFast)
        self.horizontalLayout_5.addWidget(self.widget_11)
        self.widget_12 = QtWidgets.QWidget(self.widget_13)
        self.widget_12.setObjectName("widget_12")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.widget_12)
        self.verticalLayout_7.setContentsMargins(0, -1, -1, -1)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label_8 = QtWidgets.QLabel(self.widget_12)
        self.label_8.setMinimumSize(QtCore.QSize(0, 60))
        self.label_8.setObjectName("label_8")
        self.verticalLayout_7.addWidget(self.label_8)
        self.txtHousingCloseSlow = QtWidgets.QLineEdit(self.widget_12)
        self.txtHousingCloseSlow.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingCloseSlow.setObjectName("txtHousingCloseSlow")
        self.verticalLayout_7.addWidget(self.txtHousingCloseSlow)
        self.txtHousingCloseMedium = QtWidgets.QLineEdit(self.widget_12)
        self.txtHousingCloseMedium.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingCloseMedium.setObjectName("txtHousingCloseMedium")
        self.verticalLayout_7.addWidget(self.txtHousingCloseMedium)
        self.txtHousingCloseFast = QtWidgets.QLineEdit(self.widget_12)
        self.txtHousingCloseFast.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingCloseFast.setObjectName("txtHousingCloseFast")
        self.verticalLayout_7.addWidget(self.txtHousingCloseFast)
        self.horizontalLayout_5.addWidget(self.widget_12)
        self.verticalLayout_6.addWidget(self.widget_13)
        self.widget_5 = QtWidgets.QWidget(self.groupBox)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout_7.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_12 = QtWidgets.QLabel(self.widget_5)
        self.label_12.setMinimumSize(QtCore.QSize(0, 60))
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_7.addWidget(self.label_12)
        self.txtHousingStartPosition = QtWidgets.QLineEdit(self.widget_5)
        self.txtHousingStartPosition.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingStartPosition.setObjectName("txtHousingStartPosition")
        self.horizontalLayout_7.addWidget(self.txtHousingStartPosition)
        self.verticalLayout_6.addWidget(self.widget_5)
        self.widget_14 = QtWidgets.QWidget(self.groupBox)
        self.widget_14.setObjectName("widget_14")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_14)
        self.horizontalLayout_8.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_13 = QtWidgets.QLabel(self.widget_14)
        self.label_13.setMinimumSize(QtCore.QSize(0, 60))
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_8.addWidget(self.label_13)
        self.txtHousingEndPosition = QtWidgets.QLineEdit(self.widget_14)
        self.txtHousingEndPosition.setMinimumSize(QtCore.QSize(0, 60))
        self.txtHousingEndPosition.setObjectName("txtHousingEndPosition")
        self.horizontalLayout_8.addWidget(self.txtHousingEndPosition)
        self.verticalLayout_6.addWidget(self.widget_14)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem2)
        self.verticalLayout_13.addWidget(self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(self.widget_21)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout_11 = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.widget_18 = QtWidgets.QWidget(self.groupBox_2)
        self.widget_18.setObjectName("widget_18")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.widget_18)
        self.horizontalLayout_13.setContentsMargins(-1, 20, -1, -1)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.widget_10 = QtWidgets.QWidget(self.widget_18)
        self.widget_10.setObjectName("widget_10")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_10)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_3 = QtWidgets.QLabel(self.widget_10)
        self.label_3.setText("")
        self.label_3.setObjectName("label_3")
        self.verticalLayout_5.addWidget(self.label_3)
        self.label_49 = QtWidgets.QLabel(self.widget_10)
        self.label_49.setMinimumSize(QtCore.QSize(0, 60))
        self.label_49.setAlignment(QtCore.Qt.AlignCenter)
        self.label_49.setObjectName("label_49")
        self.verticalLayout_5.addWidget(self.label_49)
        self.label_51 = QtWidgets.QLabel(self.widget_10)
        self.label_51.setMinimumSize(QtCore.QSize(0, 60))
        self.label_51.setAlignment(QtCore.Qt.AlignCenter)
        self.label_51.setObjectName("label_51")
        self.verticalLayout_5.addWidget(self.label_51)
        self.label_50 = QtWidgets.QLabel(self.widget_10)
        self.label_50.setMinimumSize(QtCore.QSize(0, 60))
        self.label_50.setAlignment(QtCore.Qt.AlignCenter)
        self.label_50.setObjectName("label_50")
        self.verticalLayout_5.addWidget(self.label_50)
        self.horizontalLayout_13.addWidget(self.widget_10)
        self.widget_6 = QtWidgets.QWidget(self.widget_18)
        self.widget_6.setObjectName("widget_6")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_6)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_15 = QtWidgets.QLabel(self.widget_6)
        self.label_15.setMinimumSize(QtCore.QSize(0, 60))
        self.label_15.setObjectName("label_15")
        self.verticalLayout_2.addWidget(self.label_15)
        self.txtTunnelMinBoundX = QtWidgets.QLineEdit(self.widget_6)
        self.txtTunnelMinBoundX.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMinBoundX.setObjectName("txtTunnelMinBoundX")
        self.verticalLayout_2.addWidget(self.txtTunnelMinBoundX)
        self.txtTunnelMinBoundY = QtWidgets.QLineEdit(self.widget_6)
        self.txtTunnelMinBoundY.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMinBoundY.setObjectName("txtTunnelMinBoundY")
        self.verticalLayout_2.addWidget(self.txtTunnelMinBoundY)
        self.txtTunnelMinBoundZ = QtWidgets.QLineEdit(self.widget_6)
        self.txtTunnelMinBoundZ.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMinBoundZ.setObjectName("txtTunnelMinBoundZ")
        self.verticalLayout_2.addWidget(self.txtTunnelMinBoundZ)
        self.horizontalLayout_13.addWidget(self.widget_6)
        self.widget_9 = QtWidgets.QWidget(self.widget_18)
        self.widget_9.setObjectName("widget_9")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget_9)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_16 = QtWidgets.QLabel(self.widget_9)
        self.label_16.setMinimumSize(QtCore.QSize(0, 60))
        self.label_16.setObjectName("label_16")
        self.verticalLayout_3.addWidget(self.label_16)
        self.txtTunnelMaxBoundX = QtWidgets.QLineEdit(self.widget_9)
        self.txtTunnelMaxBoundX.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMaxBoundX.setObjectName("txtTunnelMaxBoundX")
        self.verticalLayout_3.addWidget(self.txtTunnelMaxBoundX)
        self.txtTunnelMaxBoundY = QtWidgets.QLineEdit(self.widget_9)
        self.txtTunnelMaxBoundY.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMaxBoundY.setObjectName("txtTunnelMaxBoundY")
        self.verticalLayout_3.addWidget(self.txtTunnelMaxBoundY)
        self.txtTunnelMaxBoundZ = QtWidgets.QLineEdit(self.widget_9)
        self.txtTunnelMaxBoundZ.setMinimumSize(QtCore.QSize(0, 60))
        self.txtTunnelMaxBoundZ.setObjectName("txtTunnelMaxBoundZ")
        self.verticalLayout_3.addWidget(self.txtTunnelMaxBoundZ)
        self.horizontalLayout_13.addWidget(self.widget_9)
        self.verticalLayout_11.addWidget(self.widget_18)
        spacerItem3 = QtWidgets.QSpacerItem(20, 17, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_11.addItem(spacerItem3)
        self.verticalLayout_13.addWidget(self.groupBox_2)
        self.widget_4 = QtWidgets.QWidget(self.widget_21)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem4)
        self.btnUpdateHousingParam = QtWidgets.QPushButton(self.widget_4)
        self.btnUpdateHousingParam.setMinimumSize(QtCore.QSize(0, 62))
        self.btnUpdateHousingParam.setObjectName("btnUpdateHousingParam")
        self.horizontalLayout.addWidget(self.btnUpdateHousingParam)
        self.btnCancelHousingParam = QtWidgets.QPushButton(self.widget_4)
        self.btnCancelHousingParam.setMinimumSize(QtCore.QSize(0, 62))
        self.btnCancelHousingParam.setObjectName("btnCancelHousingParam")
        self.horizontalLayout.addWidget(self.btnCancelHousingParam)
        self.verticalLayout_13.addWidget(self.widget_4)
        self.horizontalLayout_4.addWidget(self.widget_21)
        self.horizontalLayout_6.addWidget(self.widget)

        self.retranslateUi(setting_page)
        QtCore.QMetaObject.connectSlotsByName(setting_page)

    def retranslateUi(self, setting_page):
        _translate = QtCore.QCoreApplication.translate
        setting_page.setWindowTitle(_translate("setting_page", "Form"))
        self.groupBox_4.setTitle(_translate("setting_page", "Encoder closed position"))
        self.label_14.setText(_translate("setting_page", "Setting value:"))
        self.label_17.setText(_translate("setting_page", "Encode value:"))
        self.btnSetClosedPosition.setText(_translate("setting_page", "Set"))
        self.groupBox_3.setTitle(_translate("setting_page", "Thickness setting"))
        self.label.setText(_translate("setting_page", "Target thickness (mm)"))
        self.label_2.setText(_translate("setting_page", "Tolerance (mm)"))
        self.groupBox.setTitle(_translate("setting_page", "Housing parameters"))
        self.label_11.setText(_translate("setting_page", "Slow"))
        self.label_10.setText(_translate("setting_page", "Medium"))
        self.label_9.setText(_translate("setting_page", "Fast"))
        self.label_7.setText(_translate("setting_page", "Open speed"))
        self.label_8.setText(_translate("setting_page", "Close speed"))
        self.label_12.setText(_translate("setting_page", "Start angle (deg)"))
        self.label_13.setText(_translate("setting_page", "End scan angle (deg)"))
        self.groupBox_2.setTitle(_translate("setting_page", "Tunnel bounding box"))
        self.label_49.setText(_translate("setting_page", "X"))
        self.label_51.setText(_translate("setting_page", "Y"))
        self.label_50.setText(_translate("setting_page", "Z"))
        self.label_15.setText(_translate("setting_page", "Min bound (m)"))
        self.label_16.setText(_translate("setting_page", "Max bound (m)"))
        self.btnUpdateHousingParam.setText(_translate("setting_page", "Save All"))
        self.btnCancelHousingParam.setText(_translate("setting_page", "Cancel"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    setting_page = QtWidgets.QWidget()
    ui = Ui_setting_page()
    ui.setupUi(setting_page)
    setting_page.show()
    sys.exit(app.exec_())
