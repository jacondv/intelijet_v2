# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './historyview_page.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frmHistoryView(object):
    def setupUi(self, frmHistoryView):
        frmHistoryView.setObjectName("frmHistoryView")
        frmHistoryView.resize(423, 468)
        frmHistoryView.setStyleSheet("* {\n"
"    font-size: 13pt;\n"
"}\n"
"QPushButton {\n"
"    background-color: #5C87C9;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 0px solid #3c6382;\n"
"    border-radius: 15px;\n"
"    padding: 6px 10px;\n"
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
"\n"
"}\n"
"\n"
"QLabel {\n"
"    color: #2F4F6E;           /* chữ giống TextEdit */\n"
"    font-weight: normal;\n"
"    border: 0px solid #D3D3D3;\n"
"    padding: 2px 8px;\n"
"}\n"
"\n"
"QLabel#label,\n"
"QLabel#label_2{\n"
"    color: #FFFFFF;           /* chữ giống TextEdit */\n"
"    font-weight: bold;\n"
"\n"
"}\n"
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
"\n"
"\n"
"/* Toàn bộ list widget */\n"
"QListWidget {\n"
"    background-color: #FFFFFF;      /* nền trắng */\n"
"    color: #2F4F6E;                /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;     /* viền nhẹ */\n"
"    font-size: 15pt;\n"
"    font-weight: normal;\n"
"    padding: 2px;\n"
"}\n"
"\n"
"/* Khi hover trên widget */\n"
"QListWidget:hover {\n"
"    border: 1px solid #74A9D8;\n"
"}\n"
"\n"
"/* Khi focus widget */\n"
"QListWidget:focus {\n"
"    border: 1px solid #5C87C9;\n"
"    background-color: #F9FBFF;\n"
"}\n"
"\n"
"/* Định kiểu cho mỗi item trong List */\n"
"QListWidget::item {\n"
"    padding: 2px;             /* Khoảng đệm bên trong item */\n"
"    border-bottom: 1px solid #eeeeee; /* Đường kẻ mờ phân chia các item */\n"
"    min-height: 70px;\n"
"}\n"
"\n"
"\n"
"/* Item được chọn */\n"
"QListWidget::item:selected {\n"
"    background-color: #D0E4F5;      /* màu chọn */\n"
"    color: #2F4F6E;\n"
"}\n"
"\n"
"/* Item hover */\n"
"QListWidget::item:hover {\n"
"    background-color: #E4F0FA;      /* màu hover */\n"
"}\n"
"/* Toàn bộ list widget */\n"
"QListWidget#lstJobDetail {\n"
"    font-size: 13pt;\n"
"}\n"
"\n"
"")
        self.gridLayout = QtWidgets.QGridLayout(frmHistoryView)
        self.gridLayout.setContentsMargins(0, -1, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.widget = QtWidgets.QWidget(frmHistoryView)
        self.widget.setObjectName("widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(15)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout.setContentsMargins(6, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.widget_2)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.lstJobs = QtWidgets.QListWidget(self.widget_2)
        self.lstJobs.setObjectName("lstJobs")
        self.verticalLayout.addWidget(self.lstJobs)
        self.verticalLayout_3.addWidget(self.widget_2)
        self.widget_3 = QtWidgets.QWidget(self.widget)
        self.widget_3.setObjectName("widget_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_3)
        self.verticalLayout_2.setContentsMargins(6, 6, 0, 0)
        self.verticalLayout_2.setSpacing(4)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.widget_3)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_2.addWidget(self.label_2)
        self.lstJobDetail = QtWidgets.QListWidget(self.widget_3)
        self.lstJobDetail.setObjectName("lstJobDetail")
        self.verticalLayout_2.addWidget(self.lstJobDetail)
        self.verticalLayout_3.addWidget(self.widget_3)
        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 2)
        self.gridLayout.addWidget(self.widget, 0, 0, 1, 1)

        self.retranslateUi(frmHistoryView)
        QtCore.QMetaObject.connectSlotsByName(frmHistoryView)

    def retranslateUi(self, frmHistoryView):
        _translate = QtCore.QCoreApplication.translate
        frmHistoryView.setWindowTitle(_translate("frmHistoryView", "Form"))
        self.label.setText(_translate("frmHistoryView", "Job list"))
        self.label_2.setText(_translate("frmHistoryView", "Job detail"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frmHistoryView = QtWidgets.QWidget()
    ui = Ui_frmHistoryView()
    ui.setupUi(frmHistoryView)
    frmHistoryView.show()
    sys.exit(app.exec_())
