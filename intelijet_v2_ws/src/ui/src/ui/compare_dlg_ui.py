# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'compare_dlg.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frm_MainForm(object):
    def setupUi(self, frm_MainForm):
        frm_MainForm.setObjectName("frm_MainForm")
        frm_MainForm.setWindowModality(QtCore.Qt.WindowModal)
        frm_MainForm.resize(1200, 800)
        frm_MainForm.setStyleSheet("\n"
"QPushButton {\n"
"    background-color: #5C87C9;     /* xanh sáng hơn */\n"
"    color: white;\n"
"    border: 0px solid #3c6382;\n"
"    padding: 6px 10px;\n"
"    font-size: 14px;\n"
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
"    font-size: 14px;\n"
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
"    font-size: 14px;\n"
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
"    font-size: 16px;\n"
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
"    width: 12px;\n"
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
"}")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(frm_MainForm)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.ftm_ProjectManager = QtWidgets.QWidget(frm_MainForm)
        self.ftm_ProjectManager.setStyleSheet("/* Toàn bộ list widget */\n"
"QListWidget {\n"
"    background-color: #FFFFFF;      /* nền trắng */\n"
"    color: #2F4F6E;                /* chữ xanh đậm */\n"
"    border: 1px solid #B0C4DE;     /* viền nhẹ */\n"
"    font-size: 14px;\n"
"    font-weight: bold;\n"
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
"/* Item trong list */\n"
"QListWidget::item {\n"
"    padding: 6px 10px;             /* khoảng cách chữ trong item */\n"
"    height: 30px;                  /* tăng chiều cao item */\n"
"}\n"
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
"")
        self.ftm_ProjectManager.setObjectName("ftm_ProjectManager")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.ftm_ProjectManager)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.widget_7 = QtWidgets.QWidget(self.ftm_ProjectManager)
        self.widget_7.setObjectName("widget_7")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_7)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.groupBox = QtWidgets.QGroupBox(self.widget_7)
        font = QtGui.QFont()
        # font.setPointSize(-1)
        font.setBold(True)
        font.setWeight(75)
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_2 = QtWidgets.QWidget(self.groupBox)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setContentsMargins(-1, 20, -1, -1)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.lblProjectName = QtWidgets.QLabel(self.widget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lblProjectName.setFont(font)
        self.lblProjectName.setObjectName("lblProjectName")
        self.horizontalLayout.addWidget(self.lblProjectName)
        self.verticalLayout.addWidget(self.widget_2)
        self.widget_3 = QtWidgets.QWidget(self.groupBox)
        self.widget_3.setObjectName("widget_3")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_3)
        self.verticalLayout_5.setContentsMargins(-1, 6, -1, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.widget_5 = QtWidgets.QWidget(self.widget_3)
        self.widget_5.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid rgb(200, 200, 200);\n"
"border-radius: 20px; ")
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout_3.setContentsMargins(6, 2, 6, 2)
        self.horizontalLayout_3.setSpacing(6)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btnSearchProject = QtWidgets.QPushButton(self.widget_5)
        self.btnSearchProject.setMinimumSize(QtCore.QSize(30, 30))
        self.btnSearchProject.setMaximumSize(QtCore.QSize(30, 30))
        self.btnSearchProject.setStyleSheet(" border: none;\n"
"")
        self.btnSearchProject.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/icon/search-3-48.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnSearchProject.setIcon(icon)
        self.btnSearchProject.setObjectName("btnSearchProject")
        self.horizontalLayout_3.addWidget(self.btnSearchProject)
        self.txtSearchProject = QtWidgets.QLineEdit(self.widget_5)
        self.txtSearchProject.setMinimumSize(QtCore.QSize(150, 40))
        font = QtGui.QFont()
        # font.setPointSize(-1)
        font.setBold(True)
        font.setWeight(75)
        self.txtSearchProject.setFont(font)
        self.txtSearchProject.setStyleSheet("border: none;")
        self.txtSearchProject.setClearButtonEnabled(True)
        self.txtSearchProject.setObjectName("txtSearchProject")
        self.horizontalLayout_3.addWidget(self.txtSearchProject)
        self.verticalLayout_5.addWidget(self.widget_5)
        self.widget_4 = QtWidgets.QWidget(self.widget_3)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout_2.setContentsMargins(6, 0, 6, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.btnNewProject = QtWidgets.QPushButton(self.widget_4)
        self.btnNewProject.setMaximumSize(QtCore.QSize(40, 40))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/icon/add-folder.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnNewProject.setIcon(icon1)
        self.btnNewProject.setIconSize(QtCore.QSize(40, 40))
        self.btnNewProject.setObjectName("btnNewProject")
        self.horizontalLayout_2.addWidget(self.btnNewProject)
        self.btnRenameProject = QtWidgets.QPushButton(self.widget_4)
        self.btnRenameProject.setMaximumSize(QtCore.QSize(40, 40))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/icon/icon/rename.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnRenameProject.setIcon(icon2)
        self.btnRenameProject.setIconSize(QtCore.QSize(40, 40))
        self.btnRenameProject.setObjectName("btnRenameProject")
        self.horizontalLayout_2.addWidget(self.btnRenameProject)
        self.btnDeleteProject = QtWidgets.QPushButton(self.widget_4)
        self.btnDeleteProject.setMaximumSize(QtCore.QSize(40, 40))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/icon/icon/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnDeleteProject.setIcon(icon3)
        self.btnDeleteProject.setIconSize(QtCore.QSize(40, 40))
        self.btnDeleteProject.setObjectName("btnDeleteProject")
        self.horizontalLayout_2.addWidget(self.btnDeleteProject)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.verticalLayout_5.addWidget(self.widget_4)
        self.verticalLayout.addWidget(self.widget_3)
        self.lstProject = QtWidgets.QListWidget(self.groupBox)
        self.lstProject.setObjectName("lstProject")
        self.verticalLayout.addWidget(self.lstProject)
        self.horizontalLayout_5.addWidget(self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(self.widget_7)
        font = QtGui.QFont()
        # font.setPointSize(-1)
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_2.setFont(font)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget_12 = QtWidgets.QWidget(self.groupBox_2)
        self.widget_12.setObjectName("widget_12")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_12)
        self.horizontalLayout_6.setContentsMargins(-1, 20, -1, -1)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.lblJobName = QtWidgets.QLabel(self.widget_12)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lblJobName.setFont(font)
        self.lblJobName.setObjectName("lblJobName")
        self.horizontalLayout_6.addWidget(self.lblJobName)
        self.verticalLayout_2.addWidget(self.widget_12)
        self.widget_8 = QtWidgets.QWidget(self.groupBox_2)
        self.widget_8.setObjectName("widget_8")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_8)
        self.verticalLayout_4.setContentsMargins(-1, 0, -1, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_9 = QtWidgets.QWidget(self.widget_8)
        self.widget_9.setObjectName("widget_9")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_9)
        self.horizontalLayout_8.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.verticalLayout_4.addWidget(self.widget_9)
        self.widget_6 = QtWidgets.QWidget(self.widget_8)
        self.widget_6.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid rgb(200, 200, 200);\n"
"border-radius: 20px; ")
        self.widget_6.setObjectName("widget_6")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_6)
        self.horizontalLayout_4.setContentsMargins(6, 2, 6, 2)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.btnSearchProject_2 = QtWidgets.QPushButton(self.widget_6)
        self.btnSearchProject_2.setMinimumSize(QtCore.QSize(30, 30))
        self.btnSearchProject_2.setMaximumSize(QtCore.QSize(30, 30))
        self.btnSearchProject_2.setStyleSheet(" border: none;\n"
"")
        self.btnSearchProject_2.setText("")
        self.btnSearchProject_2.setIcon(icon)
        self.btnSearchProject_2.setObjectName("btnSearchProject_2")
        self.horizontalLayout_4.addWidget(self.btnSearchProject_2)
        self.txtSearchJob = QtWidgets.QLineEdit(self.widget_6)
        self.txtSearchJob.setMinimumSize(QtCore.QSize(150, 40))
        font = QtGui.QFont()
        # font.setPointSize(-1)
        font.setBold(True)
        font.setWeight(75)
        self.txtSearchJob.setFont(font)
        self.txtSearchJob.setStyleSheet("border: none;")
        self.txtSearchJob.setClearButtonEnabled(True)
        self.txtSearchJob.setObjectName("txtSearchJob")
        self.horizontalLayout_4.addWidget(self.txtSearchJob)
        self.verticalLayout_4.addWidget(self.widget_6)
        self.widget_10 = QtWidgets.QWidget(self.widget_8)
        self.widget_10.setObjectName("widget_10")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.widget_10)
        self.horizontalLayout_9.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.btnNewJob = QtWidgets.QPushButton(self.widget_10)
        self.btnNewJob.setMinimumSize(QtCore.QSize(40, 40))
        self.btnNewJob.setMaximumSize(QtCore.QSize(40, 40))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(":/icon/icon/add-file.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnNewJob.setIcon(icon4)
        self.btnNewJob.setIconSize(QtCore.QSize(32, 32))
        self.btnNewJob.setObjectName("btnNewJob")
        self.horizontalLayout_9.addWidget(self.btnNewJob)
        self.btnEditJob = QtWidgets.QPushButton(self.widget_10)
        self.btnEditJob.setMinimumSize(QtCore.QSize(40, 40))
        self.btnEditJob.setMaximumSize(QtCore.QSize(40, 40))
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(":/icon/icon/note.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnEditJob.setIcon(icon5)
        self.btnEditJob.setIconSize(QtCore.QSize(32, 32))
        self.btnEditJob.setObjectName("btnEditJob")
        self.horizontalLayout_9.addWidget(self.btnEditJob)
        self.btnDeleteJob = QtWidgets.QPushButton(self.widget_10)
        self.btnDeleteJob.setMinimumSize(QtCore.QSize(40, 40))
        self.btnDeleteJob.setMaximumSize(QtCore.QSize(40, 40))
        self.btnDeleteJob.setIcon(icon3)
        self.btnDeleteJob.setIconSize(QtCore.QSize(32, 32))
        self.btnDeleteJob.setObjectName("btnDeleteJob")
        self.horizontalLayout_9.addWidget(self.btnDeleteJob)
        self.cbbFilterJob = QtWidgets.QComboBox(self.widget_10)
        self.cbbFilterJob.setMinimumSize(QtCore.QSize(0, 30))
        self.cbbFilterJob.setObjectName("cbbFilterJob")
        self.cbbFilterJob.addItem("")
        self.cbbFilterJob.addItem("")
        self.cbbFilterJob.addItem("")
        self.cbbFilterJob.addItem("")
        self.horizontalLayout_9.addWidget(self.cbbFilterJob)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_9.addItem(spacerItem1)
        self.verticalLayout_4.addWidget(self.widget_10)
        self.verticalLayout_2.addWidget(self.widget_8)
        self.lstJobInfo = QtWidgets.QListWidget(self.groupBox_2)
        self.lstJobInfo.setMaximumSize(QtCore.QSize(16777215, 200))
        self.lstJobInfo.setObjectName("lstJobInfo")
        self.verticalLayout_2.addWidget(self.lstJobInfo)
        self.lstJob = QtWidgets.QListWidget(self.groupBox_2)
        self.lstJob.setObjectName("lstJob")
        self.verticalLayout_2.addWidget(self.lstJob)
        self.horizontalLayout_5.addWidget(self.groupBox_2)
        self.groupBox_3 = QtWidgets.QGroupBox(self.widget_7)
        font = QtGui.QFont()
        # font.setPointSize(-1)
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_3.setFont(font)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_11 = QtWidgets.QWidget(self.groupBox_3)
        self.widget_11.setObjectName("widget_11")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.widget_11)
        self.verticalLayout_9.setContentsMargins(-1, 0, -1, 0)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.widget_18 = QtWidgets.QWidget(self.widget_11)
        self.widget_18.setObjectName("widget_18")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout(self.widget_18)
        self.horizontalLayout_15.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.btnOpenItem = QtWidgets.QPushButton(self.widget_18)
        self.btnOpenItem.setMinimumSize(QtCore.QSize(40, 40))
        self.btnOpenItem.setMaximumSize(QtCore.QSize(40, 40))
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(":/icon/icon/visual.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnOpenItem.setIcon(icon6)
        self.btnOpenItem.setIconSize(QtCore.QSize(32, 32))
        self.btnOpenItem.setObjectName("btnOpenItem")
        self.horizontalLayout_15.addWidget(self.btnOpenItem)
        self.btnDeleteItem = QtWidgets.QPushButton(self.widget_18)
        self.btnDeleteItem.setMinimumSize(QtCore.QSize(40, 40))
        self.btnDeleteItem.setMaximumSize(QtCore.QSize(40, 40))
        self.btnDeleteItem.setIcon(icon3)
        self.btnDeleteItem.setIconSize(QtCore.QSize(32, 32))
        self.btnDeleteItem.setObjectName("btnDeleteItem")
        self.horizontalLayout_15.addWidget(self.btnDeleteItem)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_15.addItem(spacerItem2)
        self.verticalLayout_9.addWidget(self.widget_18)
        self.verticalLayout_3.addWidget(self.widget_11)
        self.lstJobDetail = QtWidgets.QListWidget(self.groupBox_3)
        self.lstJobDetail.setObjectName("lstJobDetail")
        self.verticalLayout_3.addWidget(self.lstJobDetail)
        self.widget = QtWidgets.QWidget(self.groupBox_3)
        self.widget.setObjectName("widget")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem3)
        self.btnOk = QtWidgets.QPushButton(self.widget)
        self.btnOk.setMinimumSize(QtCore.QSize(0, 30))
        self.btnOk.setObjectName("btnOk")
        self.horizontalLayout_7.addWidget(self.btnOk)
        self.btnCancel = QtWidgets.QPushButton(self.widget)
        self.btnCancel.setMinimumSize(QtCore.QSize(0, 30))
        self.btnCancel.setObjectName("btnCancel")
        self.horizontalLayout_7.addWidget(self.btnCancel)
        self.verticalLayout_3.addWidget(self.widget)
        self.horizontalLayout_5.addWidget(self.groupBox_3)
        self.horizontalLayout_5.setStretch(0, 2)
        self.horizontalLayout_5.setStretch(1, 2)
        self.horizontalLayout_5.setStretch(2, 3)
        self.verticalLayout_6.addWidget(self.widget_7)
        self.verticalLayout_7.addWidget(self.ftm_ProjectManager)

        self.retranslateUi(frm_MainForm)
        QtCore.QMetaObject.connectSlotsByName(frm_MainForm)

    def retranslateUi(self, frm_MainForm):
        _translate = QtCore.QCoreApplication.translate
        self.groupBox.setTitle(_translate("frm_MainForm", "Project"))
        self.lblProjectName.setText(_translate("frm_MainForm", "#CurrnetProject"))
        self.txtSearchProject.setPlaceholderText(_translate("frm_MainForm", "Search"))
        self.groupBox_2.setTitle(_translate("frm_MainForm", "Job Number"))
        self.lblJobName.setText(_translate("frm_MainForm", "#CurrnetJob"))
        self.txtSearchJob.setPlaceholderText(_translate("frm_MainForm", "Search"))
        self.cbbFilterJob.setItemText(0, _translate("frm_MainForm", "All"))
        self.cbbFilterJob.setItemText(1, _translate("frm_MainForm", "Pending"))
        self.cbbFilterJob.setItemText(2, _translate("frm_MainForm", "Activate"))
        self.cbbFilterJob.setItemText(3, _translate("frm_MainForm", "Finish"))
        self.groupBox_3.setTitle(_translate("frm_MainForm", "Job Detail"))
        self.btnOk.setText(_translate("frm_MainForm", "Compare"))
        self.btnCancel.setText(_translate("frm_MainForm", "Close"))
from ui import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frm_MainForm = QtWidgets.QWidget()
    ui = Ui_frm_MainForm()
    ui.setupUi(frm_MainForm)
    frm_MainForm.show()
    sys.exit(app.exec_())
