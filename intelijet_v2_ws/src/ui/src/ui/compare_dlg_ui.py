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
        frm_MainForm.resize(1021, 610)
        frm_MainForm.setStyleSheet("QFrame #frame,\n"
"QFrame #frame_2,\n"
"QFrame #frame_3,\n"
"{\n"
"    border: 1px solid #ddd;\n"
"}")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(frm_MainForm)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.ftm_ProjectManager = QtWidgets.QWidget(frm_MainForm)
        self.ftm_ProjectManager.setStyleSheet("QListWidget {\n"
"    background: #ffffff;\n"
"    border: 1px solid #cccccc;\n"
"    font-family: \"Segoe UI\", \"Roboto\", sans-serif;\n"
"    font-size: 14px;\n"
"    color: #222;\n"
"    outline: none;\n"
"    padding: 0px;\n"
"}\n"
"\n"
"QListWidget::item {\n"
"    background: #ffffff;\n"
"    border: none;\n"
"    padding: 10px 12px;      /* tăng chiều cao item */\n"
"    margin: 0;\n"
"}\n"
"\n"
"QListWidget::item:selected {\n"
"    background: #e6f0ff;\n"
"    color: #003366;\n"
"}\n"
"\n"
"QListWidget::item:hover {\n"
"    background: #f5f9ff;\n"
"}\n"
"\n"
"/* ---- Scrollbar ---- */\n"
"QScrollBar:vertical {\n"
"    border: none;\n"
"    background: #f0f0f0;\n"
"    width: 18px;              /* to hơn mặc định */\n"
"    margin: 0px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical {\n"
"    background: #c0c0c0;\n"
"    min-height: 30px;\n"
"    border-radius: 6px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical:hover {\n"
"    background: #a6a6a6;\n"
"}\n"
"\n"
"QScrollBar::add-line:vertical,\n"
"QScrollBar::sub-line:vertical {\n"
"    height: 0px;              /* bỏ nút mũi tên */\n"
"}\n"
"\n"
"QScrollBar::add-page:vertical,\n"
"QScrollBar::sub-page:vertical {\n"
"    background: none;\n"
"}\n"
"\n"
"\n"
"\n"
"")
        self.ftm_ProjectManager.setObjectName("ftm_ProjectManager")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.ftm_ProjectManager)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.widget_7 = QtWidgets.QWidget(self.ftm_ProjectManager)
        self.widget_7.setObjectName("widget_7")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_7)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.frame = QtWidgets.QFrame(self.widget_7)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(0, -1, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_3 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("padding-left:10px;")
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.widget_3 = QtWidgets.QWidget(self.frame)
        self.widget_3.setObjectName("widget_3")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_3)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.widget_2 = QtWidgets.QWidget(self.widget_3)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.lblProjectName = QtWidgets.QLabel(self.widget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lblProjectName.setFont(font)
        self.lblProjectName.setObjectName("lblProjectName")
        self.horizontalLayout.addWidget(self.lblProjectName)
        self.verticalLayout_5.addWidget(self.widget_2)
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
        font.setPointSize(15)
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
        self.lstProject = QtWidgets.QListWidget(self.frame)
        self.lstProject.setStyleSheet("border: none;")
        self.lstProject.setObjectName("lstProject")
        self.verticalLayout.addWidget(self.lstProject)
        self.horizontalLayout_6.addWidget(self.frame)
        self.frame_2 = QtWidgets.QFrame(self.widget_7)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_2.setContentsMargins(0, -1, 0, -1)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_4 = QtWidgets.QLabel(self.frame_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setStyleSheet("padding-left:10px;")
        self.label_4.setObjectName("label_4")
        self.verticalLayout_2.addWidget(self.label_4)
        self.widget_8 = QtWidgets.QWidget(self.frame_2)
        self.widget_8.setObjectName("widget_8")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_8)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_9 = QtWidgets.QWidget(self.widget_8)
        self.widget_9.setObjectName("widget_9")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_9)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.lblJobName = QtWidgets.QLabel(self.widget_9)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lblJobName.setFont(font)
        self.lblJobName.setObjectName("lblJobName")
        self.horizontalLayout_8.addWidget(self.lblJobName)
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
        font.setPointSize(15)
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
        self.lstJob = QtWidgets.QListWidget(self.frame_2)
        self.lstJob.setStyleSheet("border: none;")
        self.lstJob.setObjectName("lstJob")
        self.verticalLayout_2.addWidget(self.lstJob)
        self.widget_12 = QtWidgets.QWidget(self.frame_2)
        self.widget_12.setObjectName("widget_12")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_12)
        self.horizontalLayout_5.setContentsMargins(0, 0, -1, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.verticalLayout_2.addWidget(self.widget_12)
        self.horizontalLayout_6.addWidget(self.frame_2)
        self.frame_3 = QtWidgets.QFrame(self.widget_7)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_11 = QtWidgets.QWidget(self.frame_3)
        self.widget_11.setObjectName("widget_11")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.widget_11)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_5 = QtWidgets.QLabel(self.widget_11)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setStyleSheet("padding-left:10px;")
        self.label_5.setObjectName("label_5")
        self.verticalLayout_8.addWidget(self.label_5)
        self.lstJobInfo = QtWidgets.QListWidget(self.widget_11)
        self.lstJobInfo.setMaximumSize(QtCore.QSize(16777215, 200))
        self.lstJobInfo.setStyleSheet("border: none;")
        self.lstJobInfo.setObjectName("lstJobInfo")
        self.verticalLayout_8.addWidget(self.lstJobInfo)
        self.lstJobDetail = QtWidgets.QListWidget(self.widget_11)
        self.lstJobDetail.setObjectName("lstJobDetail")
        self.verticalLayout_8.addWidget(self.lstJobDetail)
        self.verticalLayout_3.addWidget(self.widget_11)
        self.widget = QtWidgets.QWidget(self.frame_3)
        self.widget.setObjectName("widget")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem2)
        self.btnOk = QtWidgets.QPushButton(self.widget)
        self.btnOk.setMinimumSize(QtCore.QSize(0, 30))
        self.btnOk.setObjectName("btnOk")
        self.horizontalLayout_7.addWidget(self.btnOk)
        self.btnCancel = QtWidgets.QPushButton(self.widget)
        self.btnCancel.setMinimumSize(QtCore.QSize(0, 30))
        self.btnCancel.setObjectName("btnCancel")
        self.horizontalLayout_7.addWidget(self.btnCancel)
        self.verticalLayout_3.addWidget(self.widget)
        self.horizontalLayout_6.addWidget(self.frame_3)
        self.horizontalLayout_6.setStretch(0, 2)
        self.horizontalLayout_6.setStretch(1, 2)
        self.horizontalLayout_6.setStretch(2, 3)
        self.verticalLayout_6.addWidget(self.widget_7)
        self.verticalLayout_7.addWidget(self.ftm_ProjectManager)

        self.retranslateUi(frm_MainForm)
        QtCore.QMetaObject.connectSlotsByName(frm_MainForm)

    def retranslateUi(self, frm_MainForm):
        _translate = QtCore.QCoreApplication.translate
        frm_MainForm.setWindowTitle(_translate("frm_MainForm", "Form"))
        self.label_3.setText(_translate("frm_MainForm", "Project List"))
        self.lblProjectName.setText(_translate("frm_MainForm", "#CurrnetProject"))
        self.label_4.setText(_translate("frm_MainForm", "Job List"))
        self.lblJobName.setText(_translate("frm_MainForm", "#CurrnetJob"))
        self.cbbFilterJob.setItemText(0, _translate("frm_MainForm", "All"))
        self.cbbFilterJob.setItemText(1, _translate("frm_MainForm", "Pending"))
        self.cbbFilterJob.setItemText(2, _translate("frm_MainForm", "Activate"))
        self.cbbFilterJob.setItemText(3, _translate("frm_MainForm", "Finish"))
        self.label_5.setText(_translate("frm_MainForm", "Job Detail"))
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
