# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'jobsetting_page.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frmJobSetting(object):
    def setupUi(self, frmJobSetting):
        frmJobSetting.setObjectName("frmJobSetting")
        frmJobSetting.resize(515, 472)
        self.gridLayout = QtWidgets.QGridLayout(frmJobSetting)
        self.gridLayout.setObjectName("gridLayout")
        self.widget = QtWidgets.QWidget(frmJobSetting)
        self.widget.setObjectName("widget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setContentsMargins(4, 1, 4, 1)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.widget_2)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.txtJobNumber = QtWidgets.QLineEdit(self.widget_2)
        self.txtJobNumber.setMinimumSize(QtCore.QSize(150, 0))
        self.txtJobNumber.setObjectName("txtJobNumber")
        self.horizontalLayout.addWidget(self.txtJobNumber)
        self.btnCreateJob = QtWidgets.QPushButton(self.widget_2)
        self.btnCreateJob.setObjectName("btnCreateJob")
        self.horizontalLayout.addWidget(self.btnCreateJob)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.horizontalLayout.setStretch(1, 2)
        self.horizontalLayout.setStretch(2, 1)
        self.horizontalLayout.setStretch(3, 6)
        self.verticalLayout.addWidget(self.widget_2)
        self.widget_JobList = QtWidgets.QWidget(self.widget)
        self.widget_JobList.setObjectName("widget_JobList")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.widget_JobList)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.lstJobnumber = QtWidgets.QListWidget(self.widget_JobList)
        self.lstJobnumber.setObjectName("lstJobnumber")
        self.gridLayout_2.addWidget(self.lstJobnumber, 1, 0, 1, 1)
        self.widget_3 = QtWidgets.QWidget(self.widget_JobList)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.widget_3)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.txtFilter = QtWidgets.QLineEdit(self.widget_3)
        self.txtFilter.setMinimumSize(QtCore.QSize(150, 0))
        self.txtFilter.setObjectName("txtFilter")
        self.horizontalLayout_2.addWidget(self.txtFilter)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.horizontalLayout_2.setStretch(1, 2)
        self.horizontalLayout_2.setStretch(2, 6)
        self.gridLayout_2.addWidget(self.widget_3, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.widget_JobList)
        self.gridLayout.addWidget(self.widget, 0, 0, 1, 1)

        self.retranslateUi(frmJobSetting)
        QtCore.QMetaObject.connectSlotsByName(frmJobSetting)

    def retranslateUi(self, frmJobSetting):
        _translate = QtCore.QCoreApplication.translate
        frmJobSetting.setWindowTitle(_translate("frmJobSetting", "Form"))
        self.label.setText(_translate("frmJobSetting", "Job number"))
        self.btnCreateJob.setText(_translate("frmJobSetting", "Create"))
        self.label_2.setText(_translate("frmJobSetting", "Search:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frmJobSetting = QtWidgets.QWidget()
    ui = Ui_frmJobSetting()
    ui.setupUi(frmJobSetting)
    frmJobSetting.show()
    sys.exit(app.exec_())
