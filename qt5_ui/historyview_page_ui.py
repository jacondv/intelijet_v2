# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'historyview_page.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_frmHistoryView(object):
    def setupUi(self, frmHistoryView):
        frmHistoryView.setObjectName("frmHistoryView")
        frmHistoryView.resize(423, 468)
        self.gridLayout = QtWidgets.QGridLayout(frmHistoryView)
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
        self.verticalLayout.setContentsMargins(-1, 0, 0, 0)
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
        self.verticalLayout_2.setContentsMargins(9, 9, 0, 0)
        self.verticalLayout_2.setSpacing(4)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.widget_3)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_2.addWidget(self.label_2)
        self.lstJobDetail = QtWidgets.QListWidget(self.widget_3)
        self.lstJobDetail.setObjectName("lstJobDetail")
        self.verticalLayout_2.addWidget(self.lstJobDetail)
        self.verticalLayout_3.addWidget(self.widget_3)
        self.verticalLayout_3.setStretch(0, 2)
        self.verticalLayout_3.setStretch(1, 3)
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
