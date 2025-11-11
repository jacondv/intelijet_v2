# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'reportselect_dlg.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ReportSelect(object):
    def setupUi(self, ReportSelect):
        ReportSelect.setObjectName("ReportSelect")
        ReportSelect.resize(656, 621)
        ReportSelect.setStyleSheet("QLineEdit#txtSearch{\n"
"    border-radius: 20px;\n"
"    padding-left:10px;\n"
"}\n"
"\n"
"QDialogButtonBox QPushButton {\n"
"    min-width: 60px;\n"
"    min-height: 25px;\n"
"    padding: 10px 20px;\n"
"}")
        self.buttonBox = QtWidgets.QDialogButtonBox(ReportSelect)
        self.buttonBox.setGeometry(QtCore.QRect(460, 550, 171, 71))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Close)
        self.buttonBox.setObjectName("buttonBox")
        self.widget_2 = QtWidgets.QWidget(ReportSelect)
        self.widget_2.setGeometry(QtCore.QRect(0, 10, 641, 541))
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget = QtWidgets.QWidget(self.widget_2)
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(6, 0, -1, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.txtSearch = QtWidgets.QLineEdit(self.widget)
        self.txtSearch.setMinimumSize(QtCore.QSize(0, 40))
        self.txtSearch.setMaximumSize(QtCore.QSize(300, 16777215))
        self.txtSearch.setObjectName("txtSearch")
        self.horizontalLayout.addWidget(self.txtSearch)
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.cbShow = QtWidgets.QComboBox(self.widget)
        self.cbShow.setMinimumSize(QtCore.QSize(0, 40))
        self.cbShow.setObjectName("cbShow")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.cbShow.addItem("")
        self.horizontalLayout.addWidget(self.cbShow)
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.horizontalLayout.setStretch(0, 5)
        self.horizontalLayout.setStretch(3, 1)
        self.verticalLayout_2.addWidget(self.widget)
        self.lstJobs = QtWidgets.QListWidget(self.widget_2)
        self.lstJobs.setObjectName("lstJobs")
        self.verticalLayout_2.addWidget(self.lstJobs)
        self.widget_3 = QtWidgets.QWidget(self.widget_2)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setContentsMargins(0, -1, 0, -1)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lstJobItems = QtWidgets.QListWidget(self.widget_3)
        self.lstJobItems.setObjectName("lstJobItems")
        self.horizontalLayout_2.addWidget(self.lstJobItems)
        self.verticalLayout_2.addWidget(self.widget_3)

        self.retranslateUi(ReportSelect)
        self.buttonBox.accepted.connect(ReportSelect.accept)
        self.buttonBox.rejected.connect(ReportSelect.reject)
        QtCore.QMetaObject.connectSlotsByName(ReportSelect)

    def retranslateUi(self, ReportSelect):
        _translate = QtCore.QCoreApplication.translate
        ReportSelect.setWindowTitle(_translate("ReportSelect", "Dialog"))
        self.txtSearch.setPlaceholderText(_translate("ReportSelect", "Search by name or keyword…"))
        self.label.setText(_translate("ReportSelect", "Show"))
        self.cbShow.setItemText(0, _translate("ReportSelect", "10"))
        self.cbShow.setItemText(1, _translate("ReportSelect", "20"))
        self.cbShow.setItemText(2, _translate("ReportSelect", "50"))
        self.cbShow.setItemText(3, _translate("ReportSelect", "All"))
        self.label_2.setText(_translate("ReportSelect", "items"))
import resource_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ReportSelect = QtWidgets.QDialog()
    ui = Ui_ReportSelect()
    ui.setupUi(ReportSelect)
    ReportSelect.show()
    sys.exit(app.exec_())
