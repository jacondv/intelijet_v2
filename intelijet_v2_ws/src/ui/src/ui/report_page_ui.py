# -*- coding: utf-8 -*-

# Form implementation - HAND-MAINTAINED. New merged "COMPARE & REPORT"
# screen (replaces the old CompareManager/ReportViewManager modal
# dialogs - see report_page_manager.py), styled like the JOB tab
# (project_dlg_ui.py): light cards, big touch-friendly rows. Static
# shell only - the file list is built dynamically by ReportPageManager.

from PyQt5 import QtCore, QtWidgets

# ---- Same light-card palette as project_dlg_ui.py / SYSTEM tab ----
LIGHT_BG = "#f4f5f8"
CARD_BG = "#ffffff"
BORDER = "#e2e8f0"
TEXT = "#1e293b"
TEXT_MUTED = "#64748b"
ACCENT_YELLOW = "#fbc02d"
ACCENT_YELLOW_HOVER = "#f57f17"
ACCENT_RED = "#c62828"
ROW_BG = "#f8fafc"


class Ui_frm_ReportPage(object):
    def setupUi(self, frm_ReportPage):
        frm_ReportPage.setObjectName("frm_ReportPage")
        frm_ReportPage.setStyleSheet(f"""
#frm_ReportPage {{ background-color: {LIGHT_BG}; }}

#reportTopBar, #reportCard {{
    background-color: {CARD_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#reportTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 40px; font-weight: 800;
}}
#reportCardTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 36px; font-weight: 800;
    padding-bottom: 16px; border-bottom: 3px solid {BORDER};
}}
#jobSelectorLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: 22px; font-weight: 700;
}}
#cbbJobPicker {{
    background-color: {ROW_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 14px 18px;
    font-size: 26px;
    font-weight: 700;
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

#fileRow {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
#fileRowTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 26px; font-weight: 800;
}}
#fileRowSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: 18px; font-weight: 600;
}}
#emptyStateLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: 24px; font-weight: 600;
    padding: 32px;
}}
#compareBar {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#compareBarLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 24px; font-weight: 700;
}}

QCheckBox {{ background: transparent; border: none; spacing: 10px; }}
QCheckBox::indicator {{
    width: 32px; height: 32px;
    border: 2px solid {BORDER}; border-radius: 6px; background: {CARD_BG};
}}
QCheckBox::indicator:checked {{ background-color: {ACCENT_YELLOW}; border-color: {ACCENT_YELLOW_HOVER}; }}

QPushButton[cssClass="rowActionBtn"] {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 14px 20px;
    font-size: 19px;
    font-weight: 700;
}}
QPushButton[cssClass="rowActionBtn"]:hover {{ border-color: {ACCENT_YELLOW}; }}
QPushButton[cssClass="rowActionBtn"]:disabled {{ color: #cbd5e1; border-color: {BORDER}; }}
QPushButton[cssClass="rowDangerBtn"] {{
    background-color: {CARD_BG};
    color: {ACCENT_RED};
    border: 1px solid {ACCENT_RED};
    border-radius: 8px;
    padding: 14px 20px;
    font-size: 19px;
    font-weight: 700;
}}
QPushButton[cssClass="rowDangerBtn"]:hover {{ background-color: {ACCENT_RED}; color: white; }}
QPushButton[cssClass="rowPrimaryBtn"] {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
    border: none;
    border-radius: 8px;
    padding: 14px 26px;
    font-size: 19px;
    font-weight: 800;
}}
QPushButton[cssClass="rowPrimaryBtn"]:hover {{ background-color: {ACCENT_YELLOW_HOVER}; color: white; }}
QPushButton[cssClass="rowPrimaryBtn"]:disabled {{ background-color: #e2e8f0; color: #94a3b8; }}
""")
        self.rootLayout = QtWidgets.QVBoxLayout(frm_ReportPage)
        self.rootLayout.setContentsMargins(40, 40, 40, 40)
        self.rootLayout.setSpacing(28)

        # ---- Top bar: title + job picker ----
        self.reportTopBar = QtWidgets.QWidget(frm_ReportPage)
        self.reportTopBar.setObjectName("reportTopBar")
        self.reportTopBarLayout = QtWidgets.QHBoxLayout(self.reportTopBar)
        self.reportTopBarLayout.setContentsMargins(40, 28, 40, 28)
        self.reportTopBarLayout.setSpacing(16)
        self.reportTitle = QtWidgets.QLabel("Compare & Report", self.reportTopBar)
        self.reportTitle.setObjectName("reportTitle")
        self.reportTopBarLayout.addWidget(self.reportTitle)
        self.reportTopBarLayout.addStretch(1)
        self.jobSelectorLabel = QtWidgets.QLabel("Job:", self.reportTopBar)
        self.jobSelectorLabel.setObjectName("jobSelectorLabel")
        self.reportTopBarLayout.addWidget(self.jobSelectorLabel)
        self.cbbJobPicker = QtWidgets.QComboBox(self.reportTopBar)
        self.cbbJobPicker.setObjectName("cbbJobPicker")
        self.cbbJobPicker.setMinimumHeight(72)
        self.cbbJobPicker.setMinimumWidth(420)
        self.reportTopBarLayout.addWidget(self.cbbJobPicker)
        self.rootLayout.addWidget(self.reportTopBar)

        # ---- File list card ----
        self.filesCard = QtWidgets.QFrame(frm_ReportPage)
        self.filesCard.setObjectName("reportCard")
        self.filesCardLayout = QtWidgets.QVBoxLayout(self.filesCard)
        self.filesCardLayout.setContentsMargins(36, 36, 36, 36)
        self.filesCardLayout.setSpacing(20)

        self.filesCardTitle = QtWidgets.QLabel("Point Clouds", self.filesCard)
        self.filesCardTitle.setObjectName("reportCardTitle")
        self.filesCardLayout.addWidget(self.filesCardTitle)

        self.filesScroll = QtWidgets.QScrollArea(self.filesCard)
        self.filesScroll.setWidgetResizable(True)
        self.filesScrollContent = QtWidgets.QWidget()
        self.filesListLayout = QtWidgets.QVBoxLayout(self.filesScrollContent)
        self.filesListLayout.setContentsMargins(0, 0, 0, 0)
        self.filesListLayout.setSpacing(16)
        self.filesListLayout.addStretch(1)
        self.filesScroll.setWidget(self.filesScrollContent)
        self.filesCardLayout.addWidget(self.filesScroll, 1)

        self.rootLayout.addWidget(self.filesCard, 1)

        # ---- Bottom compare bar ----
        self.compareBar = QtWidgets.QWidget(frm_ReportPage)
        self.compareBar.setObjectName("compareBar")
        self.compareBarLayout = QtWidgets.QHBoxLayout(self.compareBar)
        self.compareBarLayout.setContentsMargins(28, 20, 28, 20)
        self.compareBarLabel = QtWidgets.QLabel("Select 2 point clouds to compare (0/2 selected)", self.compareBar)
        self.compareBarLabel.setObjectName("compareBarLabel")
        self.compareBarLayout.addWidget(self.compareBarLabel)
        self.compareBarLayout.addStretch(1)
        self.btnStartCompare = QtWidgets.QPushButton("Compare Selected", self.compareBar)
        self.btnStartCompare.setObjectName("btnStartCompare")
        self.btnStartCompare.setProperty("cssClass", "rowPrimaryBtn")
        self.btnStartCompare.setEnabled(False)
        self.compareBarLayout.addWidget(self.btnStartCompare)
        self.rootLayout.addWidget(self.compareBar)

        self.retranslateUi(frm_ReportPage)
        QtCore.QMetaObject.connectSlotsByName(frm_ReportPage)

    def retranslateUi(self, frm_ReportPage):
        _translate = QtCore.QCoreApplication.translate
        frm_ReportPage.setWindowTitle(_translate("frm_ReportPage", "Form"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frm_ReportPage = QtWidgets.QWidget()
    ui = Ui_frm_ReportPage()
    ui.setupUi(frm_ReportPage)
    frm_ReportPage.resize(1400, 900)
    frm_ReportPage.show()
    sys.exit(app.exec_())
