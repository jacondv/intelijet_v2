# -*- coding: utf-8 -*-

# Form implementation - HAND-MAINTAINED (feature/new_ui redesign, JOB tab).
# Replaces the old pyuic5-generated 3-column layout (Project list / Job
# list+filter / Active Work Orders) with the 2-column layout from
# docs/ui_sample/App.html's TAB 2 (JOB MANAGEMENT): a "Projects & Jobs
# List" card (search + always-expanded project blocks, no collapse) and
# a "Work Schedule" card. Project/job cards are built dynamically by
# ProjectManager (project_dlg_manager.py), not here - this file only
# lays out the static shell (top bar, the two card containers, search
# box) since the row content changes with the data.

from PyQt5 import QtCore, QtWidgets

# ---- Light-card palette, matching the SYSTEM tab (docs/ui_sample/App.html) ----
LIGHT_BG = "#f4f5f8"
CARD_BG = "#ffffff"
BORDER = "#e2e8f0"
TEXT = "#1e293b"
TEXT_MUTED = "#64748b"
ACCENT_YELLOW = "#fbc02d"
ACCENT_YELLOW_HOVER = "#f57f17"
ACCENT_RED = "#c62828"
ROW_BG = "#f8fafc"


class Ui_frm_ProjectPage(object):
    def setupUi(self, frm_ProjectPage):
        frm_ProjectPage.setObjectName("frm_ProjectPage")
        frm_ProjectPage.setStyleSheet(f"""
#frm_ProjectPage {{ background-color: {LIGHT_BG}; }}

#jobTopBar, #jobCard {{
    background-color: {CARD_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#jobTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 22px; font-weight: 800;
}}
#jobCardTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 20px; font-weight: 800;
    padding-bottom: 8px; border-bottom: 2px solid {BORDER};
}}
#txtSearch {{
    background-color: {ROW_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 16px;
    font-weight: 600;
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

#jobTopBar QPushButton {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
    border: none;
    border-radius: 8px;
    padding: 10px 18px;
    font-size: 16px;
    font-weight: 800;
}}
#jobTopBar QPushButton:hover {{ background-color: {ACCENT_YELLOW_HOVER}; color: white; }}

#projectCard {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
#projectHeaderLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 18px; font-weight: 800;
}}
#jobRow {{
    background-color: {CARD_BG};
    border: 1px solid {BORDER};
    border-radius: 6px;
}}
#jobRowTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 16px; font-weight: 700;
}}
#jobRowSubtext, #scheduleSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: 13px; font-weight: 600;
}}
#scheduleCard {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
#scheduleTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: 17px; font-weight: 800;
}}
#emptyStateLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: 15px; font-weight: 600;
    padding: 20px;
}}

/* Row action buttons are tagged via setProperty("cssClass", ...) at
   construction time (Qt Style Sheets have no CSS ".class" selector -
   only #id, type, and [attribute] selectors are supported). */
QPushButton[cssClass="rowActionBtn"] {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 13px;
    font-weight: 700;
}}
QPushButton[cssClass="rowActionBtn"]:hover {{ border-color: {ACCENT_YELLOW}; }}
QPushButton[cssClass="rowDangerBtn"] {{
    background-color: {CARD_BG};
    color: {ACCENT_RED};
    border: 1px solid {ACCENT_RED};
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 13px;
    font-weight: 700;
}}
QPushButton[cssClass="rowDangerBtn"]:hover {{ background-color: {ACCENT_RED}; color: white; }}
QPushButton[cssClass="rowPrimaryBtn"] {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
    border: none;
    border-radius: 6px;
    padding: 6px 14px;
    font-size: 13px;
    font-weight: 800;
}}
QPushButton[cssClass="rowPrimaryBtn"]:hover {{ background-color: {ACCENT_YELLOW_HOVER}; color: white; }}
""")
        self.rootLayout = QtWidgets.QVBoxLayout(frm_ProjectPage)
        self.rootLayout.setContentsMargins(20, 20, 20, 20)
        self.rootLayout.setSpacing(16)

        # ---- Top bar ----
        self.jobTopBar = QtWidgets.QWidget(frm_ProjectPage)
        self.jobTopBar.setObjectName("jobTopBar")
        self.jobTopBarLayout = QtWidgets.QHBoxLayout(self.jobTopBar)
        self.jobTopBarLayout.setContentsMargins(20, 14, 20, 14)
        self.jobTitle = QtWidgets.QLabel("HMI Execution Management", self.jobTopBar)
        self.jobTitle.setObjectName("jobTitle")
        self.jobTopBarLayout.addWidget(self.jobTitle)
        self.jobTopBarLayout.addStretch(1)
        self.btnNewProject = QtWidgets.QPushButton("+ New Project", self.jobTopBar)
        self.btnNewProject.setObjectName("btnNewProject")
        self.btnNewProject.setMinimumHeight(52)
        self.jobTopBarLayout.addWidget(self.btnNewProject)
        self.btnNewJob = QtWidgets.QPushButton("+ New Job", self.jobTopBar)
        self.btnNewJob.setObjectName("btnNewJob")
        self.btnNewJob.setMinimumHeight(52)
        self.jobTopBarLayout.addWidget(self.btnNewJob)
        self.rootLayout.addWidget(self.jobTopBar)

        # ---- Main grid: Projects&Jobs (left) / Work Schedule (right) ----
        self.mainGrid = QtWidgets.QHBoxLayout()
        self.mainGrid.setSpacing(16)
        self.rootLayout.addLayout(self.mainGrid, 1)

        # Left card
        self.projectsCard = QtWidgets.QFrame(frm_ProjectPage)
        self.projectsCard.setObjectName("jobCard")
        self.projectsCardLayout = QtWidgets.QVBoxLayout(self.projectsCard)
        self.projectsCardLayout.setContentsMargins(18, 18, 18, 18)
        self.projectsCardLayout.setSpacing(10)

        self.projectsHeaderRow = QtWidgets.QHBoxLayout()
        self.projectsCardTitle = QtWidgets.QLabel("Projects & Jobs List", self.projectsCard)
        self.projectsCardTitle.setObjectName("jobCardTitle")
        self.projectsHeaderRow.addWidget(self.projectsCardTitle)
        self.projectsHeaderRow.addStretch(1)
        self.txtSearchProject = QtWidgets.QLineEdit(self.projectsCard)
        self.txtSearchProject.setObjectName("txtSearch")
        self.txtSearchProject.setPlaceholderText("Search...")
        self.txtSearchProject.setMinimumHeight(44)
        self.txtSearchProject.setMaximumWidth(240)
        self.txtSearchProject.setClearButtonEnabled(True)
        self.projectsHeaderRow.addWidget(self.txtSearchProject)
        self.projectsCardLayout.addLayout(self.projectsHeaderRow)

        self.projectsScroll = QtWidgets.QScrollArea(self.projectsCard)
        self.projectsScroll.setWidgetResizable(True)
        self.projectsScrollContent = QtWidgets.QWidget()
        self.projectsListLayout = QtWidgets.QVBoxLayout(self.projectsScrollContent)
        self.projectsListLayout.setContentsMargins(0, 0, 0, 0)
        self.projectsListLayout.setSpacing(10)
        self.projectsListLayout.addStretch(1)
        self.projectsScroll.setWidget(self.projectsScrollContent)
        self.projectsCardLayout.addWidget(self.projectsScroll, 1)

        self.mainGrid.addWidget(self.projectsCard, 1)

        # Right card
        self.scheduleCardOuter = QtWidgets.QFrame(frm_ProjectPage)
        self.scheduleCardOuter.setObjectName("jobCard")
        self.scheduleCardOuterLayout = QtWidgets.QVBoxLayout(self.scheduleCardOuter)
        self.scheduleCardOuterLayout.setContentsMargins(18, 18, 18, 18)
        self.scheduleCardOuterLayout.setSpacing(10)

        self.scheduleCardTitle = QtWidgets.QLabel("Work Schedule", self.scheduleCardOuter)
        self.scheduleCardTitle.setObjectName("jobCardTitle")
        self.scheduleCardOuterLayout.addWidget(self.scheduleCardTitle)

        self.scheduleScroll = QtWidgets.QScrollArea(self.scheduleCardOuter)
        self.scheduleScroll.setWidgetResizable(True)
        self.scheduleScrollContent = QtWidgets.QWidget()
        self.scheduleListLayout = QtWidgets.QVBoxLayout(self.scheduleScrollContent)
        self.scheduleListLayout.setContentsMargins(0, 0, 0, 0)
        self.scheduleListLayout.setSpacing(10)
        self.scheduleListLayout.addStretch(1)
        self.scheduleScroll.setWidget(self.scheduleScrollContent)
        self.scheduleCardOuterLayout.addWidget(self.scheduleScroll, 1)

        self.mainGrid.addWidget(self.scheduleCardOuter, 1)

        self.retranslateUi(frm_ProjectPage)
        QtCore.QMetaObject.connectSlotsByName(frm_ProjectPage)

    def retranslateUi(self, frm_ProjectPage):
        _translate = QtCore.QCoreApplication.translate
        frm_ProjectPage.setWindowTitle(_translate("frm_ProjectPage", "Form"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    frm_ProjectPage = QtWidgets.QWidget()
    ui = Ui_frm_ProjectPage()
    ui.setupUi(frm_ProjectPage)
    frm_ProjectPage.resize(1400, 900)
    frm_ProjectPage.show()
    sys.exit(app.exec_())
