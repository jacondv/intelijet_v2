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

from ui.style_tokens import (
    LIGHT_BG, CARD_BG, BORDER, TEXT, TEXT_MUTED, ACCENT_YELLOW, ACCENT_YELLOW_HOVER, ACCENT_RED, ROW_BG,
    FONT_PAGE_TITLE, FONT_SECTION_TITLE, FONT_GROUP_TITLE, FONT_ROW_TITLE, FONT_VALUE, FONT_SUBTEXT,
    FONT_EMPTY_STATE, FONT_BUTTON, FONT_BUTTON_PRIMARY,
)


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
    color: {TEXT}; font-size: {FONT_PAGE_TITLE}; font-weight: 800;
}}
#jobCardTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_SECTION_TITLE}; font-weight: 800;
    padding-bottom: 16px; border-bottom: 3px solid {BORDER};
}}
#txtSearch {{
    background-color: {ROW_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 16px 22px;
    font-size: {FONT_VALUE};
    font-weight: 600;
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

#jobTopBar QPushButton {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
    border: none;
    border-radius: 10px;
    padding: 20px 34px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
#jobTopBar QPushButton:hover {{ background-color: {ACCENT_YELLOW_HOVER}; color: white; }}

#projectCard {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#projectHeaderLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_GROUP_TITLE}; font-weight: 800;
}}
#jobRow {{
    background-color: {CARD_BG};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
#jobRowTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_ROW_TITLE}; font-weight: 700;
}}
#jobRowSubtext, #scheduleSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SUBTEXT}; font-weight: 600;
}}
/* Inline New/Edit Job panel and New Project/Rename Project's inline text
   field - replaces the old JobInfoDialog/NewProjectDlg popups (see
   ProjectManager's docstring for why: a popup can't reliably dodge the
   on-screen keyboard on this touchscreen, an inline field just scrolls
   into view above it instead). */
#jobEditPanel {{
    background-color: {CARD_BG};
    border: 2px solid {ACCENT_YELLOW};
    border-radius: 8px;
}}
#jobEditField, #jobEditPanel QComboBox, #jobEditPanel QSpinBox {{
    background-color: {ROW_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 10px 14px;
    font-size: {FONT_VALUE};
    font-weight: 600;
}}
#jobEditPanel QComboBox {{
    padding-right: 52px;
}}
#jobEditFieldLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SUBTEXT}; font-weight: 700;
    margin-top: 6px;
}}
#jobEditPanel QComboBox::drop-down {{
    width: 44px;
    border: none;
}}
/* The dropdown popup is a separate top-level view, not a child of
   #jobEditPanel, so it needs its own selector - without this it falls
   back to Qt's tiny default popup styling (small font, cramped padding),
   which is what was clipping/obscuring the Status items. Row height
   itself is NOT set here (::item min-height has no effect in this Qt
   build - see _ComboRowDelegate in project_dlg_manager.py, which is
   what actually fixes the overlapping rows). */
#jobEditPanel QComboBox QAbstractItemView {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    font-size: {FONT_VALUE};
    padding: 6px;
    outline: none;
}}
#jobEditPanel QComboBox QAbstractItemView::item {{
    padding: 8px 14px;
}}
#jobEditPanel QComboBox QAbstractItemView::item:selected {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
}}
#jobEditPanel QLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_SUBTEXT}; font-weight: 700;
}}
#scheduleCard {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#scheduleTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_GROUP_TITLE}; font-weight: 800;
}}
#emptyStateLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_EMPTY_STATE}; font-weight: 600;
    padding: 32px;
}}

/* Row action buttons are tagged via setProperty("cssClass", ...) at
   construction time (Qt Style Sheets have no CSS ".class" selector -
   only #id, type, and [attribute] selectors are supported). */
QPushButton[cssClass="rowActionBtn"] {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 18px 26px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowActionBtn"]:hover {{ border-color: {ACCENT_YELLOW}; }}
QPushButton[cssClass="rowDangerBtn"] {{
    background-color: {CARD_BG};
    color: {ACCENT_RED};
    border: 1px solid {ACCENT_RED};
    border-radius: 8px;
    padding: 18px 26px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowDangerBtn"]:hover {{ background-color: {ACCENT_RED}; color: white; }}
QPushButton[cssClass="rowPrimaryBtn"] {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
    border: none;
    border-radius: 8px;
    padding: 18px 26px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowPrimaryBtn"]:hover {{ background-color: {ACCENT_YELLOW_HOVER}; color: white; }}
/* The per-job "Schedule" button once that job is already on the Work
   Schedule - a distinct filled/checked look (not the plain primary
   yellow) so it reads as "this one's active", not just a repeatable
   action button. */
QPushButton[cssClass="rowScheduledBtn"] {{
    background-color: #16a34a;
    color: white;
    border: none;
    border-radius: 8px;
    padding: 18px 26px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowScheduledBtn"]:hover {{ background-color: #15803d; }}
""")
        self.rootLayout = QtWidgets.QVBoxLayout(frm_ProjectPage)
        self.rootLayout.setContentsMargins(40, 40, 40, 40)
        self.rootLayout.setSpacing(32)

        # ---- Top bar ----
        self.jobTopBar = QtWidgets.QWidget(frm_ProjectPage)
        self.jobTopBar.setObjectName("jobTopBar")
        self.jobTopBarLayout = QtWidgets.QHBoxLayout(self.jobTopBar)
        self.jobTopBarLayout.setContentsMargins(40, 28, 40, 28)
        self.jobTitle = QtWidgets.QLabel("HMI Execution Management", self.jobTopBar)
        self.jobTitle.setObjectName("jobTitle")
        self.jobTopBarLayout.addWidget(self.jobTitle)
        self.jobTopBarLayout.addStretch(1)
        self.btnNewProject = QtWidgets.QPushButton("+ New Project", self.jobTopBar)
        self.btnNewProject.setObjectName("btnNewProject")
        self.btnNewProject.setMinimumHeight(104)
        self.jobTopBarLayout.addWidget(self.btnNewProject)
        self.rootLayout.addWidget(self.jobTopBar)

        # ---- Main grid: Projects&Jobs (left) / Work Schedule (right) ----
        self.mainGrid = QtWidgets.QHBoxLayout()
        self.mainGrid.setSpacing(32)
        self.rootLayout.addLayout(self.mainGrid, 1)

        # Left card
        self.projectsCard = QtWidgets.QFrame(frm_ProjectPage)
        self.projectsCard.setObjectName("jobCard")
        self.projectsCardLayout = QtWidgets.QVBoxLayout(self.projectsCard)
        self.projectsCardLayout.setContentsMargins(36, 36, 36, 36)
        self.projectsCardLayout.setSpacing(20)

        self.projectsHeaderRow = QtWidgets.QHBoxLayout()
        self.projectsCardTitle = QtWidgets.QLabel("Projects & Jobs List", self.projectsCard)
        self.projectsCardTitle.setObjectName("jobCardTitle")
        self.projectsHeaderRow.addWidget(self.projectsCardTitle)
        self.projectsHeaderRow.addStretch(1)
        self.cmbSortProject = QtWidgets.QComboBox(self.projectsCard)
        self.cmbSortProject.setObjectName("cmbSortProject")
        self.cmbSortProject.setMinimumHeight(88)
        self.cmbSortProject.addItem("Newest", "date_desc")
        self.cmbSortProject.addItem("Oldest", "date_asc")
        self.cmbSortProject.addItem("A to Z", "name_asc")
        self.cmbSortProject.addItem("Z to A", "name_desc")
        self.projectsHeaderRow.addWidget(self.cmbSortProject)
        self.txtSearchProject = QtWidgets.QLineEdit(self.projectsCard)
        self.txtSearchProject.setObjectName("txtSearch")
        self.txtSearchProject.setPlaceholderText("Search...")
        self.txtSearchProject.setMinimumHeight(88)
        self.txtSearchProject.setMaximumWidth(400)
        self.txtSearchProject.setClearButtonEnabled(True)
        self.projectsHeaderRow.addWidget(self.txtSearchProject)
        self.projectsCardLayout.addLayout(self.projectsHeaderRow)

        self.projectsScroll = QtWidgets.QScrollArea(self.projectsCard)
        self.projectsScroll.setWidgetResizable(True)
        self.projectsScrollContent = QtWidgets.QWidget()
        self.projectsListLayout = QtWidgets.QVBoxLayout(self.projectsScrollContent)
        self.projectsListLayout.setContentsMargins(0, 0, 0, 0)
        self.projectsListLayout.setSpacing(20)
        self.projectsListLayout.addStretch(1)
        self.projectsScroll.setWidget(self.projectsScrollContent)
        self.projectsCardLayout.addWidget(self.projectsScroll, 1)

        self.mainGrid.addWidget(self.projectsCard, 1)

        # Right card
        self.scheduleCardOuter = QtWidgets.QFrame(frm_ProjectPage)
        self.scheduleCardOuter.setObjectName("jobCard")
        self.scheduleCardOuterLayout = QtWidgets.QVBoxLayout(self.scheduleCardOuter)
        self.scheduleCardOuterLayout.setContentsMargins(36, 36, 36, 36)
        self.scheduleCardOuterLayout.setSpacing(20)

        self.scheduleCardTitle = QtWidgets.QLabel("Work Schedule", self.scheduleCardOuter)
        self.scheduleCardTitle.setObjectName("jobCardTitle")
        self.scheduleCardOuterLayout.addWidget(self.scheduleCardTitle)

        self.scheduleScroll = QtWidgets.QScrollArea(self.scheduleCardOuter)
        self.scheduleScroll.setWidgetResizable(True)
        self.scheduleScrollContent = QtWidgets.QWidget()
        self.scheduleListLayout = QtWidgets.QVBoxLayout(self.scheduleScrollContent)
        self.scheduleListLayout.setContentsMargins(0, 0, 0, 0)
        self.scheduleListLayout.setSpacing(20)
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
