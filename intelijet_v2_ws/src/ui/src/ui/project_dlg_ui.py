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
    FONT_PAGE_TITLE, FONT_SECTION_TITLE, FONT_GROUP_TITLE, FONT_VALUE, FONT_SUBTEXT,
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
    border-radius: 12px;
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
#sortProjectLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_VALUE}; font-weight: 700;
}}
/* Styled explicitly as a control (pill + colored drop-down segment +
   chevron), not the plain-text QComboBox default - operators were
   reading it as a static label instead of something tappable. */
#cmbSortProject {{
    background-color: {ROW_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 10px;
    padding: 16px 52px 16px 22px;
    font-size: {FONT_VALUE};
    font-weight: 700;
}}
#cmbSortProject:hover {{ border: 1px solid {ACCENT_YELLOW_HOVER}; }}
#cmbSortProject::drop-down {{
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 44px;
    border-left: 1px solid {BORDER};
    border-top-right-radius: 10px;
    border-bottom-right-radius: 10px;
    background-color: {ACCENT_YELLOW};
}}
#cmbSortProject::down-arrow {{
    image: none;
    width: 0px;
    height: 0px;
    border-left: 7px solid transparent;
    border-right: 7px solid transparent;
    border-top: 9px solid #153E42;
    margin-right: 16px;
}}
#cmbSortProject QAbstractItemView {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    font-size: {FONT_VALUE};
    padding: 6px;
    outline: none;
}}
#cmbSortProject QAbstractItemView::item {{ padding: 8px 14px; }}
#cmbSortProject QAbstractItemView::item:selected {{
    background-color: {ACCENT_YELLOW};
    color: #153E42;
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

#jobTopBar QPushButton {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #fbbf24, stop:1 #f59e0b);
    color: #0f172a;
    border: 1px solid #d97706;
    border-radius: 10px;
    padding: 20px 34px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
#jobTopBar QPushButton:hover {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f59e0b, stop:1 #d97706);
}}

#projectCard {{
    background-color: #f8fafc;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
}}
#projectHeaderLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_GROUP_TITLE}; font-weight: 800;
}}
#jobRow {{
    background-color: {CARD_BG};
    border: 1px solid #e2e8f0;
    border-radius: 12px;
}}
/* Same size as the project title (FONT_GROUP_TITLE) - a job's own title
   reads at the same scale as the project card it belongs to. Not used by
   the New/Edit Job panel's own title anymore - see #jobEditPanelTitle,
   which tracks that panel's (bigger) field/label size instead. */
#jobRowTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_GROUP_TITLE}; font-weight: 700;
}}
/* The New/Edit Job panel's own title - kept in sync with its label/field
   font-size (FONT_SECTION_TITLE) instead of #jobRowTitle's smaller size,
   which was leaving the title looking mismatched/undersized next to its
   own fields. */
#jobEditPanelTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_SECTION_TITLE}; font-weight: 700;
}}
#jobRowSubtext, #scheduleSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SUBTEXT}; font-weight: 600;
}}
/* Faint rule under a job/schedule card's title, separating it from the
   info+actions area below it. */
#rowDivider {{ background-color: #e2e8f0; border: none; }}
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
    font-size: {FONT_SECTION_TITLE};
    font-weight: 600;
}}
#jobEditPanel QComboBox {{
    padding-right: 52px;
}}
/* Bumped up again (now FONT_SECTION_TITLE, same as the field next to it) -
   still felt small at FONT_GROUP_TITLE next to the panel's own big title. */
#jobEditFieldLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SECTION_TITLE}; font-weight: 600;
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
#scheduleCard {{
    background-color: #f8fafc;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
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
    background-color: #ffffff;
    color: #334155;
    border: 1px solid #cbd5e1;
    border-radius: 8px;
    padding: 35px 35px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 700;
}}
QPushButton[cssClass="rowActionBtn"]:hover {{ background-color: #f8fafc; border-color: #94a3b8; color: #0f172a; }}
QPushButton[cssClass="rowDangerBtn"] {{
    background-color: #ffffff;
    color: #ef4444;
    border: 1px solid #fca5a5;
    border-radius: 8px;
    padding: 35px 35px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowDangerBtn"]:hover {{ background-color: #fef2f2; border-color: #f87171; color: #dc2626; }}
QPushButton[cssClass="rowPrimaryBtn"] {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #fbbf24, stop:1 #f59e0b);
    color: #0f172a;
    border: 1px solid #d97706;
    border-radius: 8px;
    padding: 35px 35px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowPrimaryBtn"]:hover {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f59e0b, stop:1 #d97706);
}}
/* The per-job "Schedule" button once that job is already on the Work
   Schedule - a distinct filled/checked look (not the plain primary
   amber) so it reads as "this one's active", not just a repeatable
   action button. */
QPushButton[cssClass="rowScheduledBtn"] {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0d9488, stop:1 #0f766e);
    color: white;
    border: 1px solid #115e59;
    border-radius: 8px;
    padding: 35px 35px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
QPushButton[cssClass="rowScheduledBtn"]:hover {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0f766e, stop:1 #115e59);
}}
/* Icon-only row buttons (Delete/Remove) - same colors as their text
   counterparts above, but much less padding: the icon itself already
   fills the space text would, so the full 35px text-button padding on
   top of a 48px icon made these buttons noticeably taller/wider than
   every other row button next to them. */
QPushButton[cssClass="rowDangerIconBtn"] {{
    background-color: #ffffff;
    color: #ef4444;
    border: 1px solid #fca5a5;
    border-radius: 8px;
    padding: 14px;
}}
QPushButton[cssClass="rowDangerIconBtn"]:hover {{ background-color: #fef2f2; border-color: #f87171; color: #dc2626; }}
QPushButton[cssClass="rowActionIconBtn"] {{
    background-color: #ffffff;
    color: #64748b;
    border: 1px solid #cbd5e1;
    border-radius: 8px;
    padding: 14px;
}}
QPushButton[cssClass="rowActionIconBtn"]:hover {{ background-color: #f8fafc; border-color: #94a3b8; color: #0f172a; }}
""")
        self.rootLayout = QtWidgets.QVBoxLayout(frm_ProjectPage)
        self.rootLayout.setContentsMargins(40, 40, 40, 40)
        self.rootLayout.setSpacing(20)

        # ---- Top bar ----
        self.jobTopBar = QtWidgets.QWidget(frm_ProjectPage)
        self.jobTopBar.setObjectName("jobTopBar")
        self.jobTopBarLayout = QtWidgets.QHBoxLayout(self.jobTopBar)
        self.jobTopBarLayout.setContentsMargins(40, 14, 40, 14)
        self.jobTitle = QtWidgets.QLabel("Project Management", self.jobTopBar)
        self.jobTitle.setObjectName("jobTitle")
        self.jobTopBarLayout.addWidget(self.jobTitle)
        self.jobTopBarLayout.addStretch(1)
        self.btnNewProject = QtWidgets.QPushButton("+ New Project", self.jobTopBar)
        self.btnNewProject.setObjectName("btnNewProject")
        self.btnNewProject.setMinimumHeight(68)
        self.jobTopBarLayout.addWidget(self.btnNewProject)
        self.rootLayout.addWidget(self.jobTopBar)

        # ---- Main grid: Projects&Jobs (left) / Work Schedule (right) ----
        self.mainGrid = QtWidgets.QHBoxLayout()
        self.mainGrid.setSpacing(20)
        self.rootLayout.addLayout(self.mainGrid, 1)

        # Left card
        self.projectsCard = QtWidgets.QFrame(frm_ProjectPage)
        self.projectsCard.setObjectName("jobCard")
        self.projectsCardLayout = QtWidgets.QVBoxLayout(self.projectsCard)
        self.projectsCardLayout.setContentsMargins(36, 36, 36, 36)
        self.projectsCardLayout.setSpacing(14)

        self.projectsHeaderRow = QtWidgets.QHBoxLayout()
        self.projectsCardTitle = QtWidgets.QLabel("Projects & Jobs List", self.projectsCard)
        self.projectsCardTitle.setObjectName("jobCardTitle")
        self.projectsHeaderRow.addWidget(self.projectsCardTitle)
        self.projectsHeaderRow.addStretch(1)
        self.lblSortProject = QtWidgets.QLabel("Sort:", self.projectsCard)
        self.lblSortProject.setObjectName("sortProjectLabel")
        self.projectsHeaderRow.addWidget(self.lblSortProject)
        self.cmbSortProject = QtWidgets.QComboBox(self.projectsCard)
        self.cmbSortProject.setObjectName("cmbSortProject")
        self.cmbSortProject.setMinimumHeight(88)
        self.cmbSortProject.setMinimumWidth(220)
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
        self.projectsListLayout.setSpacing(14)
        self.projectsListLayout.addStretch(1)
        self.projectsScroll.setWidget(self.projectsScrollContent)
        self.projectsCardLayout.addWidget(self.projectsScroll, 1)

        self.mainGrid.addWidget(self.projectsCard, 6)

        # Right card
        self.scheduleCardOuter = QtWidgets.QFrame(frm_ProjectPage)
        self.scheduleCardOuter.setObjectName("jobCard")
        self.scheduleCardOuterLayout = QtWidgets.QVBoxLayout(self.scheduleCardOuter)
        self.scheduleCardOuterLayout.setContentsMargins(36, 36, 36, 36)
        self.scheduleCardOuterLayout.setSpacing(14)

        self.scheduleCardTitle = QtWidgets.QLabel("Work Schedule", self.scheduleCardOuter)
        self.scheduleCardTitle.setObjectName("jobCardTitle")
        self.scheduleCardOuterLayout.addWidget(self.scheduleCardTitle)

        self.scheduleScroll = QtWidgets.QScrollArea(self.scheduleCardOuter)
        self.scheduleScroll.setWidgetResizable(True)
        self.scheduleScrollContent = QtWidgets.QWidget()
        self.scheduleListLayout = QtWidgets.QVBoxLayout(self.scheduleScrollContent)
        self.scheduleListLayout.setContentsMargins(0, 0, 0, 0)
        self.scheduleListLayout.setSpacing(14)
        self.scheduleListLayout.addStretch(1)
        self.scheduleScroll.setWidget(self.scheduleScrollContent)
        self.scheduleCardOuterLayout.addWidget(self.scheduleScroll, 1)

        self.mainGrid.addWidget(self.scheduleCardOuter, 4)

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
