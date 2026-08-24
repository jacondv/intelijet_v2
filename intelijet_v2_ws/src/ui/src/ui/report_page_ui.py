# -*- coding: utf-8 -*-

# Form implementation - HAND-MAINTAINED. New merged "COMPARE & REPORT"
# screen (replaces the old CompareManager/ReportViewManager modal
# dialogs - see report_page_manager.py), styled like the JOB tab
# (project_dlg_ui.py): light cards, big touch-friendly rows. Static
# shell only - the file list is built dynamically by ReportPageManager.

import os

from PyQt5 import QtCore, QtWidgets

from ui.style_tokens import (
    LIGHT_BG, CARD_BG, BORDER, TEXT, TEXT_MUTED, ACCENT_YELLOW, ACCENT_YELLOW_HOVER, ACCENT_RED, ROW_BG,
    FONT_PAGE_TITLE, FONT_SECTION_TITLE, FONT_GROUP_TITLE, FONT_ROW_TITLE, FONT_VALUE, FONT_FIELD_LABEL,
    FONT_SUBTEXT, FONT_EMPTY_STATE, FONT_BUTTON, FONT_BUTTON_PRIMARY,
)

# Qt's stylesheet engine only paints ::down-arrow if given a real image -
# once a QComboBox has ANY stylesheet rule, Qt switches it to the
# "styled" (QStyleSheetStyle) paint path, which does NOT fall back to the
# native style's arrow glyph the way an unstyled widget would (confirmed:
# a border-only/no-image ::down-arrow rule here rendered nothing at all,
# not even a native default - it just silently disappeared). This PNG
# (generated once via tools/gen_chevron.py, not at runtime) is the fix.
_CHEVRON_ICON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons", "chevron_down.png").replace("\\", "/")


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
    color: {TEXT}; font-size: {FONT_PAGE_TITLE}; font-weight: 800;
}}
#reportCardTitleRow {{
    border-bottom: 3px solid {BORDER};
}}
#reportCardTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_SECTION_TITLE}; font-weight: 800;
}}
/* ---- Unified field box: label + combobox drawn as ONE bordered unit,
   same idea as the header's "CURRENT JOB: ..." box (jobSelectorBox in
   intelijet_ui.py) - the box itself carries the border/background, the
   combobox inside it is borderless (just a transparent focus ring) so
   the label reads as part of the same control instead of a separate
   sibling next to it. ---- */
#fieldBox {{
    background-color: {CARD_BG};
    border: 2px solid {BORDER};
    border-radius: 10px;
}}
#fieldBox:hover {{ border-color: #cbd5e1; }}
#fieldBoxLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_FIELD_LABEL}; font-weight: 700;
}}

/* ---- Search box: a plain QLineEdit sitting above a #fieldBox combo,
   filtering that combo's item list live (see
   report_page_manager.py's _wire_search_filter) - kept as its own
   separate control instead of an editable combobox, so the combobox
   itself stays selection-only (tap to open, tap an item to choose). ---- */
QLineEdit[cssClass="pickerSearchBox"] {{
    background-color: {CARD_BG};
    border: 2px solid {BORDER};
    border-radius: 10px;
    padding: 10px 14px;
    color: {TEXT};
    font-size: {FONT_FIELD_LABEL};
    font-weight: 600;
}}
QLineEdit[cssClass="pickerSearchBox"]:focus {{ border-color: {ACCENT_YELLOW}; }}

/* ---- Modern combobox: flat field + native chevron. Selection-only
   (not editable) - a separate #pickerSearchBox QLineEdit above each one
   handles type-to-filter instead (see _build_searchable_field). The
   visible border/background now live on the #fieldBox wrapper above,
   not here - this stays borderless except for a focus ring. ---- */
#cbbProjectPicker, #cbbJobPicker, #cbbSegmentFilter {{
    background-color: transparent;
    color: {TEXT};
    border: 2px solid transparent;
    border-radius: 8px;
    padding: 12px 46px 12px 8px;
    font-size: {FONT_VALUE};
    font-weight: 700;
}}
#cbbProjectPicker:focus, #cbbJobPicker:focus, #cbbSegmentFilter:focus {{ border-color: {ACCENT_YELLOW}; }}
#cbbProjectPicker::drop-down, #cbbJobPicker::drop-down, #cbbSegmentFilter::drop-down {{
    subcontrol-origin: padding; subcontrol-position: top right;
    width: 44px; border: none; background: transparent;
}}
#cbbProjectPicker::down-arrow, #cbbJobPicker::down-arrow, #cbbSegmentFilter::down-arrow {{
    image: url({_CHEVRON_ICON_PATH});
    width: 18px; height: 18px;
    margin-right: 16px;
}}
#cbbProjectPicker QAbstractItemView, #cbbJobPicker QAbstractItemView, #cbbSegmentFilter QAbstractItemView {{
    background-color: {CARD_BG};
    border: 2px solid {BORDER};
    border-radius: 10px;
    selection-background-color: {ACCENT_YELLOW};
    selection-color: #153E42;
    padding: 6px;
    font-size: {FONT_VALUE};
    outline: none;
}}
#cbbProjectPicker QAbstractItemView::item, #cbbJobPicker QAbstractItemView::item, #cbbSegmentFilter QAbstractItemView::item {{
    padding: 18px 16px; min-height: 30px; margin: 3px 0px; border-radius: 8px;
}}
#cbbSegmentFilter {{ font-size: {FONT_FIELD_LABEL}; padding: 10px 42px 10px 8px; }}

/* Styled to match the #fieldBox comboboxes above (same white/border
   look) rather than standing out as a bright accent button. */
#btnCurrentJob {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 2px solid {BORDER};
    border-radius: 10px;
    padding: 16px 26px;
    font-size: {FONT_BUTTON_PRIMARY};
    font-weight: 800;
}}
#btnCurrentJob:hover {{ border-color: {ACCENT_YELLOW}; color: {ACCENT_YELLOW_HOVER}; }}
QScrollArea {{ background: transparent; border: none; }}
/* Segment List's vertical scrollbar, ~1.5x a default scrollbar's width
   so it's easier to grab on a touchscreen. */
#filesScroll QScrollBar:vertical {{
    width: 28px; background: {LIGHT_BG}; margin: 0px; border-radius: 6px;
}}
#filesScroll QScrollBar::handle:vertical {{
    background: #cbd5e1; min-height: 40px; border-radius: 6px;
}}
#filesScroll QScrollBar::handle:vertical:hover {{ background: #94a3b8; }}
#filesScroll QScrollBar::add-line:vertical, #filesScroll QScrollBar::sub-line:vertical {{ height: 0px; }}
#filesScroll QScrollBar::add-page:vertical, #filesScroll QScrollBar::sub-page:vertical {{ background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

/* ---- Segment grouping: one card per scan_id (Pre-Scan + its Post-Scans) ---- */
#segmentCard {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
#segmentHeader {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_GROUP_TITLE}; font-weight: 800;
}}
#segmentSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SUBTEXT}; font-weight: 600;
}}

#fileRow {{
    background-color: {CARD_BG};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
#fileRowTitle {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_ROW_TITLE}; font-weight: 800;
}}
#fileRowSubtext {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_SUBTEXT}; font-weight: 600;
}}
#emptyStateLabel {{
    background: transparent; border: none;
    color: {TEXT_MUTED}; font-size: {FONT_EMPTY_STATE}; font-weight: 600;
    padding: 32px;
}}
#compareBar {{
    background-color: {ROW_BG};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
#compareBarLabel {{
    background: transparent; border: none;
    color: {TEXT}; font-size: {FONT_ROW_TITLE}; font-weight: 700;
}}

/* ---- Compare toggle: a checkable pill button, not a native checkbox ---- */
QPushButton[cssClass="compareToggle"] {{
    background-color: {CARD_BG};
    color: {TEXT_MUTED};
    border: 2px solid {BORDER};
    border-radius: 20px;
    padding: 16px 20px;
    font-size: {FONT_BUTTON};
    font-weight: 800;
}}
QPushButton[cssClass="compareToggle"]:hover {{ border-color: {ACCENT_YELLOW}; color: {TEXT}; }}
QPushButton[cssClass="compareToggle"]:checked {{
    background-color: {ACCENT_YELLOW}; color: #153E42; border-color: {ACCENT_YELLOW_HOVER};
}}

QPushButton[cssClass="rowActionBtn"] {{
    background-color: {CARD_BG};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 20px 20px;
    font-size: {FONT_BUTTON};
    font-weight: 700;
}}
QPushButton[cssClass="rowActionBtn"]:hover {{ border-color: {ACCENT_YELLOW}; }}
QPushButton[cssClass="rowActionBtn"]:disabled {{ color: #cbd5e1; border-color: {BORDER}; }}
QPushButton[cssClass="rowDangerBtn"] {{
    background-color: {CARD_BG};
    color: {ACCENT_RED};
    border: 1px solid {ACCENT_RED};
    border-radius: 8px;
    padding: 20px 20px;
    font-size: {FONT_BUTTON};
    font-weight: 700;
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
QPushButton[cssClass="rowPrimaryBtn"]:disabled {{ background-color: #e2e8f0; color: #94a3b8; }}
""")
        self._build_top_bar_widgets(frm_ReportPage)

    def _build_field_box(self, parent, label_text, combo_object_name=None, height=72, min_width=240):
        """One #fieldBox unit = a small caption + a QComboBox, drawn as a
        single bordered control (border/background on the box, the combo
        itself borderless) - the same "label baked into the field" look
        as the header's CURRENT JOB box in intelijet_ui.py."""
        box = QtWidgets.QWidget(parent)
        box.setObjectName("fieldBox")
        layout = QtWidgets.QHBoxLayout(box)
        layout.setContentsMargins(20, 4, 12, 4)
        layout.setSpacing(10)

        label = QtWidgets.QLabel(label_text, box)
        label.setObjectName("fieldBoxLabel")
        layout.addWidget(label)

        combo = QtWidgets.QComboBox(box)
        combo.setObjectName(combo_object_name or f"cbb{label_text}Picker")
        combo.setMinimumHeight(height)
        combo.setMinimumWidth(min_width)
        combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(combo, 1)

        return box, combo

    def _build_searchable_field(self, parent, label_text, combo_object_name=None, height=72, min_width=240):
        """A #fieldBox combo (selection-only) plus its own dedicated
        #pickerSearchBox QLineEdit stacked above it, wrapped in one
        container - the search box filters the combo's items live
        (wired in report_page_manager.py), the combo itself just picks
        from whatever's currently listed."""
        container = QtWidgets.QWidget(parent)
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        search = QtWidgets.QLineEdit(container)
        search.setObjectName(f"search{label_text}Box")
        search.setProperty("cssClass", "pickerSearchBox")
        search.setPlaceholderText(f"Search {label_text.lower()}...")
        search.setMinimumHeight(44)
        layout.addWidget(search)

        box, combo = self._build_field_box(container, label_text, combo_object_name, height, min_width)
        layout.addWidget(box)

        return container, search, combo

    def _build_top_bar_widgets(self, frm_ReportPage):
        self.rootLayout = QtWidgets.QVBoxLayout(frm_ReportPage)
        self.rootLayout.setContentsMargins(40, 40, 40, 40)
        self.rootLayout.setSpacing(28)

        # ---- Top bar: title + job picker ----
        self.reportTopBar = QtWidgets.QWidget(frm_ReportPage)
        self.reportTopBar.setObjectName("reportTopBar")
        self.reportTopBarLayout = QtWidgets.QHBoxLayout(self.reportTopBar)
        self.reportTopBarLayout.setContentsMargins(40, 28, 40, 28)
        self.reportTopBarLayout.setSpacing(16)
        self.reportTitle = QtWidgets.QLabel("Reports", self.reportTopBar)
        self.reportTitle.setObjectName("reportTitle")
        self.reportTopBarLayout.addWidget(self.reportTitle)
        self.reportTopBarLayout.addSpacing(32)

        # Project/Job pickers are plain selection-only QComboBox (tap to
        # open, tap an item to choose) each paired with its own dedicated
        # search QLineEdit stacked above it (wired in
        # report_page_manager.py) rather than making the combobox itself
        # editable - keeps "type to filter" and "pick from the list" as
        # two clearly separate actions. Each field box is labeled in one
        # #fieldBox unit (border/bg on the box, not the combo) - the same
        # "label baked into the field" look as the header's CURRENT JOB
        # box. Both get layout stretch=1 (title/button stay at their
        # natural size) so they're the ones that expand to fill whatever
        # width is left in the bar.
        self.projectFieldContainer, self.searchProjectBox, self.cbbProjectPicker = self._build_searchable_field(self.reportTopBar, "Project")
        self.reportTopBarLayout.addWidget(self.projectFieldContainer, 1)

        self.jobFieldContainer, self.searchJobBox, self.cbbJobPicker = self._build_searchable_field(self.reportTopBar, "Job")
        self.reportTopBarLayout.addWidget(self.jobFieldContainer, 1)

        self.btnCurrentJob = QtWidgets.QPushButton("★ Current Job", self.reportTopBar)
        self.btnCurrentJob.setObjectName("btnCurrentJob")
        self.btnCurrentJob.setMinimumHeight(72)
        # Project/Job now stack a search box above their combo, making
        # them taller than this single-row button - bottom-align it so
        # it still lines up with the combos rather than the search boxes.
        self.reportTopBarLayout.addWidget(self.btnCurrentJob, 0, QtCore.Qt.AlignBottom)

        self.rootLayout.addWidget(self.reportTopBar)

        # ---- File list card ----
        self.filesCard = QtWidgets.QFrame(frm_ReportPage)
        self.filesCard.setObjectName("reportCard")
        self.filesCardLayout = QtWidgets.QVBoxLayout(self.filesCard)
        self.filesCardLayout.setContentsMargins(36, 36, 36, 36)
        self.filesCardLayout.setSpacing(20)

        self.filesCardHeader = QtWidgets.QWidget(self.filesCard)
        self.filesCardHeader.setObjectName("reportCardTitleRow")
        self.filesCardHeaderLayout = QtWidgets.QHBoxLayout(self.filesCardHeader)
        self.filesCardHeaderLayout.setContentsMargins(0, 0, 0, 16)
        self.filesCardHeaderLayout.setSpacing(16)

        self.filesCardTitle = QtWidgets.QLabel("Segment List", self.filesCardHeader)
        self.filesCardTitle.setObjectName("reportCardTitle")
        self.filesCardHeaderLayout.addWidget(self.filesCardTitle)
        self.filesCardHeaderLayout.addStretch(1)

        self.segmentFieldBox, self.cbbSegmentFilter = self._build_field_box(
            self.filesCardHeader, "Segment", combo_object_name="cbbSegmentFilter", height=68, min_width=280,
        )
        self.filesCardHeaderLayout.addWidget(self.segmentFieldBox)

        self.filesCardLayout.addWidget(self.filesCardHeader)

        self.filesScroll = QtWidgets.QScrollArea(self.filesCard)
        self.filesScroll.setObjectName("filesScroll")
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
