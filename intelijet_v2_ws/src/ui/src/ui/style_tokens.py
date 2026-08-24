# -*- coding: utf-8 -*-
"""Central style tokens for the light-card pages (JOB, SYSTEM, REPORT
tabs - project_dlg_ui.py / intelijet_ui.py's SYSTEM section /
report_page_ui.py). Written because font sizes had drifted out of sync
both between these pages and within the same page (e.g. two visually
equivalent row titles at 24px on one page and 26px on another) - every
page should pull from here instead of hardcoding a literal, so a future
size change is one edit in this file, not a hunt across three.

Deliberately NOT used for the 3D MAIN control panel's scan/manual-operator
buttons, or other one-off special-purpose controls (e.g. SYSTEM tab's
ON/OFF toggle pills) - those were sized for a specific touch-target/role
that isn't part of this shared text hierarchy.

Tiers, biggest to smallest:
    FONT_PAGE_TITLE    - the page's own title ("Reports", "Projects & Jobs")
    FONT_SECTION_TITLE - a card's header ("Segment List", "Device Hardware Status")
    FONT_GROUP_TITLE   - a sub-heading grouping several rows inside a card
                          (a project's name over its jobs, "Segment - Scan N")
    FONT_ROW_TITLE     - one row/item's primary text (a job's name, a file's name)
    FONT_VALUE         - editable/readonly field values (search boxes, comboboxes)
    FONT_FIELD_LABEL   - a small caption next to a value ("Project:", "Encoder Value (deg)")
    FONT_SUBTEXT       - secondary/muted text under a row title (timestamps, statuses)
    FONT_EMPTY_STATE   - "nothing here yet" placeholder messages
    FONT_BUTTON        - regular action buttons (row buttons, dialog buttons)
    FONT_BUTTON_PRIMARY- the page's one or two emphasized actions (+New Project,
                          Current Job, Compare Selected)
    FONT_BUTTON_SMALL  - small inline utility buttons (a field's own Clear button)
"""

FONT_PAGE_TITLE = "40px"
FONT_SECTION_TITLE = "36px"
FONT_GROUP_TITLE = "28px"
FONT_ROW_TITLE = "26px"
FONT_VALUE = "28px"
FONT_FIELD_LABEL = "22px"
FONT_SUBTEXT = "20px"
FONT_EMPTY_STATE = "24px"
FONT_BUTTON = "20px"
FONT_BUTTON_PRIMARY = "26px"
FONT_BUTTON_SMALL = "15px"

import os

# Shared combobox dropdown-arrow icon (see report_page_ui.py's original
# comment on why a real image is needed - once a QComboBox has any
# stylesheet rule, Qt's styled paint path won't fall back to the native
# style's arrow glyph for ::down-arrow). Generated via tools/gen_chevron.py.
CHEVRON_ICON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons", "chevron_down.png").replace("\\", "/")

# ---- Shared light-card palette (same across JOB/SYSTEM/REPORT) ----
LIGHT_BG = "#f4f5f8"
CARD_BG = "#ffffff"
BORDER = "#e2e8f0"
TEXT = "#1e293b"
TEXT_MUTED = "#64748b"
ACCENT_YELLOW = "#fbc02d"
ACCENT_YELLOW_HOVER = "#f57f17"
ACCENT_RED = "#c62828"
ROW_BG = "#f8fafc"
