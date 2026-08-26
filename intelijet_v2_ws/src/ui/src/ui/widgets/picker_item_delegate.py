# ui/widgets/picker_item_delegate.py
"""Custom-painted popup rows for a QComboBox's dropdown, shared by every
picker combobox in the app (Project/Job pickers on the REPORT tab, the
header's CURRENT JOB combobox, Status combo in the JOB tab's inline
edit panel, etc).

QSS ::item padding/margin/min-height was observed to have no effect on
row height in this Qt build (confirmed by testing - rows stayed exactly
the same size, or overlapped, no matter what padding/min-height was set
via stylesheet), so real control over row height/selection appearance
has to go through a delegate instead of a stylesheet.

"Currently selected" (checkmark + accent fill) is determined by
comparing index.row() to the combobox's own currentIndex(), not by Qt's
State_Selected flag - that flag only turns on for the item under
keyboard/mouse highlight while the popup is open, not for "this is the
field's actual current value", which is what a checkmark should mean.
State_MouseOver still gets its own (lighter, no checkmark) hover fill so
pointer/touch feedback isn't lost.
"""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QStyle, QStyledItemDelegate


class PickerItemDelegate(QStyledItemDelegate):
    ROW_HEIGHT = 68
    BG_CURRENT = QColor("#fef3c7")
    BG_HOVER = QColor("#f1f5f9")
    TEXT_CURRENT = QColor("#c2410c")
    TEXT_NORMAL = QColor("#1e293b")

    def __init__(self, combo):
        super().__init__(combo)
        self.combo = combo

    def sizeHint(self, option, index):
        size = super().sizeHint(option, index)
        size.setHeight(self.ROW_HEIGHT)
        return size

    def paint(self, painter, option, index):
        painter.save()
        is_current = index.row() == self.combo.currentIndex()
        hovered = bool(option.state & QStyle.State_MouseOver)
        rect = option.rect

        if is_current:
            painter.setBrush(self.BG_CURRENT)
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(rect.adjusted(6, 4, -6, -4), 8, 8)
        elif hovered:
            painter.setBrush(self.BG_HOVER)
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(rect.adjusted(6, 4, -6, -4), 8, 8)

        font = option.font
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(self.TEXT_CURRENT if is_current else self.TEXT_NORMAL)
        text_rect = rect.adjusted(24, 0, -44, 0)
        painter.drawText(text_rect, Qt.AlignVCenter | Qt.AlignLeft, str(index.data()))
        if is_current:
            check_rect = rect.adjusted(0, 0, -18, 0)
            painter.drawText(check_rect, Qt.AlignVCenter | Qt.AlignRight, "✓")
        painter.restore()
