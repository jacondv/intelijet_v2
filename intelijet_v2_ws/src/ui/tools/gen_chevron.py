"""One-off generator for ui/src/ui/icons/chevron_down.png - the combobox
dropdown-arrow icon report_page_ui.py points its QSS ::down-arrow rules
at. Not run at app startup: once a QComboBox has any stylesheet applied,
Qt's QStyleSheetStyle paint path takes over ::down-arrow and needs a real
image (it will not fall back to drawing the native style's arrow glyph),
so this PNG has to exist as a checked-in asset.

Re-run only if the chevron needs to look different (color/size/stroke):
    QT_QPA_PLATFORM=offscreen python3 gen_chevron.py
"""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QColor, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QApplication


def draw_chevron(path, color, size=32, stroke=4):
    pix = QPixmap(size, size)
    pix.fill(Qt.transparent)
    p = QPainter(pix)
    p.setRenderHint(QPainter.Antialiasing)
    pen = QPen(QColor(color))
    pen.setWidth(stroke)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    p.setPen(pen)
    margin = size * 0.26
    top = size * 0.36
    mid = size * 0.66
    p.drawPolyline(QPointF(margin, top), QPointF(size / 2, mid), QPointF(size - margin, top))
    p.end()
    pix.save(path, "PNG")


if __name__ == "__main__":
    app = QApplication([])
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "ui", "icons")
    os.makedirs(out_dir, exist_ok=True)
    draw_chevron(os.path.join(out_dir, "chevron_down.png"), "#64748b", size=32, stroke=4)
    print("wrote", os.path.join(out_dir, "chevron_down.png"))
