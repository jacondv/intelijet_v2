"""Header network indicator: Wi-Fi (SSID + signal bars) and/or LAN, or a red
"NO NETWORK" pill when neither is connected.

The tablet reaches the Lidar and the PCAN gateway through the router, so
"no network" explains most device-disconnected alarms - hence the red pill
(attention-grabbing on purpose, no extra notification). The reading itself is
shared.network_status; it shells out to `iw`, so it runs on the thread pool
and the GUI only ever receives the finished result.
"""
from PyQt5 import QtCore, QtGui, QtWidgets

from shared.config_loader import CONFIG as cfg
from shared.network_status import get_network_status

# Fallbacks for a last_used.yaml that predates runtime.yaml's network_status.
DEFAULT_REFRESH_SEC = 4
DEFAULT_SIGNAL_GOOD_DBM = -60
DEFAULT_SIGNAL_FAIR_DBM = -70

ICON_SIZE = 36
DIM_ALPHA = 70


def _cfg_value(name, default):
    value = getattr(getattr(cfg, "network_status", None), name, None)
    return default if value is None else value


def _wifi_pixmap(level, color):
    """Wi-Fi symbol: a dot plus 3 arcs, the first `level` arcs lit and the
    rest dimmed. level 0 = all dim (used under the "no network" cross)."""
    pm = QtGui.QPixmap(ICON_SIZE, ICON_SIZE)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    cx, cy = ICON_SIZE / 2, ICON_SIZE - 6
    for i in range(3):
        r = 7 + i * 6
        c = QtGui.QColor(color)
        if i >= level:
            c.setAlpha(DIM_ALPHA)
        p.setPen(QtGui.QPen(c, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        rect = QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r)
        p.drawArc(rect, 45 * 16, 90 * 16)
    p.setPen(QtCore.Qt.NoPen)
    p.setBrush(QtGui.QColor(color))
    p.drawEllipse(QtCore.QPointF(cx, cy), 2.5, 2.5)
    p.end()
    return pm


def _lan_pixmap(color):
    """LAN symbol: an RJ45-style socket (rounded box with a notch and pins)."""
    pm = QtGui.QPixmap(ICON_SIZE, ICON_SIZE)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    c = QtGui.QColor(color)
    p.setPen(QtGui.QPen(c, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
    p.drawRoundedRect(QtCore.QRectF(5, 8, 26, 20), 3, 3)
    p.drawLine(13, 28, 13, 31)
    p.drawLine(23, 28, 23, 31)
    p.setPen(QtGui.QPen(c, 2))
    for x in (12, 16, 20, 24):
        p.drawLine(x, 14, x, 20)
    p.end()
    return pm


def _no_network_pixmap(color):
    """Dimmed Wi-Fi symbol with a cross over it."""
    pm = _wifi_pixmap(0, color)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    p.setPen(QtGui.QPen(QtGui.QColor(color), 4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    p.drawLine(6, 6, ICON_SIZE - 6, ICON_SIZE - 6)
    p.drawLine(ICON_SIZE - 6, 6, 6, ICON_SIZE - 6)
    p.end()
    return pm


class _StatusSignals(QtCore.QObject):
    done = QtCore.pyqtSignal(dict)


class _StatusPoll(QtCore.QRunnable):
    def __init__(self, signals, good_dbm, fair_dbm):
        super().__init__()
        self._signals = signals
        self._good = good_dbm
        self._fair = fair_dbm

    def run(self):
        self._signals.done.emit(get_network_status(self._good, self._fair))


class NetworkIndicator(QtWidgets.QWidget):
    def __init__(self, parent=None, text_color="#e0f2f1", alert_color="#c62828"):
        """text_color / alert_color: the header's own chrome colors (defined in
        intelijet_ui.py, which imports this module - so passed in, not imported)."""
        super().__init__(parent)
        self._text_color = text_color
        self._alert_color = alert_color
        self.setObjectName("networkIndicator")
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        # The app's global `QLabel {}` rule (border + padding 4px 8px) would
        # shrink the 36px icon labels' drawing area and clip the icons - the
        # stylesheets set in _apply() reset border/padding/margin on them.

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(8)
        self._wifi_icon = QtWidgets.QLabel(self)
        self._lan_icon = QtWidgets.QLabel(self)
        self._text = QtWidgets.QLabel(self)
        for w in (self._wifi_icon, self._lan_icon):
            w.setFixedSize(ICON_SIZE, ICON_SIZE)
        layout.addWidget(self._lan_icon)   # outermost, so Wi-Fi + name stay together
        layout.addWidget(self._wifi_icon)
        layout.addSpacing(10)              # icon -> name gap (on top of the 8px spacing)
        layout.addWidget(self._text)

        self._good_dbm = _cfg_value("signal_good_dbm", DEFAULT_SIGNAL_GOOD_DBM)
        self._fair_dbm = _cfg_value("signal_fair_dbm", DEFAULT_SIGNAL_FAIR_DBM)
        self._signals = _StatusSignals()
        self._signals.done.connect(self._apply)
        self._polling = False

        # Blank until the first reading lands (a few ms) rather than flashing
        # a false "NO NETWORK" at startup.
        self._wifi_icon.hide()
        self._lan_icon.hide()
        self._text.setText("")

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(int(_cfg_value("refresh_sec", DEFAULT_REFRESH_SEC) * 1000))
        self._poll()

    def _poll(self):
        if self._polling:  # previous reading still running (slow `iw`)
            return
        self._polling = True
        QtCore.QThreadPool.globalInstance().start(
            _StatusPoll(self._signals, self._good_dbm, self._fair_dbm)
        )

    def _apply(self, status):
        self._polling = False
        wifi, lan = status["wifi"], status["lan"]

        if not status["connected"]:
            self.setStyleSheet(
                f"#networkIndicator {{ background-color: {self._alert_color}; border: none; border-radius: 10px; }}"
                "#networkIndicator QLabel { background: transparent; border: none; padding: 0; margin: 0; color: white;"
                " font-size: 20px; font-weight: 800; }"
            )
            self._wifi_icon.setPixmap(_no_network_pixmap("white"))
            self._wifi_icon.show()
            self._lan_icon.hide()
            self._text.setText("NO NETWORK")
            self.setToolTip("No Wi-Fi and no LAN cable - the Lidar and PLC cannot be reached")
            return

        self.setStyleSheet(
            "#networkIndicator { background: transparent; border: none; }"
            f"#networkIndicator QLabel {{ background: transparent; border: none; padding: 0; margin: 0; color: {self._text_color};"
            " font-size: 20px; font-weight: 700; }"
        )
        if wifi:
            self._wifi_icon.setPixmap(_wifi_pixmap(wifi["level"], self._text_color))
            self._wifi_icon.show()
        else:
            self._wifi_icon.hide()
        if lan:
            self._lan_icon.setPixmap(_lan_pixmap(self._text_color))
            self._lan_icon.show()
        else:
            self._lan_icon.hide()

        # Both up: the SSID is the more useful label, the LAN icon beside it
        # already says a cable is in.
        self._text.setText(wifi["ssid"] if wifi and wifi["ssid"] else ("Wi-Fi" if wifi else "LAN"))
        parts = []
        if wifi:
            sig = wifi["signal_dbm"]
            parts.append(f"Wi-Fi: {wifi['ssid']}" + (f" ({sig} dBm)" if sig is not None else ""))
        if lan:
            parts.append("LAN cable connected")
        self.setToolTip("\n".join(parts))
