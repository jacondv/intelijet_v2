"""Which network the tablet is on right now: Wi-Fi (SSID + signal) and/or LAN.

Read for the header indicator. The container runs with network_mode: host, so
/sys/class/net and nl80211 (via `iw`) show the host's real interfaces - no
extra Docker mounts needed. NetworkManager (nmcli) is NOT usable from inside
the container (no D-Bus socket), and iwgetid/psutil aren't in the image.

Interfaces are found by kind, not by name (wlp1s0 / wlan0 / enx... / eth0 all
differ between machines): a Wi-Fi interface has a wireless/ dir in sysfs, a
LAN one is a real (non-virtual) ethernet device. Docker/veth/bridge devices
live under /sys/devices/virtual and are ignored.
"""
import os
import re
import subprocess

SYS_NET = "/sys/class/net"
IW_TIMEOUT_SEC = 2

_SIGNAL_RE = re.compile(r"signal:\s*(-?\d+)\s*dBm")
_SSID_RE = re.compile(r"^\s*SSID:\s*(.*)$", re.MULTILINE)
_ESCAPE_RE = re.compile(rb"\\x([0-9a-fA-F]{2})")


def _is_virtual(iface):
    return "/virtual/" in os.path.realpath(os.path.join(SYS_NET, iface))


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _decode_ssid(raw):
    """`iw` prints non-printable bytes as \\xNN (a trailing space comes out
    as '\\x20') - turn those back into characters, then trim."""
    data = _ESCAPE_RE.sub(lambda m: bytes([int(m.group(1), 16)]), raw.encode("utf-8", "replace"))
    return data.decode("utf-8", "replace").strip()


def _wifi_link(iface):
    """(ssid, signal_dbm) if `iface` is associated to an access point, else
    None. A failing/missing `iw` also gives None - the indicator then shows
    "no Wi-Fi", which is the safe direction to be wrong in."""
    try:
        out = subprocess.run(
            ["iw", "dev", iface, "link"],
            capture_output=True, text=True, timeout=IW_TIMEOUT_SEC,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    if not out.startswith("Connected to"):
        return None
    ssid_match = _SSID_RE.search(out)
    signal_match = _SIGNAL_RE.search(out)
    return (
        _decode_ssid(ssid_match.group(1)) if ssid_match else "",
        int(signal_match.group(1)) if signal_match else None,
    )


def signal_level(signal_dbm, good_dbm=-60, fair_dbm=-70):
    """Wi-Fi signal strength as 3 (strong) / 2 (fair) / 1 (weak) bars."""
    if signal_dbm is None:
        return 1
    if signal_dbm >= good_dbm:
        return 3
    if signal_dbm >= fair_dbm:
        return 2
    return 1


def get_network_status(good_dbm=-60, fair_dbm=-70):
    """Snapshot of the current connection:
        {"wifi": {"ssid": str, "signal_dbm": int|None, "level": 1..3} | None,
         "lan": bool,          # a LAN cable is plugged in (link carrier up)
         "connected": bool}    # wifi or lan
    """
    wifi = None
    lan = False
    try:
        ifaces = sorted(os.listdir(SYS_NET))
    except OSError:
        ifaces = []
    for iface in ifaces:
        if _is_virtual(iface):
            continue
        if os.path.isdir(os.path.join(SYS_NET, iface, "wireless")):
            if wifi is None:
                link = _wifi_link(iface)
                if link:
                    ssid, signal = link
                    wifi = {"ssid": ssid, "signal_dbm": signal,
                            "level": signal_level(signal, good_dbm, fair_dbm)}
        elif _read(os.path.join(SYS_NET, iface, "type")) == "1":  # ARPHRD_ETHER
            # wwan (cellular modem) shows up as ethernet too - not a LAN cable.
            if not iface.startswith("ww") and _read(os.path.join(SYS_NET, iface, "carrier")) == "1":
                lan = True
    return {"wifi": wifi, "lan": lan, "connected": bool(wifi) or lan}
