# ===========================
# 0. Base image (ROS1 Noetic)
# ===========================
# ros-base (not desktop-full): everything actually needed (rviz, tf, pcl,
# laser_assembler, robot_state_publisher, Qt5, VTK...) is installed
# explicitly below instead of pulling desktop-full's full bundle.
FROM ros:noetic-ros-base-focal

# Prevents apt from hanging on an interactive debconf prompt (keyboard
# layout, pulled in by `onboard` below) with no TTY attached during build.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-pip \
    python3-tk \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-rosparam-shortcuts \
    ros-noetic-can-msgs \
    ros-noetic-laser-assembler \
    ros-noetic-robot-state-publisher \
    ros-noetic-cv-bridge \
    ros-noetic-diagnostic-updater \
    ros-noetic-dynamic-reconfigure \
    iputils-ping \
    libpcl-dev \
    libvtk7-dev \
    python3-vtk7 \
    libjsoncpp-dev \
    libboost-system-dev \
    libboost-serialization-dev \
    python3-pyqt5 \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    libqt5svg5 \
    libqt5multimedia5 \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libpango-1.0-0 \
    libpangocairo-1.0-0 \
    libgdk-pixbuf2.0-0 \
    libffi-dev \
    shared-mime-info \
    fonts-liberation \
    fonts-dejavu-core \
    onboard \
    dbus-x11 \
    qpdfview \
    wmctrl \
    && rm -rf /var/lib/apt/lists/*
# python3-vtk7: libvtk7-dev alone is C++ headers only, no Python bindings.
# libjsoncpp-dev/libboost-*-dev/diagnostic-updater/dynamic-reconfigure:
# build deps of the sick_scan package.
# fonts-dejavu-core: fonts-liberation only covers Latin text glyphs - it
# has no glyph for the Unicode Arrows block (U+2190-U+21FF) onboard's
# Shift (U+21E7)/Caps Lock (U+21EA) key icons use, so without a fallback
# font Pango renders the raw codepoint as a "21 E7"/"21 EA" tofu box
# instead of an arrow glyph. DejaVu Sans covers that block.
# libpango/libpangocairo/libgdk-pixbuf/libffi-dev/shared-mime-info/
# fonts-liberation: native rendering deps of WeasyPrint (PDF report export).
# onboard: on-screen keyboard, controlled via its D-Bus Show/Hide service
# (see ui/src/ui/keyboard.py) rather than killed/relaunched, to avoid
# flicker/dropped keystrokes.
# dbus-x11: provides dbus-run-session, needed by run_intelijet.sh to start
# a private D-Bus bus for onboard's Show/Hide calls to connect to.
# qpdfview: PDF viewer for report_view_dlg_manager.py - persists its own
# window geometry via QSettings and is much lighter than Okular/evince.
# wmctrl: qpdfview --unique reuses its already-running instance's window
# instead of opening a new one, so report_page_manager.py's "View Report"
# button uses wmctrl to raise/focus that existing window (Popen'ing
# qpdfview alone doesn't guarantee that window comes to front).
#
# If a package still fails to build with "missing dependency", run (inside
# the container, from intelijet_v2_ws): rosdep install --from-paths src
# --ignore-src -r -y

# ===========================
# 2. Cài Python packages
# ===========================
# No opencv-contrib-python/opencv-python here: ros-noetic-cv-bridge's
# cv_bridge_boost.so is compiled against apt's python3-opencv - a second,
# different OpenCV build from pip makes `import cv2` resolve to it instead,
# and cv_bridge's type-code lookup then fails with a bare `KeyError: 16`.
# kornia_moons transitively pulls in opencv-python, so it's installed with
# --no-deps (its other real deps - kornia, matplotlib, torch - are already
# installed below/above).
# (open3d needs its full normal deps here - a previous --no-deps attempt to
# trim its unused pandas/scikit-learn/dash broke
# `import open3d._ml3d.datasets` at runtime with a missing sklearn.)
RUN python3 -m pip config set global.retries 10 \
    && python3 -m pip config set global.timeout 120 \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install --ignore-installed "setuptools==65.5.1" "wheel==0.38.4" \
    && python3 -m pip install --ignore-installed "numpy==1.23.5"

# open3d/torch are split into their own layers: both are large downloads
# over a flaky connection, and a mid-download failure would otherwise
# discard everything installed earlier in the same RUN, forcing a full
# re-download on retry instead of resuming from the last successful layer.
RUN python3 -m pip install --ignore-installed open3d==0.19.0 rosnumpy

RUN python3 -m pip install --ignore-installed torch --index-url https://download.pytorch.org/whl/cpu

RUN python3 -m pip install --ignore-installed Pillow jinja2 weasyprint "pydyf==0.9.0" matplotlib scipy python-box \
    && python3 -m pip install --ignore-installed kornia kornia-rs \
    && python3 -m pip install --ignore-installed --no-deps kornia_moons \
    && python3 -m pip install --ignore-installed einops loguru yacs \
    && python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless \
    && apt-get update \
    && apt-get install -y ros-noetic-can-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip
# Explicit opencv uninstall as a safety net: `--no-deps` on kornia_moons
# should already keep it out, but a stale Docker layer cache once let it
# back in silently - this makes the end state correct regardless of cause.
# open3d==0.19.0 (was 0.13.0): pps/ code uses the newer
# o3d.t.geometry.PointCloud API (to_legacy()/from_legacy(), "positions"
# key), which 0.13.0 doesn't have.
# torch from the CPU-only wheel index: default PyPI torch bundles several
# GB of CUDA runtime this container (no GPU passthrough) doesn't need.
# --ignore-installed everywhere: some apt/distutils packages (PyYAML,
# pytz, zipp...) can't be cleanly upgraded by pip otherwise
# ("uninstall-distutils-installed-package" error).
# setuptools pinned to 65.5.1: newer versions break egg_info builds on
# Python 3.8's older importlib_metadata backport.
# pydyf==0.9.0: weasyprint (<=61.2) calls the old 2-arg pydyf.PDF()
# signature, which 0.11.0 removed - every PDF export raised a TypeError
# without this pin.
# python-box: pps/src/pps/utils.py (`from box import Box`).
# torch/kornia/kornia-rs/kornia_moons: ai_core_pkg (LoFTR image matcher).
# Pinned to no specific version deliberately - kornia.feature.XFeat
# needs kornia>=0.8.3, which needs Python>=3.11; this image's Python is
# 3.8 (ros:noetic-ros-base-focal), so the newest installable kornia here
# is 0.7.3 and XFeat is NOT available - use
# ai_core_pkg/matchers/disk_lightglue_matcher.py (DISK+LightGlue, both
# already in kornia 0.7.3) instead. See xfeat_matcher.py's docstring.
# einops/loguru/yacs: ai_core_pkg/matchers/efficientloftr/ (vendored
# EfficientLoFTR inference code, https://github.com/zju3dv/EfficientLoFTR -
# only these three; its own requirements.txt is for training and pulls in
# much heavier deps the vendored inference-only subset doesn't need).

RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# intelijet_v2_ws is bind-mounted from the host, not baked into the image -
# run_intelijet.sh sources its ROS setup.bash files at container startup.

CMD ["bash"]
