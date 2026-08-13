# ===========================
# 0. Base image (ROS1 Noetic)
# ===========================
# ros-base (not desktop-full): no GUI/demo tools we don't use, much smaller
# pull. Everything the app actually needs (rviz, tf, pcl, laser_assembler,
# robot_state_publisher, Qt5, VTK...) is installed explicitly below instead
# of relying on desktop-full's huge bundled package set.
FROM ros:noetic-ros-base-focal

# Without this, installing onboard below pulls in keyboard-configuration/
# console-setup as a dependency, which runs an interactive debconf prompt
# ("Country of origin for the keyboard") during apt-get install - with no
# TTY attached during `docker build`, this just hangs forever instead of
# failing. noninteractive makes debconf silently take the package's
# default answer instead of prompting.
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
    onboard \
    dbus-x11 \
    qpdfview \
    && rm -rf /var/lib/apt/lists/*
# ros-noetic-laser-assembler / robot-state-publisher: used by pps.launch
# (point_cloud2_assembler, robot_state_publisher nodes) - came for free with
# desktop-full before, must be explicit now.
# ros-noetic-cv-bridge: `from cv_bridge import CvBridge` in pps and
# ai_core_pkg (image_matcher_service.py, image_matcher_client.py).
# python3-vtk7: `import vtk` used throughout ui (vtk_viewer.py, utils.py,
# cloud_pipeline.py...) - libvtk7-dev alone is only C++ headers, no Python
# bindings.
# libjsoncpp-dev, libboost-system-dev, libboost-serialization-dev,
# ros-noetic-diagnostic-updater, ros-noetic-dynamic-reconfigure: build deps
# of the sick_scan package (see its package.xml/CMakeLists.txt find_package
# calls) - desktop-full bundled these too.
# libpango/libpangocairo/libgdk-pixbuf/libffi-dev/shared-mime-info/
# fonts-liberation: native rendering deps of WeasyPrint (PDF report export,
# ui/src/ui/tunnel_report/report_controler.py) - WeasyPrint itself is pure
# Python (installed via pip below) but needs these system libs to render.
# onboard: on-screen keyboard binary launched by ui/src/ui/keyboard.py's
# TouchKeyboard (installed as an app-wide QApplication event filter in
# app.py - fires on every QLineEdit/QTextEdit focus, e.g. typing a new
# project name). Launched once and kept running for the whole session;
# shown/hidden after that via onboard's own D-Bus service (Show/Hide) -
# NOT by killing/relaunching the process, which is what caused flicker/
# dropped keystrokes/an unmovable window under repeated show-hide cycles.
# dbus-x11: provides dbus-run-session, used by run_intelijet.sh to start a
# private D-Bus session bus for the container - required for that D-Bus
# Show/Hide call above (and for the gsettings/dconf calls that dock
# onboard to the bottom edge) to have anywhere to connect to. Without it,
# show_keyboard() silently no-ops (see the try/except around it) and
# onboard never appears at all.
# qpdfview: PDF viewer, for opening the tunnel report PDFs the app exports
# (ui/src/ui/tunnel_report/) directly on the kiosk screen - launched with
# --unique from report_view_dlg_manager.py's on_file_opened(). Chosen over
# evince: no CLI flag exists for "start maximized with a title bar" (only
# --fullscreen, which hides the title bar/close button entirely), and
# unlike evince/okular, qpdfview persists its own window geometry
# (maximized or not) across restarts on its own via QSettings - no need
# to force it with a window-manager tool (xdotool/wmctrl) after launch.
# Also much lighter than Okular (no KDE Frameworks dependency chain).
#
# If a package still fails to build with "missing dependency" after this,
# the general fix is running (inside the container, from intelijet_v2_ws):
#   rosdep install --from-paths src --ignore-src -r -y
# rather than re-adding desktop-full.

# ===========================
# 2. Cài Python packages
# ===========================
# RUN python3 -m pip install --upgrade pip \
#     && python3 -m pip install --no-cache-dir \
#        open3d \
#        opencv-contrib-python \
#        rosnumpy \
#        numpy


# No opencv-contrib-python/opencv-python here: ros-noetic-cv-bridge (apt,
# above) already pulls in python3-opencv, and cv_bridge_boost.so (its
# compiled C++ half) is built against that exact apt OpenCV. A second,
# different OpenCV build installed via pip on top makes plain `import
# cv2` in application code resolve to it instead. cv_bridge.CvBridge then
# computes one type code via its C++ extension (compiled against apt's
# OpenCV) and looks it up in a dict built from attributes of the OTHER
# cv2 module (pip's) - the two disagree, so
# `cv2_to_imgmsg(img, encoding="bgr8")` fails with a bare `KeyError: 16`
# (real message: no such key in cv_bridge's cvtype_to_name dict).
# No code in this repo uses contrib-only modules (xfeatures2d/SIFT/aruco/
# ml), so there's nothing lost by relying on apt's python3-opencv alone.
# kornia_moons hard-depends on opencv-python and pip silently pulls it in
# as a transitive dependency even though it's never named directly here
# - confirmed via `pip3 show kornia_moons` -> "Requires: kornia,
# matplotlib, opencv-python, torch" on the real deployment, and
# reproducing/fixing the resulting KeyError live. --no-deps on just that
# one package skips it; kornia_moons's other actual deps (kornia,
# matplotlib, torch) are already installed by the lines below/above.
# (open3d itself is installed with its full normal deps here - a
# previous attempt at --no-deps on open3d too, to trim its unused
# pandas/scikit-learn/dash/etc, broke `import open3d._ml3d.datasets`
# at runtime with `ModuleNotFoundError: sklearn` - the eager-import
# chain there turned out deeper than a static source read caught, so
# that one was reverted and is not worth retrying without much more
# thorough testing.)
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install --ignore-installed "setuptools==65.5.1" "wheel==0.38.4" \
    && python3 -m pip install --ignore-installed "numpy==1.23.5" \
    && python3 -m pip install --ignore-installed open3d==0.19.0 rosnumpy \
    && python3 -m pip install --ignore-installed Pillow jinja2 weasyprint "pydyf==0.9.0" matplotlib scipy python-box \
    && python3 -m pip install --ignore-installed torch --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --ignore-installed kornia kornia-rs \
    && python3 -m pip install --ignore-installed --no-deps kornia_moons \
    && python3 -m pip install --ignore-installed einops loguru yacs \
    && python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless \
    && apt-get update \
    && apt-get install -y ros-noetic-can-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip
# Safety net after all the above: `--no-deps` on kornia_moons should
# already keep opencv-python out (see the KeyError: 16 comment above),
# but that fix silently stopped applying on a real rebuild once - turned
# out to be a stale Docker layer cache, not a wrong Dockerfile, but the
# failure mode (kornia-moons dependency resolution quietly reinstalling
# opencv-python) is easy to reintroduce by accident later (e.g. adding a
# new pip package here that itself depends on opencv-python). This
# explicit uninstall makes the end state correct regardless of how any
# of the above wanted to pull it back in - `|| true` isn't used here on
# purpose: if this ever errors, that's worth noticing, not swallowing.
# open3d==0.19.0 (was 0.13.0): the pps/ point-cloud code was written
# against the newer o3d.t.geometry.PointCloud API (to_legacy()/
# from_legacy(), the "positions" tensor attribute key) throughout - 0.13.0
# used different names for both (to_legacy_pointcloud()/
# from_legacy_pointcloud(), a "points" key) and, worse, that surface
# turned out inconsistent even across builds self-reporting the same
# "0.13.0" version string in the field. Rather than keep patching around
# every naming difference one at a time, pinned to the version the code
# actually matches. Confirmed before switching: a cp38 (Python 3.8, same
# as ROS Noetic here) wheel exists on PyPI, and its only numpy constraint
# is >=1.18.0 - no conflict with numpy==1.23.5 pinned below.
# torch installed from the CPU-only wheel index: the default PyPI torch
# bundles the full NVIDIA CUDA runtime (cublas/cudnn/cusolver/cufft/...),
# several GB, which is dead weight here since this container has no GPU
# passthrough configured (no --gpus / nvidia runtime in docker-compose.yml).
# --ignore-installed on every pip install: several system packages here
# (PyYAML, pytz, zipp...) are pre-installed via apt as distutils-based
# packages, which pip cannot cleanly uninstall/upgrade when a dependency
# (e.g. open3d/torch pulling in a newer pyyaml) needs a newer version ->
# "uninstall-distutils-installed-package" error. --ignore-installed makes
# pip just install its own copy on top (shadows the apt one on sys.path)
# instead of trying to uninstall first.
# setuptools pinned to 65.5.1 (not --upgrade to latest): newer setuptools
# needs importlib_metadata features not present in the older backport that
# ships with Python 3.8 (Noetic's interpreter) -> "AttributeError: module
# 'importlib_metadata' has no attribute 'EntryPoints'" during any later
# `pip install` that has to build a package from source (egg_info step).
# PyYAML (used by shared/, ui_can_interface/): not pip-installed - already
# present as python3-yaml, pulled in transitively by ROS python tooling
# (rospkg/catkin), and it's apt/distutils-installed so pip can't cleanly
# uninstall it to "upgrade" (uninstall-distutils-installed-package error).
# Pillow: ui/ (PIL, thumbnail/preview handling).
# jinja2 + weasyprint: ui/src/ui/tunnel_report/ (HTML template -> PDF report export).
# pydyf==0.9.0: weasyprint's own PDF-object dependency, pinned because
# latest (0.11.0) is a breaking change - pydyf.PDF.__init__ dropped its
# version/identifier positional args entirely, but weasyprint (up to at
# least 61.2, latest as of writing) still calls
# pydyf.PDF((version or '1.7'), identifier) internally
# (weasyprint/pdf/__init__.py generate_pdf()), so every PDF export failed
# with "PDF.__init__() takes 1 positional argument but 3 were given".
# weasyprint's own declared dependency (pydyf>=0.8.0, no upper bound)
# doesn't protect against this - confirmed by diffing pydyf's PDF.__init__
# source across 0.8.0/0.9.0 (2 args, fine) vs 0.10.0 (2 args + deprecation
# warning) vs 0.11.0 (no args - broken). 0.9.0 chosen over 0.8.0 to skip
# straight to the last clean (no-warning) release.
# matplotlib: pps/, ui/, encoder_process/ (chart plots for the report).
# scipy: pps/, encoder_process/.
# python-box: pps/src/pps/utils.py (`from box import Box`).
# torch/kornia/kornia-rs/kornia_moons: ai_core_pkg (LoFTR image matcher) -
# kept in sync with intelijet_v2_ws/src/ai_core_pkg/requirements.txt, minus
# opencv-python/pyyaml/rospkg there (already covered above / via ROS).
# einops/loguru/yacs: ai_core_pkg/matchers/efficientloftr/ (vendored
# https://github.com/zju3dv/EfficientLoFTR inference code, an
# experimental alternative to kornia's LoFTR - see
# ai_core_pkg/matchers/efficientloftr_matcher.py). Only these three -
# the upstream repo's own requirements.txt is for training and pulls in
# pytorch-lightning/ray/albumentations/h5py/opencv==4.4.0.46/
# kornia==0.4.1, none of which the vendored inference-only subset here
# actually imports (verified by reading each vendored file's imports).

# RUN python3 -m pip install open3d
# RUN python3 -m pip install opencv-contrib-python
# RUN python3 -m pip install rosnumpy
# RUN python3 -m pip install --upgrade pip setuptools wheel
# RUN python3 -m pip install "numpy==1.23.5"
# RUN apt-get update && apt-get install -y ros-noetic-can-msgs


RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# ---------------------------
# 6. Tạo workspace ROS (ko cần, ta sẽ mount thư mục project từ ben ngoài trực tiếp vào trong)
# ---------------------------
# ENV CATKIN_WS=/root/catkin_ws
# RUN mkdir -p $CATKIN_WS/src
# WORKDIR $CATKIN_WS

# ---------------------------
# 7. Thiết lập môi trường ROS (No need, we do it from outsite container)
# ---------------------------
# RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
#     && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# ---------------------------
# 8. Lệnh mặc định khi vào container
# ---------------------------
CMD ["bash"]




#Run docker on Linux
# xhost +local:docker

# docker run -it --rm -v /home/nuc/intelijet_v2:/root/intelijet_v2 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --network host jacondv/jacon-pps-noetic

# docker run -it --rm \
#     -v /home/nuc/intelijet_v2:/root/intelijet_v2 \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     --network host \
#     jacondv/jacon-pps-noetic


# On windows
# docker run -it -e XDG_RUNTIME_DIR=/tmp/runtime-root --rm -v /c/WORK/projects/intelijet_v2:/root/intelijet_v2 -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix --network host jacon-pps-noetic


# #!/bin/bash

# # Cho phép container kết nối X server
# xhost +local:docker

# # Chạy container
# docker run -it --rm \
#     -v /home/nuc/intelijet_v2:/root/intelijet_v2 \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --network host \
#     jacon-pps-noetic
