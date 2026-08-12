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
    evince \
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
# evince: PDF viewer, for opening the tunnel report PDFs the app exports
# (ui/src/ui/tunnel_report/) directly on the kiosk screen.
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


RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install --ignore-installed "setuptools==65.5.1" "wheel==0.38.4" \
    && python3 -m pip install --ignore-installed "numpy==1.23.5" \
    && python3 -m pip install --ignore-installed open3d==0.13.0 opencv-contrib-python rosnumpy \
    && python3 -m pip install --ignore-installed Pillow jinja2 weasyprint matplotlib scipy python-box \
    && python3 -m pip install --ignore-installed torch --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --ignore-installed kornia kornia-rs kornia_moons \
    && apt-get update \
    && apt-get install -y ros-noetic-can-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /root/.cache/pip
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
# matplotlib: pps/, ui/, encoder_process/ (chart plots for the report).
# scipy: pps/, encoder_process/.
# python-box: pps/src/pps/utils.py (`from box import Box`).
# torch/kornia/kornia-rs/kornia_moons: ai_core_pkg (LoFTR image matcher) -
# kept in sync with intelijet_v2_ws/src/ai_core_pkg/requirements.txt, minus
# opencv-python/pyyaml/rospkg there (already covered above / via ROS).

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
