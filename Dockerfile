# ===========================
# 0. Base image (ROS1 Noetic)
# ===========================
FROM osrf/ros:noetic-desktop-full-focal


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
    libpcl-dev \
    libvtk7-dev \
    python3-pyqt5 \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    libqt5svg5 \
    libqt5multimedia5 \
    && rm -rf /var/lib/apt/lists/*

# ===========================
# 2. Cài Python packages
# ===========================
# RUN python3 -m pip install --upgrade pip \
#     && python3 -m pip install --no-cache-dir \
#        open3d \
#        opencv-contrib-python \
#        rosnumpy \
#        numpy

# RUN apt-get update && apt-get install -y python3-pip


RUN python3 -m pip install open3d
RUN python3 -m pip install opencv-contrib-python
RUN python3 -m pip install rosnumpy
RUN python3 -m pip install --upgrade pip setuptools wheel
RUN python3 -m pip install "numpy>=1.21,<1.27"

# Cập nhật apt và cài can-msgs (Note should update before install)
RUN apt-get update && apt-get install -y ros-noetic-can-msgs


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
# docker run -it --rm -v /c/WORK/projects/intelijet_v2:/root/intelijet_v2 -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix jacon-pps-noetic

# On windows
# docker run -it -e XDG_RUNTIME_DIR=/tmp/runtime-root --rm -v /c/WORK/projects/intelijet_v2:/root/intelijet_v2 -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix --network host jacon-pps-noetic
