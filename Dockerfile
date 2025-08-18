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
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
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


RUN python3 -m pip install --upgrade pip setuptools wheel \
    && python3 -m pip install "numpy==1.23.5" \
    && python3 -m pip install open3d==0.13.0 opencv-contrib-python rosnumpy \
    && apt-get update \
    && apt-get install -y ros-noetic-can-msgs \
    && rm -rf /var/lib/apt/lists/*

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
