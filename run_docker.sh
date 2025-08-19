#!/bin/bash

# Cho phép container kết nối X server
xhost +local:docker

sudo docker run -it --rm \
    -v /home/nuc/intelijet_v2:/root/intelijet_v2 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    --network host \
    jacondv/jacon-pps-noetic \
    bash /root/intelijet_v2/run_intelijet.sh
# xhost -local:root   
# đóng lại cho an toàn sau khi thoát container