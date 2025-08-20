#!/bin/bash

sudo usermod -aG docker nuc

# Cho phép container kết nối X server
xhost +local:docker
trap "xhost -local:docker" EXIT

CONTAINER_NAME=intelijet
IMAGE_NAME=jacondv/jacon-pps-noetic


if [ "$(sudo docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME is running, restarting..."
    sudo docker restart -t 0 $CONTAINER_NAME
    sleep 1
    sudo docker attach $CONTAINER_NAME || echo "Container exited too quickly"

elif [ "$(sudo docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME exists but stopped, starting..."
    sudo docker start -ai $CONTAINER_NAME

else

    sudo docker run -it \
        --name $CONTAINER_NAME \
        -v /home/nuc/intelijet_v2:/root/intelijet_v2 \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/dri:/dev/dri \
        --network host \
        $IMAGE_NAME \
        bash /root/intelijet_v2/run_intelijet.sh
    # xhost -local:root   
    # đóng lại cho an toàn sau khi thoát container

fi
