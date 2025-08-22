#!/bin/bash

sudo usermod -aG docker nuc

# Cho phép container kết nối X server
xhost +local:docker
trap "xhost -local:docker; echo 'Stopping container...'; sudo docker stop $CONTAINER_NAME; exit" INT


CONTAINER_NAME=intelijet
IMAGE_NAME=jacondv/jacon-pps-noetic

run_container() {
    if [ "$(sudo docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "Container $CONTAINER_NAME is already running. Restarting..."
        sudo docker exec -it $CONTAINER_NAME bash -c "cd /root/intelijet_v2 && ./run_intelijet.sh"

    elif [ "$(sudo docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        echo "Container $CONTAINER_NAME exists but stopped. Starting..."
        sudo docker start -ai $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME does not exist. Running new container..."
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
    fi


}


run_container


