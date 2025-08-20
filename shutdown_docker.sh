#!/bin/bash
# Kill ROS (nếu còn chạy)
rosnode kill -a || true
rosclean purge -y || true

# Stop và remove docker container
CONTAINER_NAME=intelijet
docker stop $CONTAINER_NAME

