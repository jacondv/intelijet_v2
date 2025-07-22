from enum import Enum


class ConnectionType(Enum):
    WIRED = 0
    WIRELESS = 1


BLK_ARC_IP = {ConnectionType.WIRED: "192.168.42.1", ConnectionType.WIRELESS: "10.1.1.1"}

# Networking & gRPC
BLK_ARC_GRPC_PORT = 8081
GRPC_SECURE_PORT = 8091

TLS_CERT_ENDPOINT = "/ssl_cert.pem"

PING_TIMEOUT = 5.0
GRPC_TIMEOUT = 3.0
GRPC_TIMEOUT_START_CAPTURE = 15.0

# LIDAR mask dimension
LIDAR_MASK_WIDTH = 2048
LIDAR_MASK_HEIGHT = 1024

# SLAM camera mask dimension
SLAM_CAM_MASK_WIDTH = 1456
SLAM_CAM_MASK_HEIGHT = 1088
