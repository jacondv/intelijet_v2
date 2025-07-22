import logging
import subprocess
import sys
from pathlib import Path
from typing import Optional

import grpc
from blk_arc_grpc import about_pb2_grpc as about_grpc
from blk_arc_grpc import \
    capture_annotations_pb2_grpc as capture_annotations_grpc
from blk_arc_grpc import capture_pb2_grpc as capture_grpc
from blk_arc_grpc import clock_pb2_grpc as clock_grpc
from blk_arc_grpc import coordinate_frames_pb2_grpc as coordinate_frames_grpc
from blk_arc_grpc import device_pb2_grpc as device_grpc
from blk_arc_grpc import fault_pb2_grpc as fault_grpc
from blk_arc_grpc import imaging_pb2_grpc as imaging_grpc
from blk_arc_grpc import masking_pb2_grpc as masking_grpc
from blk_arc_grpc import power_pb2_grpc as power_grpc
from blk_arc_grpc import scan_library_pb2_grpc as scan_library_grpc
from blk_arc_grpc import storage_pb2_grpc as storage_grpc
from blk_arc_grpc import system_pb2_grpc as system_grpc
from blk_arc_grpc import wifi_client_pb2_grpc as wifi_client_grpc
from blk_arc_grpc import network_pb2_grpc as network_grpc
from cryptography.hazmat.backends import default_backend
from cryptography.x509 import load_pem_x509_certificate, oid
from requests import get
from requests.exceptions import ConnectTimeout

from blk_arc_sample_wrapper import blk_arc_config as config


class BLK_ARC_StubHandler:

    def __init__(self):
        self.reset_grpc_stubs()
        self._grpc_secure_channel: grpc.Channel = None

    def reset_grpc_stubs(self) -> None:
        self._about_stub: about_grpc.AboutStub = None
        self._capture_annotations_stub: capture_annotations_grpc.CaptureAnnotationsStub = None
        self._capture_stub: capture_grpc.CaptureStub = None
        self._clock_stub: clock_grpc.ClockStub = None
        self._coordinate_frames_stub: coordinate_frames_grpc.CoordinateFramesStub = None
        self._device_stub: device_grpc.DeviceStub = None
        self._fault_stub: fault_grpc.FaultStub = None
        self._imaging_stub: imaging_grpc.ImagingStub = None
        self._masking_stub: masking_grpc.MaskingStub = None
        self._network: network_grpc.NetworkStub = None
        self._power_stub: power_grpc.PowerStub = None
        self._scan_library_stub: scan_library_grpc.ScanLibraryStub = None
        self._storage_stub: storage_grpc.StorageStub = None
        self._system_stub: system_grpc.SystemStub = None
        self._wifi_client_stub: wifi_client_grpc.WifiClientStub = None
        self._grpc_secure_channel: grpc.Channel = None

    def create_grpc_stubs(self, connection_type: config.ConnectionType) -> bool:
        self._grpc_secure_channel = self._create_secure_grpc_channel(connection_type=connection_type)
        if not self._grpc_secure_channel:
            return False
        try:
            self._about_stub = about_grpc.AboutStub(self._grpc_secure_channel)
            self._capture_annotations_stub = capture_annotations_grpc.CaptureAnnotationsStub(self._grpc_secure_channel)
            self._capture_stub = capture_grpc.CaptureStub(self._grpc_secure_channel)
            self._clock_stub = clock_grpc.ClockStub(self._grpc_secure_channel)
            self._coordinate_frames_stub = coordinate_frames_grpc.CoordinateFramesStub(self._grpc_secure_channel)
            self._device_stub = device_grpc.DeviceStub(self._grpc_secure_channel)
            self._fault_stub = fault_grpc.FaultStub(self._grpc_secure_channel)
            self._imaging_stub = imaging_grpc.ImagingStub(self._grpc_secure_channel)
            self._masking_stub = masking_grpc.MaskingStub(self._grpc_secure_channel)
            self._network = network_grpc.NetworkStub(self._grpc_secure_channel)
            self._power_stub = power_grpc.PowerStub(self._grpc_secure_channel)
            self._scan_library_stub = scan_library_grpc.ScanLibraryStub(self._grpc_secure_channel)
            self._storage_stub = storage_grpc.StorageStub(self._grpc_secure_channel)
            self._system_stub = system_grpc.SystemStub(self._grpc_secure_channel)
            self._wifi_client_stub = wifi_client_grpc.WifiClientStub(self._grpc_secure_channel)
            logging.info("gRPC stubs for scanner communication successfully created.")
            return True
        except Exception:
            logging.exception("Creation of gRPC stubs for scanner communication failed: ")
            return False

    def close_grpc_channel(self) -> None:
        logging.info("Closing the gRPC-channel to the scanner.")
        if self._grpc_secure_channel:
            self._grpc_secure_channel.close()
        self.reset_grpc_stubs()

    def is_pingable(self, connection_type: config.ConnectionType) -> bool:
        ip = config.BLK_ARC_IP[connection_type]
        command = ["ping", "-c", "1", ip, "-W", "5"]
        try:
            if subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                               timeout=config.PING_TIMEOUT) == 0:
                return True
        except subprocess.TimeoutExpired:
            logging.error(
                f"No BLK ARC reachable on '{ip}'. Please ensure that the device is turned on and check the connection type (IP address)."
            )
        return False

    def _create_secure_grpc_channel(self, connection_type: config.ConnectionType) -> Optional[grpc.Channel]:
        try:
            certificate_contents = self._get_certificate_from_device(connection_type=connection_type)
            if certificate_contents is None:
                return None
            subject_common_name = self._get_subject_common_name(certificate_contents)
            credentials = self._get_credentials(certificate_contents)
            channel_options = [("grpc.max_send_message_length", -1), ("grpc.max_receive_message_length", -1),
                               ("grpc.ssl_target_name_override", subject_common_name)]
            secure_channel = grpc.secure_channel(f"{config.BLK_ARC_IP[connection_type]}:{config.GRPC_SECURE_PORT}",
                                                 credentials,
                                                 options=channel_options)
        except Exception:
            logging.exception("Could not create secure channel: ")
            return None
        try:
            grpc.channel_ready_future(secure_channel).result(timeout=5)
        except grpc.FutureTimeoutError:
            logging.error("Could not connect to the secure GRPC-Server.")
            return None
        return secure_channel

    def _get_certificate_from_device(self, connection_type: config.ConnectionType) -> Optional[bytes]:
        try:
            response = get(f"http://{config.BLK_ARC_IP[connection_type]}{config.TLS_CERT_ENDPOINT}", timeout=2.0)
        except ConnectTimeout:
            logging.error("Certificate could not be loaded on time.")
            return None

        if not response.ok:
            logging.error(f"Could not get certificate from device on IP {config.BLK_ARC_IP[connection_type]}")
            return None
        return response.content

    def _get_subject_common_name(self, certificate_bytes: bytes) -> str:
        certificate_object = load_pem_x509_certificate(certificate_bytes, default_backend())
        return certificate_object.subject.get_attributes_for_oid(oid.NameOID.COMMON_NAME)[0].value

    def _get_credentials(self, root_certificate_bytes: bytes) -> grpc.ChannelCredentials:
        current_path = Path(__file__)
        base_path = Path(getattr(sys, "_MEIPASS", current_path.parent))
        try:
            with open(base_path / "certificates/blkarc_integrators.cakey.pem", "rb") as private_key_file:
                private_key_bytes = private_key_file.read()
            with open(base_path / "certificates/blkarc_integrators.cacert.pem", "rb") as certificate_chain_file:
                certificate_chain_bytes = certificate_chain_file.read()
            credentials = grpc.ssl_channel_credentials(root_certificate_bytes, private_key_bytes,
                                                       certificate_chain_bytes)
            return credentials
        except FileNotFoundError:
            raise FileNotFoundError("Failed to find key and certificate!")

    def is_connected(self) -> bool:
        return self._grpc_secure_channel is not None

    @property
    def about_stub(self) -> about_grpc.AboutStub:
        return self._about_stub

    @property
    def capture_annotations_stub(self) -> capture_annotations_grpc.CaptureAnnotationsStub:
        return self._capture_annotations_stub

    @property
    def capture_stub(self) -> capture_grpc.CaptureStub:
        return self._capture_stub

    @property
    def clock_stub(self) -> clock_grpc.ClockStub:
        return self._clock_stub

    @property
    def coordinate_frames_stub(self) -> coordinate_frames_grpc.CoordinateFramesStub:
        return self._coordinate_frames_stub

    @property
    def device_stub(self) -> device_grpc.DeviceStub:
        return self._device_stub

    @property
    def fault_stub(self) -> fault_grpc.FaultStub:
        return self._fault_stub

    @property
    def imaging_stub(self) -> imaging_grpc.ImagingStub:
        return self._imaging_stub

    @property
    def masking_stub(self) -> masking_grpc.MaskingStub:
        return self._masking_stub

    @property
    def network_stub(self) -> network_grpc.NetworkStub:
        return self._network

    @property
    def power_stub(self) -> power_grpc.PowerStub:
        return self._power_stub

    @property
    def scan_library_stub(self) -> scan_library_grpc.ScanLibraryStub:
        return self._scan_library_stub

    @property
    def storage_stub(self) -> storage_grpc.StorageStub:
        return self._storage_stub

    @property
    def system_stub(self) -> system_grpc.SystemStub:
        return self._system_stub

    @property
    def wifi_client_stub(self) -> wifi_client_grpc.WifiClientStub:
        return self._wifi_client_stub
