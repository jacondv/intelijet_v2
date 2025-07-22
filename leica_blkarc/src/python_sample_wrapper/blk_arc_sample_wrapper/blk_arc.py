import logging
import time
from pathlib import Path
from typing import Generator, List, Optional

import numpy as np
from blk_arc_grpc import about_pb2 as about_message
from blk_arc_grpc import capture_pb2 as capture_message
from blk_arc_grpc import clock_pb2 as clock_message
from blk_arc_grpc import coordinate_frames_pb2 as coordinate_frames_message
from blk_arc_grpc import device_pb2 as device_message
from blk_arc_grpc import fault_pb2 as fault_message
from blk_arc_grpc import imaging_pb2 as imaging_message
from blk_arc_grpc import masking_pb2 as masking_message
from blk_arc_grpc import power_pb2 as power_message
from blk_arc_grpc import scan_library_pb2 as scan_library_message
from blk_arc_grpc import wifi_client_pb2 as wifi_client_message
from blk_arc_grpc import network_pb2 as network_message
from google.protobuf import empty_pb2
from google.protobuf.wrappers_pb2 import StringValue
from PIL import Image

from blk_arc_sample_wrapper import blk_arc_config as config
from blk_arc_sample_wrapper.blk_arc_decorators import (false_if_not_connected, false_if_rpc_error,
                                                       none_if_not_connected, none_if_rpc_error)
from blk_arc_sample_wrapper.blk_arc_stub_handler import BLK_ARC_StubHandler


class BLK_ARC:
    """
        A simple API-layer to BLK ARC devices using the gRPC-interface.
    """

    def __init__(self):
        self._grpc_stub_handler: BLK_ARC_StubHandler = BLK_ARC_StubHandler()

    def connect(self, connection_type: config.ConnectionType = config.ConnectionType.WIRED) -> bool:
        """
        Connects to a BLK ARC device.

        Args:
            connection_type (config.ConnectionType): Connection method of the BLK ARC (Wired or wireless).
        Returns:
            bool: True if a connection could be established, else False.
        """

        logging.info(f"Connecting through {'USB' if connection_type == config.ConnectionType.WIRED else 'WiFi'}.")

        if not self._grpc_stub_handler.is_pingable(connection_type=connection_type):
            return False

        if not self._grpc_stub_handler.create_grpc_stubs(connection_type=connection_type):
            self.disconnect()
            return False
        return self._grpc_stub_handler.is_connected()

    def is_connected(self) -> bool:
        """
        Checks whether a device is currently connected.

        Returns:
            bool: True if a device is currently connected, else False.
        """

        return self._grpc_stub_handler.is_connected()

    def disconnect(self) -> bool:
        """
        Disconnects from a BLK ARC device.

        Returns:
            bool: True if the device could be disconnected, else False.
        """

        self._grpc_stub_handler.close_grpc_channel()
        return True

    @none_if_not_connected
    @none_if_rpc_error
    def get_device_info(self) -> Optional[about_message.ManufactureInfoResponse]:
        """
        Reads the name, the serial-number and the article-number from the device. See the proto definition for further documentation.

        Returns:
            Optional[about_message.ManufactureInfoResponse]: Proto-message containing the device-information or None if the call failed.
        """

        manufacture_info = self._grpc_stub_handler.about_stub.GetManufactureInfo(empty_pb2.Empty(),
                                                                                 timeout=config.GRPC_TIMEOUT)
        return manufacture_info

    @none_if_not_connected
    @none_if_rpc_error
    def get_firmware_version(self) -> Optional[str]:
        """
        Reads the firmware-version from the device.

        Returns:
            Optional[str]: Firmware version or None if the call failed.
        """

        software_info = self._grpc_stub_handler.about_stub.GetSoftwareInfo(empty_pb2.Empty(),
                                                                           timeout=config.GRPC_TIMEOUT)
        firmware_version = software_info.sw_versions["firmware"]
        return firmware_version

    @none_if_not_connected
    @none_if_rpc_error
    def get_device_status(self) -> Optional[device_message.DeviceStateResponse]:
        """
        Reads the status from the device. See the proto definition for further documentation.

        Returns:
            Optional[device_message.DeviceStateResponse]: Proto-message containing the device-state or None if the call failed.
        """

        device_status = self._grpc_stub_handler.device_stub.DeviceState(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return device_status

    @none_if_not_connected
    @none_if_rpc_error
    def get_free_disk_space_percentage(self) -> Optional[float]:
        """
        Reads the free disk-space from the device.

        Returns:
            Optional[float]: Free disk-space percentage or None if the call failed.
        """

        disk_status = self._grpc_stub_handler.storage_stub.GetDiskStatus(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return disk_status.free_disk_space_percent

    @none_if_not_connected
    @none_if_rpc_error
    def start_capture(self) -> Optional[capture_message.StartCaptureResponse]:
        """
        Starts capturing a new scan.

        Returns:
            Optional[capture_message.StartCaptureResponse]: Proto-message containing the scan-id, starting time and a
            uuid of the started scan or None if the call failed.
        """
        start_capture_response = self._grpc_stub_handler.capture_stub.StartCapture(
            empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT_START_CAPTURE)
        return start_capture_response

    @none_if_not_connected
    @none_if_rpc_error
    def stop_capture(self) -> Optional[capture_message.StopCaptureResponse]:
        """
        Stops the currently running scan.

        Returns:
            Optional[capture_message.StopCaptureResponse]: Proto-message containing the scan-id, starting time, duration, a
            uuid and some other statistics of the finished scan or None if the call failed.
        """

        stop_capture_response = self._grpc_stub_handler.capture_stub.StopCapture(empty_pb2.Empty(),
                                                                                 timeout=config.GRPC_TIMEOUT)
        return stop_capture_response

    @false_if_not_connected
    @false_if_rpc_error
    def begin_static_pose(self) -> bool:
        """
        Allows you to mark the start of a static pose while scanning. This enables you to process static-parts of the scan
        separately, which can result in improved accuraccy of the data.

        Returns:
            bool: True if the start of a static scan could be marked in the running scan, else False.
        """

        if not self.is_scanning():
            logging.warning("Please start a scan first.")
            return False
        if self.is_in_static_pose():
            logging.warning("You are already in a static pose. First end it with end_static_pose().")
            return False
        self._grpc_stub_handler.capture_annotations_stub.BeginStaticPose(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return True

    @false_if_not_connected
    @false_if_rpc_error
    def end_static_pose(self) -> bool:
        """
        Allows you to mark the end of a static pose while scanning. See 'begin_static_pose()' for more details.

        Returns:
            bool: True if the end of the static scan could be marked in the running scan, else False.
        """

        if not self.is_scanning():
            logging.warning("Please start a scan first.")
            return False
        if not self.is_in_static_pose():
            logging.warning("You are not yet in a static pose. First start it with begin_static_pose().")
            return False
        self._grpc_stub_handler.capture_annotations_stub.EndStaticPose(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return True

    def is_scanning(self) -> bool:
        """
        Returns whether a scan is ongoing.

        Returns:
            bool: True if the call worked and a scan is ongoing, else False.
        """

        device_status = self.get_device_status()
        if device_status is None:
            return False
        return device_status.state in [
            device_message.DeviceStateResponse.State.CAPTURE_RUNNING,
            device_message.DeviceStateResponse.State.CAPTURE_RUNNING_STATIC
        ]

    def is_in_static_pose(self) -> bool:
        """
        Returns whether a static-scan is ongoing. Static-scans are started/stopped with 'begin_static_pose()'
        and 'end_static_pose()'.

        Returns:
            bool: True if the call worked and a static-scan is ongoing, else False.
        """

        device_status = self.get_device_status()
        if device_status is None:
            return False
        return device_status.state == device_message.DeviceStateResponse.State.CAPTURE_RUNNING_STATIC

    @none_if_not_connected
    @none_if_rpc_error
    def start_scan_data_stream(
        self,
        max_point_frequency: int = 10000,
        enable_low_latency_trajectory: bool = False
    ) -> Optional[Generator[capture_message.CaptureStreamMessage, None, None]]:
        """
        Starts the streaming of scan-data. Scan-data contain the trajectory of the scanner and the pointcloud in scanner-frame.

        Args:
            max_point_frequency (int): Upper limit for the number of pointcloud-points which should be streamed. The maximum number
                                       you will be able to actually receive depends on various factors like the connection or the
                                       efficiency of the code receiving the points. The number of received points will always be
                                       smaller than this limit due to point-filtering on the scanner.
            enable_low_latency_trajectory (bool): Enables trajectory-output with lower latency compared to the slightly delayed
                                                  SLAM-trajectory output. A lower latency is achieved by integration of IMU-data,
                                                  thus also reducing the accuracy of the signal as compared to the usual
                                                  SLAM-based output.
        Returns:
            Optional[Generator[capture_message.CaptureStreamMessage, None, None]]: A generator streaming the scan-data or None
            if the call failed.
        """

        stream_request = capture_message.LiveStreamRequest(
            options=capture_message.StreamMessageOptions(max_point_frequency=max_point_frequency,
                                                         enable_trajectory=False, #Default is True
                                                         enable_baser=True, #Default is True
                                                         enable_slam=False, #Default is True
                                                         enable_scan_events=True,
                                                         enable_low_latency_trajectory=enable_low_latency_trajectory))
        capture_stream = self._grpc_stub_handler.capture_stub.StreamLiveCapture(stream_request)
        return capture_stream

    @none_if_not_connected
    @none_if_rpc_error
    def list_scans(self) -> Optional[List[scan_library_message.ItemInfo]]:
        """
        Lists metadata of all the scans which are stored on the scanner.

        Returns:
            Optional[List[scan_library_message.ItemInfo]]: A list of metadata of all the scans on the scanner or None if the
            call failed.
        """

        items_request = scan_library_message.ListItemsRequest(selector=u'/scan-data/scans')
        data_items = self._grpc_stub_handler.scan_library_stub.ListItems(items_request, timeout=config.GRPC_TIMEOUT)
        return data_items.items

    @none_if_not_connected
    def get_scan_info(self, scan_id: int) -> Optional[scan_library_message.ItemInfo]:
        """
        Returns the metadata of a specific scan identified by its scan-id.

        Args:
            scan_id (int): The id of the scan.
        Returns:
            Optional[scan_library_message.ItemInfo]: Metadata of the specified scan or None if the call failed.
        """

        scans = self.list_scans()
        if not scans:
            logging.warning("Could not get the scans from the scanner.")
            return None
        scan_info = [scan for scan in scans if scan.metadata.properties['item/id'] == str(scan_id)]
        if len(scan_info) > 0:
            return scan_info[-1]
        return None

    @false_if_not_connected
    @false_if_rpc_error
    def delete_scan(self, scan_id: int) -> bool:
        """
        Deletes a scan from the scanner, identified by its scan-id.

        Args:
            scan_id (int): The id of the scan which should be deleted.
        Returns:
            bool: True if the scan could be successfully deleted, else False.
        """

        items_request = scan_library_message.ListItemsRequest(selector=u'/scan-data/scans/{}'.format(scan_id))
        self._grpc_stub_handler.scan_library_stub.DeleteItem(items_request, timeout=config.GRPC_TIMEOUT)
        return True

    @false_if_not_connected
    def download_scan(self, scan_id: int, path: Path) -> bool:
        """
        Downloads a scan from the scanner to the device calling this method. The scan is identified by the scan-id.

        Args:
            scan_id (int): The id of the scan which should be deleted.
            path (Path): The path to the directory where the scan should be downloaded to.
        Returns:
            bool: True if the scan could be successfully downloaded, else False.
        """

        if path.is_file():
            logging.warning(f"The download path must be a directory, not a file. Provided path: {path}")
            return False
        path.mkdir(parents=True, exist_ok=True)

        scan_info = self.get_scan_info(scan_id)
        if not scan_info:
            logging.warning(f"Could not find scan with ID '{scan_id}' on the scanner.")
            return False

        scan_size = scan_info.size_bytes
        filename = f"{scan_id}.b2g"

        with open(path / filename, 'wb') as file:

            read_item_request = scan_library_message.ReadItemRequest(path=scan_info.path, offset=0, length=0)
            chunk_stream = self._grpc_stub_handler.scan_library_stub.ReadItem(read_item_request)
            last_logged_percentage = 0
            for chunk in chunk_stream:
                file.write(chunk.data)
                percentage = round(100.0 * chunk.offset / scan_size)
                if percentage >= last_logged_percentage + 10:
                    logging.info(f"Downloaded {percentage}%")
                    last_logged_percentage = percentage
        return True

    @false_if_not_connected
    @false_if_rpc_error
    def trigger_detail_image(self) -> bool:
        """
        Triggers the capture of an image with the detailed-camera. Capturing a detail-image is only possible while
        a scan is ongoing. The image is stored in the scan (.b2g).

        Returns:
            bool: True if the capture of the detail-image could be triggered, else False.
        """

        if not self.is_scanning():
            logging.warning("Please start a scan first.")
            return False
        detail_image_request = imaging_message.TakeDetailImageRequest(exclude_preview=True)
        self._grpc_stub_handler.imaging_stub.TakeDetailImage(detail_image_request, timeout=config.GRPC_TIMEOUT)
        detail_panorama_request = imaging_message.TakePanoramaImageRequest(exclude_preview=False)
        self._grpc_stub_handler.imaging_stub.TakePanoramaImage(detail_panorama_request, timeout=config.GRPC_TIMEOUT)
        return True

    # yapf: disable
    @none_if_not_connected
    @none_if_rpc_error
    def stream_pano_images_scanning(
            self,
            count: int = 1) -> Optional[Generator[imaging_message.ImageStreamResponse, None, None]]:
        # yapf: enable
        """
        Starts the streaming of panoramic images during a scan.

        Args:
            count (int): Number of pano image triplets (Left, Front & Right camera) to be streamed.

        Returns:
            Optional[Generator[imaging_message.ImageStreamResponse, None, None]]: A generator streaming the panoramic
                                                                                  images or None if the call failed.
        """

        if not self.is_scanning():
            logging.warning("Please start a scan first.")
            return False

        # NOTE: While streaming pano images during a scan, the `exposure_mode` is required to be set to `AUTO`
        #       (`exposure_in_ms` and `gain` will be ignored), else an error will be thrown.
        pano_image_stream_request = imaging_message.ImageStreamRequest(count=count,
                                                                       max_frames_per_second=0,
                                                                       compression=imaging_message.ImageCompression.RAW,
                                                                       exposure_mode=imaging_message.ExposureMode.AUTO,
                                                                       exposure_in_ms=0,
                                                                       gain=0,
                                                                       image_size=imaging_message.ImageSize.FULL)
        pano_image_stream = self._grpc_stub_handler.imaging_stub.StreamPanoramaImages(pano_image_stream_request)
        return pano_image_stream

    @none_if_not_connected
    @none_if_rpc_error
    def stream_pano_images_idle(self,
                                camera_id: imaging_message.PanoramaCameraStreamRequest.CameraID,
                                count: int = 1,
                                exposure_in_ms: int = -1,
                                gain: int = -1) -> Optional[Generator[imaging_message.ImageStreamResponse, None, None]]:
        """
        Starts the streaming of panoramic images while no scan is on-going.

        Args:
            camera_id (imaging_message.PanoramaCameraStreamRequest.CameraID): The ID of the camera of which the images
                                                                              should be streamed.
            count (int): Number of images to be streamed.
            exposure_in_ms (int): The exposure is the time the shutter stays open, expressed in milliseconds.
                                  A higher exposure value will generate a brighter image.
            gain (int): The gain is the equivalent of the ISO value in a camera. It artificially increases the
                        brightness of the image without requiring a longer exposure.

        Returns:
            Optional[Generator[imaging_message.ImageStreamResponse, None, None]]: A generator streaming the panoramic
                                                                                  images or None if the call failed.
        """

        if self.is_scanning():
            logging.warning("Not possible while scanning, use stream_panorama_images instead.")
            return False

        # if the exposure time and the gain is given by the integrator, then the exposure mode is set to MANUAL, else it is set to AUTO
        exposure_mode = imaging_message.ExposureMode.MANUAL
        if (exposure_in_ms == -1 or gain == -1):
            exposure_mode = imaging_message.ExposureMode.AUTO
            exposure_in_ms = 0
            gain = 0

        stream_pano_camera_request = imaging_message.PanoramaCameraStreamRequest(count=count,
                                                                                 camera_id=camera_id,
                                                                                 exposure_mode=exposure_mode,
                                                                                 exposure_in_ms=exposure_in_ms,
                                                                                 gain=gain)
        pano_camera_stream = self._grpc_stub_handler.imaging_stub.StreamPanoramaCamera(stream_pano_camera_request)
        return pano_camera_stream

    @false_if_not_connected
    @false_if_rpc_error
    def reboot(self) -> bool:
        """
        Triggers a reboot of the scanner. The reboot will take a while and you will loose conntection to the device.
        Therefore you must reconnect using 'connect()' once the scanner is running again.

        Returns:
            bool: True if the reboot was triggered and the connection was closed, else False.
        """

        self._grpc_stub_handler.system_stub.Reboot(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return self.disconnect()

    @none_if_not_connected
    @none_if_rpc_error
    def power_status(self) -> Optional[power_message.PowerStatusResponse]:
        """
        Reads information about the power-source from the scanner. This allows to determine whether the scanner
        is in USB-only mode (no scanning allowed) or properly connected to a power-supply.

        Returns:
            Optional[power_message.PowerStatusResponse]: A proto-message containing information about the power-source
            or None if the call failed.
        """

        power_status = self._grpc_stub_handler.power_stub.GetPowerStatus(empty_pb2.Empty(), timeout=config.GRPC_TIMEOUT)
        return power_status

    def data_only_mode_active(self) -> bool:
        """
        Returns whether the scanner is in USB-only mode (only connected by USB-cable). In this mode you can still
        interact with the scanner and download scans. However scanning is not possible.

        Returns:
            bool: True if the call succeeded and the scanner is in data-only-mode, else False.
        """

        power_status = self.power_status()
        if not power_status:
            logging.warning("Could not get power-status.")
            return False
        return power_status.power_source == power_message.PowerStatusResponse.PowerSource.USB

    @none_if_not_connected
    @none_if_rpc_error
    def get_time_from_clock_sources(self) -> Optional[clock_message.TimestampsResponse]:
        """
        Reads the different clock-sources from the scanner. This can be used in order to establish time-synchronisation.

        Returns:
            Optional[clock_message.TimestampsResponse]: A proto-message containing the time of all clock-sources
            or None if the call failed.
        """

        current_clock_status = self._grpc_stub_handler.clock_stub.GetCurrentDateTime(empty_pb2.Empty(),
                                                                                     timeout=config.GRPC_TIMEOUT)
        return current_clock_status

    @false_if_not_connected
    @false_if_rpc_error
    def acknowledge_faults(self) -> bool:
        """
        Some less serious faults on the scanner are resolvable by acknowledgement. This call allows you to acknowledge
        all the active and acknowledgeable faults on the scanner. Please refer to the 'fault_example.py' for further
        documentation about the handling of faults on the scanner.

        Returns:
            bool: True if it was possible to successfully acknowledge active faults, else False.
        """

        fault_request = fault_message.AcknowledgeFaultRequest(location="")
        self._grpc_stub_handler.fault_stub.AcknowledgeFault(fault_request, timeout=config.GRPC_TIMEOUT)
        return True

    def wait_till_idle(self, timeout: float) -> bool:
        timeout_time = time.time() + timeout
        timed_out = False
        device_ready = False
        while not timed_out and not device_ready:
            device_status = self.get_device_status()
            device_ready = device_status.state == device_message.DeviceStateResponse.State.IDLE
            timed_out = time.time() > timeout_time
            time.sleep(1.0)
        if timed_out:
            logging.warning(f"Device still not ready to capture after waiting {timeout} seconds")
        return device_ready

    @none_if_not_connected
    @none_if_rpc_error
    def get_coordinate_frames(self) -> Optional[coordinate_frames_message.AlignedExtrinsics]:
        """
        Reads the transformations between important coordinate-frames of the scanner.
        All transformations describe the transformation of the respective frame with respect to the
        baser-frame. The baser-frame is located in the middle of the dome of the scanner and is
        the frame of the streamed pointcloud-points. Also the trajectory tracks the baser-frame with
        respect to a gravity aligned global-frame.

        Returns:
            Optional[coordinate_frames_message.AlignedExtrinsics]: A proto-message containing the transformations
            to different frames with respect to the baser-frame, or None if the call failed.
        """

        get_transformation_data_request = coordinate_frames_message.TransformationRequest()
        extrinsics = self._grpc_stub_handler.coordinate_frames_stub.GetTransformationData(
            get_transformation_data_request, timeout=config.GRPC_TIMEOUT)
        return extrinsics

    @false_if_not_connected
    @false_if_rpc_error
    def reset_mask(self, mask_type: masking_message.MaskIdentifier) -> bool:
        """
        Resets a custom mask on the scanner, which was previously set by 'set_mask()'.

        Args:
            mask_type (masking_message.MaskIdentifier): The mask which should be reset.
        Returns:
            bool: True if it was possible to reset the mask, else False.
        """

        if mask_type == masking_message.MaskIdentifier.TYPE_UNSET:
            logging.warning(f"Mask of type '{mask_type}' not known.")
            return False
        mask_request = masking_message.MaskRequest(id=mask_type)
        self._grpc_stub_handler.masking_stub.ResetMask(mask_request, timeout=config.GRPC_TIMEOUT)
        return True

    @none_if_not_connected
    @none_if_rpc_error
    def get_mask(self, mask_type: masking_message.MaskIdentifier) -> Optional[masking_message.MaskDescription]:
        """
        Gets a custom mask on the scanner, which was previously set by 'set_mask()'.

        Args:
            mask_type (masking_message.MaskIdentifier): The mask which should be read.
        Returns:
            Optional[masking_message.MaskDescription]: The custom mask or None if the call failed.
        """

        if mask_type == masking_message.MaskIdentifier.TYPE_UNSET:
            logging.warning(f"Mask of type '{mask_type}' not known.")
            return None
        mask_request = masking_message.MaskRequest(id=mask_type)
        mask = self._grpc_stub_handler.masking_stub.GetMask(mask_request, timeout=config.GRPC_TIMEOUT)
        return mask

    @false_if_not_connected
    @false_if_rpc_error
    def download_mask(self, mask_type: masking_message.MaskIdentifier, path: Path) -> bool:
        """
        Gets a custom mask on the scanner, which was previously set by 'set_mask()' and saves it as an image.

        Args:
            mask_type (masking_message.MaskIdentifier): The mask which should be read.
            path (Path): The filename and path where the image should be stored.
        Returns:
            bool: True if it successfully saved the mask as an image, else False.
        """

        if path.suffix != ".png":
            logging.warning("The provided filename is not in .png format.")
            return False

        mask = self.get_mask(mask_type=mask_type)
        if mask is None:
            logging.warning("Failed to download the mask.")
            return False

        if mask_type == masking_message.MaskIdentifier.LIDAR:
            byte_mask = mask.lidar_mask.byte_mask
            image = Image.frombytes('L', (byte_mask.width, byte_mask.height), byte_mask.data)
            image.save(path)
        else:
            slam_bit_mask = mask.slam_mask.bit_mask
            mask_unpacked = np.unpackbits(np.frombuffer(slam_bit_mask.data, dtype=np.uint8)).reshape(
                slam_bit_mask.image_height_pixels, slam_bit_mask.image_width_pixels).astype(np.uint8)
            mask_unpacked *= 255
            image = Image.frombytes('L', (slam_bit_mask.image_width_pixels, slam_bit_mask.image_height_pixels),
                                    mask_unpacked)
            image.save(path)
        return True

    @false_if_not_connected
    @false_if_rpc_error
    def set_lidar_mask(self, mask_filename: Path) -> bool:
        """
        Sets a custom lidar mask on the scanner. This allows you to filter out pointcloud-points. Please refer to
        'lidar_masking_example.py' for an example on how to properly use masks.

        Args:
            mask_filename (Path): The path to the lidar mask image.
        Returns:
            bool: True if it was possible to set the lidar mask, else False.
        """

        if mask_filename.suffix != ".png":
            logging.warning("The input image is not in .png format. Please provide an image mask in .png format.")
            return False

        mask_img = Image.open(mask_filename).convert('L')
        lidar_mask = self._convert_mask_img_to_lidarmask(mask_img)
        if not lidar_mask:
            return False
        mask_description = masking_message.MaskDescription(id=masking_message.MaskIdentifier.LIDAR,
                                                           lidar_mask=lidar_mask)
        self._grpc_stub_handler.masking_stub.SetMask(mask_description, timeout=config.GRPC_TIMEOUT)
        return True

    def _convert_mask_img_to_lidarmask(self, mask_img: Image) -> masking_message.LidarMask:
        width, height = mask_img.size
        if width != config.LIDAR_MASK_WIDTH or height != config.LIDAR_MASK_HEIGHT:
            logging.warning(
                f"The provided mask image does not have the correct size of {config.LIDAR_MASK_WIDTH} x {config.LIDAR_MASK_HEIGHT}."
            )
            return None

        # Convert the image to a NumPy array
        img_array = np.array(mask_img, dtype=np.uint8)

        # create the byte array based on the input mask, pixels above 128 in the grey-scale image will be masked
        byte_array = np.where(img_array > 128, 255, 0).astype(np.uint8).flatten()

        byte_mask = masking_message.ByteMask(width=width, height=height, data=bytes(byte_array))
        return masking_message.LidarMask(byte_mask=byte_mask)

    @false_if_not_connected
    @false_if_rpc_error
    def set_slam_mask(self, mask_filename: Path, camera_id: masking_message.MaskIdentifier) -> bool:
        """
        Sets a custom slam-camera mask on the scanner. This allows you to filter out pixels when colorizing the
        pointcloud-points. Please refer to 'slam_camera_masking_example.py' for an example on how to properly use
        slam-camera masks.

        Args:
            mask_filename (Path): The path to the slam camera mask image.
            camera_id (masking_message.MaskIdentifier): The camera id for which camera the mask should be used.
        Returns:
            bool: True if it was possible to set the slam mask, else False.
        """

        if mask_filename.suffix != ".png":
            logging.warning("The input image is not in .png format. Please provide an image mask in .png format.")
            return False

        mask_img = Image.open(mask_filename).convert('L')
        slam_mask = self._convert_mask_img_to_slam_camera_mask(mask_img)
        if not slam_mask:
            return False
        mask_description = masking_message.MaskDescription(id=camera_id, slam_mask=slam_mask)
        self._grpc_stub_handler.masking_stub.SetMask(mask_description, timeout=config.GRPC_TIMEOUT)
        return True

    def _convert_mask_img_to_slam_camera_mask(self, mask_img: Image) -> Optional[masking_message.BitMask]:
        width, height = mask_img.size

        if width != config.SLAM_CAM_MASK_WIDTH or height != config.SLAM_CAM_MASK_HEIGHT:
            logging.warning(
                f"The provided mask image does not have the correct size of {config.SLAM_CAM_MASK_WIDTH} x {config.SLAM_CAM_MASK_HEIGHT}."
            )
            return None

        # Convert the image to a NumPy array of floats
        img_array = np.array(mask_img, dtype=np.uint8)

        # apply mask to byte_array
        bool_mask = img_array > 128
        bit_array = np.packbits(bool_mask, axis=1).flatten()

        bytes_per_row = width // 8
        bit_mask = masking_message.BitMask(image_width_pixels=width,
                                           image_height_pixels=height,
                                           bytes_per_row=bytes_per_row,
                                           data=bytes(bit_array))
        return masking_message.SLAMCameraMask(bit_mask=bit_mask)

    @none_if_not_connected
    @none_if_rpc_error
    def get_wifi_client_status(self) -> Optional[wifi_client_message.WifiClientStatusResponse]:
        """
        Returns current information about the Wifi client mode, such as the currently connected Wifi network and whether it has access to the internet.

        Returns:
            Optional[wifi_client_message.WifiClientStatusResponse]: The Wifi client status response.
        """

        return self._grpc_stub_handler.wifi_client_stub.GetWifiClientStatus(empty_pb2.Empty())

    @none_if_not_connected
    @none_if_rpc_error
    def scan_wifi_networks(self) -> Optional[wifi_client_message.ScanWifiNetworksResponse]:
        """
        Returns a list of nearby Wifi networks detected in the last scan.

        Returns:
            Optional[wifi_client_message.ScanWifiNetworksResponse]: A list of all visible Wifi Networks.
        """
        return self._grpc_stub_handler.wifi_client_stub.ScanWifiNetworks(empty_pb2.Empty())

    @none_if_not_connected
    @none_if_rpc_error
    def connect_wifi_network(self,
                             ssid: str,
                             pwd: str,
                             encryption: wifi_client_message.Encryption,
                             autoconnect: bool = False) -> Optional[wifi_client_message.WifiClientStatusResponse]:
        """
        Connects the BLK ARC to the specified network using WPA/WPA2/WPA3 encryption. The connection attempt is limited to 15 seconds. The method may block up to this time.

        Returns:
            Optional[wifi_client_message.WifiClientStatusResponse]: A list of all visible Wifi Networks.
        """
        new_wifi_connection_request = wifi_client_message.NewWifiConnectionRequest(encryption=encryption,
                                                                                   ssid=ssid,
                                                                                   psk=StringValue(value=pwd),
                                                                                   autoconnect=autoconnect)
        return self._grpc_stub_handler.wifi_client_stub.ConnectToNewWifiNetwork(new_wifi_connection_request)

    @none_if_not_connected
    @none_if_rpc_error
    def get_known_wifi_networks(self) -> Optional[wifi_client_message.KnownWifiListResponse]:
        """
        Returns a list of the currently saved Wifi networks, sorted by priority.

        Returns:
            Optional[wifi_client_message.KnownWifiListResponse]: A list of all known Wifi Networks.
        """
        return self._grpc_stub_handler.wifi_client_stub.GetKnownWifiNetworks(empty_pb2.Empty())

    @false_if_not_connected
    @false_if_rpc_error
    def delete_known_wifi_network(self, ssid: str) -> bool:
        """
        Deletes a network from the list of currently saved networks. If the device is currently connected to the specified network, it will be disconnected first.

        Returns:
            bool: True if the service was successfully triggered. Attention: No feedback if the Wifi network got successfully deleted or not.
        """
        wifi_delete_Request = wifi_client_message.WifiDeleteRequest(ssid=ssid)
        self._grpc_stub_handler.wifi_client_stub.DeleteKnownWifiNetwork(wifi_delete_Request)
        return True

    @false_if_not_connected
    @false_if_rpc_error
    def clear_known_wifi_networks(self) -> bool:
        """
        Clears the list of known Wifi networks. If the device is currently connected to a network, it will be disconnected first.

        Returns:
            bool: True if the service was successfully triggered. Attention: No feedback if the Wifi networks got successfully cleared or not.
        """
        self._grpc_stub_handler.wifi_client_stub.ClearKnownWifiNetworks(empty_pb2.Empty())
        return True

    @none_if_not_connected
    @none_if_rpc_error
    def get_network_information(self) -> Optional[network_message.NetworkInformationResponse]:
        """
        Get a list of the available BLK device network interfaces, paired with their respective IP.

        Returns:
            Optional[network_message.NetworkInformationResponse]: A list of the available network interfaces.
        """
        return self._grpc_stub_handler.network_stub.GetNetworkInformation(empty_pb2.Empty())



# DANH TODO ------------------------------------

    @none_if_not_connected
    @none_if_rpc_error
    def get_calibration_info(self) -> Optional[about_message.HardwareInfoResponse]:
        """
        Reads the name, the serial-number and the article-number from the device. See the proto definition for further documentation.

        Returns:
            Optional[about_message.ManufactureInfoResponse]: Proto-message containing the device-information or None if the call failed.
        """

        manufacture_info = self._grpc_stub_handler.about_stub.GetHardwareInfo(empty_pb2.Empty(),
                                                                                 timeout=config.GRPC_TIMEOUT)
        return manufacture_info
