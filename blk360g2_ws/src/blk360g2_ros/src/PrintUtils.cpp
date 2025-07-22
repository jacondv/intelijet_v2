#include "PrintUtils.hpp"
#include "UuidUtils.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace common
{
template<size_t Size>
std::ostream& operator<<(std::ostream& os, const double (&matrix)[Size])
{
    const int rows = static_cast<int>(std::sqrt(Size));
    for (int i = 0; i < rows; ++i)
    {
        os << '\t';
        for (int j = 0; j < rows; ++j)
        {
            os << std::to_string(matrix[j + i * rows]) << ", ";
        }
        os << '\n';
    }
    return os;
}

std::string margin(int width)
{
    return std::string(width, ' ');
}

std::string timestampToUtc(uint64_t tm)
{
    if (tm != 0)
    {
        auto tt = static_cast<time_t>(tm / 1000000000);
        std::tm* gmt = std::gmtime(&tt);
        std::ostringstream os;
        os << std::put_time(gmt, "%F %T %Z");
        return os.str();
    }
    return "N/A";
}

const char* strConnectionStatus(Blk360G2_ConnectionStatus status)
{
    switch (status)
    {
    case Blk360G2_ConnectionStatus_OK:
        return "OK";
    case Blk360G2_ConnectionStatus_ForceDisconnected:
        return "ForceDisconnected";
    case Blk360G2_ConnectionStatus_Disconnected:
        return "Disconnected";
    case Blk360G2_ConnectionStatus_Reconnect:
        return "Reconnect";
    default:
        return "Unknown";
    }
}

const char* strMeasurementAccuracy(Blk360G2_MeasurementAccuracy_t accuracy)
{
    switch (accuracy)
    {
    case Blk360G2_MeasurementAccuracy_Invalid:
        return "Invalid";
    case Blk360G2_MeasurementAccuracy_Unset:
        return "Unset";
    case Blk360G2_MeasurementAccuracy_Inaccurate:
        return "Inaccurate";
    case Blk360G2_MeasurementAccuracy_Accurate:
        return "Accurate";
    default:
        return "Unknown";
    }
}

const char* strPointCloudDensity(Blk360G2_PointCloudDensity_t density)
{
    switch (density)
    {
    case Blk360G2_PointCloudDensity_Invalid:
        return "Invalid";
    case Blk360G2_PointCloudDensity_High:
        return "High";
    case Blk360G2_PointCloudDensity_Medium:
        return "Medium";
    case Blk360G2_PointCloudDensity_Low:
        return "Low";
    case Blk360G2_PointCloudDensity_Ultralow:
        return "Ultralow";
    default:
        return "Unknown";
    }
}

const char* strImagingMode(Blk360G2_ImagingMode_t imagingMode)
{
    switch (imagingMode)
    {
    case Blk360G2_ImagingMode_Invalid:
        return "Invalid";
    case Blk360G2_ImagingMode_LDR:
        return "LDR";
    case Blk360G2_ImagingMode_HDR:
        return "HDR";
    default:
        return "Unknown";
    }
}

const char* strVisMode(Blk360G2_VisMode_t visMode)
{
    switch (visMode)
    {
    case Blk360G2_VisMode_Invalid:
        return "Invalid";
    case Blk360G2_VisMode_Disabled:
        return "Disabled";
    case Blk360G2_VisMode_Enabled:
        return "Enabled";
    default:
        return "Unknown";
    }
}

const char* strCaptureState(Blk360G2_CaptureState_t state)
{
    switch (state)
    {
    case Blk360G2_CaptureState_Invalid:
        return "Invalid";
    case Blk360G2_CaptureState_InProgress:
        return "InProgress";
    case Blk360G2_CaptureState_Completed:
        return "Completed";
    case Blk360G2_CaptureState_Aborted:
        return "Aborted";
    case Blk360G2_CaptureState_Cancelled:
        return "Cancelled";
    default:
        return "Unknown";
    }
}

const char* strGetImageStatus(const Blk360G2_GetImageProgressStatus_t status)
{
    switch (status)
    {
    case Blk360G2_GetImageProgressStatus_Downloading:
        return "downloading";
    case Blk360G2_GetImageProgressStatus_Processing:
        return "processing";
    case Blk360G2_GetImageProgressStatus_Done:
        return "done";
    default:
        return "<<invalid>>";
    }
}

const char* strCameraPosition(Blk360G2_CameraPosition position)
{
    switch (position)
    {
    case Blk360G2_CameraPosition_Invalid:
        return "Invalid";
    case Blk360G2_CameraPosition_RightBottom:
        return "RightBottom";
    case Blk360G2_CameraPosition_RightTop:
        return "RightTop";
    case Blk360G2_CameraPosition_LeftTop:
        return "LeftTop";
    case Blk360G2_CameraPosition_LeftBottom:
        return "LeftBottom";
    default:
        return "Unknown";
    }
}
const char* strPixelPattern(Blk360G2_PixelPattern_t pattern)
{
    switch (pattern)
    {
    case Blk360G2_PixelPattern_INVALID:
        return "INVALID";
    case Blk360G2_PixelPattern_BAYER_GRBG:
        return "BAYER_GRBG";
    case Blk360G2_PixelPattern_BAYER_RGGB:
        return "BAYER_RGGB";
    case Blk360G2_PixelPattern_BAYER_BGGR:
        return "BAYER_BGGR";
    case Blk360G2_PixelPattern_BAYER_GBRG:
        return "BAYER_GBRG";
    case Blk360G2_PixelPattern_GRAY:
        return "GRAY";
    case Blk360G2_PixelPattern_COLOR_RGBA:
        return "COLOR_RGBA";
    case Blk360G2_PixelPattern_COLOR_RGB:
        return "COLOR_RGB";
    case Blk360G2_PixelPattern_COLOR_BGR:
        return "COLOR_BGR";
    case Blk360G2_PixelPattern_COLOR_RGB_FLOAT:
        return "COLOR_RGB_FLOAT";
    default:
        return "Unknown";
    }
}

const char* strImagePlane(Blk360G2_ImagePlane_t plane)
{
    switch (plane)
    {
    case Blk360G2_ImagePlane_INVALID:
        return "INVALID";
    case Blk360G2_ImagePlane_GRAY:
        return "GRAY";
    case Blk360G2_ImagePlane_BAYER_CHANNEL_0:
        return "BAYER_CHANNEL_0";
    case Blk360G2_ImagePlane_BAYER_CHANNEL_1:
        return "BAYER_CHANNEL_1";
    case Blk360G2_ImagePlane_BAYER_CHANNEL_2:
        return "BAYER_CHANNEL_2";
    case Blk360G2_ImagePlane_BAYER_CHANNEL_3:
        return "BAYER_CHANNEL_3";
    case Blk360G2_ImagePlane_COLOR_CHANNEL_0:
        return "COLOR_CHANNEL_0";
    case Blk360G2_ImagePlane_COLOR_CHANNEL_1:
        return "COLOR_CHANNEL_1";
    case Blk360G2_ImagePlane_COLOR_CHANNEL_2:
        return "COLOR_CHANNEL_2";
    case Blk360G2_ImagePlane_COLOR_CHANNEL_3:
        return "COLOR_CHANNEL_3";
    default:
        return "Unknown";
    }
}
const char* strProjectionType(Blk360G2_ProjectionType_t type)
{
    switch (type)
    {
    case Blk360G2_ProjectionType_INVALID:
        return "Invalid";
    case Blk360G2_ProjectionType_PINHOLE:
        return "PINHOLE";
    default:
        return "Unknown";
    }
}

const char* strStatus(const Blk360G2_DeviceStatus& in)
{
    switch (in.status)
    {
    case Blk360G2_Status_OK:
        return "OK";
    case Blk360G2_Status_Error:
        return "non-fatal device error";
    case Blk360G2_Status_Fatal:
        return "fatal device error";
    default:
        return "invalid device status";
    }
}

const char* strLedColor(const Blk360G2_DeviceStatus& in)
{
    switch (in.ledStatus.color)
    {
    case Blk360G2_LedColor_None:
        return "None";
    case Blk360G2_LedColor_Red:
        return "Red";
    case Blk360G2_LedColor_Green:
        return "Green";
    case Blk360G2_LedColor_Yellow:
        return "Yellow";
    case Blk360G2_LedColor_Blue:
        return "Blue";
    case Blk360G2_LedColor_Orange:
        return "Orange";
    case Blk360G2_LedColor_White:
        return "White";
    case Blk360G2_LedColor_Off:
        return "Off";
    case Blk360G2_LedColor_GreenBright:
        return "Bright Green";
    case Blk360G2_LedColor_GreenDark:
        return "Dark Green";
    case Blk360G2_LedColor_YellowBrigth:
        return "Bright Yellow";
    case Blk360G2_LedColor_YellowDark:
        return "Dark Yellow";
    default:
        return "Invalid";
    }
}

const char* strLedState(const Blk360G2_DeviceStatus& in)
{
    switch (in.ledStatus.state)
    {
    case Blk360G2_LedState_None:
        return "None";
    case Blk360G2_LedState_Off:
        return "Off";
    case Blk360G2_LedState_Pulsating:
        return "Pulsating";
    case Blk360G2_LedState_Blinking:
        return "Blinking";
    case Blk360G2_LedState_On:
        return "On";
    case Blk360G2_LedState_Progress:
        return "Progress";
    case Blk360G2_LedState_Spinning:
        return "Spinning";
    default:
        return "Invalid";
    }
}

const char* strStorageAlert(const Blk360G2_DeviceStatus& in)
{
    switch (in.storage.storageAlert)
    {
    case Blk360G2_StorageAlert_Invalid:
        return "Invalid";
    case Blk360G2_StorageAlert_OK:
        return "OK";
    case Blk360G2_StorageAlert_Low:
        return "Warning Low";
    case Blk360G2_StorageAlert_Crit:
        return "Critical Low";
    default:
        return "Invalid";
    }
}

const char* strErrorAction(const Blk360G2_ErrorDetail& in)
{
    switch (in.action)
    {
    case Blk360G2_ErrorAction_Acknowledge:
        return "device requires refresh of error status (acknowledgement)";
    case Blk360G2_ErrorAction_Reboot:
        return "device requires a reboot";
    case Blk360G2_ErrorAction_SoftwareUpdate:
        return "device requires a software update";
    case Blk360G2_ErrorAction_Service:
        return "device requires service";
    default:
        return "invalid device status";
    }
}

std::string strErrorCode(const Blk360G2_ErrorDetail& in)
{
    switch (in.action)
    {
    case Blk360G2_ErrorAction_Acknowledge:
        return std::to_string(in.code.acknowledge);
    case Blk360G2_ErrorAction_Reboot:
        return std::to_string(in.code.reboot);
    case Blk360G2_ErrorAction_SoftwareUpdate:
        return std::to_string(in.code.softwareUpdate);
    case Blk360G2_ErrorAction_Service:
        return std::to_string(in.code.service);
    default:
        return {};
    }
}

void print(const Blk360G2_ErrorEvent& error)
{
    std::cerr << "Received Error event\n"
              << "Error code: " << error.errorCode << "\n"
              << "Error message: " << error.message << std::endl;
}

void print(const Blk360G2_PointCloudDownloadProgressEvent& event)
{
    const float downloadProgress = static_cast<float>(event.downloadedSize) / event.totalSize * 100.0f;
    std::cout << "Received PointCloudDownloadProgress event\n"
              << "Download: " << downloadProgress << '%' << std::endl;
}

void print(const Blk360G2_PointCloudProcessProgressEvent& event)
{
    std::cout << "Received PointCloudProcessProgress event\n"
              << "Process: " << event.progress << '%' << std::endl;
}

void print(const Blk360G2_MeasurementProgressEvent& event)
{
    std::cout << "Received MeasurementProgress event\n"
              << "Measurement: " << event.progress << "%" << std::endl;
}

void print(const Blk360G2_SetupStartedEvent& event)
{
    std::cout << "Received SetupStarted event\n"
              << "New setup uuid: " << getStringUuid(event.setupUuid).uuid << std::endl;
}

void print(const Blk360G2_VisRunningEvent& event)
{
    std::cout << "Received VisRunning event\n"
              << "Source setup uuid: "
              << Blk360G2_UUID_Serialize(event.sourceSetupUuid).uuid << '\n'
              << "Device moving status: "
              << std::boolalpha << event.deviceMoving << '\n'
              << "Current delta pose:\n"
              << event.deltaPose.elements << std::endl;
}

void print(const Blk360G2_SaveImagesToFileEvent& event)
{
    std::cout << "Received SaveImagesToFile event\n"
              << "Image: " << event.currentImage << "/" << event.totalImages << "\n"
              << "bytes: " << event.currentByte << " out of " << event.totalBytes << std::endl;
}

void print(const Blk360G2_SetupPackLoadingImagesSuccessEvent& event)
{
    std::cout << "Received LoadingImagesFromFileSuccess event\n"
              << "Loading images from file finished. Number of loaded images = " << event.totalImages << std::endl;
}

void print(const Blk360G2_GetOfficeImagesProgressEvent& event)
{
    std::cout << "Received GetOfficeImagesProgress event\n"
              << "GetOfficeImages Status: " << strGetImageStatus(event.progressStatus) << "\n"
              << "GetOfficeImages Progress: " << event.progress << '%' << std::endl;
}

void print(const Blk360G2_ImageReadyEvent& event)
{
    std::cout << "Received ImageReady event\n"
              << "Processed images: " << event.imageNumber + 1
              << " out of " << event.totalImages << std::endl;
}

void print(const Blk360G2_JobMetadata& metadata)
{
    std::cout << "Job " << getStringUuid(metadata.uuid).uuid << ":" << '\n'
              << '\t' << "Thumbnail Uuid: " << getStringUuid(metadata.thumbnailUuid).uuid << '\n'
              << '\t' << "Created: " << metadata.created << '\n'
              << '\t' << "Updated: " << metadata.updated << '\n';
}

void print(const Blk360G2_SetupMetadata& metadata)
{
    std::cout << "Setup " << getStringUuid(metadata.uuid).uuid << ':' << '\n'
              << '\t' << "Updated: " << timestampToUtc(metadata.updated) << '\n'
              << '\t' << "Created: " << timestampToUtc(metadata.created) << '\n'
              << '\t' << "Size: " << metadata.size << '\n'
              << '\t' << "Read only: " << std::boolalpha << metadata.readOnly << '\n';

    if (metadata.hasThumbnail)
    {
        std::cout << '\t' << "Thumbnail UUID: " << getStringUuid(metadata.thumbnailUuid).uuid << '\n';
    }

    if (metadata.hasVisResult)
    {
        std::cout << '\t' << "Delta pose: " << '\n'
                  << metadata.visResult.pose.deltaPose.elements << '\n'
                  << '\t' << "Position Uncertainty: " << '\n'
                  << metadata.visResult.pose.positionUncertainty.elements << '\n'
                  << '\t' << "Orientation Uncertainty: " << '\n'
                  << metadata.visResult.pose.orientationUncertainty.elements << '\n'
                  << '\t' << "Quality: " << metadata.visResult.pose.quality << "%" << '\n'
                  << '\t' << "Source Setup Uuid: " << getStringUuid(metadata.visResult.sourceSetupUuid).uuid << '\n'
                  << '\t' << "Accuracy: " << strMeasurementAccuracy(metadata.visResult.accuracy) << '\n';
    }

    std::cout << '\t' << "Scan Resolution: " << strPointCloudDensity(metadata.scanResolution) << '\n'
              << '\t' << "Scan Raster horizontal: " << metadata.scanRaster.horizontal << '\n'
              << '\t' << "Scan Raster vertical: " << metadata.scanRaster.vertical << '\n'
              << '\t' << "Imaging Mode: " << strImagingMode(metadata.imagingMode) << '\n'
              << '\t' << "Pose: " << '\n'
              << metadata.pose.elements << '\n'
              << '\t' << "Tilt matrix: " << '\n'
              << metadata.tiltMatrix.elements << '\n'
              << "Capture state: " << strCaptureState(metadata.captureState) << '\n';
}

void print(const Blk360G2_ConnectionStatus& status)
{
    std::cout << "Connection status: " << strConnectionStatus(status) << std::endl;
}

void print(const Blk360G2_ImagePlane& imagePlane)
{
    std::cout << margin(2) << "Image plane: " << common::strImagePlane(imagePlane) << '\n';
}

void print(const Blk360G2_CameraPosition& position)
{
    std::cout << "Camera position: " << strCameraPosition(position) << "\n";
}

void print(const Blk360G2_CameraCalibration& calibration)
{
    std::cout << "Bitdepth: " << calibration.bitdepth << "\n"
              << "Lens focal Length: " << calibration.lensFocalLength << "\n"
              << "Lens FStop: " << calibration.lensFStop << "\n"
              << "Pixel Pattern: " << strPixelPattern(calibration.pixelPattern) << "\n"
              << "Pixel Size: " << calibration.pixelSize << "\n";

    const auto& center = calibration.extrinsic.projectionCenter;
    const auto& rotation = calibration.extrinsic.rotation.elements;

    std::cout << "Extrinsic:\n";

    std::cout << margin(2) << "Projection center: {" << center.x << ", " << center.y << ", " << center.z << "}\n";

    std::cout << margin(2) << "Rotation matrix: {\n"
              << margin(4) << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "\n"
              << margin(4) << rotation[3] << ", " << rotation[4] << ", " << rotation[5] << "\n"
              << margin(4) << rotation[6] << ", " << rotation[7] << ", " << rotation[8] << '\n'
              << margin(2) << "}\n";
}

void print(const Blk360G2_CameraModel& cameraModel)
{
    std::cout << margin(4) << "Projection Type: " << strProjectionType(cameraModel.type) << "\n"
              << margin(6) << "Principal point: {" << cameraModel.projection.cx << ", " << cameraModel.projection.cy << "}\n"
              << margin(6) << "Focal point: {" << cameraModel.projection.fx << ", " << cameraModel.projection.fx << "}\n"
              << margin(6) << "s: " << cameraModel.projection.s << "\n"
              << margin(6) << "k1: " << cameraModel.distortion.k1 << "\n"
              << margin(6) << "k2: " << cameraModel.distortion.k2 << "\n"
              << margin(6) << "k3: " << cameraModel.distortion.k3 << "\n"
              << margin(6) << "kd1: " << cameraModel.distortion.kd1 << "\n"
              << margin(6) << "kd2: " << cameraModel.distortion.kd2 << "\n"
              << margin(6) << "kd3: " << cameraModel.distortion.kd3 << "\n"
              << margin(6) << "p1: " << cameraModel.distortion.p1 << "\n"
              << margin(6) << "p2: " << cameraModel.distortion.p2 << "\n"
              << margin(6) << "p3: " << cameraModel.distortion.p3 << "\n"
              << margin(6) << "p4: " << cameraModel.distortion.p4 << "\n";
}

void print(const Blk360G2_DeviceInfo& deviceInfo)
{
    std::cout << "Device name: " << deviceInfo.name << "\n";
    std::cout << "Serial number: " << deviceInfo.serialNumber << "\n";
    std::cout << "Article number: " << deviceInfo.articleNumber << "\n";
    std::cout << "Has VIS: " << std::boolalpha << deviceInfo.hasVIS << "\n";
    std::cout << "Firmware: " << deviceInfo.fwVersion << "\n";
    std::cout << "Hardware: " << deviceInfo.hwVersion << "\n";
}

void print(const Blk360G2_DeviceStatus& deviceStatus)
{
    std::cout << "Battery percentage: " << deviceStatus.batteryStatus.percentage << "\n";
    std::cout << "Battery alert: " << deviceStatus.batteryStatus.batteryAlert << "\n";
    std::cout << "Power source: " << deviceStatus.batteryStatus.powerSource << "\n";
    std::cout << "Free storage space: " << deviceStatus.storage.freeBytes << " bytes\n";
    std::cout << "Total storage space: " << deviceStatus.storage.totalBytes << " bytes\n";
    std::cout << "Storage alert: " << strStorageAlert(deviceStatus) << "\n";
    std::cout << "Led color: " << strLedColor(deviceStatus) << "\n";
    std::cout << "Led state: " << strLedState(deviceStatus) << "\n";
    std::cout << "Status: " << strStatus(deviceStatus) << "\n";
    if (deviceStatus.status != Blk360G2_Status_OK)
    {
        std::cout << "Error action: " << strErrorAction(deviceStatus.errorDetail) << "\n";
        std::cout << "Error code: " << strErrorCode(deviceStatus.errorDetail) << "\n";
    }

    Blk360G2_EnvironmentInfo environment = deviceStatus.environment;
    std::for_each(std::begin(environment.cpuBoardTemp), std::end(environment.cpuBoardTemp),
        [cpuBoardCount = 1](const auto& board) mutable {
            std::cout << "CPU Board " << cpuBoardCount << " temperature: " << board << "\n";
            cpuBoardCount++;
        });

    std::cout << "Conn Board temperature: " << environment.connBoardTemp << "\n";
    std::cout << "Battery temperature: " << environment.batteryTemp << "\n";
    std::cout << "lidar sys temperature: " << environment.lidar.sysTemp << "\n";
    std::cout << "lidar fpga temperature: " << environment.lidar.fpgaTemp << "\n";
    std::cout << "lidar APD temperature: " << environment.lidar.apdTemp << "\n";
    std::cout << "lidar laser temperature: " << environment.lidar.laserTemp << "\n";

    std::for_each(std::begin(environment.cpu.coreTemp), std::end(environment.cpu.coreTemp),
        [cpuCoreCount = 1](const auto& core) mutable {
            std::cout << "CPU Core " << cpuCoreCount << " temperature: " << core << "\n";
            cpuCoreCount++;
        });

    std::cout << "GPU temperature: " << environment.cpu.gpuTemp << "\n";
    std::cout << "POP memory temperature: " << environment.cpu.popMemoryTemp << "\n";
}

void print(const Blk360G2_MeasurementParameters& measurementParameters)
{
    std::cout << "Measurement parameters: ";
    if (measurementParameters.enablePointCloud)
        std::cout << "\n\t- Scan Resolution: " << strPointCloudDensity(measurementParameters.scanConfig.density);
    if (measurementParameters.enableImages)
        std::cout << "\n\t- Imaging configuration: " << strImagingMode(measurementParameters.imagingConfig.imagingMode);
    std::cout << "\n\t- Vis configuration: " << strVisMode(measurementParameters.visConfig.visMode);
    std::cout << std::endl;
}
}
