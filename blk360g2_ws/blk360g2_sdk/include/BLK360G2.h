#ifndef BLK360G2_H
#define BLK360G2_H

#ifdef _WIN32
    #define BLK360G2_SHARED __declspec(dllexport)
#else
    #define BLK360G2_SHARED __attribute__((visibility("default")))
#endif

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
    #include <stdbool.h>
#endif

#define BLK360G2_LIBRARY_VERSION 0x04010200

#if __cplusplus
extern "C" {
#endif

/****************** Type definitions ************************/

typedef uint32_t Blk360G2_version_t;
typedef uint32_t Blk360G2_ErrorCode_t;
typedef uint32_t Blk360G2_ConnectionStatus_t;
typedef uint32_t Blk360G2_ImagingMode_t;
typedef uint32_t Blk360G2_ImageDirection_t;
typedef uint32_t Blk360G2_ImageType_t;
typedef uint32_t Blk360G2_CameraPosition_t;
typedef uint32_t Blk360G2_ImageFormat_t;
typedef uint32_t Blk360G2_PixelPattern_t;
typedef uint32_t Blk360G2_ImagePlane_t;
typedef uint32_t Blk360G2_ProjectionType_t;
typedef uint32_t Blk360G2_MeasurementAccuracy_t;
typedef uint64_t Blk360G2_Handle;
typedef uint64_t Blk360G2_milliseconds_t;
typedef uint64_t Blk360G2_seconds_t;
typedef uint64_t Blk360G2_nanoseconds_t;
typedef uint64_t Blk360G2_CancelledDownload_t;
typedef uint32_t Blk360G2_percent_t;
typedef uint64_t Blk360G2_index_t;
typedef uint32_t Blk360G2_PointCloudDensity_t;
typedef uint32_t Blk360G2_Status_t;
typedef uint32_t Blk360G2_LedColor_t;
typedef uint32_t Blk360G2_LedState_t;
typedef uint32_t Blk360G2_ErrorDetail_Acknowledge_t;
typedef uint32_t Blk360G2_ErrorDetail_Reboot_t;
typedef uint32_t Blk360G2_ErrorDetail_SoftwareUpdate_t;
typedef uint32_t Blk360G2_ErrorDetail_Service_t;
typedef uint32_t Blk360G2_ErrorAction_t;
typedef float Blk360G2_celsius_t;
typedef double Blk360G2_meter_t;
typedef uint32_t Blk360G2_LinkType_t;
typedef uint32_t Blk360G2_GetImageProgressStatus_t;
typedef uint32_t Blk360G2_CaptureState_t;
typedef uint32_t Blk360G2_StorageAlert_t;
typedef uint32_t Blk360G2_ProcessingBackend_t;
typedef uint32_t Blk360G2_ImagePipeline_t;
typedef uint32_t Blk360G2_VisMode_t;
typedef uint32_t Blk360G2_StitchedPanoramaType_t;
typedef uint32_t Blk360G2_BatteryAlert_t;
typedef uint32_t Blk360G2_PowerSource_t;

/****************** Handles ************************/

BLK360G2_SHARED Blk360G2_Handle Blk360G2_Handle_Null();
BLK360G2_SHARED bool Blk360G2_Handle_IsNull(Blk360G2_Handle handle);

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_EventQueueHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SubscriptionHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SessionHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_PointCloudChunkHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_DeviceConfigWorkflowHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_MeasurementWorkflowHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_ProcessingWorkflowHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_DataManipulationWorkflowHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_ImageHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SetupEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SetupLinkEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_ImageEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SetupHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SetupLinkHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_BundleHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_BundleEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_ContainerEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_JobHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_JobEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_ThumbnailHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_TagHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_TagEnumeratorHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_StreamHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_StitcherHandle;

typedef struct
{
    Blk360G2_Handle handle;
} Blk360G2_SetupPackHandle;

/****************** Data container handles ************************/

typedef Blk360G2_ContainerEnumeratorHandle Blk360G2_CameraModel_ContainerEnumeratorHandle;
typedef Blk360G2_ContainerEnumeratorHandle Blk360G2_CalibrationIntrinsic_ContainerEnumeratorHandle;

/****************** Data objects ************************/

typedef struct
{
    bool truncateFile;
    char pathToLogFile[255 + 1];
} BLK360G2_Encoded_Logger_Config;

typedef struct
{
    bool loggerEnabled;
    BLK360G2_Encoded_Logger_Config loggerConfig;
} BLK360G2_Api_Config;

typedef struct
{
    uint8_t uuid[16];
} Blk360G2_UUID;

typedef struct
{
    char uuid[36 + 1];
} Blk360G2_StringUUID;

typedef struct
{
    Blk360G2_milliseconds_t callTimeout;
    Blk360G2_milliseconds_t sessionTimeout;
    Blk360G2_milliseconds_t statusTimeout;
    char clientName[127 + 1];
    bool forceConnection;
} Blk360G2_SessionParameters;

typedef enum
{
    Blk360G2_ConnectionStatus_OK = 0,
    Blk360G2_ConnectionStatus_ForceDisconnected = 1,
    Blk360G2_ConnectionStatus_Disconnected = 2,
    Blk360G2_ConnectionStatus_Reconnect = 3
} Blk360G2_ConnectionStatus;

typedef struct
{
    char name[127 + 1];
    char serialNumber[31 + 1];
    char articleNumber[31 + 1];
    char hwVersion[31 + 1];
    char fwVersion[31 + 1];
    bool hasVIS;
} Blk360G2_DeviceInfo;

typedef enum
{
    Blk360G2_BatteryAlert_None = 0,
    Blk360G2_BatteryAlert_OK = 1,
    Blk360G2_BatteryAlert_Low = 2,
    Blk360G2_BatteryAlert_Crit = 3
} Blk360G2_BatteryAlert;

typedef enum
{
    Blk360G2_PowerSource_None = 0,
    Blk360G2_PowerSource_Battery = 1,
    Blk360G2_PowerSource_External = 2
} Blk360G2_PowerSource;

typedef struct
{
    Blk360G2_percent_t percentage;
    Blk360G2_BatteryAlert_t batteryAlert;
    Blk360G2_PowerSource_t powerSource;
} Blk360G2_BatteryStatus;

typedef enum
{
    Blk360G2_Status_Invalid = 0,
    Blk360G2_Status_OK = 1,
    Blk360G2_Status_Error = 2,
    Blk360G2_Status_Fatal = 3,
} Blk360G2_Status;

typedef enum
{
    Blk360G2_ErrorAction_Invalid = 0,
    Blk360G2_ErrorAction_Acknowledge = 1,
    Blk360G2_ErrorAction_Reboot = 2,
    Blk360G2_ErrorAction_SoftwareUpdate = 3,
    Blk360G2_ErrorAction_Service = 4,
} Blk360G2_ErrorAction;

typedef enum
{
    Blk360G2_ErrorDetail_Acknowledge_Invalid = 0,
    Blk360G2_ErrorDetail_Acknowledge_Motorization = 200,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzPosit = 201,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzRepositionLimitReached = 202,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzCannotReachVelocity = 203,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzConstantVelocity = 204,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzTimeout = 205,
    Blk360G2_ErrorDetail_Acknowledge_MotorizationAzInvalidStream = 206,
    Blk360G2_ErrorDetail_Acknowledge_Edm = 300,
    Blk360G2_ErrorDetail_Acknowledge_EdmDustDirtDetected = 301,
    Blk360G2_ErrorDetail_Acknowledge_Vis = 400,
    Blk360G2_ErrorDetail_Acknowledge_VisStarting = 401,
    Blk360G2_ErrorDetail_Acknowledge_VisProcessing = 402,
    Blk360G2_ErrorDetail_Acknowledge_VisStopping = 403,
    Blk360G2_ErrorDetail_Acknowledge_VisResultMissing = 404,
    Blk360G2_ErrorDetail_Acknowledge_Imaging = 500,
    Blk360G2_ErrorDetail_Acknowledge_ImagingMipiCalibration = 501,
    Blk360G2_ErrorDetail_Acknowledge_ImagingAcquisitionIrq = 502,
    Blk360G2_ErrorDetail_Acknowledge_ImagingIspIrq = 503,
    Blk360G2_ErrorDetail_Acknowledge_ImagingCorruptImage = 504,
    Blk360G2_ErrorDetail_Acknowledge_ImagingHistogram = 505,
    Blk360G2_ErrorDetail_Acknowledge_ImagingSetup = 506,
    Blk360G2_ErrorDetail_Acknowledge_ImagingUnsynchronizedVideo = 507,
    Blk360G2_ErrorDetail_Acknowledge_ImagingEncoding = 508,
    Blk360G2_ErrorDetail_Acknowledge_ImagingStitching = 509,
    Blk360G2_ErrorDetail_Acknowledge_ImagingCapture = 510,
    Blk360G2_ErrorDetail_Acknowledge_ImagingNoImage = 511,
    Blk360G2_ErrorDetail_Acknowledge_ImagingTimeout = 512,
    Blk360G2_ErrorDetail_Acknowledge_Scan = 600,
    Blk360G2_ErrorDetail_Acknowledge_ScanTimeout = 601,
    Blk360G2_ErrorDetail_Acknowledge_Command = 700,
    Blk360G2_ErrorDetail_Acknowledge_Storage = 1100,
    Blk360G2_ErrorDetail_Acknowledge_StorageNoSpace = 1101,
    Blk360G2_ErrorDetail_Acknowledge_StorageWriteFile = 1102,
    Blk360G2_ErrorDetail_Acknowledge_StorageWriteDb = 1103,
    Blk360G2_ErrorDetail_Acknowledge_StorageDbRepaired = 1104,
    Blk360G2_ErrorDetail_Acknowledge_StorageRemoveDataset = 1105,
    Blk360G2_ErrorDetail_Acknowledge_Tilt = 1200,
    Blk360G2_ErrorDetail_Acknowledge_TiltCalculationFailed = 1201,
    Blk360G2_ErrorDetail_Acknowledge_System = 2000,
    Blk360G2_ErrorDetail_Acknowledge_SystemSuddenShutdown = 2001,
} Blk360G2_ErrorDetail_Acknowledge;

typedef enum
{
    Blk360G2_ErrorDetail_Reboot_Invalid = 0,
    Blk360G2_ErrorDetail_Reboot_Imu = 100,
    Blk360G2_ErrorDetail_Reboot_ImuDeviceUnavailable = 101,
    Blk360G2_ErrorDetail_Reboot_Motorization = 200,
    Blk360G2_ErrorDetail_Reboot_MotorizationSensorUnavailable = 201,
    Blk360G2_ErrorDetail_Reboot_MotorizationAzAxis = 202,
    Blk360G2_ErrorDetail_Reboot_MotorizationElAxis = 203,
    Blk360G2_ErrorDetail_Reboot_Edm = 300,
    Blk360G2_ErrorDetail_Reboot_EdmHeadUnavailable = 301,
    Blk360G2_ErrorDetail_Reboot_EdmNoScanPointsReceived = 302,
    Blk360G2_ErrorDetail_Reboot_EdmAzAngle = 303,
    Blk360G2_ErrorDetail_Reboot_EdmElAngle = 304,
    Blk360G2_ErrorDetail_Reboot_EdmLaserSafety = 305,
    Blk360G2_ErrorDetail_Reboot_Vis = 400,
    Blk360G2_ErrorDetail_Reboot_VisSensorInit = 401,
    Blk360G2_ErrorDetail_Reboot_Imaging = 500,
    Blk360G2_ErrorDetail_Reboot_ImagingAcquisitionIrq = 501,
    Blk360G2_ErrorDetail_Reboot_ImagingIspIrq = 502,
    Blk360G2_ErrorDetail_Reboot_ImagingWrongDataType = 503,
    Blk360G2_ErrorDetail_Reboot_ImagingHistogram = 504,
    Blk360G2_ErrorDetail_Reboot_ImagingSensorUnavailable = 505,
    Blk360G2_ErrorDetail_Reboot_Led = 600,
    Blk360G2_ErrorDetail_Reboot_LedUnavailable = 601,
    Blk360G2_ErrorDetail_Reboot_Button = 700,
    Blk360G2_ErrorDetail_Reboot_ButtonUnavailable = 701,
    Blk360G2_ErrorDetail_Reboot_Fan = 800,
    Blk360G2_ErrorDetail_Reboot_FanUnavailable = 801,
    Blk360G2_ErrorDetail_Reboot_Usb = 900,
    Blk360G2_ErrorDetail_Reboot_UsbUnavailable = 901,
    Blk360G2_ErrorDetail_Reboot_System = 2000,
    Blk360G2_ErrorDetail_Reboot_SystemTemperatureSensorUnavailable = 2001,
    Blk360G2_ErrorDetail_Reboot_Wifi = 3000,
    Blk360G2_ErrorDetail_Reboot_Wifi_Access_Point_Unavailable = 3001,
} Blk360G2_ErrorDetail_Reboot;

typedef enum
{
    Blk360G2_ErrorDetail_SoftwareUpdate_Invalid = 0,
    Blk360G2_ErrorDetail_SoftwareUpdate_EdmFpgaVersionMismatch = 301,
    Blk360G2_ErrorDetail_SoftwareUpdate_MainFpgaVersionMismatch = 501,
    Blk360G2_ErrorDetail_SoftwareUpdate_SystemMissingDriver = 2001,
    Blk360G2_ErrorDetail_SoftwareUpdate_StorageDatabaseSchemaMismatch = 1101,
} Blk360G2_ErrorDetail_SoftwareUpdate;

typedef enum
{
    Blk360G2_ErrorDetail_Service_Invalid = 0,
    Blk360G2_ErrorDetail_Service_Calibration = 1000,
    Blk360G2_ErrorDetail_Service_CalibrationMissing = 1001,
    Blk360G2_ErrorDetail_Service_CalibrationCorrupted = 1002,
    Blk360G2_ErrorDetail_Service_Storage = 1100,
    Blk360G2_ErrorDetail_Service_StorageCorruptDatabase = 1101,
    Blk360G2_ErrorDetail_Service_System = 2000,
    Blk360G2_ErrorDetail_Service_SystemCorruptFilesystem = 2001,
    Blk360G2_ErrorDetail_Service_SystemMissingSerial = 2002,
} Blk360G2_ErrorDetail_Service;

typedef union
{
    Blk360G2_ErrorDetail_Acknowledge_t acknowledge;
    Blk360G2_ErrorDetail_Reboot_t reboot;
    Blk360G2_ErrorDetail_SoftwareUpdate_t softwareUpdate;
    Blk360G2_ErrorDetail_Service_t service;
} Blk360G2_ErrorDetailCode;

typedef struct
{
    Blk360G2_ErrorAction_t action;
    Blk360G2_ErrorDetailCode code;
} Blk360G2_ErrorDetail;

typedef struct
{
    Blk360G2_celsius_t sysTemp;
    Blk360G2_celsius_t fpgaTemp;
    Blk360G2_celsius_t apdTemp;
    Blk360G2_celsius_t laserTemp;
} Blk360G2_LidarTemperatures;

typedef struct
{
    Blk360G2_celsius_t coreTemp[4];
    Blk360G2_celsius_t gpuTemp;
    Blk360G2_celsius_t popMemoryTemp;
} Blk360G2_CpuTemperatures;

typedef struct
{
    Blk360G2_celsius_t cpuBoardTemp[3];
    Blk360G2_celsius_t connBoardTemp;
    Blk360G2_celsius_t batteryTemp;
    Blk360G2_LidarTemperatures lidar;
    Blk360G2_CpuTemperatures cpu;
} Blk360G2_EnvironmentInfo;

typedef enum
{
    Blk360G2_StorageAlert_Invalid = 0,
    Blk360G2_StorageAlert_OK = 1,
    Blk360G2_StorageAlert_Low = 2,
    Blk360G2_StorageAlert_Crit = 3
} Blk360G2_StorageAlert;

typedef struct
{
    uint64_t totalBytes;
    uint64_t freeBytes;
    Blk360G2_StorageAlert_t storageAlert;
} Blk360G2_StorageStatus;

typedef enum
{
    Blk360G2_LedColor_None = 0,
    Blk360G2_LedColor_Red = 1,
    Blk360G2_LedColor_Green = 2,
    Blk360G2_LedColor_Yellow = 3,
    Blk360G2_LedColor_Blue = 4,
    Blk360G2_LedColor_Orange = 5,
    Blk360G2_LedColor_White = 6,
    Blk360G2_LedColor_Off = 7,
    Blk360G2_LedColor_GreenBright = 8,
    Blk360G2_LedColor_GreenDark = 9,
    Blk360G2_LedColor_YellowBrigth = 10,
    Blk360G2_LedColor_YellowDark = 11
} Blk360G2_LedColor;

typedef enum
{
    Blk360G2_LedState_None = 0,
    Blk360G2_LedState_Off = 1,
    Blk360G2_LedState_Pulsating = 2,
    Blk360G2_LedState_Blinking = 3,
    Blk360G2_LedState_On = 4,
    Blk360G2_LedState_Progress = 5,
    Blk360G2_LedState_Spinning = 6
} Blk360G2_LedState;

typedef struct
{
    Blk360G2_LedColor_t color;
    Blk360G2_LedState_t state;
} Blk360G2_LedStatus;

typedef struct
{
    Blk360G2_BatteryStatus batteryStatus;
    Blk360G2_Status_t status;
    Blk360G2_ErrorDetail errorDetail;
    Blk360G2_EnvironmentInfo environment;
    Blk360G2_StorageStatus storage;
    Blk360G2_LedStatus ledStatus;
} Blk360G2_DeviceStatus;

typedef enum
{
    Blk360G2_PointCloudDensity_Invalid = 0,
    Blk360G2_PointCloudDensity_High = 1,
    Blk360G2_PointCloudDensity_Medium = 2,
    Blk360G2_PointCloudDensity_Low = 3,
    Blk360G2_PointCloudDensity_Ultralow = 4,
} Blk360G2_PointCloudDensity;

typedef enum
{
    Blk360G2_ImageType_Invalid = 0,
    Blk360G2_ImageType_Raw = 1,
    Blk360G2_ImageType_LDR = 2,
    Blk360G2_ImageType_LDR_Pano = 3,
    Blk360G2_ImageType_HDR = 4,
    Blk360G2_ImageType_HDR_Pano = 5,
    Blk360G2_ImageType_LDR_Fulldome = 6,
    Blk360G2_ImageType_HDR_Fulldome = 7,

} Blk360G2_ImageType;

typedef enum
{
    Blk360G2_ImagingMode_Invalid = 0,
    Blk360G2_ImagingMode_LDR = 1,
    Blk360G2_ImagingMode_HDR = 2,
} Blk360G2_ImagingMode;

typedef enum
{
    Blk360G2_VisMode_Invalid = 0,
    Blk360G2_VisMode_Disabled = 1,
    Blk360G2_VisMode_Enabled = 2,
} Blk360G2_VisMode;

typedef struct
{
    Blk360G2_PointCloudDensity_t density;
} Blk360G2_ScanConfig;

typedef struct
{
    Blk360G2_ImagingMode_t imagingMode;
} Blk360G2_ImagingConfig;

typedef struct
{
    Blk360G2_VisMode_t visMode;
} Blk360G2_VisConfig;

typedef struct
{
    Blk360G2_ScanConfig scanConfig;
    Blk360G2_ImagingConfig imagingConfig;
    Blk360G2_VisConfig visConfig;
    bool enablePointCloud;
    bool enableImages;
} Blk360G2_MeasurementParameters;

typedef enum
{
    Blk360G2_ProcessingBackend_CPU = 0,
    Blk360G2_ProcessingBackend_GPU = 1,
} Blk360G2_ProcessingBackend;

typedef struct
{
    Blk360G2_ProcessingBackend_t backend;
} Blk360G2_ProcessingParameters;

typedef struct
{
    Blk360G2_seconds_t selfTimer;
    Blk360G2_MeasurementParameters measurementParameters;
} Blk360G2_ButtonPressConfig;

typedef enum
{
    Blk360G2_ImageDirection_Invalid = 0,
    Blk360G2_ImageDirection_Front = 1,
    Blk360G2_ImageDirection_Back = 2,
} Blk360G2_ImageDirection;

typedef enum
{
    Blk360G2_ImageFormat_Invalid = 0,
    Blk360G2_ImageFormat_DNL = 1,
    Blk360G2_ImageFormat_JPG = 2,
    Blk360G2_ImageFormat_JXR = 3,
    Blk360G2_ImageFormat_DNG = 4,
    Blk360G2_ImageFormat_RGB_FLOAT = 5,
    Blk360G2_ImageFormat_RGB_UINT8 = 6,
    Blk360G2_ImageFormat_RGB_UINT16 = 7
} Blk360G2_ImageFormat;

typedef enum
{
    Blk360G2_CameraPosition_Invalid = 0,
    Blk360G2_CameraPosition_RightBottom = 1,
    Blk360G2_CameraPosition_RightTop = 2,
    Blk360G2_CameraPosition_LeftTop = 3,
    Blk360G2_CameraPosition_LeftBottom = 4,
} Blk360G2_CameraPosition;

typedef struct
{
    Blk360G2_UUID uuid;
    Blk360G2_nanoseconds_t timestamp;
    uint64_t dataSize;
    Blk360G2_ImageDirection_t imageDirection;
    Blk360G2_ImageType_t imageType;
    Blk360G2_CameraPosition_t cameraPosition;
    Blk360G2_ImageFormat_t imageFormat;
    uint32_t imageWidth;
    uint32_t imageHeight;
    double azimuthRad;
    double exposure;
} Blk360G2_ImageMetadata;

typedef enum
{
    Blk360G2_PixelPattern_INVALID = 0,
    Blk360G2_PixelPattern_BAYER_GRBG = 1,
    Blk360G2_PixelPattern_BAYER_RGGB = 2,
    Blk360G2_PixelPattern_BAYER_BGGR = 3,
    Blk360G2_PixelPattern_BAYER_GBRG = 4,
    Blk360G2_PixelPattern_GRAY = 5,
    Blk360G2_PixelPattern_COLOR_RGBA = 6,
    Blk360G2_PixelPattern_COLOR_RGB = 7,
    Blk360G2_PixelPattern_COLOR_BGR = 8,
    Blk360G2_PixelPattern_COLOR_RGB_FLOAT = 9,
} Blk360G2_PixelPattern;

typedef enum
{
    Blk360G2_ImagePlane_INVALID = 0,
    Blk360G2_ImagePlane_GRAY = 1,
    Blk360G2_ImagePlane_BAYER_CHANNEL_0 = 2,
    Blk360G2_ImagePlane_BAYER_CHANNEL_1 = 3,
    Blk360G2_ImagePlane_BAYER_CHANNEL_2 = 4,
    Blk360G2_ImagePlane_BAYER_CHANNEL_3 = 5,
    Blk360G2_ImagePlane_COLOR_CHANNEL_0 = 6,
    Blk360G2_ImagePlane_COLOR_CHANNEL_1 = 7,
    Blk360G2_ImagePlane_COLOR_CHANNEL_2 = 8,
    Blk360G2_ImagePlane_COLOR_CHANNEL_3 = 9,
} Blk360G2_ImagePlane;

typedef enum
{
    Blk360G2_ProjectionType_INVALID = 0,
    Blk360G2_ProjectionType_PINHOLE = 1,
} Blk360G2_ProjectionType;

typedef struct
{
    double fx;
    double fy;
    double cx;
    double cy;
    double s;
} Blk360G2_ProjectionParameters;

typedef struct
{
    double k1;
    double k2;
    double k3;
    double kd1;
    double kd2;
    double kd3;
    double p1;
    double p2;
    double p3;
    double p4;
} Blk360G2_DistortionParameters;

typedef struct
{
    Blk360G2_ProjectionType_t type;
    Blk360G2_ProjectionParameters projection;
    Blk360G2_DistortionParameters distortion;
} Blk360G2_CameraModel;

typedef struct
{
    Blk360G2_ImagePlane_t imagePlane;
    Blk360G2_CameraModel_ContainerEnumeratorHandle modelsHandle;
} Blk360G2_CalibrationIntrinsic;

typedef struct
{
    char id[40 + 1];
} Blk360G2_Transaction;

typedef struct
{
    // Column major arrangement
    double elements[3 * 3];
} Blk360G2_Matrix3;

typedef struct
{
    // Column major arrangement
    double elements[4 * 4];
} Blk360G2_Matrix4;

typedef struct
{
    double x;
    double y;
    double z;
} Blk360G2_Point;

typedef struct
{
    Blk360G2_Point projectionCenter; // in meters
    Blk360G2_Matrix3 rotation;
} Blk360G2_CalibrationExtrinsic;

typedef struct
{
    Blk360G2_meter_t pixelSize;
    uint32_t bitdepth;
    Blk360G2_PixelPattern_t pixelPattern;

    Blk360G2_meter_t lensFocalLength;
    double lensFStop;

    Blk360G2_CalibrationIntrinsic_ContainerEnumeratorHandle intrinsicHandle;
    Blk360G2_CalibrationExtrinsic extrinsic;
} Blk360G2_CameraCalibration;

typedef struct
{
    // 4x4 camera projection matrix(16 values). Column - major; focal length is
    // relative to image plane of size 2x2 units, principal point offset is
    // relative to center with Y pointing up(flip sign if going from OpenCV).
    Blk360G2_Matrix4 projection;

    // 4x4 camera transformation matrix(16 values, rotation + translation).
    // Column - major; X points to the right, Y up and Z backward; translation is
    // in meters.
    Blk360G2_Matrix4 transformation;
} Blk360G2_ImageCalibration;

typedef struct
{
    Blk360G2_Matrix4 deltaPose;
    Blk360G2_Matrix3 positionUncertainty;
    Blk360G2_Matrix3 orientationUncertainty;
    Blk360G2_percent_t quality;
} Blk360G2_VisPose;

typedef enum
{
    Blk360G2_MeasurementAccuracy_Invalid = 0,
    Blk360G2_MeasurementAccuracy_Unset = 1,
    Blk360G2_MeasurementAccuracy_Inaccurate = 2,
    Blk360G2_MeasurementAccuracy_Accurate = 3,
} Blk360G2_MeasurementAccuracy;

typedef struct
{
    Blk360G2_VisPose pose;
    Blk360G2_UUID sourceSetupUuid;
    Blk360G2_MeasurementAccuracy_t accuracy;
} Blk360G2_VisResult;

typedef enum
{
    Blk360G2_StitchedPanoramaType_Invalid = 0,
    Blk360G2_StitchedPanoramaType_Halfdome = 1,
    Blk360G2_StitchedPanoramaType_Fulldome = 2,
} Blk360G2_StitchedPanoramaType;

typedef struct
{
    uint32_t horizontal;
    uint32_t vertical;
} Blk360G2_PlanarResolution;

typedef struct
{
    Blk360G2_UUID uuid;
    Blk360G2_UUID thumbnailUuid;
    Blk360G2_nanoseconds_t created;
    Blk360G2_nanoseconds_t updated;
    uint32_t version;

    bool hasThumbnail;
} Blk360G2_JobMetadata;

typedef struct
{
    Blk360G2_UUID uuid;
    uint32_t version;
    Blk360G2_nanoseconds_t created;
} Blk360G2_BundleMetadata;

typedef struct
{
    Blk360G2_SetupHandle setup;
    Blk360G2_StitchedPanoramaType_t panoramaType;
} Blk360G2_StitcherConfiguration;

typedef struct
{
    char name[255 + 1];
} Blk360G2_MimeType;

typedef struct
{
    Blk360G2_UUID uuid;
    Blk360G2_MimeType type;
    uint32_t version;
} Blk360G2_ThumbnailMetadata;

typedef struct
{
    Blk360G2_MimeType type;
    uint32_t version;
} Blk360G2_BlobHeader;

typedef struct
{
    Blk360G2_UUID uuid;
    Blk360G2_Point position;
    Blk360G2_UUID thumbnailUuid;
    Blk360G2_BlobHeader blobHeader;
    uint32_t version;
    Blk360G2_nanoseconds_t created;

    bool hasPosition;
    bool hasThumbnail;
    bool hasBlob;
} Blk360G2_TagMetadata;

typedef enum
{
    BLK360G2_ImagePipeline_Invalid = 0,
    BLK360G2_ImagePipeline_Field = 1,
    BLK360G2_ImagePipeline_Office = 2,
    BLK360G2_ImagePipeline_Both = 3,
} Blk360G2_ImagePipeline;

typedef struct
{
    Blk360G2_ImagePipeline_t imagePipeline;
} Blk360G2_LoadImagesFromFileOptions;

typedef struct
{
    bool withPointCloud;
    bool withImages;
    Blk360G2_ImagePipeline_t imagePipeline;
} Blk360G2_SetupPack_SaveOptions;

typedef struct
{
    Blk360G2_ImagePipeline_t imagePipeline;
} Blk360G2_SetupPack_LoadImagesOptions;

/****************** Error Handling ************************/

enum Blk360G2_ErrorCode
{
    /// General API errors
    Blk360G2_Error_Ok = 0x00000000,
    Blk360G2_Error_ApiNotInitialized = 0x00000001,
    Blk360G2_Error_IncompatibleVersion = 0x00000002,
    /// Invalid API operations
    Blk360G2_Error_InvalidOperation = 0x00010000,
    Blk360G2_Error_InvalidOperation_ApiAlreadyInitialized = 0x00010001,
    Blk360G2_Error_InvalidOperation_DataDownloading = 0x00010002,
    Blk360G2_Error_InvalidOperation_DataAlreadyDownloaded = 0x00010003,
    Blk360G2_Error_InvalidOperation_DataProcessing = 0x00010004,
    Blk360G2_Error_InvalidOperation_DataAlreadyProcessed = 0x00010005,
    Blk360G2_Error_InvalidOperation_DataNotDownloaded = 0x00010006,
    Blk360G2_Error_InvalidOperation_DataNotProcessed = 0x00010007,
    Blk360G2_Error_InvalidOperation_UnsupportedImageFormat = 0x00010008,
    Blk360G2_Error_InvalidOperation_EventQueueEmpty = 0x00010009,
    Blk360G2_Error_InvalidOperation_SetupReleased = 0x0001000A,
    Blk360G2_Error_InvalidOperation_ChunkReleased = 0x0001000B,
    Blk360G2_Error_InvalidOperation_PointCloudProcessingAlreadyStarted = 0x0001000C,
    Blk360G2_Error_InvalidOperation_ImageReleased = 0x0001000D,
    Blk360G2_Error_InvalidOperation_NotImplemented = 0x0001000E,
    Blk360G2_Error_InvalidOperation_WrongContainerType = 0x0001000F,
    Blk360G2_Error_InvalidOperation_HdrBracketsNotAvailable = 0x00010010,
    Blk360G2_Error_InvalidOperation_CommitTransactionError = 0x00010011,
    Blk360G2_Error_InvalidOperation_InvalidCameraCalibration = 0x00010012,
    Blk360G2_Error_InvalidOperation_NoData = 0x00010013,
    Blk360G2_Error_InvalidOperation_StitcherComplete = 0x00010014,
    Blk360G2_Error_InvalidOperation_SingleHalfdomeStitcherAllowed = 0x00010015,
    Blk360G2_Error_InvalidOperation_DataUploading = 0x00010016,
    Blk360G2_Error_InvalidOperation_DataNotUploaded = 0x00010017,
    Blk360G2_Error_InvalidOperation_InvalidSetupState = 0x00010018,
    Blk360G2_Error_InvalidOperation_NotSupported = 0x00010019,
    Blk360G2_Error_InvalidOperation_StreamCancelled = 0x0001001A,
    Blk360G2_Error_InvalidOperation_VisStreamAlreadyStarted = 0x0001001B,
    /// Invalid API parameter
    Blk360G2_Error_InvalidInstanceHandle = 0x00020000,
    Blk360G2_Error_InvalidInstanceHandle_Workflow = 0x00020001,
    Blk360G2_Error_InvalidInstanceHandle_Session = 0x00020002,
    Blk360G2_Error_InvalidInstanceHandle_Enumerator = 0x00020003,
    Blk360G2_Error_InvalidInstanceHandle_EventQueue = 0x00020004,
    Blk360G2_Error_InvalidInstanceHandle_Setup = 0x00020005,
    Blk360G2_Error_InvalidInstanceHandle_PointCloudChunk = 0x00020006,
    Blk360G2_Error_InvalidInstanceHandle_Image = 0x00020007,
    Blk360G2_Error_InvalidInstanceHandle_ContainerEnumerator = 0x00020008,
    Blk360G2_Error_InvalidInstanceHandle_Job = 0x00020009,
    Blk360G2_Error_InvalidInstanceHandle_Subscription = 0x0002000A,
    Blk360G2_Error_InvalidInstanceHandle_Bundle = 0x0002000B,
    Blk360G2_Error_InvalidInstanceHandle_Thumbnail = 0x0002000C,
    Blk360G2_Error_InvalidInstanceHandle_Tag = 0x0002000D,
    Blk360G2_Error_InvalidInstanceHandle_SetupLink = 0x0002000E,
    Blk360G2_Error_InvalidInstanceHandle_Stream = 0x0002000F,
    Blk360G2_Error_InvalidInstanceHandle_Stitcher = 0x00020010,
    Blk360G2_Error_InvalidInstanceHandle_SetupPack = 0x00020011,
    /// Invalid input parameter
    Blk360G2_Error_InvalidArgument = 0x00040000,
    Blk360G2_Error_InvalidArgument_EnumOutOfRange = 0x00040001,
    Blk360G2_Error_InvalidArgument_WrongFormat = 0x00040002,
    Blk360G2_Error_InvalidArgument_Empty = 0x00040003,
    Blk360G2_Error_InvalidArgument_WrongImageType = 0x00040004,
    Blk360G2_Error_InvalidArgument_MissingNullTerminator = 0x00040005,
    Blk360G2_Error_InvalidArgument_WrongStitcherType = 0x00040006,
    Blk360G2_Error_InvalidArgument_ImageAlreadyAdded = 0x00040007,
    Blk360G2_Error_InvalidArgument_DataTooLarge = 0x00040008,
    Blk360G2_Error_InvalidArgument_WrongMeasurementParameters = 0x00040009,
    Blk360G2_Error_InvalidArgument_IncompleteSetup = 0x0004000A,
    Blk360G2_Error_InvalidArgument_NothingSelectedToDownload = 0x0004000B,
    Blk360G2_Error_InvalidArgument_Null = 0x0004000C,
    Blk360G2_Error_InvalidArgument_FileNotFound = 0x0004000D,
    Blk360G2_Error_InvalidArgument_WrongImagePipeline = 0x0004000E,
    /// Internal errors
    Blk360G2_Error_InternalError = 0x00080000,
    Blk360G2_Error_InternalError_MemoryAllocation = 0x00080001,
    Blk360G2_Error_InternalError_InvalidAsynchronousOperation = 0x00080002,
    Blk360G2_Error_InternalError_InvalidDataAccess = 0x00080003,
    Blk360G2_Error_InternalError_HandleAssignment = 0x00080004,
    Blk360G2_Error_InternalError_SessionError = 0x00080005,
    Blk360G2_Error_InternalError_InvalidEvent = 0x00080006,
    Blk360G2_Error_InternalError_ProcessingInitFailed = 0x00080007,
    Blk360G2_Error_InternalError_ProcessingFailed = 0x00080008,
    Blk360G2_Error_InternalError_GPUMemoryAllocation = 0x00080009,
    Blk360G2_Error_InternalError_DataSizeMismatch = 0x0008000A,
    Blk360G2_Error_InternalError_DatabaseError = 0x0008000B,
    Blk360G2_Error_InternalError_FileAccess = 0x0008000C,
    Blk360G2_Error_InternalError_OfflineSession = 0x0008000D,
    Blk360G2_Error_InternalError_Unexpected = 0x000800FF,
    /// Device errors
    Blk360G2_Error_DeviceError = 0x00100000,
    Blk360G2_Error_DeviceError_Occupied = 0x00100001,
    Blk360G2_Error_DeviceError_ResourceExhausted = 0x00100002,
    Blk360G2_Error_DeviceError_Timeout = 0x00100003,
    Blk360G2_Error_DeviceError_Unavailable = 0x00100004,
    Blk360G2_Error_DeviceError_InvalidArgument = 0x00100005,
    Blk360G2_Error_DeviceError_Unimplemented = 0x00100006,
    Blk360G2_Error_DeviceError_InvalidMetadata = 0x00100007,
    Blk360G2_Error_DeviceError_InvalidResponse = 0x00100008,
    Blk360G2_Error_DeviceError_Measurement = 0x00100009,
    Blk360G2_Error_DeviceError_HalfTurn = 0x0010000A,
    Blk360G2_Error_DeviceError_MissingHDRBracket = 0x0010000B,
    Blk360G2_Error_DeviceError_Aborted = 0x0010000C,
    Blk360G2_Error_DeviceError_PermissionDenied = 0x0010000D,
    Blk360G2_Error_DeviceError_Unexpected = 0x001000FF,
};

typedef struct
{
    Blk360G2_ErrorCode_t code;
    char message[251 + 1];
} Blk360G2_Error;

typedef struct
{
    uint32_t abi;
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
} Blk360G2_Version;

typedef enum
{
    Blk360G2_LinkType_Invalid = 0,
    Blk360G2_LinkType_Manual = 1,
    Blk360G2_LinkType_ICPChecked = 2,
} Blk360G2_LinkType;

typedef enum
{
    Blk360G2_GetImageProgressStatus_Invalid = 0,
    Blk360G2_GetImageProgressStatus_Downloading = 1,
    Blk360G2_GetImageProgressStatus_Processing = 2,
    Blk360G2_GetImageProgressStatus_Done = 3,
} Blk360G2_GetImageProgressStatus;

typedef enum
{
    Blk360G2_CaptureState_Invalid = 0,
    Blk360G2_CaptureState_InProgress = 1,
    Blk360G2_CaptureState_Completed = 2,
    Blk360G2_CaptureState_Aborted = 3,
    Blk360G2_CaptureState_Cancelled = 4,
} Blk360G2_CaptureState;

typedef struct
{
    Blk360G2_UUID targetSetupUuid;
    Blk360G2_LinkType_t type;
} Blk360G2_SetupLink;

typedef struct
{
    Blk360G2_UUID uuid;
    Blk360G2_UUID thumbnailUuid;
    bool hasThumbnail;

    Blk360G2_nanoseconds_t created;
    Blk360G2_nanoseconds_t updated;

    bool readOnly;

    uint64_t size;
    Blk360G2_PointCloudDensity_t scanResolution;
    Blk360G2_PlanarResolution scanRaster;
    bool hasVisResult;
    Blk360G2_VisResult visResult;
    Blk360G2_ImagingMode_t imagingMode;
    Blk360G2_CaptureState_t captureState;

    Blk360G2_Matrix4 pose;
    bool hasPose;
    Blk360G2_Matrix4 tiltMatrix;
    uint32_t version;
} Blk360G2_SetupMetadata;

typedef struct
{
    Blk360G2_UUID sourceSetupUuid;
    Blk360G2_UUID targetSetupUuid;
    Blk360G2_LinkType_t type;
} Blk360G2_SetupLinkMetadata;

typedef enum
{
    Blk360G2_CancelledDownload_PointCloud = 1,
    Blk360G2_CancelledDownload_Image = 3,
    Blk360G2_CancelledDownload_SetupToFile = 4,
} Blk360G2_CancelledDownload;

BLK360G2_SHARED Blk360G2_Error Blk360G2_Api_GetLastError();

/****************** API ************************/

BLK360G2_SHARED void Blk360G2_Api_New(Blk360G2_version_t version);
BLK360G2_SHARED void Blk360G2_Api_New_With_Config(Blk360G2_version_t version, BLK360G2_Api_Config config);
BLK360G2_SHARED void Blk360G2_Api_Release();

/****************** Events ************************/

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_ErrorCode_t errorCode;
    char message[244];
} Blk360G2_ErrorEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_ConnectionStatus_t status;
} Blk360G2_ConnectionStateChangedEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    uint64_t downloadedSize;
    uint64_t totalSize;
} Blk360G2_PointCloudDownloadProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_CancelledDownload_t cancelledAction;
} Blk360G2_DownloadCancelEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_DeviceStatus deviceStatus;
} Blk360G2_DeviceStatusEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_percent_t progress;
    Blk360G2_UUID setupUuid;
} Blk360G2_MeasurementProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_UUID setupUuid;
} Blk360G2_SetupStartedEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_UUID setupUuid;
} Blk360G2_MeasurementCancelledEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_percent_t progress;
} Blk360G2_RotationProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_PointCloudChunkHandle handle;
    Blk360G2_index_t chunkIndex;
    Blk360G2_index_t totalChunks;
} Blk360G2_PointCloudChunkAvailableEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_percent_t progress;
} Blk360G2_PointCloudProcessProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_ImageHandle imageHandle;
    Blk360G2_percent_t progress;
} Blk360G2_ImageDownloadProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_nanoseconds_t timestamp;
} Blk360G2_VisStartedEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_TagHandle tagHandle;
    uint64_t downloadedSize;
    uint64_t totalSize;
} Blk360G2_BlobDownloadProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_nanoseconds_t timestamp;
} Blk360G2_VisStoppedEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_nanoseconds_t timestamp;
    Blk360G2_ErrorCode_t errorCode;
    char message[235 + 1];
} Blk360G2_VisErrorEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_nanoseconds_t timestamp;
    Blk360G2_Matrix4 deltaPose;
    bool deviceMoving;
    Blk360G2_UUID sourceSetupUuid;
} Blk360G2_VisRunningEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_TagHandle tagHandle;
} Blk360G2_BlobDownloadCancelEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_UUID setupUuid;
    Blk360G2_ImageMetadata imageMetadata;
} Blk360G2_ImageCapturedEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_TagHandle tagHandle;
    uint64_t uploadedSize;
    uint64_t totalSize;
} Blk360G2_BlobUploadProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_TagHandle tagHandle;
} Blk360G2_BlobUploadCancelEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_ImageHandle imageHandle;
    Blk360G2_GetImageProgressStatus_t progressStatus;
    Blk360G2_percent_t progress;
} Blk360G2_GetImageProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
} Blk360G2_PointCloudProcessCancelEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_GetImageProgressStatus_t progressStatus;
    Blk360G2_percent_t progress;
} Blk360G2_GetOfficeImagesProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_SetupHandle setupHandle;
    Blk360G2_ImageHandle imageHandle;
    uint32_t imageNumber;
    uint32_t totalImages;
} Blk360G2_ImageReadyEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    uint64_t totalImages;
} Blk360G2_LoadingImagesFromFileSuccessEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;

    uint64_t currentImage;
    uint64_t totalImages;

    uint64_t currentByte;
    uint64_t totalBytes;
} Blk360G2_SaveImagesToFileEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_StitcherHandle stitcher;
    Blk360G2_percent_t progress;
    Blk360G2_ImageHandle stitchedImage;
} Blk360G2_StitchingProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;

    uint64_t currentSetup;
    uint64_t totalSetups;

    Blk360G2_SetupHandle setupHandle;
} Blk360G2_SetupPackSaveProgressEvent;

typedef struct
{
    Blk360G2_SubscriptionHandle sender;
    uint64_t totalImages;
} Blk360G2_SetupPackLoadingImagesSuccessEvent;

typedef union
{
    Blk360G2_SubscriptionHandle sender;
    Blk360G2_ConnectionStateChangedEvent connectionStatusChanged;
    Blk360G2_PointCloudDownloadProgressEvent pointCloudDownloadProgress;
    Blk360G2_PointCloudChunkAvailableEvent pointCloudChunkAvailable;
    Blk360G2_PointCloudProcessProgressEvent pointCloudProcessProgress;
    Blk360G2_PointCloudProcessCancelEvent pointCloudProcessCancelled;
    Blk360G2_ImageDownloadProgressEvent imageDownloadProgress;
    Blk360G2_DownloadCancelEvent downloadCancelled;
    Blk360G2_DeviceStatusEvent deviceStatus;
    Blk360G2_MeasurementCancelledEvent measurementCancelled;
    Blk360G2_MeasurementProgressEvent measurementProgress;
    Blk360G2_SetupStartedEvent setupStarted;
    Blk360G2_RotationProgressEvent rotationProgress;
    Blk360G2_BlobDownloadProgressEvent blobDownloadProgress;
    Blk360G2_BlobDownloadCancelEvent blobDownloadCancelled;
    Blk360G2_BlobUploadProgressEvent blobUploadProgress;
    Blk360G2_BlobUploadCancelEvent blobUploadCancelled;
    Blk360G2_ErrorEvent error;
    Blk360G2_VisStartedEvent visStarted;
    Blk360G2_VisStoppedEvent visStopped;
    Blk360G2_VisErrorEvent visError;
    Blk360G2_VisRunningEvent visRunning;
    Blk360G2_ImageCapturedEvent imageCaptured;
    Blk360G2_GetImageProgressEvent getImageProgress;
    Blk360G2_LoadingImagesFromFileSuccessEvent loadingImagesFromFileSuccess;
    Blk360G2_GetOfficeImagesProgressEvent getOfficeImagesProgress;
    Blk360G2_ImageReadyEvent imageReady;
    Blk360G2_SaveImagesToFileEvent saveImagesToFile;
    Blk360G2_StitchingProgressEvent stitchingProgress;
    Blk360G2_SetupPackSaveProgressEvent setupPackSaveProgress;
    Blk360G2_SetupPackLoadingImagesSuccessEvent setupPackLoadingImagesSuccess;
    char padding[512];
} Blk360G2_Event;

/****************** Enumerators ************************/

BLK360G2_SHARED void Blk360G2_JobEnumerator_Release(Blk360G2_JobEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_JobEnumerator_MoveNext(Blk360G2_JobEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_JobHandle Blk360G2_JobEnumerator_GetCurrent(Blk360G2_JobEnumeratorHandle instance);

BLK360G2_SHARED void Blk360G2_SetupEnumerator_Release(Blk360G2_SetupEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_SetupEnumerator_MoveNext(Blk360G2_SetupEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_SetupHandle Blk360G2_SetupEnumerator_GetCurrent(Blk360G2_SetupEnumeratorHandle instance);

BLK360G2_SHARED void Blk360G2_SetupLinkEnumerator_Release(Blk360G2_SetupLinkEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_SetupLinkEnumerator_MoveNext(Blk360G2_SetupLinkEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_SetupLinkHandle Blk360G2_SetupLinkEnumerator_GetCurrent(Blk360G2_SetupLinkEnumeratorHandle instance);

BLK360G2_SHARED void Blk360G2_ImageEnumerator_Release(Blk360G2_ImageEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_ImageEnumerator_MoveNext(Blk360G2_ImageEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_ImageHandle Blk360G2_ImageEnumerator_GetCurrent(Blk360G2_ImageEnumeratorHandle instance);

BLK360G2_SHARED void Blk360G2_BundleEnumerator_Release(Blk360G2_BundleEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_BundleEnumerator_MoveNext(Blk360G2_BundleEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_BundleHandle Blk360G2_BundleEnumerator_GetCurrent(Blk360G2_BundleEnumeratorHandle instance);

BLK360G2_SHARED void Blk360G2_TagEnumerator_Release(Blk360G2_TagEnumeratorHandle instance);
BLK360G2_SHARED bool Blk360G2_TagEnumerator_MoveNext(Blk360G2_TagEnumeratorHandle instance);
BLK360G2_SHARED Blk360G2_TagHandle Blk360G2_TagEnumerator_GetCurrent(Blk360G2_TagEnumeratorHandle instance);

BLK360G2_SHARED Blk360G2_EventQueueHandle Blk360G2_EventQueue_New();
BLK360G2_SHARED void Blk360G2_EventQueue_Release(Blk360G2_EventQueueHandle instance);
BLK360G2_SHARED Blk360G2_Event Blk360G2_EventQueue_Pop(Blk360G2_EventQueueHandle instance);
BLK360G2_SHARED void Blk360G2_EventQueue_Clear(Blk360G2_EventQueueHandle instance);
BLK360G2_SHARED bool Blk360G2_EventQueue_IsEmpty(Blk360G2_EventQueueHandle instance);
BLK360G2_SHARED bool Blk360G2_EventQueue_Wait(Blk360G2_EventQueueHandle instance, Blk360G2_milliseconds_t timeout);

BLK360G2_SHARED void Blk360G2_Subscription_Release(Blk360G2_SubscriptionHandle subscription);

/****************** Session ************************/

BLK360G2_SHARED Blk360G2_SessionParameters Blk360G2_SessionParameters_New();

BLK360G2_SHARED Blk360G2_SessionHandle Blk360G2_Session_New_Default(const char* serverAddress);
BLK360G2_SHARED Blk360G2_SessionHandle Blk360G2_Session_New(const char* serverAddress, Blk360G2_SessionParameters sessionParameters);
BLK360G2_SHARED Blk360G2_SessionHandle Blk360G2_Session_New_Offline();
BLK360G2_SHARED void Blk360G2_Session_Release(Blk360G2_SessionHandle instance);
BLK360G2_SHARED void Blk360G2_Session_Reconnect(Blk360G2_SessionHandle instance);
BLK360G2_SHARED Blk360G2_SessionParameters Blk360G2_Session_GetParameters(Blk360G2_SessionHandle instance);
BLK360G2_SHARED void Blk360G2_Session_SetParameters(Blk360G2_SessionHandle instance, Blk360G2_SessionParameters sessionParameters);

BLK360G2_SHARED bool Blk360G2_Session_IsConnected(Blk360G2_SessionHandle instance);
BLK360G2_SHARED bool Blk360G2_Session_IsConnectionLost(Blk360G2_SessionHandle instance);
BLK360G2_SHARED Blk360G2_ConnectionStatus_t Blk360G2_Session_GetConnectionStatus(Blk360G2_SessionHandle instance);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_Session_OnConnectionStatusChanged(Blk360G2_SessionHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_Version Blk360G2_Api_GetLibraryVersion();
BLK360G2_SHARED const char* Blk360G2_Api_GetDetailedLibraryVersion();
BLK360G2_SHARED Blk360G2_Version Blk360G2_Api_DecodeVersion(Blk360G2_version_t version);

/****************** DeviceConfigWorkflow ************************/

BLK360G2_SHARED Blk360G2_DeviceConfigWorkflowHandle Blk360G2_DeviceConfigWorkflow_Create(Blk360G2_SessionHandle instance);
BLK360G2_SHARED void Blk360G2_DeviceConfigWorkflow_Release(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_ButtonPressConfig Blk360G2_ButtonPressConfig_GetDefault();
BLK360G2_SHARED Blk360G2_ButtonPressConfig Blk360G2_DeviceConfigWorkflow_GetButtonPressConfig(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_DeviceConfigWorkflow_SetButtonPressConfig(Blk360G2_DeviceConfigWorkflowHandle instance, Blk360G2_ButtonPressConfig config);
BLK360G2_SHARED void Blk360G2_DeviceConfigWorkflow_ResetButtonPressConfig(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_DeviceInfo Blk360G2_DeviceConfigWorkflow_GetDeviceInfo(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_DeviceStatus Blk360G2_DeviceConfigWorkflow_GetDeviceStatus(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_nanoseconds_t Blk360G2_DeviceConfigWorkflow_GetDeviceTime(Blk360G2_DeviceConfigWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_DeviceConfigWorkflow_SetDeviceTime(Blk360G2_DeviceConfigWorkflowHandle instance, Blk360G2_nanoseconds_t ns);
BLK360G2_SHARED const char* Blk360G2_DeviceConfigWorkflow_GetSuggestedSetupName(Blk360G2_DeviceConfigWorkflowHandle instance, Blk360G2_UUID jobUuid);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DeviceConfigWorkflow_OnDeviceStatus(Blk360G2_DeviceConfigWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Containers ************************/

BLK360G2_SHARED void Blk360G2_ContainerEnumerator_Release(Blk360G2_ContainerEnumeratorHandle handle);
BLK360G2_SHARED bool Blk360G2_ContainerEnumerator_MoveNext(Blk360G2_ContainerEnumeratorHandle handle);

BLK360G2_SHARED Blk360G2_CameraModel Blk360G2_ContainerEnumerator_GetCurrent_CameraModel(Blk360G2_ContainerEnumeratorHandle handle);
BLK360G2_SHARED Blk360G2_CalibrationIntrinsic Blk360G2_ContainerEnumerator_GetCurrent_CalibrationIntrinsic(Blk360G2_ContainerEnumeratorHandle handle);

/****************** Streams ************************/

BLK360G2_SHARED void Blk360G2_Stream_Release(Blk360G2_StreamHandle handle);

BLK360G2_SHARED void Blk360G2_Stream_Write(Blk360G2_StreamHandle handle, const char* buffer, uint32_t size);
BLK360G2_SHARED uint32_t Blk360G2_Stream_Read(Blk360G2_StreamHandle handle, char* buffer, uint32_t size);

BLK360G2_SHARED uint32_t Blk360G2_Stream_Size(Blk360G2_StreamHandle handle);
BLK360G2_SHARED void Blk360G2_Stream_Cancel(Blk360G2_StreamHandle handle);

/****************** Data manipulation workflow ************************/

BLK360G2_SHARED Blk360G2_DataManipulationWorkflowHandle Blk360G2_DataManipulationWorkflow_Create(Blk360G2_SessionHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Release(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Cancel(Blk360G2_DataManipulationWorkflowHandle instance);

BLK360G2_SHARED Blk360G2_SetupEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListSetups(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_ImageEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListImages(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_DownloadPointCloud(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_DownloadImage(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ImageHandle image);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_GetFieldImage(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ImageHandle image);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_GetOfficeImages(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_GetSourceImageForPanorama(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ImageHandle image);

BLK360G2_SHARED Blk360G2_Transaction Blk360G2_DataManipulationWorkflow_Transaction_Open(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Transaction_Commit(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_Transaction transaction);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Transaction_Rollback(Blk360G2_DataManipulationWorkflowHandle instance);

BLK360G2_SHARED Blk360G2_CameraCalibration Blk360G2_DataManipulationWorkflow_GetCameraCalibration(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_CameraPosition_t cameraPosition);
BLK360G2_SHARED Blk360G2_ImageCalibration Blk360G2_DataManipulationWorkflow_GetImageCalibration(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ImageHandle image, Blk360G2_meter_t distance);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnPointCloudDownloadProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnImageDownloadProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnSaveImagesToFile(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnGetImageProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnGetOfficeImagesProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnError(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnCancel(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnImageReady(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Job ************************/

BLK360G2_SHARED void Blk360G2_Job_Release(Blk360G2_JobHandle instance);
BLK360G2_SHARED Blk360G2_JobMetadata Blk360G2_JobMetadata_New(Blk360G2_UUID uuid);
BLK360G2_SHARED Blk360G2_JobEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListJobs(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_JobHandle Blk360G2_DataManipulationWorkflow_GetJobBySetupUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID setupUuid);
BLK360G2_SHARED Blk360G2_JobHandle Blk360G2_DataManipulationWorkflow_GetJobByUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID jobUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_RefreshMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_JobHandle job);
BLK360G2_SHARED Blk360G2_JobMetadata Blk360G2_DataManipulationWorkflow_Job_GetRefreshedMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_JobHandle job);
BLK360G2_SHARED Blk360G2_UUID Blk360G2_DataManipulationWorkflow_GetActiveJob(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetActiveJob(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID jobUuid);
BLK360G2_SHARED Blk360G2_JobHandle Blk360G2_DataManipulationWorkflow_Job_Create(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_JobMetadata metadata, const char* jobName);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_Delete(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_JobHandle job);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_AddSetup(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID setupUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_RemoveSetup(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID setupUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_AddBundle(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID bundleUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_RemoveBundle(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID bundleUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_AddTag(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID tagUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_RemoveTag(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID tagUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_SetThumbnail(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, Blk360G2_UUID thumbnailUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_ClearThumbnail(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle);
BLK360G2_SHARED Blk360G2_SetupEnumeratorHandle Blk360G2_DataManipulationWorkflow_Job_ListSetups(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle);

BLK360G2_SHARED Blk360G2_JobMetadata Blk360G2_Job_GetMetadata(Blk360G2_JobHandle instance);
BLK360G2_SHARED const char* Blk360G2_Job_GetName(Blk360G2_JobHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_SetName(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, const char* name);
BLK360G2_SHARED const char* Blk360G2_Job_GetNameTemplate(Blk360G2_JobHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_SetNameTemplate(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, const char* nameTemplate);
BLK360G2_SHARED const char* Blk360G2_Job_GetDescription(Blk360G2_JobHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_SetDescription(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, const char* description);
BLK360G2_SHARED const char* Blk360G2_Job_GetLocation(Blk360G2_JobHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Job_SetLocation(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_JobHandle jobHandle, const char* location);

/****************** Setup ************************/

BLK360G2_SHARED void Blk360G2_Setup_Release(Blk360G2_SetupHandle instance);
BLK360G2_SHARED void Blk360G2_SetupLink_Release(Blk360G2_SetupLinkHandle instance);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_Delete(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_Delete_Force(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_Update(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, Blk360G2_SetupMetadata metadata);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_SetName(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, const char* name);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_SetDescription(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, const char* description);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_SetLocation(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, const char* location);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_SetThumbnail(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, Blk360G2_UUID thumbnailUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_ClearThumbnail(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);

BLK360G2_SHARED Blk360G2_SetupHandle Blk360G2_DataManipulationWorkflow_GetSetupByUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID uuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_RefreshMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED Blk360G2_SetupMetadata Blk360G2_DataManipulationWorkflow_Setup_GetRefreshedMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup);
BLK360G2_SHARED Blk360G2_SetupMetadata Blk360G2_Setup_GetMetadata(Blk360G2_SetupHandle instance);
BLK360G2_SHARED const char* Blk360G2_Setup_GetName(Blk360G2_SetupHandle instance);
BLK360G2_SHARED const char* Blk360G2_Setup_GetDescription(Blk360G2_SetupHandle instance);
BLK360G2_SHARED const char* Blk360G2_Setup_GetLocation(Blk360G2_SetupHandle instance);

BLK360G2_SHARED Blk360G2_SetupEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListLinkedSetups(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setupHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_AddSetupLink(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setupHandle, Blk360G2_SetupLink setupLink);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_RemoveSetupLink(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setupHandle, Blk360G2_UUID setupUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_AddTag(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setupHandle, Blk360G2_UUID tagUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Setup_RemoveTag(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setupHandle, Blk360G2_UUID tagUuid);

BLK360G2_SHARED Blk360G2_SetupLinkEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListSetupLinksByJobUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID jobUuid);
BLK360G2_SHARED Blk360G2_SetupLinkMetadata Blk360G2_SetupLink_GetMetadata(Blk360G2_SetupLinkHandle instance);

/****************** Database ************************/

BLK360G2_SHARED Blk360G2_SetupPackHandle Blk360G2_SetupPack_New(const char* filePath);
BLK360G2_SHARED void Blk360G2_SetupPack_Release(Blk360G2_SetupPackHandle setupPackHandle);

BLK360G2_SHARED void Blk360G2_SetupPack_AddSetup(Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_SetupHandle setupHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_AddSetupsFromBundle(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_BundleHandle bundleHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_AddSetupsFromJob(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_JobHandle jobHandle);

BLK360G2_SHARED Blk360G2_SetupPack_SaveOptions Blk360G2_SetupPack_SaveOptions_New();
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_Save(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_SetupPack_SaveOptions options);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_SetupPack_OnSaveProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

BLK360G2_SHARED Blk360G2_SetupPack_LoadImagesOptions Blk360G2_SetupPack_LoadImagesOptions_New();
BLK360G2_SHARED Blk360G2_SetupEnumeratorHandle Blk360G2_DataManipulationWorkflow_SetupPack_Load(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_LoadPointCloud(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_SetupHandle setupHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_LoadImages(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupPackHandle setupPackHandle, Blk360G2_SetupHandle setupHandle, Blk360G2_SetupPack_LoadImagesOptions options);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_SetupPack_OnLoadingImagesSuccess(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_SetupPack_DownloadDuringMeasurement(Blk360G2_DataManipulationWorkflowHandle dataManipulationWorkflow, Blk360G2_MeasurementWorkflowHandle measurementWorkflow, Blk360G2_ImagePipeline imagePipeline, const char* outputFilePath, Blk360G2_MeasurementParameters measurementParameters);

/****************** Database - Deprecated ************************/

BLK360G2_SHARED Blk360G2_LoadImagesFromFileOptions Blk360G2_LoadImagesFromFileOptions_New();
BLK360G2_SHARED Blk360G2_SetupHandle Blk360G2_DataManipulationWorkflow_LoadSetupFromFile(Blk360G2_DataManipulationWorkflowHandle instance, const char* inputFilePath);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_LoadImagesFromFile(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_LoadImagesFromFileOptions options, const char* inputFilePath);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnLoadingImagesFromFileSuccess(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Bundle ************************/

BLK360G2_SHARED Blk360G2_BundleHandle Blk360G2_DataManipulationWorkflow_Bundle_Create(Blk360G2_DataManipulationWorkflowHandle instance, const char* name);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Bundle_Delete(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_BundleHandle bundle);
BLK360G2_SHARED void Blk360G2_Bundle_Release(Blk360G2_BundleHandle instance);
BLK360G2_SHARED Blk360G2_BundleMetadata Blk360G2_Bundle_GetMetadata(Blk360G2_BundleHandle instance);
BLK360G2_SHARED const char* Blk360G2_Bundle_GetName(Blk360G2_BundleHandle bundleHandle);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Bundle_SetName(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_BundleHandle bundleHandle, const char* name);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Bundle_AddSetup(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_BundleHandle bundleHandle, Blk360G2_UUID setupUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Bundle_RemoveSetup(Blk360G2_DataManipulationWorkflowHandle workflowHandle, Blk360G2_BundleHandle bundleHandle, Blk360G2_UUID setupUuid);

BLK360G2_SHARED Blk360G2_BundleEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListAllBundles(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_BundleEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListJobBundles(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID uuid);
BLK360G2_SHARED Blk360G2_BundleHandle Blk360G2_DataManipulationWorkflow_GetBundleBySetupUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID setupUuid);
BLK360G2_SHARED Blk360G2_BundleHandle Blk360G2_DataManipulationWorkflow_GetBundleByUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID uuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Bundle_RefreshMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_BundleHandle bundleHandle);
BLK360G2_SHARED Blk360G2_BundleMetadata Blk360G2_DataManipulationWorkflow_Bundle_GetRefreshedMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_BundleHandle bundleHandle);
BLK360G2_SHARED Blk360G2_SetupEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListBundleSetups(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID uuid);

/****************** Thumbnail ************************/

BLK360G2_SHARED void Blk360G2_Thumbnail_Release(Blk360G2_ThumbnailHandle instance);
BLK360G2_SHARED Blk360G2_ThumbnailHandle Blk360G2_DataManipulationWorkflow_GetThumbnailByUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID thumbnailUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Thumbnail_RefreshMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ThumbnailHandle thumbnail);
BLK360G2_SHARED Blk360G2_ThumbnailMetadata Blk360G2_DataManipulationWorkflow_Thumbnail_GetRefreshedMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ThumbnailHandle thumbnail);

BLK360G2_SHARED Blk360G2_ThumbnailHandle Blk360G2_DataManipulationWorkflow_Thumbnail_Create(Blk360G2_DataManipulationWorkflowHandle instance, uint64_t size, const char* data, Blk360G2_MimeType mimeType);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Thumbnail_Delete(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_ThumbnailHandle thumbnail);

BLK360G2_SHARED Blk360G2_ThumbnailMetadata Blk360G2_Thumbnail_GetMetadata(Blk360G2_ThumbnailHandle instance);
BLK360G2_SHARED const char* Blk360G2_Thumbnail_GetData(Blk360G2_ThumbnailHandle instance);
BLK360G2_SHARED uint64_t Blk360G2_Thumbnail_GetDataSizeInBytes(Blk360G2_ThumbnailHandle instance);

/****************** Tag ************************/

BLK360G2_SHARED void Blk360G2_Tag_Release(Blk360G2_TagHandle instance);
BLK360G2_SHARED Blk360G2_TagMetadata Blk360G2_TagMetadata_New();

BLK360G2_SHARED Blk360G2_TagEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListAllTags(Blk360G2_DataManipulationWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_TagEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListJobTags(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID jobUuid);
BLK360G2_SHARED Blk360G2_TagEnumeratorHandle Blk360G2_DataManipulationWorkflow_ListSetupTags(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID setupUuid);
BLK360G2_SHARED Blk360G2_TagHandle Blk360G2_DataManipulationWorkflow_GetTagByUuid(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_UUID tagUuid);
BLK360G2_SHARED Blk360G2_TagHandle Blk360G2_DataManipulationWorkflow_Tag_Create(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagMetadata metadata, const char* name, const char* description);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_Delete(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_Update(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, Blk360G2_TagMetadata metadata);

BLK360G2_SHARED Blk360G2_TagMetadata Blk360G2_Tag_GetMetadata(Blk360G2_TagHandle instance);
BLK360G2_SHARED const char* Blk360G2_Tag_GetName(Blk360G2_TagHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_SetName(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, const char* name);
BLK360G2_SHARED const char* Blk360G2_Tag_GetDescription(Blk360G2_TagHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_SetDescription(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, const char* description);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_SetThumbnail(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, Blk360G2_UUID thumbnailUuid);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_ClearThumbnail(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_RefreshMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);
BLK360G2_SHARED Blk360G2_TagMetadata Blk360G2_DataManipulationWorkflow_Tag_GetRefreshedMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);

/****************** Blob ************************/

BLK360G2_SHARED const char* Blk360G2_Tag_GetBlobFilename(Blk360G2_TagHandle instance);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_SetBlobFilename(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, const char* filename);
BLK360G2_SHARED const char* Blk360G2_Tag_GetBlobData(Blk360G2_TagHandle instance);
BLK360G2_SHARED uint64_t Blk360G2_Tag_GetBlobDataSizeInBytes(Blk360G2_TagHandle instance);

BLK360G2_SHARED void Blk360G2_Tag_ReleaseBlob(Blk360G2_TagHandle instance);

BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_DownloadBlobData(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);
BLK360G2_SHARED void Blk360G2_DataManipulationWorkflow_Tag_UploadBlobData(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, uint64_t size, const char* data, const Blk360G2_MimeType*);
BLK360G2_SHARED Blk360G2_StreamHandle Blk360G2_DataManipulationWorkflow_Tag_UploadBlobDataStream(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag, uint64_t size, const Blk360G2_MimeType*);
BLK360G2_SHARED Blk360G2_StreamHandle Blk360G2_DataManipulationWorkflow_Tag_DownloadBlobDataStream(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_TagHandle tag);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnBlobDownloadProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnBlobDownloadCancel(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnBlobUploadProgress(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_DataManipulationWorkflow_OnBlobUploadCancel(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Image ************************/

BLK360G2_SHARED Blk360G2_ImageHandle Blk360G2_DataManipulationWorkflow_GetImageByMetadata(Blk360G2_DataManipulationWorkflowHandle instance, Blk360G2_SetupHandle setup, Blk360G2_ImageMetadata metadata);
BLK360G2_SHARED Blk360G2_ImageMetadata Blk360G2_Image_GetMetadata(Blk360G2_ImageHandle instance);
BLK360G2_SHARED void Blk360G2_Image_Release(Blk360G2_ImageHandle instance);
BLK360G2_SHARED const char* Blk360G2_Image_GetData(Blk360G2_ImageHandle image);
BLK360G2_SHARED uint64_t Blk360G2_Image_GetDataSizeInBytes(Blk360G2_ImageHandle image);
BLK360G2_SHARED Blk360G2_ImageEnumeratorHandle Blk360G2_Image_GetHdrImageBrackets(Blk360G2_ImageHandle instance);

/****************** ProcessingWorkflow ************************/

BLK360G2_SHARED Blk360G2_ProcessingWorkflowHandle Blk360G2_ProcessingWorkflow_Create(Blk360G2_SessionHandle instance, Blk360G2_ProcessingParameters parameters);
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_Release(Blk360G2_ProcessingWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_ProcessingParameters Blk360G2_ProcessingParameters_New();
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_ProcessPointCloud(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_SetupHandle setupHandle);
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_ProcessImage(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_ImageHandle image);
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_Cancel(Blk360G2_ProcessingWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_ToneMapHDRImage(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_ImageHandle image);
BLK360G2_SHARED void Blk360G2_ProcessingWorkflow_BalanceHalfDomeColors(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_ImageHandle image);

BLK360G2_SHARED void Blk360G2_PointCloudChunk_Release(Blk360G2_PointCloudChunkHandle handle);
BLK360G2_SHARED const char* Blk360G2_PointCloudChunk_GetData(Blk360G2_PointCloudChunkHandle handle);
BLK360G2_SHARED uint64_t Blk360G2_PointCloudChunk_GetDataSizeInBytes(Blk360G2_PointCloudChunkHandle handle);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnPointCloudChunkAvailable(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnPointCloudProcessProgress(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnPointCloudProcessError(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnPointCloudProcessCancel(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Panoramas ************************/

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnStitchingProgress(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_ProcessingWorkflow_OnStitchingError(Blk360G2_ProcessingWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

BLK360G2_SHARED Blk360G2_StitcherHandle Blk360G2_HalfdomeStitcher_New(Blk360G2_ProcessingWorkflowHandle processing, Blk360G2_DataManipulationWorkflowHandle dataManipulation, Blk360G2_SetupHandle setup);
BLK360G2_SHARED Blk360G2_StitcherHandle Blk360G2_FulldomeStitcher_New(Blk360G2_ProcessingWorkflowHandle processing, Blk360G2_DataManipulationWorkflowHandle dataManipulation, Blk360G2_SetupHandle setup);
BLK360G2_SHARED Blk360G2_StitcherConfiguration Blk360G2_Stitcher_GetConfiguration(Blk360G2_StitcherHandle stitcher);
BLK360G2_SHARED void Blk360G2_Stitcher_AddImage(Blk360G2_StitcherHandle stitcher, Blk360G2_ImageHandle image);

BLK360G2_SHARED bool Blk360G2_Stitcher_IsOutputReady(Blk360G2_StitcherHandle instance);
BLK360G2_SHARED Blk360G2_ImageHandle Blk360G2_Stitcher_GetPanorama(Blk360G2_StitcherHandle instance);
BLK360G2_SHARED void Blk360G2_Stitcher_Release(Blk360G2_StitcherHandle instance);

/****************** MeasurementWorkflow ************************/

BLK360G2_SHARED Blk360G2_MeasurementWorkflowHandle Blk360G2_MeasurementWorkflow_Create(Blk360G2_SessionHandle instance);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_Release(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED Blk360G2_MeasurementParameters Blk360G2_MeasurementParameters_New();
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_Start(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_MeasurementParameters parameters, const char* setupName);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_DoHalfTurn(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_Cancel(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_ButtonScanEvents_Start(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_ButtonScanEvents_Stop(Blk360G2_MeasurementWorkflowHandle instance);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnMeasurementProgress(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnSetupStarted(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnHalfTurnProgress(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnImageCapture(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnError(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnCancel(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_StartVisStream(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED void Blk360G2_MeasurementWorkflow_StopVisStream(Blk360G2_MeasurementWorkflowHandle instance);
BLK360G2_SHARED bool Blk360G2_MeasurementWorkflow_IsVisStreamStarted(Blk360G2_MeasurementWorkflowHandle instance);

BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnVisStarted(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnVisStopped(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnVisRunning(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);
BLK360G2_SHARED Blk360G2_SubscriptionHandle Blk360G2_MeasurementWorkflow_OnVisError(Blk360G2_MeasurementWorkflowHandle instance, Blk360G2_EventQueueHandle queue);

/****************** Helpers ************************/
BLK360G2_SHARED Blk360G2_StringUUID Blk360G2_UUID_Serialize(Blk360G2_UUID uuid);
BLK360G2_SHARED Blk360G2_UUID Blk360G2_UUID_Deserialize(Blk360G2_StringUUID uuid);
BLK360G2_SHARED Blk360G2_StringUUID Blk360G2_UUID_Parse(const char* uuid);

#ifdef __cplusplus // extern "C" {
}
#endif

#endif // BLK360G2_H
