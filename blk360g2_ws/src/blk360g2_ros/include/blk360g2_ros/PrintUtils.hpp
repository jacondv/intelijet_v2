#pragma once

#include "BLK360G2.h"

namespace common
{
void print(const Blk360G2_ErrorEvent&);
void print(const Blk360G2_PointCloudDownloadProgressEvent&);
void print(const Blk360G2_PointCloudProcessProgressEvent&);
void print(const Blk360G2_MeasurementProgressEvent&);
void print(const Blk360G2_SetupStartedEvent&);
void print(const Blk360G2_VisRunningEvent&);
void print(const Blk360G2_SaveImagesToFileEvent&);
void print(const Blk360G2_SetupPackLoadingImagesSuccessEvent&);
void print(const Blk360G2_GetOfficeImagesProgressEvent&);
void print(const Blk360G2_ImageReadyEvent&);

void print(const Blk360G2_JobMetadata&);
void print(const Blk360G2_SetupMetadata&);

void print(const Blk360G2_MeasurementParameters&);
void print(const Blk360G2_ConnectionStatus&);
void print(const Blk360G2_ImagePlane&);
void print(const Blk360G2_CameraPosition&);
void print(const Blk360G2_CameraCalibration&);
void print(const Blk360G2_CameraModel&);
void print(const Blk360G2_DeviceInfo&);
void print(const Blk360G2_DeviceStatus&);

const char* strGetImageStatus(const Blk360G2_GetImageProgressStatus_t);
}
