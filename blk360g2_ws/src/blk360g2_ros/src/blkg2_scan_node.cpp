// ROS node to expose BLK360G2 start_scan service and publish pointcloud

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <BLK360G2.h>
#include <mutex>
#include <thread>
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <blk360g2_ros/PointcloudUtils.hpp>

using namespace common;

namespace blk360g2
{
std::mutex mtx;
bool scanInProgress = false;
ros::Publisher pointcloud_pub;

Blk360G2_SessionHandle session{Blk360G2_Handle_Null()};
Blk360G2_MeasurementWorkflowHandle measurementWorkflow{Blk360G2_Handle_Null()};
Blk360G2_DataManipulationWorkflowHandle dataManipulationWorkflow{Blk360G2_Handle_Null()};
Blk360G2_ProcessingWorkflowHandle processingWorkflow{Blk360G2_Handle_Null()};
Blk360G2_SetupHandle setupToDownload{Blk360G2_Handle_Null()};
Blk360G2_EventQueueHandle queue{Blk360G2_Handle_Null()};

Blk360G2_SubscriptionHandle errorSubscription{Blk360G2_Handle_Null()};
Blk360G2_SubscriptionHandle progressSubscription{Blk360G2_Handle_Null()};

void checkError(const char* context = "")
{
    const Blk360G2_Error error = Blk360G2_Api_GetLastError();
    if (error.code != Blk360G2_Error_Ok)
    {
        ROS_ERROR_STREAM("[" << context << "] BLK360 Error: " << error.message);
        throw std::runtime_error(error.message);
    }
}

void init()
{
    Blk360G2_Api_New(BLK360G2_LIBRARY_VERSION);
    checkError("Api_New");

    session = Blk360G2_Session_New_Default("10.10.1.1");
    checkError("Session_New_Default");

    measurementWorkflow = Blk360G2_MeasurementWorkflow_Create(session);
    checkError("MeasurementWorkflow_Create");

    dataManipulationWorkflow = Blk360G2_DataManipulationWorkflow_Create(session);
    checkError("DataManipulationWorkflow_Create");

    queue = Blk360G2_EventQueue_New();
    checkError("EventQueue_New");
}

void cleanup()
{
    Blk360G2_Setup_Release(setupToDownload);
    Blk360G2_EventQueue_Release(queue);
    Blk360G2_ProcessingWorkflow_Release(processingWorkflow);
    Blk360G2_DataManipulationWorkflow_Release(dataManipulationWorkflow);
    Blk360G2_MeasurementWorkflow_Release(measurementWorkflow);
    Blk360G2_Session_Release(session);
    Blk360G2_Api_Release();
}

Blk360G2_UUID doScan()
{
    auto parameters = Blk360G2_MeasurementParameters_New();
    parameters.scanConfig.density = Blk360G2_PointCloudDensity_Ultralow;
    parameters.enablePointCloud = true;
    parameters.enableImages = false;
    parameters.visConfig.visMode = Blk360G2_VisMode_Disabled;

    errorSubscription = Blk360G2_MeasurementWorkflow_OnError(measurementWorkflow, queue);
    checkError("OnError");
    progressSubscription = Blk360G2_MeasurementWorkflow_OnMeasurementProgress(measurementWorkflow, queue);
    checkError("OnProgress");
    auto setupStarted = Blk360G2_MeasurementWorkflow_OnSetupStarted(measurementWorkflow, queue);
    checkError("OnSetupStarted");

    Blk360G2_MeasurementWorkflow_Start(measurementWorkflow, parameters, nullptr);
    checkError("StartMeasurement");

    Blk360G2_UUID setupUuid{};
    while (Blk360G2_EventQueue_Wait(queue, 10000))
    {
        checkError("Wait");

        if (Blk360G2_EventQueue_IsEmpty(queue)) break;
        const auto event = Blk360G2_EventQueue_Pop(queue);
        checkError("Pop");

        if (event.sender.handle == setupStarted.handle)
            setupUuid = event.setupStarted.setupUuid;
        else if (event.sender.handle == errorSubscription.handle)
            throw std::runtime_error("Scan error");
        else if (event.sender.handle == progressSubscription.handle && event.measurementProgress.progress == 100)
            break;
    }
    return setupUuid;
}

void processPointCloud(const Blk360G2_UUID& uuid)
{
    setupToDownload = Blk360G2_DataManipulationWorkflow_GetSetupByUuid(dataManipulationWorkflow, uuid);
    checkError("GetSetup");

    Blk360G2_DataManipulationWorkflow_DownloadPointCloud(dataManipulationWorkflow, setupToDownload);
    checkError("DownloadPointCloud");

    Blk360G2_ProcessingParameters params{};
    processingWorkflow = Blk360G2_ProcessingWorkflow_Create(session, params);
    checkError("CreateProcessingWorkflow");

    auto chunkSub = Blk360G2_ProcessingWorkflow_OnPointCloudChunkAvailable(processingWorkflow, queue);
    auto progSub = Blk360G2_ProcessingWorkflow_OnPointCloudProcessProgress(processingWorkflow, queue);
    auto errSub = Blk360G2_ProcessingWorkflow_OnPointCloudProcessError(processingWorkflow, queue);
    checkError("SubscribeProcessingEvents");

    Blk360G2_ProcessingWorkflow_ProcessPointCloud(processingWorkflow, setupToDownload);
    checkError("StartProcessing");

    std::vector<uint8_t> full_data;
    while (Blk360G2_EventQueue_Wait(queue, 20000))
    {
        if (Blk360G2_EventQueue_IsEmpty(queue)) continue;
        const auto event = Blk360G2_EventQueue_Pop(queue);
        checkError("PopProcessing");

        if (event.sender.handle == chunkSub.handle)
        {
            const char* data = Blk360G2_PointCloudChunk_GetData(event.pointCloudChunkAvailable.handle);
            size_t size = Blk360G2_PointCloudChunk_GetDataSizeInBytes(event.pointCloudChunkAvailable.handle);
            full_data.insert(full_data.end(), data, data + size);
            Blk360G2_PointCloudChunk_Release(event.pointCloudChunkAvailable.handle);
        }
        else if (event.sender.handle == errSub.handle)
        {
            throw std::runtime_error("Processing error event");
        }
        else if (event.sender.handle == progSub.handle && event.pointCloudProcessProgress.progress == 100)
        {
            break;
        }
    }

    std::vector<uint8_t> cloud_data = common::convertPolarBufferToPointCloud2Data(full_data);
    
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.height = 1;
    msg.point_step = 16;
    msg.is_dense = false;
    msg.is_bigendian = false;

    msg.fields.resize(4);
    msg.fields[0].name = "x";         msg.fields[0].offset = 0;  msg.fields[0].datatype = 7; msg.fields[0].count = 1;
    msg.fields[1].name = "y";         msg.fields[1].offset = 4;  msg.fields[1].datatype = 7; msg.fields[1].count = 1;
    msg.fields[2].name = "z";         msg.fields[2].offset = 8;  msg.fields[2].datatype = 7; msg.fields[2].count = 1;
    msg.fields[3].name = "intensity"; msg.fields[3].offset = 12; msg.fields[3].datatype = 7; msg.fields[3].count = 1;

    msg.width = cloud_data.size() / msg.point_step;
    msg.row_step = msg.width * msg.point_step;
    msg.data = std::move(cloud_data);

    pointcloud_pub.publish(msg);
}

bool startScanCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (scanInProgress)
    {
        res.success = false;
        res.message = "Scan already in progress.";
        return true;
    }

    scanInProgress = true;
    std::thread([]() {
        try
        {
            init();
            Blk360G2_UUID uuid = doScan();
            processPointCloud(uuid);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Scan failed: " << e.what());
        }
        cleanup();
        scanInProgress = false;
    }).detach();

    res.success = true;
    res.message = "Scan started asynchronously.";
    return true;
}
} // namespace blk360g2

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blk360g2");
    ros::NodeHandle nh;

    blk360g2::pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/blk360g2/pointcloud", 1);
    ros::ServiceServer service = nh.advertiseService("/blk360g2/start_scan", blk360g2::startScanCallback);

    ROS_INFO("BLK360 G2 scan service ready.");
    ros::spin();

    return 0;
}
