// ROS node to expose BLK360G2 start_scan **action** and publish pointcloud

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <actionlib/server/simple_action_server.h>
#include <blk360g2_ros/StartScanAction.h>

#include <BLK360G2.h>
#include <blk360g2_ros/PointcloudUtils.hpp>
#include <mutex>
#include <thread>
#include <vector>

using namespace common;

namespace blk360g2
{
std::mutex mtx;
bool scanInProgress = false;
ros::Publisher pointcloud_pub;

// BLK `handles
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
    progressSubscription = Blk360G2_MeasurementWorkflow_OnMeasurementProgress(measurementWorkflow, queue);
    auto setupStarted = Blk360G2_MeasurementWorkflow_OnSetupStarted(measurementWorkflow, queue);

    Blk360G2_MeasurementWorkflow_Start(measurementWorkflow, parameters, nullptr);

    Blk360G2_UUID setupUuid{};
    while (Blk360G2_EventQueue_Wait(queue, 10000))
    {
        const auto event = Blk360G2_EventQueue_Pop(queue);
        if (event.sender.handle == setupStarted.handle)
            setupUuid = event.setupStarted.setupUuid;
        else if (event.sender.handle == errorSubscription.handle)
            throw std::runtime_error("Scan error");
        else if (event.sender.handle == progressSubscription.handle && event.measurementProgress.progress == 100)
            break;
    }
    return setupUuid;
}

void publishCloud(const Blk360G2_UUID& uuid, const std::string& topic, ros::NodeHandle& nh)
{
    setupToDownload = Blk360G2_DataManipulationWorkflow_GetSetupByUuid(dataManipulationWorkflow, uuid);
    Blk360G2_DataManipulationWorkflow_DownloadPointCloud(dataManipulationWorkflow, setupToDownload);

    Blk360G2_ProcessingParameters params{};
    processingWorkflow = Blk360G2_ProcessingWorkflow_Create(session, params);

    auto chunkSub = Blk360G2_ProcessingWorkflow_OnPointCloudChunkAvailable(processingWorkflow, queue);
    auto progSub = Blk360G2_ProcessingWorkflow_OnPointCloudProcessProgress(processingWorkflow, queue);

    Blk360G2_ProcessingWorkflow_ProcessPointCloud(processingWorkflow, setupToDownload);

    std::vector<uint8_t> full_data;
    while (Blk360G2_EventQueue_Wait(queue, 20000))
    {
        if (Blk360G2_EventQueue_IsEmpty(queue)) continue;
        const auto event = Blk360G2_EventQueue_Pop(queue);

        if (event.sender.handle == chunkSub.handle)
        {
            const char* data = Blk360G2_PointCloudChunk_GetData(event.pointCloudChunkAvailable.handle);
            size_t size = Blk360G2_PointCloudChunk_GetDataSizeInBytes(event.pointCloudChunkAvailable.handle);
            full_data.insert(full_data.end(), data, data + size);
            Blk360G2_PointCloudChunk_Release(event.pointCloudChunkAvailable.handle);
        }
        else if (event.sender.handle == progSub.handle && event.pointCloudProcessProgress.progress == 100)
        {
            break;
        }
    }

    std::vector<uint8_t> cloud_data = convertPolarBufferToPointCloud2Data(full_data);
    ROS_INFO("Warning: PointCloud2 data size: %zu bytes", cloud_data.size());
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.height = 1;
    msg.point_step = 16;
    msg.is_dense = false;
    msg.is_bigendian = false;
    msg.width = cloud_data.size() / msg.point_step;
    msg.row_step = msg.width * msg.point_step;

    msg.fields.resize(4);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0; msg.fields[0].datatype = 7; msg.fields[0].count = 1;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4; msg.fields[1].datatype = 7; msg.fields[1].count = 1;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8; msg.fields[2].datatype = 7; msg.fields[2].count = 1;
    msg.fields[3].name = "intensity"; msg.fields[3].offset = 12; msg.fields[3].datatype = 7; msg.fields[3].count = 1;
    msg.data = std::move(cloud_data);

    // ros::NodeHandle nh;
    if (!pointcloud_pub || pointcloud_pub.getTopic() != topic)
    {
        pointcloud_pub.shutdown();
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
    }

    pointcloud_pub.publish(msg);
}

class StartScanActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<blk360g2_ros::StartScanAction> as_;
    std::string action_name_;
    blk360g2_ros::StartScanResult result_;

public:
    StartScanActionServer(const std::string& name) :
        as_(nh_, name, boost::bind(&StartScanActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    void executeCB(const blk360g2_ros::StartScanGoalConstPtr& goal)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (scanInProgress)
        {
            result_.success = false;
            result_.message = "Scan already in progress.";
            as_.setAborted(result_);
            return;
        }

        if (goal->output_topic.empty()) {
            result_.success = false;
            result_.message = "Output topic is empty.";
            as_.setAborted(result_);
            return;
        }

        scanInProgress = true;
        try {
            init();
            Blk360G2_UUID uuid = doScan();
            publishCloud(uuid, goal->output_topic, nh_);
            result_.success = true;
            result_.message = "Scan completed successfully.";
            as_.setSucceeded(result_);
        } catch (const std::exception& e) {
            result_.success = false;
            result_.message = e.what();
            as_.setAborted(result_);
        }
        cleanup();
        scanInProgress = false;
    }
};

} // namespace blk360g2

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blk360g2");
    ros::NodeHandle nh;
    blk360g2::StartScanActionServer server("/blk360g2/start_scan");
    ROS_INFO("BLK360 G2 Action Server ready.");
    ros::spin();
    return 0;
}
