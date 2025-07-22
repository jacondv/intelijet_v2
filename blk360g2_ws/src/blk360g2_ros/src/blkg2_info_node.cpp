#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <BLK360G2.h>

#include <cstdlib>
#include <iostream>
#include <string>



Blk360G2_SessionHandle session{Blk360G2_Handle_Null()};
Blk360G2_DeviceConfigWorkflowHandle deviceConfigWorkflow{Blk360G2_Handle_Null()};

void checkError() {
    Blk360G2_Error error = Blk360G2_Api_GetLastError();
    if (error.code != Blk360G2_Error_Ok) {
        ROS_ERROR("BLK360G2 error: %s", error.message);
        Blk360G2_DeviceConfigWorkflow_Release(deviceConfigWorkflow);
        Blk360G2_Session_Release(session);
        Blk360G2_Api_Release();
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "blkg2_info_node");
    ros::NodeHandle nh("~");

    std::string ip = "10.10.1.1";
    nh.getParam("device_ip", ip);

    Blk360G2_Api_New(BLK360G2_LIBRARY_VERSION);
    checkError();

    session = Blk360G2_Session_New_Default(ip.c_str());
    checkError();

    deviceConfigWorkflow = Blk360G2_DeviceConfigWorkflow_Create(session);
    checkError();

    auto deviceInfo = Blk360G2_DeviceConfigWorkflow_GetDeviceInfo(deviceConfigWorkflow);
    checkError();

    auto deviceStatus = Blk360G2_DeviceConfigWorkflow_GetDeviceStatus(deviceConfigWorkflow);
    checkError();

    ROS_INFO("Model: %d", deviceStatus);
    // ROS_INFO("Firmware: %s", deviceInfo.firmwareVersion);
    // ROS_INFO("Status: Battery %.1f%%, Temp %.1f°C", deviceStatus.batteryLevel, deviceStatus.temperature);
    // ROS_INFO("Model: %s", deviceStatus);
    
    Blk360G2_DeviceConfigWorkflow_Release(deviceConfigWorkflow);
    Blk360G2_Session_Release(session);
    Blk360G2_Api_Release();

    return 0;
}
