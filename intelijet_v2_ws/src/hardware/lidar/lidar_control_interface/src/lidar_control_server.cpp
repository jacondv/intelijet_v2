#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// Trạng thái nội bộ
bool lidar_is_scanning = false;

bool handleStart(std_srvs::Trigger::Request &req,
                 std_srvs::Trigger::Response &res)
{
    if (lidar_is_scanning)
    {
        res.success = false;
        res.message = "LIDAR is already scanning.";
    }
    else
    {
        lidar_is_scanning = true;
        ROS_INFO("[LIDAR] Scan started.");
        // TODO: Gửi lệnh điều khiển phần cứng
        res.success = true;
        res.message = "LIDAR scan started.";
    }
    return true;
}

bool handleStop(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res)
{
    if (!lidar_is_scanning)
    {
        res.success = false;
        res.message = "LIDAR is not scanning.";
    }
    else
    {
        lidar_is_scanning = false;
        ROS_INFO("[LIDAR] Scan stopped.");
        // TODO: Gửi lệnh điều khiển phần cứng
        res.success = true;
        res.message = "LIDAR scan stopped.";
    }
    return true;
}

bool handleStatus(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
{
    if (lidar_is_scanning)
    {
        res.success = true;
        res.message = "LIDAR is scanning.";
    }
    else
    {
        res.success = false;
        res.message = "LIDAR is idle.";
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_control_server");
    ros::NodeHandle nh;

    ros::ServiceServer start_srv = nh.advertiseService("start_scan", handleStart);
    ros::ServiceServer stop_srv = nh.advertiseService("stop_scan", handleStop);
    ros::ServiceServer status_srv = nh.advertiseService("get_scan_status", handleStatus);

    ROS_INFO("LIDAR control interface ready.");
    ros::spin();
    return 0;
}
