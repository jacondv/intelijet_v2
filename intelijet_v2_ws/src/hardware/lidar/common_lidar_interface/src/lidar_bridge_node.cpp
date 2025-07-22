// File: lidar_bridge_node.cpp
// Location: modules/lidar_bridge/src/lidar_bridge_node.cpp
// All Lidar output will be in PointCloud2 format at topic /lidar/points

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pub_cloud;
ros::Subscriber sub_cloud;
std::string input_topic, output_topic;
std::string input_type;  // "scan" or "cloud"

laser_geometry::LaserProjection projector;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_msg, cloud);
    cloud.header.frame_id = scan_msg->header.frame_id;
    cloud.header.stamp = scan_msg->header.stamp;
    pub_cloud.publish(cloud);
    ROS_INFO_ONCE("[lidar_bridge] Converting LaserScan to PointCloud2");
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pub_cloud.publish(msg);
    ROS_INFO_ONCE("[lidar_bridge] Forwarding PointCloud2");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_bridge_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("input_topic", input_topic, "/scan");
    nh.param<std::string>("output_topic", output_topic, "/pointcloud");
    nh.param<std::string>("input_type", input_type, "scan");  // or "cloud"

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

    if (input_type == "scan")
    {
        sub_cloud = nh.subscribe(input_topic, 1, scanCallback);
    }
    else if (input_type == "cloud")
    {
        sub_cloud = nh.subscribe(input_topic, 1, cloudCallback);
    }
    else
    {
        ROS_ERROR("[lidar_bridge] Invalid input_type: must be 'scan' or 'cloud'");
        return 1;
    }

    ROS_INFO_STREAM("[lidar_bridge] Subscribed to: " << input_topic);
    ROS_INFO_STREAM("[lidar_bridge] Publishing to: " << output_topic);

    ros::spin();
    return 0;
}
