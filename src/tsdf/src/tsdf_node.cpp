// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// DLIO
// #include "dlio/dlio.h"
#include "dlio_stuff.hpp"

// OTHER
#include "tsdf_map.hpp"

class TSDF_Node {
public:
    TSDF_Node(ros::NodeHandle nh) {
        subPath = nh.subscribe("robot/dlio/odom_node/path", queueSize, &TSDF_Node::callback_path, this);
        subPcl = nh.subscribe("robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDF_Node::callback_pcl_deskewed, this);
    }

public:
    void callback_path(const nav_msgs::PathConstPtr& msg) {
        double now = ros::Time::now().toSec();
        bufferedPath = { msg, now };

        if (now - bufferedPcl.second < syncThreshhold) {
            tsdfMap.add_pointcloud(bufferedPath.first, bufferedPcl.first);
        }
    }
    void callback_pcl_deskewed(const sensor_msgs::PointCloud2ConstPtr& msg) {
        double now = ros::Time::now().toSec();
        bufferedPcl = { msg, now };

        if (now - bufferedPath.second < syncThreshhold) {
            tsdfMap.add_pointcloud(bufferedPath.first, bufferedPcl.first);
        }
    }

private:
    TSDF_Map tsdfMap;
    uint32_t queueSize = 10;
    ros::Subscriber subPath;
    ros::Subscriber subPcl;

    // buffer with timestamp
    std::pair<nav_msgs::PathConstPtr, double> bufferedPath;
    std::pair<sensor_msgs::PointCloud2ConstPtr, double> bufferedPcl;

    // configure this
    double syncThreshhold = 0.01;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf_node");
    ros::NodeHandle nh;

    for (int8_t i = -127; i < 127; i++) {
        TSDF_Point point(i);
        ROS_INFO("%f", point.get());
    }
    TSDF_Point point(127);
    ROS_INFO("%f", point.get());

    TSDF_Node tsdfNode(nh);
    // ros::spin();
    return 0;
}