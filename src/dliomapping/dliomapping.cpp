#include <ctime>
#include <iostream>
#include <iterator>
#include <locale>
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
#include <pcl/io/ply_io.h>

struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
      std::uint32_t t; // time since beginning of scan in nanoseconds
      float time; // time since beginning of scan in seconds
      double timestamp; // absolute timestamp in seconds
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (float, time, time)
    (double, timestamp, timestamp))

class Dliomapping_Node {
public:
    Dliomapping_Node(ros::NodeHandle nh) {
        subPcl = nh.subscribe("robot/dlio/odom_node/pointcloud/deskewed", queueSize, &Dliomapping_Node::callback_pcl_deskewed, this);

        std::time_t time = std::time({});
        timeString = "yyyy-mm-dd_hh-mm_";
        std::strftime(std::data(timeString), std::size(timeString),"%F_%H-%M", std::gmtime(&time));
        timeString = timeString.substr(0, timeString.size() - 1); // because strftime is cursed to hell and back
        std::cout << "Starting recording at timestamp: " << timeString << '\n';

    }

public:
    void callback_pcl_deskewed(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*msg, pointcloud);

        rawGlobalMap += pointcloud;
        iClouds++;
        if (iClouds >= nMaxCloudsPerPly - 1) {
            std::string path = "/root/repo/maps/" + timeString + "_" + std::to_string(iPly++) + ".ply";
            std::cout << "saving temporary map to " << path << "..." << std::endl;
            int res = pcl::io::savePLYFile(path, rawGlobalMap);
            // int res = pcl::io::savePCDFile(path, rawGlobalMap);
            rawGlobalMap = {};
            iClouds = 0;
            if (res) std::cout << "failed saving ply file: " << res << std::endl;
        }
    }

private:
    uint32_t queueSize = 100;
    ros::Subscriber subPcl;
    std::string timeString;

    // temporary stuff for saving raw maps:
    pcl::PointCloud<Point> rawGlobalMap;
    size_t nMaxCloudsPerPly = 100;
    size_t iClouds = 0;
    size_t iPly = 0;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "dliomapping_node");
    ros::NodeHandle nh;
    Dliomapping_Node dliomappingNode(nh);
    ros::spin();
    return 0;
}