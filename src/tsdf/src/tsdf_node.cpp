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

// DLIO
// #include "dlio/dlio.h"
#include "dlio_stuff.hpp"

// OTHER
#include "tsdf_map.hpp"


// TODO: only reallyt need deskewed pointcloud, points are already transformed

class TSDF_Node {
public:
    TSDF_Node(ros::NodeHandle nh) {
        subPcl = nh.subscribe("robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDF_Node::callback_pcl_deskewed, this);
        // tsdfMap.DEBUGGING_INSERT();
    }

public:
    void callback_pcl_deskewed(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*msg, pointcloud);

        // tsdfMap.DEBUGGING_INSERT();

        rawGlobalMap += pointcloud;
        if (iClouds >= nMaxCloudsPerPly - 1) {
            std::string path = "/root/repo/maps/testoutput" + std::to_string(iPly++) + ".ply";
            std::cout << "saving temporary map to" << path << std::endl;
            int res = pcl::io::savePLYFile(path, rawGlobalMap);
            if (!res) std::cout << "failed saving ply file" << std::endl;
        }
    }

private:
    TSDF_Map tsdfMap;
    uint32_t queueSize = 10;
    ros::Subscriber subPath;
    ros::Subscriber subPcl;

    // temporary stuff for saving raw maps:
    pcl::PointCloud<Point> rawGlobalMap;
    size_t nMaxCloudsPerPly = 500;
    size_t iClouds = 0;
    size_t iPly = 0;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf_node");
    ros::NodeHandle nh;
    TSDF_Node tsdfNode(nh);
    ros::spin();
    return 0;
}