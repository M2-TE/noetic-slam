// ROS
#ifndef __INTELLISENSE__
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
#endif
#include <Eigen/Eigen>
#include <chrono>
#include <random>
#include <iostream>
//
#include "dlio_stuff.hpp"
#include "dag_map.hpp"


class TSDF_Node {
public:
    TSDF_Node(ros::NodeHandle nh) {
        subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDF_Node::callback_pcl_deskewed, this);
        // subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDF_Node::callback_pcl_deskewed, this);

        // generate random point data
        std::vector<Eigen::Vector3f> points(100'000);
        std::random_device rd;
        std::mt19937 gen(420);
        std::uniform_real_distribution<float> dis(-10.0f, 10.0f);

        // insert into tsdf DAG
        Eigen::Vector3f position = {};
        Eigen::Quaternionf rotation = {};
        for (size_t i = 0; i < 1; i++) {
            std::random_device rd;
            std::mt19937 gen(420);
            std::uniform_real_distribution<float> dis(-10.0f, 10.0f);
            for (auto& point: points) {
                point = {
                    dis(gen),
                    dis(gen),
                    dis(gen)
                };
                // point.normalize();
                // point *= 10.0f;
            }
            dagMap.insert_scan(position, rotation, points);
        }
        exit(0);
    }

public:
    void callback_pcl_deskewed(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*msg, pointcloud);

        // insert raw points into a non-pcl vector
        std::vector<Eigen::Vector3f> points;
        points.reserve(pointcloud.size());
        for (auto cur = pointcloud.begin(); cur != pointcloud.end(); cur++) {
            points.emplace_back(cur->getVector3fMap());
        }

        auto now = std::chrono::steady_clock::now();
        std::cout << pointcloud.size() << " points with a window of ";
        pointcloud.clear();
        std::cout << std::chrono::duration<double, std::milli>(now - prev).count() << " ms" << std::endl;
        prev = now;

        // insert into tsdf DAG
        Eigen::Vector3f position = {};
        Eigen::Quaternionf rotation = {};
        // auto start = std::chrono::steady_clock::now();
        dagMap.insert_scan(position, rotation, points);
        // auto end = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
        // exit(0);
    }

private:
    DAG::Map dagMap;
    uint32_t queueSize = 300000;
    ros::Subscriber subPath;
    ros::Subscriber subPcl;

    // timers
    std::chrono::time_point<std::chrono::steady_clock> prev;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf_node");
    ros::NodeHandle nh;
    TSDF_Node tsdfNode(nh);
    ros::spin();
    return 0;
}