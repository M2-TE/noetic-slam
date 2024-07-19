#include <chrono>
#include <cstddef>
#include <fstream>
#include <random>
#include <iostream>

// ros crap
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
//
#include <Eigen/Eigen>
#include "dlio_stuff.hpp"
#include "dag/dag.hpp"

// todo: move headers to the places that need them
// todo: separate source for vulkan stuff as well
// todo: use fmt for logging

class TSDFMap {
public:
    TSDFMap(ros::NodeHandle nh) {
        subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        // subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        
        if (false) {
            // generate random point data
            std::vector<Eigen::Vector3f> points(100'000);
            std::random_device rd;
            std::mt19937 gen(420);
            std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

            // insert into tsdf DAG
            Eigen::Vector3f position { 0, 0, 0 };
            // Eigen::Vector3f position { 10, 10, 10 };
            // Eigen::Vector3f position { -10, -10, -10 };
            for (size_t i = 0; i < 1; i++) {
                for (auto& point: points) {
                    Eigen::Vector3d pointd = {
                        dis(gen),
                        dis(gen),
                        dis(gen)
                    };
                    pointd.normalize();
                    pointd *= 5.0;
                    point = pointd.cast<float>();
                    point += position;
                }
                dag.insert_scan(position, Eigen::Quaternionf::Identity(), points);
            }
            // std::ofstream output;
            // output.open("sphere.ascii");
            // for (auto& point: points) {
            //     output << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
            // }
            // output.close();
            
            dag.print_stats();
            dag.reconstruct();
            exit(0);
        }
        else if (true) {
            
            std::ifstream file;
            file.open("debugpoints.bin", std::ofstream::binary);
            size_t n_points;
            file.read(reinterpret_cast<char*>(&n_points), sizeof(size_t));
            std::vector<Eigen::Vector3f> points(n_points);
            for (auto& point: points) {
                file.read(reinterpret_cast<char*>(&point), sizeof(Eigen::Vector3f));
            }
            Eigen::Vector3f position { 0, 0, 0 };
            dag.insert_scan(position, Eigen::Quaternionf::Identity(), points);
            
            dag.print_stats();
            dag.reconstruct();
            exit(0);
        }
    }
    ~TSDFMap() {
        if (false) {
            std::ofstream file;
            file.open("debugpoints.bin", std::ofstream::binary | std::ofstream::trunc);
            size_t n_points = ALL_POINTS_DEBUG.size();
            file.write(reinterpret_cast<char*>(&n_points), sizeof(size_t));
            for (auto& point: ALL_POINTS_DEBUG) {
                file.write(reinterpret_cast<char*>(&point), sizeof(Eigen::Vector3f));
            }
            std::cout << "Wrote points to debugpoints.bin\n";
        }
        
        dag.print_stats();
        dag.reconstruct();
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
        pointcloud.clear();

        // insert into tsdf DAG
        Eigen::Vector3f position = {};
        Eigen::Quaternionf rotation = {};
        std::cout << "Inserting " << points.size() << " points.\n";
        
        // write raw points to file for debug purposes
        if (false) {
            ALL_POINTS_DEBUG.insert(ALL_POINTS_DEBUG.end(), points.cbegin(), points.cend());
        }
        
        // dag.insert_scan(position, rotation, points);
        
        // DEBUG
        if (false) {
            dag.print_stats();
            dag.reconstruct();
            exit(0);
        }
    }

private:
    Dag dag;
    uint32_t queueSize = 100;
    ros::Subscriber subPath;
    ros::Subscriber subPcl;
    std::vector<Eigen::Vector3f> ALL_POINTS_DEBUG;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf_map_node");
    ros::NodeHandle nh;
    TSDFMap node { nh };
    ros::spin();
    return 0;
}