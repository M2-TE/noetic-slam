#include <chrono>
#include <fstream>
#include <random>
#include <iostream>

// // VDBFusion (benchmarking)
// #include <openvdb/openvdb.h>
// #include <vdbfusion/VDBVolume.h>
// #include <matplot/matplot.h>

#include <Eigen/Eigen>
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
#include "dlio_stuff.hpp"
#include "dag_map.hpp"


class TSDF_Node {
public:
    TSDF_Node(ros::NodeHandle nh) {
        // subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDF_Node::callback_pcl_deskewed, this);
        subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDF_Node::callback_pcl_deskewed, this);
        
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
                dagMap.insert_scan(position, Eigen::Quaternionf::Identity(), points);
            }
            // std::ofstream output;
            // output.open("sphere.ascii");
            // for (auto& point: points) {
            //     output << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
            // }
            // output.close();

            // // benchmarking VDBFusion
            // std::vector<Eigen::Vector3d> pointsD(100'000);
            // std::uniform_real_distribution<double> disD(-10.0f, 10.0f);
            // openvdb::initialize();
            // vdbfusion::VDBVolume tsdf_volume(0.02, 0.06, true);
            // Eigen::Vector3d origin(0.0, 0.0, 0.0);
            // for (size_t i = 0; i < 10; i++) {
            //     for (auto& point: pointsD) {
            //         point = {
            //             disD(gen),
            //             disD(gen),
            //             disD(gen)
            //         };
            //         // point.normalize();
            //         // point *= 10.0f;
            //     }
            //     auto beg = std::chrono::steady_clock::now();
            //     tsdf_volume.Integrate(pointsD, origin, [](float){return 1.0;});
            //     auto end = std::chrono::steady_clock::now();
            //     auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            //     std::cout << "VDBFusion: " << dur << " ms" << std::endl;
            // }
            
            dagMap.print_stats();
            dagMap.save_h5();
            exit(0);
        }
    }
    ~TSDF_Node() {
        dagMap.print_stats();
        dagMap.save_h5();
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

        auto now = std::chrono::steady_clock::now();
        prev = now;

        // insert into tsdf DAG
        Eigen::Vector3f position = {};
        Eigen::Quaternionf rotation = {};
        std::cout << "Inserting " << points.size() << " points.\n";
        dagMap.insert_scan(position, rotation, points);
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