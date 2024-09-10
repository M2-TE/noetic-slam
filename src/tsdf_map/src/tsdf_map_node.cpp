#include <chrono>
#include <thread>
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
#include <fmt/base.h>
// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// Voxblox
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
// VDBFusion
#include <vdbfusion/VDBVolume.h>
// Other
#include <Eigen/Eigen>
#include "dag/dag.hpp"
#include "chad_grid.hpp"
#include "chad_reconstruction.hpp"


// todo: move headers to the places that need them
// todo: separate source for vulkan stuff as well

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
}
EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (float, time, time)
    (double, timestamp, timestamp))

class TSDFMap {
public:
    TSDFMap(ros::NodeHandle nh) {
        sub_pcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        // sub_pcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        sub_pos = nh.subscribe("/robot/dlio/odom_node/pose", queueSize, &TSDFMap::callback_pos, this);
        return;

        std::vector<Eigen::Vector3f> points;
        // simulate sphere
        if (false) {
            // generate random point data
            points.resize(1'000);
            std::random_device rd;
            std::mt19937 gen(420);
            std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

            // insert into tsdf DAG
            std::vector<Eigen::Vector3f> positions = {
                { 0, 0, 0 },
            };
            for (auto i = 0; i < 1000; i++) {
                positions.emplace_back(0.0f, 0.0f, 0.0f);
            }
            for (size_t i = 0; i < positions.size(); i++) {
                for (auto& point: points) {
                    Eigen::Vector3d pointd = {
                        dis(gen),
                        dis(gen),
                        dis(gen)
                    };
                    pointd.normalize();
                    pointd *= 1.0;
                    point = pointd.cast<float>();
                    point += positions[i];
                }
                dag.insert(points, positions[i], Eigen::Quaternionf::Identity());
                // dag.print_stats();
            }
            reconstruct();
            exit(0);
        }
        // load a bunch of points
        else if (true) {
            std::ifstream file;
            file.open("debugpoints.bin", std::ofstream::binary);
            size_t n_points;
            file.read(reinterpret_cast<char*>(&n_points), sizeof(size_t));
            points.resize(n_points);
            for (auto& point: points) {
                file.read(reinterpret_cast<char*>(&point), sizeof(Eigen::Vector3f));
            }
        }

        // std::this_thread::sleep_for(std::chrono::seconds(10)); // pause to allow more precise measurement of heap allocs
        // chad_tsdf backend
        if (true) {
            Eigen::Vector3f position{ 0, 0, 0 };
            Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
            dag.insert(points, position, rotation);
            dag.print_stats();
            reconstruct();
            exit(0);
        }
        // octomap backend
        else if (false) {
            octomap::Pointcloud cloud;
            for (auto& point: points) {
                cloud.push_back({ point.x(), point.y(), point.z() });
            }

            auto beg = std::chrono::high_resolution_clock::now();
            octomap::OcTree tree{ LEAF_RESOLUTION };
            tree.insertPointCloud(cloud, { 0, 0, 0 });
            tree.updateInnerOccupancy();
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
            fmt::println("octomap {}", dur.count());

            // fmt::println("{} MiB", (double)tree.memoryFullGrid() / 1024.0 / 1024.0);
            fmt::println("{} MiB", (double)tree.memoryUsage() / 1024.0 / 1024.0);

            exit(0);
        }
        // voxblox backend
        else if (false) {
            // set up map
            voxblox::TsdfMap::Config cfg;
            cfg.tsdf_voxel_size = LEAF_RESOLUTION;
            cfg.tsdf_voxels_per_side = 16;
            voxblox::TsdfMap map{ cfg };
            // set up integrator
            voxblox::TsdfIntegratorBase::Config int_cfg;
            int_cfg.default_truncation_distance = 0.1;
            // voxblox::FastTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // faster integration
            voxblox::MergedTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // smaller footprint
            // voxblox::SimpleTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // ew
            // write into voxblox pointcloud
            voxblox::Pointcloud pointcloud;
            voxblox::Colors colors;
            for (auto& point: points) {
                pointcloud.emplace_back(point.x(), point.y(), point.z());
                colors.emplace_back(0, 255, 0);
            }
            // integrate pointcloud

            auto beg = std::chrono::high_resolution_clock::now();
            voxblox::Transformation T_G_C;
            integrator.integratePointCloud(T_G_C, pointcloud, colors);
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
            fmt::println("voxblox ctor {}", dur.count());
            exit(0);
        }
        // VDBFusion backend
        else if (false) {
            // truncation of same size as voxel size
            vdbfusion::VDBVolume volume(LEAF_RESOLUTION, LEAF_RESOLUTION);
            // vdbfusion::VDBFusion fusion(volume);
            Eigen::Vector3d position{ 0, 0, 0 };
            // convert to double prec points
            std::vector<Eigen::Vector3d> pointsd;
            pointsd.reserve(points.size());
            for (auto& point: points) {
                pointsd.emplace_back(point.cast<double>());
            }
            // integrate into tsdf volume
            auto beg = std::chrono::high_resolution_clock::now();
            volume.Integrate(pointsd, position, [](float f){ return f; });
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
            fmt::println("vdbfusion ctor {}", dur.count());
            // fusion.save("vdbfusion.vdb");
            exit(0);
        }
    }
    ~TSDFMap() {
        dag.print_stats();
        reconstruct();
    }
    
    void reconstruct() {
        typedef lvr2::BaseVector<float> VecT;
        typedef lvr2::BilinearFastBox<VecT> BoxT;
        
        // create hash grid from entire tree
        // generate mesh from hash grid
        lvr2::PMPMesh<VecT> mesh{};
        std::string decomp_type = "PMC";
        if (decomp_type == "MC") {
        }
        else if (decomp_type == "PMC") {
            auto node_levels = dag.get_node_levels();
            auto leaf_level = dag.get_leaf_level();
            auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, leaf_level, LEAF_RESOLUTION);
            grid_p->saveGrid("hashgrid.grid");
            
            ChadReconstruction<VecT, BoxT> reconstruction { grid_p };
            reconstruction.getMesh(mesh);
        }
        
        // generate mesh buffer from reconstructed mesh
        auto norm_face = lvr2::calcFaceNormals(mesh);
        auto norm_vert = lvr2::calcVertexNormals(mesh, norm_face);
        lvr2::MeshBufferPtr mesh_buffer_p;
        if (false) {
            // coloring
            auto cluster_map = lvr2::planarClusterGrowing(mesh, norm_face, 0.85);
            lvr2::ClusterPainter cluster_painter { cluster_map };
            lvr2::ColorGradient::GradientType t = lvr2::ColorGradient::gradientFromString("GREY");
            auto cluster_colors = boost::optional<lvr2::DenseClusterMap<lvr2::RGB8Color>>(cluster_painter.colorize(mesh, t));
            lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer { cluster_map };
            finalizer.setClusterColors(*cluster_colors);
            finalizer.setVertexNormals(norm_vert);
            mesh_buffer_p = finalizer.apply(mesh);
        }
        else {
            // calc normals for vertices
            lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
            finalizer.setNormalData(norm_vert);
            mesh_buffer_p = finalizer.apply(mesh);
        }

        // save to disk
        auto model_p = std::make_shared<lvr2::Model>(mesh_buffer_p);
        lvr2::ModelFactory::saveModel(model_p, "yeehaw.ply");
        std::cout << "Saved mesh to yeehaw.ply\n";
    }
    void callback_pos(const geometry_msgs::PoseStampedConstPtr& msg) {
        // extract position
        cur_pos = {
            (float)msg->pose.position.x,
            (float)msg->pose.position.y,
            (float)msg->pose.position.z
        };
        // extract rotation
        cur_rot = {
            (float)msg->pose.orientation.w,
            (float)msg->pose.orientation.x,
            (float)msg->pose.orientation.y,
            (float)msg->pose.orientation.z
        };
    }
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
        fmt::println("Inserting {} points at pos {} {} {}", points.size(), cur_pos.x(), cur_pos.y(), cur_pos.z());
        dag.insert(points, cur_pos, cur_rot);
        
        // write raw points to file for debug purposes
        // fmt::println("writing {} points", points.size());
        // ALL_POINTS_DEBUG.insert(ALL_POINTS_DEBUG.end(), points.cbegin(), points.cend());

        // DEBUG
        static int i = 0;
        if (++i >= 80) {
            // std::ofstream file;
            // file.open("debugpoints.bin", std::ofstream::binary | std::ofstream::trunc);
            // size_t n_points = ALL_POINTS_DEBUG.size();
            // file.write(reinterpret_cast<char*>(&n_points), sizeof(size_t));
            // for (auto& point: ALL_POINTS_DEBUG) {
            //     file.write(reinterpret_cast<char*>(&point), sizeof(Eigen::Vector3f));
            // }
            // std::cout << "Wrote points to debugpoints.bin\n";
            dag.print_stats();
            reconstruct();
            exit(0);
        }
    }

private:
    DAG dag;
    Eigen::Vector3f cur_pos = { 0, 0, 0 };
    Eigen::Quaternionf cur_rot = {};
    uint32_t queueSize = 100;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_pos;
    // std::vector<Eigen::Vector3f> ALL_POINTS_DEBUG;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf_map_node");
    ros::NodeHandle nh;
    TSDFMap node { nh };
    ros::spin();
    return 0;
}