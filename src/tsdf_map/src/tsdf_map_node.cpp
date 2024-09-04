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
#include "dag/dag.hpp"
#include "chad_grid.hpp"
#include "chad_reconstruction.hpp"

// todo: move headers to the places that need them
// todo: separate source for vulkan stuff as well
// todo: use fmt for logging

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

bool DEBUG_RECORD_POINTS = false;

class TSDFMap {
public:
    TSDFMap(ros::NodeHandle nh) {
        subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        // subPcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        
        if (true) {
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
                dag.insert(points, position, Eigen::Quaternionf::Identity());
            }
            dag.print_stats();
            reconstruct();
            // std::ofstream output;
            // output.open("sphere.ascii");
            // for (auto& point: points) {
            //     output << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
            // }
            // output.close();
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
            dag.insert(points, position, Eigen::Quaternionf::Identity());
            dag.print_stats();
            reconstruct();
            exit(0);
        }
    }
    ~TSDFMap() {
        if (DEBUG_RECORD_POINTS) {
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
        // dag.reconstruct();
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
            double voxel_res = dag.get_voxel_resolution();
            auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, leaf_level, voxel_res);
            
            // // store all the points into .grid file
            // std::ofstream output;
            // output.open("hashgrid.grid", std::ofstream::trunc | std::ofstream::binary);
            // // store header data
            // float voxel_res = leafResolution;
            // output.write(reinterpret_cast<char*>(&voxel_res), sizeof(float));
            // size_t n_scan_points = grid_points.size();
            // output.write(reinterpret_cast<char*>(&n_scan_points), sizeof(size_t));
            // size_t n_query_points = pGrid->getQueryPoints().size();
            // output.write(reinterpret_cast<char*>(&n_query_points), sizeof(size_t));
            // size_t n_cells = pGrid->getNumberOfCells();
            // output.write(reinterpret_cast<char*>(&n_cells), sizeof(size_t));
            // // store raw scan points (vec3)
            // for (auto cur = grid_points.cbegin(); cur != grid_points.cend(); cur++) {
            //     const Eigen::Vector3f* ptr = &(*cur);
            //     output.write(reinterpret_cast<const char*>(ptr), sizeof(Eigen::Vector3f));
            // }
            // // store query points (vec3 + float)
            // auto& query_points = pGrid->getQueryPoints();
            // for (auto cur = query_points.cbegin(); cur != query_points.cend(); cur++) {
            //     auto _pos = cur->m_position;
            //     Eigen::Vector3f pos { _pos.x, _pos.y, _pos.z };
            //     float signed_distance = cur->m_distance;
            //     output.write(reinterpret_cast<const char*>(&pos), sizeof(Eigen::Vector3f));
            //     output.write(reinterpret_cast<const char*>(&signed_distance), sizeof(float));
            // }
            // // store cells (8x uint32_t)
            // for (auto cur = pGrid->firstCell(); cur != pGrid->lastCell(); cur++) {
            //     auto* cell = cur->second;
            //     for (size_t i = 0; i < 8; i++) {
            //         uint32_t i_query_point = cell->getVertex(i);
            //         output.write(reinterpret_cast<const char*>(&i_query_point), sizeof(uint32_t));
            //     }
            // }
            // output.close();
            
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
        if (DEBUG_RECORD_POINTS) {
            ALL_POINTS_DEBUG.insert(ALL_POINTS_DEBUG.end(), points.cbegin(), points.cend());
        }
        
        // dag.insert_scan(position, rotation, points);
        
        // DEBUG
        if (false) {
            dag.print_stats();
            exit(0);
        }
    }

private:
    DAG dag;
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