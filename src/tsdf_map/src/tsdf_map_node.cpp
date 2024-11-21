#include <chrono>
#include <linux/sysinfo.h>
#include <thread>
#include <cstddef>
#include <fstream>
#include <random>
#include <iostream>

// ros crap
#include <ros/ros.h>
#include <ros/callback_queue.h>
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
#include <fmt/format.h>
// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// Voxblox
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
// VDBFusion
#include <vdbfusion/VDBVolume.h>
#include </root/repo/build/tsdf_map/_deps/libigl-src/include/igl/write_triangle_mesh.h> // temp fix for docker
// Other
#include <Eigen/Eigen>
#include "dag/dag.hpp"
#include "chad_grid.hpp"
#include "chad_reconstruction.hpp"
#include "utils.hpp"

#define MAP_BACKEND_IDX 0

class TSDFMap {
public:
    TSDFMap(ros::NodeHandle nh) {
        // sub_pcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        sub_pcl = nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", queueSize, &TSDFMap::callback_pcl_deskewed, this);
        sub_pos = nh.subscribe("/robot/dlio/odom_node/pose", queueSize, &TSDFMap::callback_pos, this);

        // initialize backend of choice
        switch (MAP_BACKEND_IDX) {
            case 0:
                dag_p = new DAG{};
                break;
            case 1:
                ocmap_p = new octomap::OcTree{ LEAF_RESOLUTION };
                break;
            case 2: {
                    voxblox::TsdfMap::Config cfg;
                    cfg.tsdf_voxel_size = LEAF_RESOLUTION;
                    cfg.tsdf_voxels_per_side = 16;
                    voxmap_p = new voxblox::TsdfMap{ cfg };
                    voxblox::TsdfIntegratorBase::Config int_cfg;
                    int_cfg.default_truncation_distance = LEAF_RESOLUTION * 2;
                    integrator_p = new voxblox::FastTsdfIntegrator{ int_cfg, voxmap_p->getTsdfLayerPtr() }; // faster integration
                    // integrator_p = new voxblox::MergedTsdfIntegrator{ int_cfg, voxmap_p->getTsdfLayerPtr() }; // smaller footprint
                    // integrator_p = new voxblox::SimpleTsdfIntegrator{ int_cfg, voxmap_p->getTsdfLayerPtr() }; // ew
                }
                break;
            case 3:
                openvdb::initialize();
                vdbmap_p = new vdbfusion::VDBVolume{ LEAF_RESOLUTION, LEAF_RESOLUTION * 2, false };
                break;
            default: break;
        }

        // // chad_tsdf backend
        // if (true) {
        //     Eigen::Vector3f position{ 0, 0, 0 };
        //     Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
        //     dag.insert(points, position, rotation);
        //     save_chad();
        //     exit(0);
        // }
        // // octomap backend
        // else if (false) {
        //     octomap::Pointcloud cloud;
        //     for (auto& point: points) {
        //         cloud.push_back({ point.x(), point.y(), point.z() });
        //     }
        //     auto beg = std::chrono::high_resolution_clock::now();
        //     octomap::OcTree tree{ LEAF_RESOLUTION };
        //     tree.insertPointCloud(cloud, { 0, 0, 0 });
        //     tree.updateInnerOccupancy();
        //     auto end = std::chrono::high_resolution_clock::now();
        //     auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
        //     fmt::println("octomap {}", dur.count());
        //     // fmt::println("{} MiB", (double)tree.memoryFullGrid() / 1024.0 / 1024.0);
        //     fmt::println("{} MiB", (double)tree.memoryUsage() / 1024.0 / 1024.0);
        //     exit(0);
        // }
        // // voxblox backend
        // else if (false) {
        //     // set up map
        //     voxblox::TsdfMap::Config cfg;
        //     cfg.tsdf_voxel_size = LEAF_RESOLUTION;
        //     cfg.tsdf_voxels_per_side = 16;
        //     voxblox::TsdfMap map{ cfg };
        //     // set up integrator
        //     voxblox::TsdfIntegratorBase::Config int_cfg;
        //     int_cfg.default_truncation_distance = 0.1;
        //     // voxblox::FastTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // faster integration
        //     voxblox::MergedTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // smaller footprint
        //     // voxblox::SimpleTsdfIntegrator integrator{ int_cfg, map.getTsdfLayerPtr() }; // ew
        //     // write into voxblox pointcloud
        //     voxblox::Pointcloud pointcloud;
        //     voxblox::Colors colors;
        //     for (auto& point: points) {
        //         pointcloud.emplace_back(point.x(), point.y(), point.z());
        //         colors.emplace_back(0, 255, 0);
        //     }
        //     // integrate pointcloud
        //     auto beg = std::chrono::high_resolution_clock::now();
        //     voxblox::Transformation T_G_C;
        //     integrator.integratePointCloud(T_G_C, pointcloud, colors);
        //     auto end = std::chrono::high_resolution_clock::now();
        //     auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
        //     fmt::println("voxblox ctor {}", dur.count());
        //     exit(0);
        // }
        // // VDBFusion backend
        // else if (false) {
        //     // truncation of same size as voxel size
        //     vdbfusion::VDBVolume volume(LEAF_RESOLUTION, LEAF_RESOLUTION);
        //     // vdbfusion::VDBFusion fusion(volume);
        //     Eigen::Vector3d position{ 0, 0, 0 };
        //     // convert to double prec points
        //     std::vector<Eigen::Vector3d> pointsd;
        //     pointsd.reserve(points.size());
        //     for (auto& point: points) {
        //         pointsd.emplace_back(point.cast<double>());
        //     }
        //     // integrate into tsdf volume
        //     auto beg = std::chrono::high_resolution_clock::now();
        //     volume.Integrate(pointsd, position, [](float f){ return f; });
        //     auto end = std::chrono::high_resolution_clock::now();
        //     auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
        //     fmt::println("vdbfusion ctor {}", dur.count());
        //     // fusion.save("vdbfusion.vdb");
        //     exit(0);
        // }
    }
    ~TSDFMap() {
        // TODO: write final timings and memory footprint
        fmt::println("min: {} ms", min.count());
        fmt::println("max: {} ms", max.count());
        fmt::println("avg: {} ms", total.count() / frame_count);
        #if MAP_BACKEND_IDX == 0
            dag_p->print_stats();
            auto beg = std::chrono::high_resolution_clock::now();
            auto count = dag_p->debug_iterate_all_leaves_of_subtree(1);
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            fmt::println("Root total iteration time: {}, leaf count: {}", dur, count);
            // save_points();
            save_chad();
        #elif MAP_BACKEND_IDX == 1
        // lets not..
        #elif MAP_BACKEND_IDX == 2
        // Create a MeshIntegrator
        voxblox::MeshIntegratorConfig mesh_config;
        voxblox::MeshLayer mesh_layer(voxmap_p->block_size());
        voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(mesh_config, voxmap_p->getTsdfLayer(), &mesh_layer);
        mesh_integrator.generateMesh(false, false);
        
        // Save the mesh to a file
        std::string filename = "maps/mesh.ply";
        voxblox::outputMeshLayerAsPly(filename, mesh_layer);

        #elif MAP_BACKEND_IDX == 3
        // generate mesh as per example in repo
        auto [vertices, triangles] = vdbmap_p->ExtractTriangleMesh(true);
        Eigen::MatrixXd V(vertices.size(), 3);
        for (size_t i = 0; i < vertices.size(); i++) {
            V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
        }
        Eigen::MatrixXi F(triangles.size(), 3);
        for (size_t i = 0; i < triangles.size(); i++) {
            F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
        }
        std::string filename = "maps/mesh.ply";
        igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
        #endif
    }
    
    void reconstruct(uint32_t root_addr, std::string_view mesh_name, bool save_grid) {
        typedef lvr2::BaseVector<float> VecT;
        typedef lvr2::BilinearFastBox<VecT> BoxT;
        
        // create hash grid from entire tree
        // generate mesh from hash grid
        lvr2::PMPMesh<VecT> mesh{};
        std::string decomp_type = "PMC";
        if (decomp_type == "MC") {
        }
        else if (decomp_type == "PMC") {
            auto node_levels = dag_p->get_node_levels();
            auto leaf_level = dag_p->get_leaf_level();
            auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, leaf_level, root_addr, LEAF_RESOLUTION);
            if (save_grid) grid_p->saveGrid("hashgrid.grid");
            
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
        lvr2::ModelFactory::saveModel(model_p, mesh_name.data());
        fmt::println("Saved mesh to {}", mesh_name);
    }
    void save_points() {
        std::ofstream file;
        file.open("maps/points.pts", std::ofstream::trunc);
        for (auto& point: all_points) {
            file << point.x() << ',' << point.y() << ',' << point.z() << '\n';
        }
        file.close();
        all_points.clear();
    }
    void save_chad() {
        dag_p->print_stats();
        // for (uint32_t addr_i = 0; addr_i < dag_p->_subtrees.size(); addr_i++) {
        //     uint32_t addr = dag_p->_subtrees[addr_i]._root_addr;
        //     fmt::println("Reconstructing subtree at address {}", addr);
        //     reconstruct(addr, fmt::format("maps/mesh_{}.ply", addr_i), false);
        // }
        // create global map and reconstruct it
        dag_p->merge_all_subtrees();
        // reconstruct(1, "maps/mesh.ply", true);
    }
    
    void insert(
    #if MAP_BACKEND_IDX == 3
        std::vector<Eigen::Vector3d>& pointsd
    #else
        std::vector<Eigen::Vector3f>& points
    #endif
    ) {
        #if MAP_BACKEND_IDX == 0
            dag_p->insert(points, cur_pos, cur_rot);
        #elif MAP_BACKEND_IDX == 1
            octomap::Pointcloud cloud;
            for (auto& point: points) {
                cloud.push_back({ point.x(), point.y(), point.z() });
            }
            ocmap_p->insertPointCloud(cloud, { cur_pos.x(), cur_pos.y(), cur_pos.z() });
            ocmap_p->updateInnerOccupancy();
        #elif MAP_BACKEND_IDX == 2
            voxblox::Pointcloud pointcloud;
            voxblox::Colors colors;
            for (auto& point: points) {
                pointcloud.emplace_back(point.x(), point.y(), point.z());
                colors.emplace_back(0, 0, 0);
            }
            voxblox::Transformation T_G_C { cur_rot, cur_pos };
            integrator_p->integratePointCloud(T_G_C, pointcloud, colors);
        #elif MAP_BACKEND_IDX == 3
            Eigen::Vector3d pos = cur_pos.cast<double>();
            vdbmap_p->Integrate(pointsd, pos, [](float /*unused*/) { return 1.0f; });
        #endif
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
        #if MAP_BACKEND_IDX == 3
            std::vector<Eigen::Vector3d> points;
        #else
            std::vector<Eigen::Vector3f> points;
        #endif
        points.reserve(pointcloud.size());
        for (auto cur = pointcloud.begin(); cur != pointcloud.end(); cur++) {
            auto vec = cur->getVector3fMap();
            points.emplace_back(vec.x(), vec.y(), vec.z());
        }
        // DEBUG
        // all_points.insert(all_points.end(), points.begin(), points.end());
        // static Eigen::Vector3f bb_lower = { 0, 0, 0 };
        // static Eigen::Vector3f bb_upper = { 0, 0, 0 };
        // for (auto& point: points) {
        //     bb_lower = bb_lower.cwiseMin(point);
        //     bb_upper = bb_upper.cwiseMax(point);
        // }

        // insert points
        fmt::println("Inserting {} points", points.size());
        auto beg = std::chrono::high_resolution_clock::now();
        insert(points);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
        fmt::println("insertion time: {}", dur.count());

        // measure physical memory footprint
        #if MAP_BACKEND_IDX == 0
        double mb = (double)read_phys_mem_kb() / 1024.0;
        double mb_read = dag_p->get_readonly_size();
        double mb_hash = dag_p->get_hash_size();
        fmt::println("proc: {} MiB\nread: {} MiB\nhash: {} MiB", mb, mb_read, mb_hash);
        #else
        double mb = (double)read_phys_mem_kb() / 1024.0;
        fmt::println("Memory: {} MiB", mb);
        #endif
        
        // update trackers
        frame_count++;
        total += dur;
        min = std::min(min, dur);
        max = std::max(max, dur);

        #if MAP_BACKEND_IDX == 0
        // iterate through leaves and count
        beg = std::chrono::high_resolution_clock::now();
        uint32_t leafcount = dag_p->debug_iterate_all_leaves_of_subtree(dag_p->_subtrees.back()._root_addr);
        end = std::chrono::high_resolution_clock::now();
        auto dur_iteration = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("iteration time: {}, leaf count: {}", dur_iteration, leafcount);
        #endif


        // write to csv
        std::ofstream file;
        file.open("insertion_times.csv", std::ofstream::app);
        file << frame_count << ',' 
            << mb << ',' 
            << dur.count()
        #if MAP_BACKEND_IDX == 0
            << ',' << dur_iteration << ',' << leafcount
        #endif
            << '\n';
        file.close();
    }

private:
    // data structures for testing:
    DAG* dag_p;
    octomap::OcTree* ocmap_p;
    voxblox::TsdfMap* voxmap_p;
    voxblox::TsdfIntegratorBase* integrator_p;
    vdbfusion::VDBVolume* vdbmap_p;
    std::vector<Eigen::Vector3f> all_points; // DEBUG
    //
    Eigen::Vector3f cur_pos = { 0, 0, 0 };
    Eigen::Quaternionf cur_rot = {};
    uint32_t queueSize = 10;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_pos;
    int baseline_memory;
    std::chrono::milliseconds min = std::chrono::milliseconds::max();
    std::chrono::milliseconds max = std::chrono::milliseconds::min();
    std::chrono::milliseconds total = std::chrono::milliseconds::zero();
    size_t frame_count = 0;
};

int main(int argc, char **argv) {
    fmt::println("Starting tsdf_map_node");
    ros::init(argc, argv, "tsdf_map_node");
    ros::NodeHandle nh;
    TSDFMap node { nh };
    int baseline_memory = read_phys_mem_kb();
    std::ofstream file;
    file.open("insertion_times.csv", std::ofstream::trunc);
    file << "frame_count" << ',' 
        << "memory" << ',' 
        << "duration"
        #if MAP_BACKEND_IDX == 0
        << ',' << "iteration_time" 
        << ',' << "leaf_count"
        #endif
        << '\n';
    file.close();
    ros::spin();
    return 0;
}