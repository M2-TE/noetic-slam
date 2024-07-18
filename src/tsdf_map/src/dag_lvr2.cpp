#include "dag/dag.hpp"
//
#include <placeholder/HashGridDag.tcc>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/geometry/PMPMesh.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
//
#include "dag/node_levels.hpp"

void Dag::reconstruct() {
    std::vector<std::vector<uint32_t>*> nodeLevelRef;
    for (auto i = 0; i < 63/3; i++) nodeLevelRef.push_back(&node_levels[i].data);
    nodeLevelRef.push_back(&leaf_level->data);
    
    ///////////////////////// LVR2 ///////////////////////////
    typedef lvr2::BaseVector<float> VecT;
    
    // create hash grid from entire tree
    // generate mesh from hash grid
    lvr2::PMPMesh<VecT> mesh{};
    constexpr std::string_view decompositionType = "MC";
    if (decompositionType == "MC") {
        auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::FastBox<VecT>>>(nodeLevelRef, leafResolution);
        
        // store all the points into .grid file
        std::ofstream output;
        output.open("hashgrid.grid", std::ofstream::trunc | std::ofstream::binary);
        // store header data
        float voxel_res = leafResolution;
        output.write(reinterpret_cast<char*>(&voxel_res), sizeof(float));
        size_t n_scan_points = grid_points.size();
        output.write(reinterpret_cast<char*>(&n_scan_points), sizeof(size_t));
        size_t n_query_points = pGrid->getQueryPoints().size();
        output.write(reinterpret_cast<char*>(&n_query_points), sizeof(size_t));
        size_t n_cells = pGrid->getNumberOfCells();
        output.write(reinterpret_cast<char*>(&n_cells), sizeof(size_t));
        // store raw scan points (vec3)
        for (auto cur = grid_points.cbegin(); cur != grid_points.cend(); cur++) {
            const Eigen::Vector3f* ptr = &(*cur);
            output.write(reinterpret_cast<const char*>(ptr), sizeof(Eigen::Vector3f));
        }
        // store query points (vec3 + float)
        auto& query_points = pGrid->getQueryPoints();
        for (auto cur = query_points.cbegin(); cur != query_points.cend(); cur++) {
            auto _pos = cur->m_position;
            Eigen::Vector3f pos { _pos.x, _pos.y, _pos.z };
            float signed_distance = cur->m_distance;
            output.write(reinterpret_cast<const char*>(&pos), sizeof(Eigen::Vector3f));
            output.write(reinterpret_cast<const char*>(&signed_distance), sizeof(float));
        }
        // store cells (8x uint32_t)
        for (auto cur = pGrid->firstCell(); cur != pGrid->lastCell(); cur++) {
            auto* cell = cur->second;
            for (size_t i = 0; i < 8; i++) {
                uint32_t i_query_point = cell->getVertex(i);
                output.write(reinterpret_cast<const char*>(&i_query_point), sizeof(uint32_t));
            }
        }
        output.close();
        
        
        lvr2::FastReconstruction<VecT, lvr2::FastBox<VecT>> reconstruction(pGrid);
        reconstruction.getMesh(mesh);
    }
    // else if (decompositionType == "PMC") {
    //     auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::BilinearFastBox<VecT>>>(nodeLevelRef, leafResolution);
    //     lvr2::FastReconstruction<VecT, lvr2::BilinearFastBox<VecT>> reconstruction(pGrid);
    //     reconstruction.getMesh(mesh);
    // }
    
    // generate mesh buffer from reconstructed mesh
    auto faceNormals = lvr2::calcFaceNormals(mesh);
    auto vertexNormals = lvr2::calcVertexNormals(mesh, faceNormals);
    lvr2::MeshBufferPtr meshBuffer;
    if (false) {
        // coloring
        auto clusterBiMap = lvr2::planarClusterGrowing(mesh, faceNormals, 0.85);
        lvr2::ClusterPainter painter(clusterBiMap);
        lvr2::ColorGradient::GradientType t = lvr2::ColorGradient::gradientFromString("GREY");
        auto clusterColors = boost::optional<lvr2::DenseClusterMap<lvr2::RGB8Color>>(painter.colorize(mesh, t));
        lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer(clusterBiMap);
        finalizer.setClusterColors(*clusterColors);
        finalizer.setVertexNormals(vertexNormals);
        meshBuffer = finalizer.apply(mesh);
    }
    else {
        // Calc normals for vertices
        lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
        finalizer.setNormalData(vertexNormals);
        meshBuffer = finalizer.apply(mesh);
    }

    // save to disk
    auto model = std::make_shared<lvr2::Model>(meshBuffer);
    lvr2::ModelFactory::saveModel(model, "yeehaw.ply");
}