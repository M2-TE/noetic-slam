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
        lvr2::FastReconstruction<VecT, lvr2::FastBox<VecT>> reconstruction(pGrid);
        reconstruction.getMesh(mesh);
    }
    else if (decompositionType == "PMC") {
        auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::BilinearFastBox<VecT>>>(nodeLevelRef, leafResolution);
        lvr2::FastReconstruction<VecT, lvr2::BilinearFastBox<VecT>> reconstruction(pGrid);
        reconstruction.getMesh(mesh);
    }
    
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