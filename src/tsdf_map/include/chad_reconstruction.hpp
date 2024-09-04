#pragma once
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include "chad_grid.hpp"

template<typename BaseVecT, typename BoxT>
struct ChadReconstruction: public lvr2::FastReconstructionBase<BaseVecT> {
    ChadReconstruction(shared_ptr<ChadGrid<BaseVecT, BoxT>> grid) {
        m_grid = grid;
    }
    virtual ~ChadReconstruction() {
    }
    void getMesh(
        lvr2::BaseMesh<BaseVecT>& mesh,
        lvr2::BoundingBox<BaseVecT>& bb,
        vector<unsigned int>& duplicates,
        float comparePrecision) override {
    }
    void getMesh(lvr2::BaseMesh<BaseVecT> &mesh) override {
        // Status message for mesh generation
        string comment = lvr2::timestamp.getElapsedTime() + "Creating mesh ";
        lvr2::ProgressBar progress(m_grid->getNumberOfCells(), comment);

        // Some pointers
        BoxT* b;
        unsigned int global_index = mesh.numVertices();

        // Iterate through cells and calculate local approximations
        typename unordered_map<size_t, BoxT*>::iterator it;
        for(it = m_grid->firstCell(); it != m_grid->lastCell(); it++)
        {
            b = it->second;
            b->getSurface(mesh, m_grid->getQueryPoints(), global_index);
            if(!lvr2::timestamp.isQuiet())
                ++progress;
        }

        if(!lvr2::timestamp.isQuiet())
            cout << endl;

        lvr2::BoxTraits<BoxT> traits;

        if(traits.type == "SharpBox")  // Perform edge flipping for extended marching cubes
        {
            string SFComment = lvr2::timestamp.getElapsedTime() + "Flipping edges  ";
            lvr2::ProgressBar SFProgress(this->m_grid->getNumberOfCells(), SFComment);
            for(it = this->m_grid->firstCell(); it != this->m_grid->lastCell(); it++)
            {

                lvr2::SharpBox<BaseVecT>* sb;
                sb = reinterpret_cast<lvr2::SharpBox<BaseVecT>* >(it->second);
                if(sb->m_containsSharpFeature)
                {
                    lvr2::OptionalVertexHandle v1;
                    lvr2::OptionalVertexHandle v2;
                    lvr2::OptionalEdgeHandle e;

                    if(sb->m_containsSharpCorner)
                    {
                        // 1
                        v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][0]];
                        v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][1]];

                        if(v1 && v2)
                        {
                            e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                            if(e)
                            {
                                mesh.flipEdge(e.unwrap());
                            }
                        }

                        // 2
                        v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][2]];
                        v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][3]];

                        if(v1 && v2)
                        {
                            e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                            if(e)
                            {
                                mesh.flipEdge(e.unwrap());
                            }
                        }

                        // 3
                        v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][4]];
                        v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][5]];

                        if(v1 && v2)
                        {
                            e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                            if(e)
                            {
                                mesh.flipEdge(e.unwrap());
                            }
                        }

                    }
                    else
                    {
                        // 1
                        v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][0]];
                        v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][1]];

                        if(v1 && v2)
                        {
                            e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                            if(e)
                            {
                                mesh.flipEdge(e.unwrap());
                            }
                        }

                        // 2
                        v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][4]];
                        v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][5]];

                        if(v1 && v2)
                        {
                            e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                            if(e)
                            {
                                mesh.flipEdge(e.unwrap());
                            }
                        }
                    }
                }
                ++SFProgress;
            }
            cout << endl;
        }

        if(traits.type == "BilinearFastBox")
        {
            string comment = lvr2::timestamp.getElapsedTime() + "Optimizing plane contours  ";
            lvr2::ProgressBar progress(this->m_grid->getNumberOfCells(), comment);
            for(it = this->m_grid->firstCell(); it != this->m_grid->lastCell(); it++)
            {
            // F... type safety. According to traits object this is OK!
                lvr2::BilinearFastBox<BaseVecT>* box = reinterpret_cast<lvr2::BilinearFastBox<BaseVecT>*>(it->second);
                box->optimizePlanarFaces(mesh, 5);
                ++progress;
            }
            cout << endl;
        }
    }
private:
    shared_ptr<ChadGrid<BaseVecT, BoxT>> m_grid;
};