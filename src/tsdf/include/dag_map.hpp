#pragma once
#include <algorithm>
#include <atomic>
#include <execution>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <array>
#include <cmath>
#include <span>
#include <thread>
#include <parallel/algorithm>
#include <cstdint>
//
#include <eigen3/Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <morton-nd/mortonND_BMI2.h>
// #include <highfive/highfive.hpp>
// #include <highfive/boost.hpp>
// #include <highfive/eigen.hpp>
#include <highfive/H5File.hpp>
#include <placeholder/HashGridDag.tcc>
#include <lvr2/reconstruction/HashGrid.hpp>
//
#include "dag_constants.hpp"
#include "octree.hpp"
#include "dag_structs.hpp"
#include "leaf_cluster.hpp"
#include "lvr2/geometry/PMPMesh.hpp"
#include "lvr2/reconstruction/FastBox.hpp"
#include "lvr2/reconstruction/FastReconstruction.hpp"
#include "lvr2/algorithm/NormalAlgorithms.hpp"


// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
static Eigen::Vector3f normal_from_neighbourhood(std::vector<Eigen::Vector3f>& points) {
	// calculate centroid by through coefficient average
	Eigen::Vector3f centroid { 0, 0, 0 };
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		centroid += *p;
	}
	double recip = 1.0 / (double)points.size();
	centroid *= recip;

	// covariance matrix excluding symmetries
	double xx = 0.0; double xy = 0.0; double xz = 0.0;
	double yy = 0.0; double yz = 0.0; double zz = 0.0;
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		auto r = *p - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	xx *= recip;
	xy *= recip;
	xz *= recip;
	yy *= recip;
	yz *= recip;
	zz *= recip;

	// weighting linear regression based on square determinant
	Eigen::Vector3f weighted_dir = { 0, 0, 0 };

	// determinant x
	{
		double det_x = yy*zz - yz*yz;
		Eigen::Vector3f axis_dir = {
			(float)(det_x),
			(float)(xz*yz - xy*zz),
			(float)(xy*yz - xz*yy)
		};
		double weight = det_x * det_x;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant y
	{
		double det_y = xx*zz - xz*xz;
		Eigen::Vector3f axis_dir = {
			(float)(xz*yz - xy*zz),
			(float)(det_y),
			(float)(xy*xz - yz*xx)
		};
		double weight = det_y * det_y;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant z
	{
		double det_z = xx*yy - xy*xy;
		Eigen::Vector3f axis_dir = {
			(float)(xy*yz - xz*yy),
			(float)(xy*xz - yz*xx),
			(float)(det_z)
		};
		double weight = det_z * det_z;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}

	// return normalized weighted direction as surface normal
	weighted_dir.normalize();
	return weighted_dir;
}
static Eigen::Vector3f normal_from_neighbourhood(std::vector<Eigen::Vector4d>& points) {
	// calculate centroid by through coefficient average
	Eigen::Vector4d centroid { 0, 0, 0, 0 };
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		centroid += *p;
	}
	double recip = 1.0 / (double)points.size();
	centroid *= recip;

	// covariance matrix excluding symmetries
	double xx = 0.0; double xy = 0.0; double xz = 0.0;
	double yy = 0.0; double yz = 0.0; double zz = 0.0;
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		auto r = *p - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	xx *= recip;
	xy *= recip;
	xz *= recip;
	yy *= recip;
	yz *= recip;
	zz *= recip;

	// weighting linear regression based on square determinant
	Eigen::Vector4d weighted_dir = { 0, 0, 0, 0 };

	// determinant x
	{
		double det_x = yy*zz - yz*yz;
		Eigen::Vector4d axis_dir = {
			det_x,
			xz*yz - xy*zz,
			xy*yz - xz*yy,
			0.0
		};
		double weight = det_x * det_x;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant y
	{
		double det_y = xx*zz - xz*xz;
		Eigen::Vector4d axis_dir = {
			xz*yz - xy*zz,
			det_y,
			xy*xz - yz*xx,
			0.0
		};
		double weight = det_y * det_y;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant z
	{
		double det_z = xx*yy - xy*xy;
		Eigen::Vector4d axis_dir = {
			xy*yz - xz*yy,
			xy*xz - yz*xx,
			det_z,
			0.0
		};
		double weight = det_z * det_z;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}

	// return normalized weighted direction as surface normal
	weighted_dir.normalize();
	return { (float)weighted_dir.x(), (float)weighted_dir.y(), (float)weighted_dir.z() };
}

namespace DAG {
	static std::vector<std::pair<double, std::string>> measurements;
	struct Map {
		static auto calc_morton(std::vector<Eigen::Vector3f>& points) {
			auto beg = std::chrono::steady_clock::now();
			
			// create a vector to hold sortable morton codes alongside position
			std::vector<std::pair<MortonCode, Eigen::Vector3f>> mortonCodes;
			mortonCodes.reserve(points.size());
			
			// iterate through all points to populate morton code vector
			for (auto it_points = points.cbegin(); it_points != points.cend(); it_points++) {
				Eigen::Vector3f position = *it_points;
				// transform into chunk position (leaf chunk in this case)
				Eigen::Vector3f chunkPosFloat = position * (1.0 / leafResolution);
				Eigen::Vector3i chunkPos = chunkPosFloat.cast<int32_t>();
				// calculate morton code and insert into vector
				MortonCode mc(chunkPos);
				mortonCodes.emplace_back(mc.val, position);
			}
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "mort calc");
			return mortonCodes;
		}
		static void sort_points(std::vector<Eigen::Vector3f>& points, std::vector<std::pair<MortonCode, Eigen::Vector3f>>& mortonCodes) {
			auto beg = std::chrono::steady_clock::now();
			
			// sort morton code vector
			auto sorter = [](const auto& a, const auto& b){
				return std::get<0>(a) > std::get<0>(b);
			};
			std::sort(std::execution::par_unseq, mortonCodes.begin(), mortonCodes.end(), sorter);
			
			// insert sorted morton code points back into points vector
			auto it_points = points.begin();
			for (auto it_morton = mortonCodes.cbegin(); it_morton != mortonCodes.cend(); it_morton++) {
				*it_points++ = std::get<1>(*it_morton);
			}
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "pnts sort");
		}
		static auto calc_normals(Pose pose, std::vector<std::pair<MortonCode, Eigen::Vector3f>>& mortonCodes) {
			auto beg = std::chrono::steady_clock::now();
			
			// construct neighbourhoods of points
			struct Neighbourhood {
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_beg;
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_end;
			};
			auto calc_neigh_map = [&mortonCodes](size_t neighLevel) {
				// the level up to which is checked to see if two morton codes belong to the same neighbourhood
				size_t mask = std::numeric_limits<size_t>::max() << neighLevel * 3;
				phmap::flat_hash_map<typeof(MortonCode::val), Neighbourhood> neighMap;
				// phmap::btree_map<typeof(MortonCode::val), Neighbourhood> neighMap;
				// std::map<typeof(MortonCode::val), Neighbourhood> neighMap;
				
				// insert the first neighbourhood
				MortonCode mc_neigh = std::get<0>(mortonCodes.front());
				mc_neigh.val &= mask;
				auto it_neigh = neighMap.emplace(mc_neigh.val, Neighbourhood(mortonCodes.cbegin(), mortonCodes.cbegin())).first;
				
				// iterate through all points to build neighbourhoods
				for (auto it_morton = mortonCodes.cbegin() + 1; it_morton != mortonCodes.cend(); it_morton++) {
					// get morton codes from current point and current neighbourhood
					MortonCode mc_point = std::get<0>(*it_morton);
					MortonCode mc_neigh = std::get<0>(*it_neigh->second.it_beg);
					mc_point.val &= mask;
					mc_neigh.val &= mask;
					
					// check if the current point doesnt fit the neighbourhood
					if (mc_neigh != mc_point) {
						// set end() iterator for current neighbourhood
						it_neigh->second.it_end = it_morton;
						// create a new neighbourhood starting at current point
						it_neigh = neighMap.emplace(mc_point.val, Neighbourhood(it_morton, it_morton)).first;
					}
				}
				// set end() iterator for final neighbourhood
				it_neigh->second.it_end = mortonCodes.cend();
				
				return neighMap;
			};
			// use chunk at level 2 (relative from leaves) for neighbourhoods by default
			static size_t neighLevel = 2;
			auto neighMap = calc_neigh_map(neighLevel);
			// rough approximation of points per neighbourhood
			size_t nPts = mortonCodes.size() / neighMap.size();
			// decrease neigh level if too many points per neighbourhood are present
			while (nPts > 20) {
				neighLevel--;
				neighMap = calc_neigh_map(neighLevel);
				// rough approximation of points per neighbourhood
				nPts = mortonCodes.size() / neighMap.size();
				std::cout << "decreased neighbourhood level to: " << neighLevel << '\n';
			}
			// increase neigh level until the desired amount of points per neighbourhood is reached
			while (nPts < 6) {
				neighLevel++;
				neighMap = calc_neigh_map(neighLevel);
				// rough approximation of points per neighbourhood
				nPts = mortonCodes.size() / neighMap.size();
				std::cout << "increased neighbourhood level to: " << neighLevel << '\n';
			}
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "neig calc");
			beg = std::chrono::steady_clock::now();
			
			// build normals using local neighbourhoods
			float maxDist = leafResolution * (1 << neighLevel);
			std::vector<Eigen::Vector3f> normals(mortonCodes.size());
			typedef decltype(neighMap)::const_iterator NeighIter;
			auto normFnc = [&normals, &neighMap, &mortonCodes, pose, maxDist](NeighIter beg, NeighIter end) {
				for (auto it_neigh = beg; it_neigh != end; it_neigh++) {
					// get morton code for current neighbourhood
					MortonCode mc_neigh = std::get<0>(*it_neigh);
					const Neighbourhood& neigh = std::get<1>(*it_neigh);
					Eigen::Vector3i pos_neigh = mc_neigh.decode();
					
					// gather adjacent neighbourhoods
					std::vector<Neighbourhood*> adjNeighs;
					for (int32_t x = -1; x <= +1; x++) {
						for (int32_t y = -1; y <= +1; y++) {
							for (int32_t z = -1; z <= +1; z++) {
								Eigen::Vector3i offset { x, y, z };
								offset *= 1 << neighLevel;
								MortonCode mc_near(pos_neigh + offset);
								// attempt to find adjacent neighbourhood in map
								auto it_near = neighMap.find(mc_near.val);
								if (it_near != neighMap.cend()) {
									Neighbourhood& adj = std::get<1>(*it_near);
									adjNeighs.push_back(&adj);	
								}
							}
						}
					}
					
					// count total points in all adjacent neighbourhoods
					size_t n_points = 0;
					for (auto it_neigh = adjNeighs.cbegin(); it_neigh != adjNeighs.cend(); it_neigh++) {
						Neighbourhood& neigh = **it_neigh;
						n_points += neigh.it_end - neigh.it_beg;
					}
					// collect points from adjacent neighbours
					std::vector<Eigen::Vector3f> localPoints;
					localPoints.reserve(n_points);
					for (auto it_neigh = adjNeighs.cbegin(); it_neigh != adjNeighs.cend(); it_neigh++) {
						Neighbourhood& neigh = **it_neigh;
						for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
							localPoints.push_back(std::get<1>(*it_point));
						}
					}
					
					// reserve some generous space for nearest points
					std::vector<Eigen::Vector4d> nearestPoints;
					nearestPoints.reserve(n_points);
					// for every point within the current neighbourhood it_neigh, find its nearest neighbours for normal calc
					for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
						Eigen::Vector3f point = std::get<1>(*it_point);
						
						// go over all points and store the ones within the boundary
						nearestPoints.clear();
						for (auto it_other = localPoints.cbegin(); it_other != localPoints.cend(); it_other++) {
							Eigen::Vector3f diff = *it_other - point;
							float distSqr = diff.squaredNorm();
							if (distSqr <= maxDist*maxDist) {
								Eigen::Vector3f other = *it_other;
								nearestPoints.emplace_back(other.x(), other.y(), other.z(), 0.0);
							}
						}
						
						// use these filtered nearest points to calc normal
						Eigen::Vector3f normal;
						if (nearestPoints.size() >= 3) normal = normal_from_neighbourhood(nearestPoints);
						else normal = (point - pose.pos).normalized();
						
						// flip normal if needed
						Eigen::Vector3f posToPoint = point - pose.pos;
						float dot = normal.dot(posToPoint);
						if (dot < 0.0f) normal *= -1.0f;
						
						// figure out index of point
						size_t index = it_point - mortonCodes.cbegin();
						normals[index] = normal;
					}
				}
			};
			// balance load across several threads
			std::vector<std::jthread> threads;
			threads.reserve(std::jthread::hardware_concurrency());
			for (size_t iThread = 0; iThread < threads.capacity(); iThread++) {
				// neighbours per thread
				size_t nNeighs = neighMap.size() / threads.capacity();
				// prepare iterators
				NeighIter beg = neighMap.cbegin();
				std::advance(beg, nNeighs * iThread);
				NeighIter end;
				if (iThread == threads.capacity() - 1) {
					end = neighMap.cend();
				}
				else {
					end = beg;
					std::advance(end, nNeighs);
				}
				threads.emplace_back(normFnc, beg, end);
			}
			threads.clear();
			
			end = std::chrono::steady_clock::now();
			dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "norm calc");
			return normals;
		}
		static auto build_trie_whatnot(Octree& octree, Eigen::Vector3f inputPos, Eigen::Vector3f inputNorm, uint32_t tid) {
			// calculate chunk position as a leaf
			constexpr float recip = 1.0 / leafResolution;
			Eigen::Vector3i baseClusterChunk = (inputPos * recip).cast<int32_t>();
			// %2 to get the parent chunk with 2x2x2 leaf clusters
			baseClusterChunk = baseClusterChunk.unaryExpr([](int32_t val) { return val - val%2; });
			// convert back to real-world coordinates
			Eigen::Vector3f baseClusterPos = baseClusterChunk.cast<float>() * leafResolution;
			
			// DEBUG
			if (tid == 0) {
				// std::cout << clusterPos.x() << ' ' << clusterPos.y() << ' ' << clusterPos.z() << '\t';
				// std::cout << inputPos.x() << ' ' << inputPos.y() << ' ' << inputPos.z() << '\n';
				// std::cout << inputNorm.x() << ' ' << inputNorm.y() << ' ' << inputNorm.z() << '\n';
			}
			
			// // offsets of leaves within
			// std::array<float, 8> leaves;
			// auto pLeaf = leaves.begin();
			// for (auto xl = 0; xl < 2; xl++) {
			// 	for (auto yl = 0; yl < 2; yl++) {
			// 		for (auto zl = 0; zl < 2; zl++) {
			// 			// leaf position
			// 			Eigen::Vector3f leafOffset = Eigen::Vector3f(xl, yl, zl) * leafResolution;
			// 			Eigen::Vector3f pos = clusterPosReal + leafOffset;
			// 			*pLeaf = inputNorm.dot(pos - inputPos);		
			// 			// DEBUG
			// 			// float optimalLeaf = pos.norm() - 5.0f;
			// 			// *pLeaf = optimalLeaf;
			// 			// DEBUG END
			// 			pLeaf++;
			// 		}
			// 	}
			// }
			// // // compare to other existing leaves
			// MortonCode code(clusterPos * 2); // swallows a layer (reverted on reconstruction)
			// auto& cluster = octree.find(code.val);
			// if (cluster != 0) {
			// 	LeafCluster lc(leaves);
			// 	LeafCluster other(cluster);
			// 	lc.merge(other);
			// 	cluster = lc.cluster;
			// }
			// // simply insert
			// else {
			// 	LeafCluster lc(leaves);
			// 	cluster = lc.cluster;
			// }
			// return;

			// calculate signed distances for neighbours of current leaf cluster as well
			for (int32_t x = -1; x <= +1; x++) {
				for (int32_t y = -1; y <= +1; y++) {
					for (int32_t z = -1; z <= +1; z++) {
						// this is be a leaf cluster containing 8 individual leaves
						Eigen::Vector3i clusterChunk = baseClusterChunk + Eigen::Vector3i(x, y, z) * 2;
						Eigen::Vector3f clusterPos = baseClusterPos + Eigen::Vector3f(x, y, z) * 2 * leafResolution;
						// Eigen::Vector3f clusterOffset = clusterPos.cast<float>() * leafResolution;

						// offsets of leaves within
						std::array<float, 8> leaves;
						auto pLeaf = leaves.begin();
						for (auto xl = 0; xl < 2; xl++) {
							for (auto yl = 0; yl < 2; yl++) {
								for (auto zl = 0; zl < 2; zl++) {
									// leaf position
									Eigen::Vector3f leafOffset = Eigen::Vector3f(xl, yl, zl) * leafResolution;
									Eigen::Vector3f pos = clusterPos + leafOffset;
									*pLeaf++ = inputNorm.dot(pos - inputPos);
								}
							}
						}
						
						// compare to other existing leaves
						MortonCode code(clusterChunk);
						// since we do not explicitly encode leaf positions into morton code, 3 LSB wont be set
						code.val = code.val >> 3; // EXPERIMENTAL
						auto& cluster = octree.find(code.val);
						
						if (cluster != 0) {
							LeafCluster lc(leaves);
							LeafCluster other(cluster);
							lc.merge(other);
							cluster = lc.cluster;
						}
						// simply insert
						else {
							LeafCluster lc(leaves);
							cluster = lc.cluster;
						}
					}
				}
			}
		}
		auto get_trie(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) {
			auto beg = std::chrono::steady_clock::now();

			// round threads down to nearest power of two
			uint32_t lz = __builtin_clz(std::jthread::hardware_concurrency());
			size_t nThreads = 1 << (32 - lz - 1);
			// std::cout << "Using " << nThreads << " threads for trie ctor\n";
			std::vector<std::jthread> threads;
			threads.reserve(nThreads);
			std::vector<Octree> octrees(nThreads);

			// thread groups throughout the stages
			struct ThreadGroup {
				std::unique_ptr<std::atomic_uint8_t> nCompletedThreads;
				std::unique_ptr<std::atomic_bool> bPrepared;
				std::vector<Octree::Key> collisions;
				std::array<Octree*, 2> trees;
				uint32_t iLeadThread;
				uint32_t collisionDepth;
			};
			struct Stage {
				uint32_t groupSize; // threads per group
				std::vector<ThreadGroup> groups;
			};
			// calc number of total stages
			uint32_t nStages = std::sqrt(nThreads);
			std::vector<Stage> stages(nStages);
			// prepare thread groups for each stage
			for (uint32_t i = 0; i < nStages; i++) {
				auto& stage = stages[i];

				// figure out how many groups there are
				stage.groupSize = 2 << i;
				uint32_t nGroups = nThreads / stage.groupSize;

				// fill each group with info data
				stage.groups.resize(nGroups);
				for (uint32_t iGrp = 0; iGrp < nGroups; iGrp++) {
					auto& grp = stage.groups[iGrp];
					grp.iLeadThread = iGrp * stage.groupSize;
					grp.bPrepared = std::make_unique<std::atomic_bool>(false);
					grp.nCompletedThreads = std::make_unique<std::atomic_uint8_t>(0);
				}
			}
			
			// build smaller octrees on several threads
			size_t progress = 0;
			for (size_t id = 0; id < nThreads; id++) {
				size_t nElements = points.size() / nThreads;
				if (id == nThreads - 1) nElements = 0; // special value for final thread to read the rest
				// build one octree per thread
				threads.emplace_back([&points, &normals, &octrees, id, progress, nElements](){
					auto pCur = points.cbegin() + progress;
					auto pEnd = (nElements == 0) ? (points.cend()) : (pCur + nElements);
					auto pNorm = normals.cbegin() + progress;
					Octree& octree = octrees[id];
					// Octree::PathCache cache(octree);
					for (; pCur < pEnd; pCur++, pNorm++) {
						build_trie_whatnot(octree, *pCur, *pNorm, id);
					}
				});
				progress += nElements;
			}
			threads.clear(); // join
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "trie ctor");
			
			// merge the smaller octrees into one
			beg = std::chrono::steady_clock::now();
			for (size_t id = 0; id < nThreads; id++) {
				threads.emplace_back([&octrees, &stages, id]() {
					// combine octrees via multiple stages using thread groups
					for (uint32_t iStage = 0; iStage < stages.size(); iStage++) {
						auto& stage = stages[iStage];
						uint32_t iGrp = id / stage.groupSize;
						uint32_t iLocal = id % stage.groupSize; // block-local index
						auto& grp = stage.groups[iGrp];

						if (grp.iLeadThread == id) { // leader thread
							// wait for all dependent groups to finish
							if (iStage > 0) {
								auto& prevStage = stages[iStage - 1];
								uint32_t iPrevGrp = id / prevStage.groupSize;
								auto& grpA = prevStage.groups[iPrevGrp];
								auto& grpB = prevStage.groups[iPrevGrp + 1];
								uint8_t res = 0;
								// wait for group A to be done
								res = grpA.nCompletedThreads->load();
								while (res < prevStage.groupSize) {
									res = grpA.nCompletedThreads->load();
								}
								// wait for group B to be done
								res = grpB.nCompletedThreads->load();
								while (res < prevStage.groupSize) {
									res = grpB.nCompletedThreads->load();
								}
							}
							// set up pointers for group trees
							grp.trees[0] = &octrees[id];
							grp.trees[1] = &octrees[id + stage.groupSize/2];

							// in order to keep a balanced load, have N collisions per thread
							uint32_t nTargetCollisions = stage.groupSize;
							std::tie(grp.collisions, grp.collisionDepth) = grp.trees[0]->merge(*grp.trees[1], nTargetCollisions);

							// signal that this group thread is fully prepared
							grp.bPrepared->store(true);
							grp.bPrepared->notify_all();
						}
						grp.bPrepared->wait(false);
						// resolve assigned collisions
						uint32_t colIndex = iLocal;
						while (colIndex < grp.collisions.size()) {
							grp.trees[0]->merge(*grp.trees[1], grp.collisions[colIndex], grp.collisionDepth,
								[](Octree::Leaf& leaf_a, Octree::Leaf& leaf_b) {
								// merge leaves into a
								LeafCluster lc_a(leaf_a);
								LeafCluster lc_b(leaf_b);
								lc_a.merge(lc_b);
								leaf_a = lc_a.cluster;
							});
							colIndex += stage.groupSize;
						}
						// increment completion atomic
						(*grp.nCompletedThreads)++;
					}
				});
			}
			threads.clear(); // join

			end = std::chrono::steady_clock::now();
			dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "trie merg");
			return std::move(octrees[0]);
		}
		auto insert_octree2(Octree& octree) {
			auto beg = std::chrono::steady_clock::now();
			
			struct DagNode {
				DagNode() = delete;
				~DagNode() = delete;
				static DagNode* conv(uint32_t& addr) {
					return reinterpret_cast<DagNode*>(&addr);
				}
				// check header mask for a specific child
				bool contains_child(uint32_t iChild) const {
					uint32_t childBit = 1 << iChild;
					return header & iChild;
				}
				uint32_t get_child(uint32_t iChild) const {
					// popcount lookup table: https://stackoverflow.com/a/51388543
					constexpr uint8_t bitcount[] = {
						0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
						1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
						1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
						1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
						2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
						3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
						3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
						4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
					};
					// count how many children come before this one
					uint32_t childBit = 1 << iChild;
					uint32_t masked = header & (childBit - 1);
					uint32_t nChildren = bitcount[masked];
					// return actual index to child
					return children[nChildren + 1];
				}
				uint32_t header; // always valid
				std::array<uint32_t, 8> children; // sparse and compacted
			};
			
			// trackers that will be updated during traversal
			std::array<uint8_t, 63/3> path;
			std::array<const DagNode*, 		63/3> dagNodes;
			std::array<const Octree::Node*, 63/3> octNodes;
			std::array<std::array<uint32_t, 8>, 63/3> newNodes;
			// reset arrays
			path.fill(0);
			octNodes.fill(nullptr);
			for (auto& level: newNodes) level.fill(0);
			// set starting values
			int64_t depth = 0;
			octNodes[0] = octree.get_root();
			dagNodes[0] = DagNode::conv(nodeLevels[0].data[0]);
			
			while (depth >= 0) {
				auto iChild = path[depth]++;
				
				// todo
				if (iChild == 8) {
					depth--;
				}
				else if (depth < 63/3 - 1) {
					// check if the octree contains this child
					auto* octChild = octNodes[depth]->children[iChild];
					if (octChild == nullptr) continue;
					// check if the DAG tree contains this child
					auto* dagNode = dagNodes[depth];
					if (dagNode->contains_child(iChild)) {
						// walk further down as both octree and dag contain this child
						uint32_t dagChildAddr = dagNode->get_child(iChild);
						depth++;
						dagNodes[depth] = nodeLevels[depth].data[dagChildAddr];
						octNodes[depth] = octChild;
						depth++;
					}
					else {
						// node is missing from dag, add it and all its subsequent children
						// TODO
					}
					
					
				}
				// todo
				else {
					
				}
			}
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "dag  ctor");
		}
		// todo: make octree2 call a mini version of this, starting from nodes that didnt exist in DAG
		auto insert_octree(Octree& octree) {
			if (nodeLevels[0].data.size() > 1) {
				insert_octree2(octree);
				return;
			}
			auto beg = std::chrono::steady_clock::now();
			
			// trackers that will be updated during traversal
			std::array<uint8_t, 63/3> path;
			std::array<std::array<uint32_t, 8>, 63/3> dagNodes;
			std::array<const Octree::Node*, 63/3> octNodes;
			// reset arrays
			path.fill(0);
			octNodes.fill(nullptr);
			for (auto& level: dagNodes) level.fill(0);
			// set starting values
			int64_t depth = 0;
			octNodes[0] = octree.get_root();
			
			// iterate through octree and insert into hashDAG
			while (depth >= 0) {
				auto iChild = path[depth]++;
				
				// when all children were iterated
				if (iChild == 8) {
					// gather all children for this new node
					std::vector<uint32_t> children(1);
					for (auto i = 0; i < 8; i++) {
						if (dagNodes[depth][i] == 0) continue;
						children.push_back(dagNodes[depth][i]);
						children[0] |= 1 << i; // child mask
					}
					// add child count to mask
					children[0] |= (children.size() - 1) << 8;
					// reset node tracker for used-up nodes
					dagNodes[depth].fill(0);
					
					// check path in parent depth to know this node's child ID
					uint32_t indexInParent;
					if (depth == 0) indexInParent = 0;
					else indexInParent = path[depth - 1] - 1;
					
					// resize data if necessary and then copy over
					auto& level = nodeLevels[depth];
					level.data.resize(level.nOccupied + children.size());
					std::memcpy(
						level.data.data() + level.nOccupied,
						children.data(),
						children.size() * sizeof(uint32_t));
					// check if the same node existed previously
					uint32_t temporary = level.nOccupied;
					auto [pIndex, bNew] = level.hashSet.emplace(temporary);
					if (bNew) {
						level.nOccupied += children.size();
						uniques[depth]++;
						dagNodes[depth][indexInParent] = temporary;
					}
					else {
						dupes[depth]++;
						dagNodes[depth][indexInParent] = *pIndex;
					}
					
					depth--;
				}
				// node contains children
				else if (depth < 63/3 - 1) {
					// retrieve child node
					auto* pChild = octNodes[depth]->children[iChild];
					if (pChild == nullptr) continue;
					// walk deeper
					depth++;
					octNodes[depth] = pChild;
					path[depth] = 0;
				}
				// node contains leaves
				else {
					// go over every leaf(-cluster)
					for (auto i = 0; i < 8; i++) {
						LeafCluster lc(octNodes[depth]->leaves[i]);
						if (lc.cluster == LeafCluster::max) continue;
						if (lc.cluster == LeafCluster::min) continue;
						
						// check if this leaf cluster already exists
						auto temporaryIndex = leafLevel.data.size();
						auto [pIter, bNew] = leafLevel.hashMap.emplace(lc.cluster, temporaryIndex);
						if (bNew) {
							uniques[depth+1]++;
							// insert as new leaf cluster
							auto [part0, part1] = lc.get_parts();
							leafLevel.data.push_back(part0);
							leafLevel.data.push_back(part1);
							dagNodes[depth][iChild] = temporaryIndex;
						}
						else {
							dupes[depth+1]++;
							// simply update references to the existing cluster
							dagNodes[depth][iChild] = pIter->second;
						}
					}
					// update path tracker to signal that all "children" were iterated
					path[depth] = 8;
				}
				
			}
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "dag  ctor");
		}
		void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
			auto beg = std::chrono::steady_clock::now();
			
			// update bounding box
			for (auto cur = points.cbegin(); cur != points.cend(); cur++) {
				float x = cur->x();
				float y = cur->y();
				float z = cur->z();
				if (x < lowerLeft.x) lowerLeft.x = x;
				if (y < lowerLeft.y) lowerLeft.y = y;
				if (z < lowerLeft.z) lowerLeft.z = z;
				if (x > upperRight.x) upperRight.x = x;
				if (y > upperRight.y) upperRight.y = y;
				if (z > upperRight.z) upperRight.z = z;
			}
			Pose pose = { position, rotation };
			// TODO: transform points here, if not already transformed
			auto mortonCodes = calc_morton(points);
			sort_points(points, mortonCodes);
			auto normals = calc_normals(pose, mortonCodes);
			auto octree = get_trie(points, normals);
			insert_octree(octree);

			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "FULL time");
			if (true) {
				std::cout << std::setprecision(2);
				for (auto& pair: measurements) {
					std::cout << pair.second << " " << (double)pair.first << " ms\n";
				}
				std::cout << std::setprecision(6);
				std::cout << "\n";
			}
			measurements.clear();
		}
		void save_h5() {
			// HighFive::File file("test.h5", HighFive::File::Truncate);
			// for (uint32_t i = 0; i < dagLevels.size(); i++) {
			//     HighFive::DataSet dataset = file.createDataSet<uint32_t>(std::to_string(i), HighFive::DataSpace::From(dagLevels[i].data));
			//     dataset.write(dagLevels[i].data);
			// }

			// HighFive::File reader("test.h5", HighFive::File::ReadOnly);
			// for (uint32_t i = 0; i < dagLevels.size(); i++) {
			//     HighFive::DataSet dataset = file.getDataSet(std::to_string(i));
			//     std::vector<uint32_t> data;
			//     dataset.read(data);
			//     std::cout << data.size() << '\n';
			// }
			return;
			lvr2::BoundingBox<lvr2::BaseVector<float>> boundingBox(lowerLeft, upperRight);
			std::vector<std::vector<uint32_t>*> nodeLevelRef;
			for (auto& level: nodeLevels) nodeLevelRef.push_back(&level.data);
			nodeLevelRef.push_back(&leafLevel.data);
			
			///////////////////////// LVR2 ///////////////////////////
			typedef lvr2::BaseVector<float> VecT;
			
			// create hash grid from entire tree
			// generate mesh from hash grid
			lvr2::PMPMesh<VecT> mesh{};
			constexpr std::string_view decompositionType = "MC";
			if (decompositionType == "MC") {
				auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::FastBox<VecT>>>(boundingBox, nodeLevelRef, leafResolution);
				lvr2::FastReconstruction<VecT, lvr2::FastBox<VecT>> reconstruction(pGrid);
				reconstruction.getMesh(mesh);
			}
			else if (decompositionType == "PMC") {
				auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::BilinearFastBox<VecT>>>(boundingBox, nodeLevelRef, leafResolution);
				lvr2::FastReconstruction<VecT, lvr2::BilinearFastBox<VecT>> reconstruction(pGrid);
				reconstruction.getMesh(mesh);
			}
			// lvr2::reconstruct::Options options(0, "");
			// optimizeMesh(options, mesh);
			
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
		void print_stats() {
			size_t nUniques = 0;
			size_t nDupes = 0;
			size_t nBytes = 0;
			for (size_t i = 0; i < uniques.size(); i++) {
				std::cout << std::fixed;
				std::cout << "Level " << i << ": " << uniques[i] << " uniques, " << dupes[i] << " dupes, ";
				std::cout << static_cast<double>(uniques[i] * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB \n";
				nUniques += uniques[i];
				nDupes += dupes[i];
			}
			std::cout << static_cast<double>(nUniques * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB used in total\n";
		}

	private:
		static constexpr float min = -10000000;
		static constexpr float max = +10000000;
		
		lvr2::BaseVector<float> lowerLeft = {max, max, max};
		lvr2::BaseVector<float> upperRight = {min, min, min};
		std::array<uint32_t, 63/3+1> uniques = {};
		std::array<uint32_t, 63/3+1> dupes = {};

		// new //
		std::array<NodeLevel, 63/3> nodeLevels;
		LeafLevel leafLevel;
		struct Scan {
			Pose pose;
			uint32_t root;
			Eigen::Vector3f lowerLeft = {max, max, max};
			Eigen::Vector3f upperRight = {min, min, min};
		};
		std::vector<Scan> scans;
	};
}