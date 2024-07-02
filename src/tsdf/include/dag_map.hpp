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
#include "dag_structs.hpp"
#include "leaf_cluster.hpp"
#include "lvr2/geometry/PMPMesh.hpp"
#include "lvr2/reconstruction/DMCReconstruction.hpp"
#include "lvr2/reconstruction/FastBox.hpp"
#include "lvr2/reconstruction/FastReconstruction.hpp"
#include "lvr2/algorithm/NormalAlgorithms.hpp"
#include "trie.hpp"


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
			
			
			// the level up to which is checked to see if two morton codes belong to the same neighbourhood
			constexpr size_t neighLevel = 2;
			constexpr size_t mask = std::numeric_limits<size_t>::max() << neighLevel * 3;
			
			// construct neighbourhoods of points
			struct Neighbourhood {
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_beg;
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_end;
			};
			// todo: check performance of real hash vs sorted red-black
			// phmap::flat_hash_map<typeof(MortonCode::val), Neighbourhood> neighMap;
			std::map<typeof(MortonCode::val), Neighbourhood> neighMap;
			{
				// insert the first neighbourhood
				MortonCode mc_neigh = std::get<0>(mortonCodes.front());
				mc_neigh.val &= mask;
				auto it_neigh = neighMap.emplace(mc_neigh.val, Neighbourhood(mortonCodes.cbegin(), mortonCodes.cbegin())).first;
				
				// iterate through all points to build neighbourhoods
				for(auto it_morton = mortonCodes.cbegin() + 1; it_morton != mortonCodes.cend(); it_morton++) {
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
			}
			
			// build normals using local neighbourhoods
			std::vector<Eigen::Vector3f> normals(mortonCodes.size());
			for (auto it_neigh = neighMap.cbegin(); it_neigh != neighMap.cend(); it_neigh++) {
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
				// std::cout << adjNeighs.size() << '\n';
				
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
				// std::cout << n_points<< '\n';
				
				// reserve some generous space for nearest points
				std::vector<Eigen::Vector4d> nearestPoints;
				nearestPoints.reserve(n_points);
				// for every point within the current neighbourhood it_neigh, find its nearest neighbours for normal calc
				constexpr float maxDist = leafResolution * (1 << neighLevel);
				for(auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
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
					// std::cout << normal.dot(posToPoint) << '\t';
					// std::cout << "pos: " << point.x() << ' ' << point.y() << ' ' << point.z() << '\t';
					// std::cout << "norm: " << normal.x() << ' ' << normal.y() << ' ' << normal.z() << '\n';
					
					
					// figure out index of point
					size_t index = it_point - mortonCodes.cbegin();
					normals[index] = normal;
				}
			}
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "norm calc");
			return normals;
		}
		static auto build_trie_whatnot(Octree& octree, Octree::PathCache& cache, const Eigen::Vector3f inputPos, const Eigen::Vector3f& inputNorm, uint32_t tid) {
			// voxels per unit at leaf level
			constexpr float recip = 1.0 / leafResolution;
			// morton code will be generated using leaf cluster position, not the leaves themselves
			Eigen::Vector3i mainClusterPos = (inputPos * recip).cast<int32_t>();
			mainClusterPos = mainClusterPos.unaryExpr([](int32_t val) { return val - val%2; });
			
			// DEBUG
			// if (tid == 0) {
				// std::cout << inputPos.x() << ' ' << inputPos.y() << ' ' << inputPos.z() << '\t';
				// std::cout << inputNorm.x() << ' ' << inputNorm.y() << ' ' << inputNorm.z() << '\n';
			// }

			// calculate signed distances for neighbours of current leaf cluster as well
			for (int32_t x = -1; x <= +1; x++) {
				for (int32_t y = -1; y <= +1; y++) {
					for (int32_t z = -1; z <= +1; z++) {
						// this is be a leaf cluster containing 8 individual leaves
						Eigen::Vector3i clusterPos = mainClusterPos + Eigen::Vector3i(x, y, z) * 2;
						Eigen::Vector3f clusterOffset = clusterPos.cast<float>() * leafResolution;

						// offsets of leaves within
						std::array<float, 8> leaves;
						auto pLeaf = leaves.begin();
						for (auto xl = 0; xl < 2; xl++) {
							for (auto yl = 0; yl < 2; yl++) {
								for (auto zl = 0; zl < 2; zl++) {
									// leaf position
									Eigen::Vector3f leafOffset = Eigen::Vector3f(xl, yl, zl) * leafResolution;
									Eigen::Vector3f pos = clusterOffset + leafOffset;
									*pLeaf = inputNorm.dot(pos - inputPos);
									
									// DEBUG
									float optimalLeaf = pos.norm() - 5.0f;
									// float diff = std::abs(*pLeaf - optimalLeaf);
									// if (diff > 0.1) {
									// 	std::cout << std::setprecision(4);
									// 	if (true) {
									// 		std::cout << diff << '\t';
									// 		std::cout << *pLeaf << '\t' << optimalLeaf << '\n';
									// 		std::cout << "pos: " << inputPos.x() << ' ' << inputPos.y() << ' ' << inputPos.z() << '\t';
									// 		std::cout << "norm: " << inputNorm.x() << ' ' << inputNorm.y() << ' ' << inputNorm.z() << '\n';
									// 	}
									// }
									// std::ostringstream oss;
									// oss << *pLeaf << '\t' << optimalLeaf << '\t' << diff << '\n';
									// std::cout << oss.str();
									// *pLeaf = optimalLeaf;
									// DEBUG END
									pLeaf++;
								}
							}
						}
						
						// compare to other existing leaves
						MortonCode code(clusterPos * 2); // swallows a layer (reverted on reconstruction)
						auto& cluster = octree.find_cached(code.val, cache);
						
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
			std::vector<Octree> octrees;
			octrees.reserve(nThreads);

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
			size_t progress = 0;
			for (size_t id = 0; id < nThreads; id++) {
				size_t nElements = points.size() / nThreads;
				if (id == nThreads - 1) nElements = 0; // special value for final thread to read the rest
				octrees.emplace_back(1'000'000); // hardcoded for now
				// build one octree per thread
				threads.emplace_back([&points, &normals, &octrees, id, progress, nElements](){
					auto pCur = points.cbegin() + progress;
					auto pEnd = (nElements == 0) ? (points.cend()) : (pCur + nElements);
					auto pNorm = normals.cbegin() + progress;
					Octree& octree = octrees[id];
					Octree::PathCache cache(octree);
					for (; pCur < pEnd; pCur++, pNorm++) {
						build_trie_whatnot(octree, cache, *pCur, *pNorm, id);
					}
				});
				progress += nElements;
			}

			// join all threads
			threads.clear();
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "trie ctor");

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
							std::tie(grp.collisions, grp.collisionDepth) = grp.trees[0]->find_collisions_and_merge(*grp.trees[1], nTargetCollisions);

							// signal that this group thread is fully prepared
							grp.bPrepared->store(true);
							grp.bPrepared->notify_all();
						}
						grp.bPrepared->wait(false);
						// resolve assigned collisions
						uint32_t colIndex = iLocal;
						while (colIndex < grp.collisions.size()) {
							grp.trees[0]->resolve_collisions(*grp.trees[1], grp.collisions[colIndex], grp.collisionDepth,
								[](Octree::Node* pA, Octree::Node* pB) {
								// compare two leaf clusters a and b during collision
								// merge results into node a

								// iterate over leaf cluster parents
								for (uint32_t i = 0; i < 8; i++) {
									LeafCluster lcA(pA->leaves[i]);
									LeafCluster lcB(pB->leaves[i]);
									lcA.merge(lcB);
									pA->leaves[i] = lcA.cluster;
								}
							});
							colIndex += stage.groupSize;
						}
						// increment completion atomic
						(*grp.nCompletedThreads)++;
					}
				});
			}
			// join threads
			threads.clear();

			end = std::chrono::steady_clock::now();
			dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "trie mrge");
			return octrees[0];
		}
		void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
			auto full_beg = std::chrono::steady_clock::now();
			
			// std::random_device rd;
			// std::mt19937 gen(0);
			// std::uniform_real_distribution<float> dis(-.06f, .06f);
			// std::cout << std::setprecision(4);
			// for (auto i = 0; i < 1; i++) {
			// 	// gen
			// 	std::array<float, 8> leaves;
			// 	for (auto iLeaf = 0; iLeaf < 8; iLeaf++) leaves[iLeaf] = dis(gen);
			// 	LeafCluster lc(leaves);
				
			// 	// merge
			// 	std::array<float, 8> otherLeaves;
			// 	for (auto iLeaf = 0; iLeaf < 8; iLeaf++) otherLeaves[iLeaf] = dis(gen);
			// 	LeafCluster lcmerge(otherLeaves);
			// 	lc.merge(lcmerge);
				
			// 	// conversion
			// 	auto [part0, part1] = lc.get_parts();
			// 	LeafCluster lc2(part0, part1);
				
			// 	// output
			// 	for (auto iLeaf = 0; iLeaf < 8; iLeaf++) {
			// 		std::cout << leaves[iLeaf] << "  \tmerged with " << otherLeaves[iLeaf] << "  \toutput: " << lc2.get_sd(iLeaf) << '\n';
			// 	}
			// 	std::cout << '\n';
			// }
			// exit(0);
			
			
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
			auto trie = get_trie(points, normals);
			auto beg = std::chrono::steady_clock::now();

			// path leading to current node
			std::array<uint8_t, nDagLevels> path;
			// collection of nodes that exist along the path
			std::array<std::array<uint32_t, 8>, nDagLevels> nodes;
			// nodes of the temporary octree
			std::array<Octree::Node*, nDagLevels> octNodes;
			// keep track of tree depth
			uint_fast32_t depth;
			
			// initialize
			for (auto& level: nodes) level.fill(0); // reset nodes tracker
			path.fill(0); // reset path
			octNodes[0] = trie.pNodes; // begin at octree root
			depth = 0; // depth 0 being the root
			
			// begin insertion into hashDAG (bottom-up)
			while (true) {
				auto iChild = path[depth]++;
				if (iChild >= 8) {
					// insert normal/root node
					if (depth < nDagLevels - 1) {
						// gather all children for this new node
						std::vector<uint32_t> children(1);
						for (auto i = 0; i < 8; i++) {
							if (nodes[depth+1][i] == 0) continue;
							children.push_back(nodes[depth+1][i]);
							children[0] |= 1 << i; // child mask
						}
						// add child count to mask
						children[0] |= (children.size() - 1) << 8;
						// reset node tracker for used-up nodes
						nodes[depth+1].fill(0);
						
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
						uint32_t indexInParent = path[depth - 1] - 1;
						if (depth == 0) indexInParent = 0;
						if (bNew) {
							level.nOccupied += children.size();
							uniques[depth]++;
							nodes[depth][indexInParent] = temporary;
						}
						else {
							dupes[depth]++;
							nodes[depth][indexInParent] = *pIndex;
						}
					}
					if (depth == 0) break;
					depth--;
					continue;
				}

				// child node is a leaf cluster of 8 leaves
				if (depth == nDagLevels - 1) {
					// retrieve leaf cluster
					LeafCluster lc { octNodes[depth]->leaves[iChild] };
					if (lc.cluster == 0) continue;
					// check if this leaf cluster already exists
					auto temporaryIndex = leafLevel.data.size();
					auto [pIter, bNew] = leafLevel.hashMap.emplace(lc.cluster, temporaryIndex);
					if (bNew) {
						uniques[depth]++;
						// update reference to leaf cluster
						nodes[depth][iChild] = temporaryIndex;
						// insert into (uint32) data array
						auto [part0, part1] = lc.get_parts();
						// std::cout << std::bitset<64>(lc.cluster) << '\n';
						leafLevel.data.push_back(part0);
						leafLevel.data.push_back(part1);
					}
					else {
						dupes[depth]++;
						// simply update references to the existing cluster
						nodes[depth][iChild] = pIter->second;
					}
				}
				// child node is a simple node
				else {
					// retrieve child node
					auto* pChild = octNodes[depth]->children[iChild];
					if (pChild == nullptr) continue;
					// walk deeper
					depth++;
					octNodes[depth] = pChild;
					// reset child tracker
					path[depth] = 0;
				}
			}

			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "trie iter");
			dur = std::chrono::duration<double, std::milli> (end - full_beg).count();
			measurements.emplace_back(dur, "FULL");
			if (true) {
				std::vector<double> times;
				std::vector<std::string> labels;
				std::cout << std::setprecision(2);
				for (auto& pair: measurements) {
					std::cout << pair.second << " " << (uint32_t)pair.first << " ms\n";
					// std::cout << (uint32_t)pair.first << '\t';
					if (pair.second == "FULL") continue;
					times.push_back(pair.first);
					labels.push_back(pair.second);
				}
				std::cout << std::setprecision(6);
				std::cout << "\n";
			}
			measurements.clear();

			size_t nUniques = 0;
			size_t nDupes = 0;
			size_t nBytes = 0;
			for (size_t i = 0; i < uniques.size(); i++) {
				std::cout << "Level " << i << ": " << uniques[i] << " uniques, " << dupes[i] << " dupes, ";
				std::cout << static_cast<double>(uniques[i] * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB \n";
				nUniques += uniques[i];
				nDupes += dupes[i];
			}
			std::cout << static_cast<double>(nUniques * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB used in total\n";
			// // std::cout << "Memory footprint: " << nBytes << " bytes (" << (double)nBytes / 1'000'000 << " MB)\n";
			// std::cout << "Total: " << nUniques << " uniques, " << nDupes << " dupes\n";
			// size_t pointsBytes = points.size() * sizeof(Eigen::Vector3f);
			// std::cout << "Pointcloud footprint: " << pointsBytes << " bytes (" << (double)pointsBytes / 1'000'000 << "MB)\n";
			//// find the average of all points
			// Eigen::Vector3d acc = {0, 0, 0};
			// for (auto& point: points) {
			//     acc += point.cast<double>();
			// }
			// acc /= (double)points.size();
			// std::cout << acc << std::endl;
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
			// return;
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
			else if (decompositionType == "DMC") {
				// auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::BilinearFastBox<VecT>>>(boundingBox, nodeLevelRef, leafResolution);
				// lvr2::DMCReconstruction<VecT, lvr2::FastBox<VecT>> reconstruction(pGrid);
				// reconstruction.getMesh(mesh);
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

	private:
		static constexpr float min = -10000000;
		static constexpr float max = +10000000;
		
		lvr2::BaseVector<float> lowerLeft = {max, max, max};
		lvr2::BaseVector<float> upperRight = {min, min, min};
		std::array<uint32_t, nDagLevels> uniques = {};
		std::array<uint32_t, nDagLevels> dupes = {};

		// new //
		std::array<NodeLevel, nDagLevels - 1> nodeLevels;
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