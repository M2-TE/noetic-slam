#pragma once
#include <algorithm>
#include <atomic>
#include <execution>
#include <iomanip>
#include <iostream>
#include <limits>
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
#include <lvr2/reconstruction/HashGrid.hpp>
//
#include "dag_structs.hpp"
#include "lvr2/geometry/PMPMesh.hpp"
#include "lvr2/reconstruction/FastBox.hpp"
#include "lvr2/reconstruction/FastReconstruction.hpp"
#include "lvr2/algorithm/NormalAlgorithms.hpp"
#include "trie.hpp"


// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
static Eigen::Vector3f normal_from_neighbourhood(std::span<Eigen::Vector3f> points) {
	typedef double prec;
	// calculate centroid by through coefficient average
	Eigen::Vector3d centroid = { 0, 0, 0 };
	for (auto p = points.begin(); p != points.end(); p++) {
		centroid += p->cast<prec>();
	}
	centroid /= (prec)points.size();

	// covariance matrix excluding symmetries
	prec xx = 0.0;
	prec xy = 0.0;
	prec xz = 0.0;
	prec yy = 0.0;
	prec yz = 0.0;
	prec zz = 0.0;
	for (auto p = points.begin(); p != points.end(); p++) {
		auto r = p->cast<prec>() - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	xx /= (prec)points.size();
	xy /= (prec)points.size();
	xz /= (prec)points.size();
	yy /= (prec)points.size();
	yz /= (prec)points.size();
	zz /= (prec)points.size();

	// weighting linear regression based on square determinant
	Eigen::Vector3d weighted_dir = {};
	Eigen::Vector3d axis_dir = {};
	prec weight = 0.0;

	// determinant x
	prec det_x = yy*zz - yz*yz;
	axis_dir = {
		det_x,
		xz*yz - xy*zz,
		xy*yz - xz*yy
	};
	weight = det_x * det_x;
	if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
	weighted_dir += axis_dir * weight;

	// determinant y
	prec det_y = xx*zz - xz*xz;
	axis_dir = {
		xz*yz - xy*zz,
		det_y,
		xy*xz - yz*xx
	};
	weight = det_y * det_y;
	if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
	weighted_dir += axis_dir * weight;

	// determinant z
	prec det_z = xx*yy - xy*xy;
	axis_dir = {
		xy*yz - xz*yy,
		xy*xz - yz*xx,
		det_z
	};
	weight = det_z * det_z;
	if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
	weighted_dir += axis_dir * weight;

	// return normalized weighted direction as surface normal
	return weighted_dir.normalized().cast<float>();
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
		static auto calc_normals(Pose pose, std::vector<Eigen::Vector3f>& points, std::vector<std::pair<MortonCode, Eigen::Vector3f>>& mortonCodes) {
			auto beg = std::chrono::steady_clock::now();
			
			// sort points via morton code for next steps (plus better cache coherency)
			sort_points(points, mortonCodes);
			
			// construct neighbourhoods of points
			struct Neighbourhood {
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_beg;
				std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator it_end;
			};
			std::vector<Neighbourhood> neighbourhoods;
			neighbourhoods.emplace_back(mortonCodes.cbegin(), mortonCodes.cbegin());
			for(auto it_morton = mortonCodes.cbegin() + 1; it_morton != mortonCodes.cend(); it_morton++) {
				// get latest neighbourhood
				auto& neighbourhood = neighbourhoods.back();
				// check if current point is still within said neighbourhood
				MortonCode mc_point = std::get<0>(*it_morton);
				MortonCode mc_neigh = std::get<0>(*neighbourhood.it_beg);
				
				// the level up to which is checked to see if two morton codes belong to the same neighbourhood
				constexpr size_t neighLevel = 4;
				// mask to remove the LSB, where 3 bits are one level
				constexpr size_t mask = std::numeric_limits<size_t>::max() << neighLevel * 3;
				
				if ((mc_neigh.val & mask) == (mc_point.val & mask)) {
					// same neighbourhood // TRODO
				}
				else {
					// different neighbourhood // TODO
				}
				
				// TODO:
				// read point at neighbourhood beginning
				// define how many bits deep the morton code should be checked
				// each depth is 3 bits
			}
			std::cout << neighbourhoods.size() << '\n';
			// TODO: set it_end of final neighbourhood
			
			std::vector<Eigen::Vector3f> normals;
			normals.reserve(points.size());
			// TODO
			
			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "norm calc");
			return normals;
		}
		auto get_normals(Pose pose, std::vector<Eigen::Vector3f>& points) {
			auto beg = std::chrono::steady_clock::now();
			// points to be sorted via morton code
			std::vector<std::tuple<MortonCode, Eigen::Vector3f>> sorted;
			sorted.reserve(points.size());
			for (auto pCur = points.begin(); pCur != points.end(); pCur++) {
				// calculate leaf voxel position
				Eigen::Vector3f fPos = *pCur * (1.0 / leafResolution);
				Eigen::Vector3f safetyOffset = Eigen::Vector3f(leafResolution, leafResolution, leafResolution) / 2.0;
				Eigen::Vector3i vPos = (fPos + safetyOffset).cast<int32_t>();
				// assign to voxel chunk
				vPos /= (int32_t)dagSizes[idx::leaf - 4];
				// create 63-bit morton code from 3x21-bit fields
				sorted.emplace_back(vPos, *pCur);
			}
			// sort via morton codes
			// std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b){
			// __gnu_parallel::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b){
			std::sort(std::execution::par_unseq, sorted.begin(), sorted.end(), [](const auto& a, const auto& b){
				return std::get<0>(a) > std::get<0>(b);
			});

			// sort original points via sorted vector (redundant for now, helpful later on)
			auto pOut = points.begin();
			for (auto pCur = sorted.begin(); pCur != sorted.end(); pCur++, pOut++) {
				*pOut = std::get<1>(*pCur);
			}

			// create map lookup map for unique morton codes
			phmap::flat_hash_map<MortonCode, uint32_t> map;
			MortonCode last = std::get<0>(sorted.front());
			map.emplace(last, 0);
			uint32_t i = 0;
			for (auto pCur = sorted.begin(); pCur != sorted.end(); pCur++, i++) {
				if (std::get<0>(*pCur) == last) continue;
				last = std::get<0>(*pCur);
				map.emplace(last, i);
			}

			auto end = std::chrono::steady_clock::now();
			auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "norm sort");
			beg = std::chrono::steady_clock::now();

			// set up threads
			std::vector<std::jthread> threads;
			size_t nThreads = std::jthread::hardware_concurrency();
			threads.reserve(nThreads);
			// std::cout << "Using " << nThreads << " threads for normal calc\n";

			// estimate normals based on nearest morton code neighbours
			size_t progress = 0;
			std::vector<Eigen::Vector3f> normals(points.size());
			for (size_t i = 0; i < nThreads; i++) {
				size_t nElements = points.size() / nThreads;
				if (i == nThreads - 1) nElements = 0; // special value for final thread

				// launch thread
				threads.emplace_back([&sorted, &normals, &map, progress, nElements, pose](){
					auto pCur = sorted.cbegin() + progress;
					auto pEnd = (nElements == 0) ? (sorted.cend()) : (pCur + nElements);
					size_t normIndex = progress;
					for (; pCur != pEnd; pCur++) {
						Eigen::Vector3f point = std::get<1>(*pCur);
						std::vector<Eigen::Vector3f> neighbours;
						neighbours.push_back(point);

						// traverse morton code neighbours for nearest neighbour search
						auto [xC, yC, zC] = mortonnd::MortonNDBmi_3D_64::Decode(std::get<0>(*pCur).val);
						constexpr decltype(xC) off = 1; // offset (1 = 3x3x3 morton neighbourhood)
						for (auto x = xC - off; x <= xC + off; x++) {
							for (auto y = yC - off; y <= yC + off; y++) {
								for (auto z = zC - off; z <= zC + off; z++) {
									// generate morton code from new coordinates
									MortonCode mc = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);

									// look up index for the point corresponding to mc
									auto iter = map.find(mc);
									if (iter == map.end()) continue;
									uint32_t index = iter->second;
									auto pSorted = sorted.cbegin() + index;

									// iterate over all values for current key
									while (pSorted != sorted.cend() && std::get<0>(*pSorted) == mc) {
										neighbours.push_back(std::get<1>(*pSorted));
										pSorted++;
									}
								}
							}
						}
						// std::cout << neighbours.size() << std::endl;
						Eigen::Vector3f normal;
						if (neighbours.size() > 1) normal = normal_from_neighbourhood(neighbours);
						else normal = (pose.pos - point).normalized();
						normals[normIndex++] = normal;
					}
				});
				progress += nElements;
			}
			threads.clear();

			end = std::chrono::steady_clock::now();
			dur = std::chrono::duration<double, std::milli> (end - beg).count();
			measurements.emplace_back(dur, "norm calc");
			return normals;
		}
		static auto build_trie_whatnot(Octree& octree, Octree::PathCache& cache, const Eigen::Vector3f inputPos, const Eigen::Vector3f& inputNorm, uint32_t tid) {
			// voxels per unit at leaf level
			constexpr float nVoxelsPerUnit = 1.0 / leafResolution;
			// morton code will be generated using leaf cluster position, not the leaves themselves
			Eigen::Vector3f safetyOffset = Eigen::Vector3f(leafResolution, leafResolution, leafResolution) / 2.0;
			Eigen::Vector3i mainClusterPos = (nVoxelsPerUnit * inputPos + safetyOffset).cast<int32_t>();
			mainClusterPos = mainClusterPos.unaryExpr([](int32_t val) { return val - val%2; });

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
									*pLeaf = inputNorm.dot(inputPos - pos);
									pLeaf++;
								}
							}
						}
						
						// compare to other existing leaves
						MortonCode code(clusterPos * 2); // swallows a layer (reverted on reconstruction)
						auto& cluster = octree.find_cached(code.val, cache);
						
						if (cluster != 0) {
							LeafCluster lc(leaves);
							LeafCluster other((uint32_t)cluster);
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
					for (; pCur < pEnd; pCur++) {
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
			auto mortonCodes = calc_morton(points);
			auto normals2 = calc_normals(pose, points, mortonCodes);
			
			
			auto normals = get_normals(pose, points);
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
			return;
			lvr2::BoundingBox<lvr2::BaseVector<float>> boundingBox(lowerLeft, upperRight);
			std::vector<std::vector<uint32_t>*> nodeLevelRef;
			for (auto& level: nodeLevels) nodeLevelRef.push_back(&level.data);
			nodeLevelRef.push_back(&leafLevel.data);
			
			///////////////////////// LVR2 ///////////////////////////
			typedef lvr2::BaseVector<float> VecT;
			
			// create hash grid from entire tree
			auto pGrid = std::make_shared<lvr2::HashGrid<VecT, lvr2::FastBox<VecT>>>(boundingBox, nodeLevelRef, leafResolution);
			
			// generate mesh from hash grid
			lvr2::FastReconstruction<VecT, lvr2::FastBox<VecT>> reconstruction(pGrid);
			lvr2::PMPMesh<VecT> mesh{};
			reconstruction.getMesh(mesh);			
			// lvr2::reconstruct::Options options(0, "");
			// optimizeMesh(options, mesh);
			
			// generate mesh buffer from reconstructed mesh
			auto faceNormals = lvr2::calcFaceNormals(mesh);
    		auto clusterBiMap = lvr2::planarClusterGrowing(mesh, faceNormals, 0.85);
			lvr2::ClusterPainter painter(clusterBiMap);
    		lvr2::ColorGradient::GradientType t = lvr2::ColorGradient::gradientFromString("GREY");
			auto clusterColors = boost::optional<lvr2::DenseClusterMap<lvr2::RGB8Color>>(painter.colorize(mesh, t));
			auto vertexNormals = lvr2::calcVertexNormals(mesh, faceNormals);
			// auto matResult = lvr2::Materializer<VecT>(mesh, clusterBiMap, faceNormals, *surface).generateMaterials();

			// Calc normals for vertices
			// lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
			// finalizer.setNormalData(vertexNormals);
			// finalizer.setColorData(const VertexMap<RGB8Color> &colorData);
			
			lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer(clusterBiMap);
			finalizer.setClusterColors(*clusterColors);
			finalizer.setVertexNormals(vertexNormals);
			// finalizer.setMaterializerResult(matResult);
			
			// save to disk
			auto meshBuffer = finalizer.apply(mesh);
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

		// std::array<DAG::Level, nDagLevels> dagLevels;

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