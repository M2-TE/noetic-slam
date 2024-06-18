#pragma once
#include <algorithm>
#include <atomic>
#include <execution>
#include <iomanip>
#include <iostream>
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
		auto get_normals(Pose pose, std::vector<Eigen::Vector3f>& points) {
			auto beg = std::chrono::steady_clock::now();
			// points to be sorted via morton code
			std::vector<std::tuple<MortonCode, Eigen::Vector3f, uint32_t>> sorted;
			sorted.reserve(points.size());
			uint32_t i = 0;
			for (auto pCur = points.begin(); pCur != points.end(); pCur++) {
				// calculate leaf voxel position
				Eigen::Matrix<int32_t, 3, 1> vPos = (*pCur * (1.0 / leafResolution)).cast<int32_t>();
				// assign to voxel chunk
				vPos /= (int32_t)dagSizes[mortonVolume];
				// create 63-bit morton code from 3x21-bit fields
				sorted.emplace_back(vPos, *pCur, i++);
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
			i = 0;
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
						normals[std::get<2>(*pCur)] = normal;
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
		static auto build_trie_whatnot(Octree& octree, const Eigen::Vector3f inputPos, const Eigen::Vector3f& inputNorm) {
			// leaf cluster position that p belongs to
			Eigen::Vector3i voxelPos = (inputPos * (0.5f/leafResolution)).cast<int32_t>();
			Octree::PathCache cache(octree);

			// calculate signed distances for neighbours of current leaf cluster as well
			constexpr int32_t off = 1; // offset (1 = 3x3x3)
			for (int32_t x = -off; x <= +off; x++) {
				for (int32_t y = -off; y <= +off; y++) {
					for (int32_t z = -off; z <= +off; z++) {
						// this is be a leaf cluster containing 8 individual leaves
						Eigen::Vector3i cPos = voxelPos + Eigen::Vector3i(x, y, z);
						// actual floating position of cluster
						Eigen::Vector3f fPos = cPos.cast<float>() * 2.0f * leafResolution;

						MortonCode code(cPos);
						auto& cluster = octree.find_cached(code.val, cache);

						// offsets of leaves within
						std::array<float, 8> leaves;
						auto pLeaf = leaves.begin();
						for (auto xl = 0; xl < 2; xl++) {
							for (auto yl = 0; yl < 2; yl++) {
								for (auto zl = 0; zl < 2; zl++) {
									// leaf position
									Eigen::Vector3f offset = Eigen::Vector3f(xl, yl, zl) * leafResolution;
									Eigen::Vector3f lPos = fPos + offset;
									*pLeaf = inputNorm.dot(inputPos - lPos);
									pLeaf++;
								}
							}
						}

						// pack all leaves into 32/64 bits
						uint64_t packedLeaves = 0;
						for (uint64_t i = 0; i < 8; i++) {
							// todo: sd max should be turned into a parameter
							constexpr double sdMax = leafResolution * 2; // max range of 2 voxels
							float sdNormalized = leaves[i] * (1.0 / sdMax); // normalize signed distance (not clipped between -1 and 1)
							constexpr size_t nBits = 4; // 1b sign, 3b data
							constexpr float range = (float)(1 << (nBits-1));
							float sdScaled = sdNormalized * range;
							// cast to 8-bit integer and clamp between given range
							int8_t sdScaledInt = (int8_t)sdScaled;
							sdScaledInt = std::clamp<int8_t>(sdScaledInt, -range, range);
							// add offset such that values are represented linearly
							// 0 = -range, 8 = +range (both of these should be seen as "too far away" and discarded)
							uint8_t sdScaledUint = (uint8_t)(sdScaledInt + (int8_t)range);
							// pack the 4 bits of this value into the leaf cluster
							packedLeaves |= (uint64_t)sdScaledUint << i*4;
						}
						// when cluster already contains data, resolve collision
						if (cluster != 0) {
							for (uint64_t i = 0; i < 8; i++) {
								uint64_t baseMask = 0b1111;
								uint8_t a = (cluster      >> i*4) & baseMask;
								uint8_t b = (packedLeaves >> i*4) & baseMask;
								int8_t ai = (int8_t)a - 4;
								int8_t bi = (int8_t)b - 4;
								// ruleset (in order of priority):
								// 1. positive signed distance takes precedence over negative
								// 2. smaller value takes precedence over larger value
								bool bOverwrite = false;
								if (std::signbit(bi) < std::signbit(ai)) bOverwrite = true;
								else if (std::signbit(bi) == std::signbit(ai) && std::abs(bi) < std::abs(ai)) bOverwrite = true;
								if (bOverwrite) {
									// mask out the relevant bits
									uint64_t mask = baseMask << i*4;
									mask = ~mask;
									// overwrite cluster bits with new value
									cluster = (cluster & mask) | (b << i*4);
								}
							}
						}
						else cluster = packedLeaves;
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
				octrees.emplace_back(100'000); // hardcoded for now
				threads.emplace_back([&points, &normals, &octrees, id, progress, nElements](){
					auto pCur = points.cbegin() + progress;
					auto pEnd = (nElements == 0) ? (points.cend()) : (pCur + nElements);
					auto pNorm = normals.cbegin() + progress;
					// build one octree per thread
					for (; pCur < pEnd; pCur++) {
						build_trie_whatnot(octrees[id], *pCur, *pNorm);
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
									auto& leafA = pA->leaves[i];
									auto& leafB = pB->leaves[i];
									// iterate over leaf clusters
									for (uint64_t i = 0; i < 8; i++) {
										uint64_t baseMask = 0b1111;
										uint8_t a = (leafA >> i*4) & baseMask;
										uint8_t b = (leafB >> i*4) & baseMask;
										int8_t ai = (int8_t)a - 4;
										int8_t bi = (int8_t)b - 4;
										// ruleset (in order of priority):
										// 1. positive signed distance takes precedence over negative
										// 2. smaller value takes precedence over larger value
										bool bOverwrite = false;
										if (std::signbit(bi) < std::signbit(ai)) bOverwrite = true;
										else if (std::signbit(bi) == std::signbit(ai) && std::abs(bi) < std::abs(ai)) bOverwrite = true;
										if (bOverwrite) {
											// mask out the relevant bits
											uint64_t mask = baseMask << i*4;
											mask = ~mask;
											// overwrite cluster bits with new value
											leafA = (leafA & mask) | (b << i*4);
										}
									}
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
					if (depth < nDagLevels - 2) {
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
						uint32_t indexInParent = depth > 0 ? path[depth - 1] - 1 : 0;
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
					// go back up to parent
					if (depth == 0) break;
					depth--;
					continue;
				}

				// retrieve child node
				auto* pChild = octNodes[depth]->children[iChild];
				if (pChild == nullptr) continue;
				
				// child node contains 8 leaf clusters (1 cluster = 8 leaves; 64 leaves)
				if (depth == nDagLevels - 2) {
					// resize data if necessary and then copy over
					leafLevel.data.resize(leafLevel.nOccupied + 8);
					std::memcpy(
						leafLevel.data.data() + leafLevel.nOccupied,
						pChild->leaves.data(),
						8 * sizeof(uint32_t));
					// check if the same node existed previously
					uint32_t temporary = leafLevel.nOccupied;
					auto [pIndex, bNew] = leafLevel.hashSet.emplace(temporary);
					if (bNew) {
						leafLevel.nOccupied += 8;
						uniques[depth]++;
						nodes[depth][iChild] = temporary;
					}
					else {
						dupes[depth]++;
						nodes[depth][iChild] = *pIndex;
					}
				}
				// child node is a simple node
				else {
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
					// std::cout << pair.second << " " << pair.first << " ms\n";
					std::cout << pair.first << '\t';
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

			lvr2::BoundingBox<lvr2::BaseVector<float>> boundingBox(lowerLeft, upperRight);
			std::vector<std::vector<uint32_t>*> nodeLevelRef;
			for (auto& level: nodeLevels) nodeLevelRef.push_back(&level.data);
			nodeLevelRef.push_back(&leafLevel.data);
			
			// create hash grid from entire tree
			auto pGrid = std::make_shared<lvr2::HashGrid<lvr2::BaseVector<float>, lvr2::FastBox<lvr2::BaseVector<float>>>>(boundingBox, nodeLevelRef);
			
			// generate mesh from hash grid
			lvr2::FastReconstruction<lvr2::BaseVector<float>, lvr2::FastBox<lvr2::BaseVector<float>>> reconstruction(pGrid);
			lvr2::PMPMesh<lvr2::BaseVector<float>> mesh{};
			reconstruction.getMesh(mesh);
			
			// generate mesh buffer from reconstructed mesh
			// lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer;
			lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
			auto meshBuffer = finalizer.apply(mesh);
			
			// save to disk
			auto model = std::make_shared<lvr2::Model>(meshBuffer);
			lvr2::ModelFactory::saveModel(model, "yeehaw.obj");
		}

	private:
		lvr2::BaseVector<float> lowerLeft = {0, 0, 0};
		lvr2::BaseVector<float> upperRight = {0, 0, 0};
		std::array<uint32_t, nDagLevels> uniques = {};
		std::array<uint32_t, nDagLevels> dupes = {};

		// std::array<DAG::Level, nDagLevels> dagLevels;

		// new //
		std::array<NodeLevel, nDagLevels - 1> nodeLevels;
		LeafLevel leafLevel;
		struct Scan {
			Pose pose;
			uint32_t root;
			Eigen::Vector3f lowerLeft = {0,0,0};
			Eigen::Vector3f upperRight = {0,0,0};
		};
		std::vector<Scan> scans;
	};
}