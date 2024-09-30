# Noetic Slam

TSDF Slam using CHAD TSDF as its mapping backend

## Description

Noetic Slam was built to provide a testbed for comparison between CHAD TSDF, Voxblox, VDBFusion and Octomap.
It runs within a docker container and therefore requires no dependencies, apart from docker itself.
Scripts are provided to ease usage of pointcloud recording and playback, as well as registration and map writes.

## Getting Started

### Acquiring pointclouds
The scripts are mainly tailored towards Ouster scanners, but should work with other setups that provide pointcloud and IMU topics.

### 0. Docker container launch

Build and launch the docker container. The parameter allows passing through a GPU for rendering.

Pick one:
```
bash scripts/docker-run.sh none
bash scripts/docker-run.sh integrated
bash scripts/docker-run.sh nvidia
```

Once the container is up and running, three separate consoles are needed.
They are not required to be run within a console attached to the docker container.


### 1. DLIO launch

[DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git) (Direct LiDAR-Inertial Odometry) is used for registration. Topics can be adjusted either in the dockerfile within the project root or in the DLIO launch script itself.
```
bash scripts/dlio-launch.sh
```

### 2. Mapping node launch

The mapping node will use one of the mapping backends of choice using registered points from DLIO. Adjust MAP_BACKEND_IDX in [tsdf_map_node.cpp](src/tsdf_map/src/tsdf_map_node.cpp) to choose a specific backend:
* 0: CHAD TSDF
* 1: Octomap
* 2: Voxblox
* 3: VDBFusion
```
rosrun tsdf_map tsdf_map_node
```
### 3: Pointcloud and IMU playback

Once the DLIO and mapping nodes are running, replay or stream a pointcloud and IMU topic; this example shows three ways using either an Ouster or generic bagfile.
```
bash scripts/ouster-replay.sh bags/<bagfile>
bash scripts/ouster-stream.sh bags/<bagfile>
rosbag replay bags/<bagfile>
```

## Authors

Jan Kuhlmann

## Version History

* 0.1
    * Initial Release

## Acknowledgments

* [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)
* [Octomap](https://github.com/OctoMap/octomap)
* [Voxblox](https://github.com/ethz-asl/voxblox)
* [VDBFusion](https://github.com/PRBonn/vdbfusion)
