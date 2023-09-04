// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("map callback");
}
void keyframes_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ROS_INFO("keyframes callback");
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("odom callback");
}
void pcl_keyframe_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("pcl keyframe callback");
}
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("pose callback");
}

// buffer with timestamp
std::pair<nav_msgs::PathConstPtr, double> bufferedPath;
std::pair<sensor_msgs::PointCloud2ConstPtr, double> bufferedPcl;
// configure this
double syncThreshhold = 0.01;

// temporary solution
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
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (double, timestamp, timestamp))

void combined_callback() {
    ROS_INFO("combined callback");

    // extract pose
    auto pose = bufferedPath.first->poses.back().pose;
    // extract pcl
    pcl::PointCloud<Point> pcl = {};
    pcl::fromROSMsg(*bufferedPcl.first, pcl);

    for (int i = 0; i < 100; i++) {
        ROS_INFO("Point: %f, %f, %f, %f", pcl[i].data[0], pcl[i].data[1], pcl[i].data[2], pcl[i].data[3]);
    }
}
void path_callback(const nav_msgs::PathConstPtr& msg) {
    double now = ros::Time::now().toSec();
    bufferedPath = { msg, now };

    if (now - bufferedPcl.second < syncThreshhold) {
        combined_callback();
    }
}
void pcl_deskewed_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    double now = ros::Time::now().toSec();
    bufferedPcl = { msg, now };

    if (now - bufferedPath.second < syncThreshhold) {
        combined_callback();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf");
    ros::NodeHandle nh;
    // ros::Subscriber sub0 = n.subscribe("robot/dlio/map_node/map", 10, map_callback);
    // ros::Subscriber sub1 = n.subscribe("robot/dlio/odom_node/keyframes", 10, keyframes_callback);
    // ros::Subscriber sub2 = n.subscribe("robot/dlio/odom_node/odom", 10, odom_callback);
    // ros::Subscriber sub5 = n.subscribe("robot/dlio/odom_node/pointcloud/keyframe", 10, pcl_keyframe_callback);
    // ros::Subscriber sub6 = n.subscribe("robot/dlio/odom_node/pose", 10, pose_callback);
    ros::Subscriber subPath = nh.subscribe("robot/dlio/odom_node/path", 10, path_callback);
    ros::Subscriber subPcl = nh.subscribe("robot/dlio/odom_node/pointcloud/deskewed", 10, pcl_deskewed_callback);
    ros::spin();
    return 0;
}