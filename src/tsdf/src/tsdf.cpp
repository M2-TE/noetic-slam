#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("map callback");
}
void keyframes_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ROS_INFO("keyframes callback");
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("odom callback");
}

void path_callback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("path callback");
}

void pcl_deskewed_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("pcl deskewed callback");
}

void pcl_keyframe_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("pcl keyframe callback");
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("pose callback");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tsdf-backend");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("robot/dlio/odom_node/pose", 10, pose_callback);
    ros::spin();

    return 0;
}