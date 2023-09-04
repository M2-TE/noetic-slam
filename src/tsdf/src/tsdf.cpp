#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

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
    ros::init(argc, argv, "tsdf");
    ros::NodeHandle n;
    // ros::Subscriber sub0 = n.subscribe("robot/dlio/map_node/map", 10, map_callback);
    // ros::Subscriber sub1 = n.subscribe("robot/dlio/odom_node/keyframes", 10, keyframes_callback);
    // ros::Subscriber sub2 = n.subscribe("robot/dlio/odom_node/odom", 10, odom_callback);
    // ros::Subscriber sub5 = n.subscribe("robot/dlio/odom_node/pointcloud/keyframe", 10, pcl_keyframe_callback);
    // ros::Subscriber sub6 = n.subscribe("robot/dlio/odom_node/pose", 10, pose_callback);
    ros::Subscriber sub3 = n.subscribe("robot/dlio/odom_node/path", 10, path_callback);
    ros::Subscriber sub4 = n.subscribe("robot/dlio/odom_node/pointcloud/deskewed", 10, pcl_deskewed_callback);
    ros::spin();

    return 0;
}