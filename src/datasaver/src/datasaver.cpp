#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class LidarDataSaver {
public:
    LidarDataSaver(ros::NodeHandle nh) {
        sub_pcl = nh.subscribe("/os_cloud_node/points", 10, &LidarDataSaver::callbackPointCloud, this);
    }

    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        YAML::Node amplitudeNode, intensityNode;

        for (const auto& point : cloud.points) {
            YAML::Node pointNode;
            pointNode.push_back(point.x);
            pointNode.push_back(point.y);
            pointNode.push_back(point.z);
            amplitudeNode["points"].push_back(pointNode);
            YAML::Node intensityValue;
            intensityValue = point.intensity;
            intensityNode["intensities"].push_back(intensityValue);

        }

        saveYAML("amplitude.yaml", amplitudeNode);
        saveYAML("intensities.yaml", intensityNode);
        ROS_INFO("Saved amplitude and intensity data.");
    }

private:
    ros::Subscriber sub_pcl;

    void saveYAML(const std::string& filename, const YAML::Node& node) {
        std::ofstream outFile(filename);
        outFile << node;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_data_saver");
    ros::NodeHandle nh;
    LidarDataSaver saver(nh);
    ros::spin();
    return 0;
}
