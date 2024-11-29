#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class LidarDatasaver {
public:
    LidarDatasaver(ros::NodeHandle nh) {
        std::cout << "********************************************" << std::endl;
        std::cout << "Lidar datasaver node was started." << std::endl;
        std::cout << "********************************************" << std::endl;
        
        sub_pcl = nh.subscribe("/ouster/points", 10, &LidarDatasaver::callbackPointCloud, this);
    }

    // Read point cloud data and save it to seperate files
    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // std::ofstream dataFileAmplitude("amplitude.data", std::ios::binary);
        std::ofstream dataFileIntensities("intensities.data", std::ios::binary);
        
        for (const auto& point : cloud.points) {
            float intensities = point.intensity;
            dataFileIntensities.write(reinterpret_cast<const char*>(&intensities), sizeof(float));
        }

        dataFileIntensities.close();


        createYAML("intensities.yaml", "intensities", "channel", "float", "[59463419, 1]");
        ROS_INFO("Saved intensities data.");

        // TODO: weiter dateien erstellen -> welche Werte werden benötigt?
        // TODO: meta.yaml erstellen

        // TODO: Wie zusamenführen mit Kameradaten?
    }

private:
    ros::Subscriber sub_pcl;

    // TODO: Ordner anpassen
    // create YAML file
    void createYAML(const std::string& filename, const std::string& name, const std::string& entity, const std::string& datatype, const std::string& shape) {
        YAML::Node yamlNode;
        yamlNode["entity"] = entity;
        yamlNode["data_type"] = datatype;
        yamlNode["type"] = "array";
        yamlNode["shape"] = YAML::Load(shape);
        yamlNode["name"] = name;

        std::ofstream outFile(filename);
        if (outFile.is_open()) {
            outFile << yamlNode;
            outFile.close();
        } else {
            ROS_ERROR_STREAM("Failed to create " << filename);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidardatasaver");
    ros::NodeHandle nh;
    LidarDatasaver saver(nh);
    ros::spin();
    return 0;
}
