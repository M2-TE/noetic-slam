#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

class LidarDatasaver {
public:
    LidarDatasaver(ros::NodeHandle nh) {
        std::cout << "********************************************" << std::endl;
        std::cout << "Lidar datasaver node was started." << std::endl;
        std::cout << "********************************************" << std::endl;
        
        setLidarTargetDir();

        sub_pcl = nh.subscribe("/ouster/points", 10, &LidarDatasaver::callbackPointCloud, this);
    }

    // Read point cloud data and save it to seperate files
    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        fs::create_directories(getNextDir());

        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);


        // std::ofstream dataFileAmplitude("amplitude.data", std::ios::binary);
        std::ofstream dataFileIntensities(getNextDir() + "intensities.data", std::ios::binary);
        
        for (const auto& point : cloud.points) {
            float intensities = point.intensity;
            dataFileIntensities.write(reinterpret_cast<const char*>(&intensities), sizeof(float));
        }

        dataFileIntensities.close();


        createYAML("intensities.yaml", "intensities", "channel", "float", "[59463419, 1]");
        ROS_INFO("Saved intensities data.");

        nextDirNr++;

        // TODO: weiter dateien erstellen -> welche Werte werden benötigt?
        // TODO: meta.yaml erstellen

        // TODO: Wie zusamenführen mit Kameradaten?
    }

private:
    ros::Subscriber sub_pcl;
    std::string lidarTargetDir;

    int nextDirNr = 0;

    // TODO: Ordner anpassen
    // create YAML file
    void createYAML(const std::string& filename, const std::string& name, const std::string& entity, const std::string& datatype, const std::string& shape) {
        YAML::Node yamlNode;
        yamlNode["entity"] = entity;
        yamlNode["data_type"] = datatype;
        yamlNode["type"] = "array";
        yamlNode["shape"] = YAML::Load(shape);
        yamlNode["name"] = name;

        std::ofstream outFile(getNextDir() + filename);
        if (outFile.is_open()) {
            outFile << yamlNode;
            outFile.close();
        } else {
            ROS_ERROR_STREAM("Failed to create " << filename);
        }
    }

    void setLidarTargetDir() {
        std::string baseTargetDir = fs::absolute(fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()).string() + "/sampledata/raw/";
        std::cout << "*******Base dir: " << baseTargetDir << "*********" << std::endl;
       
        int maxDir = 0;
        for (const auto& entry : fs::directory_iterator(baseTargetDir)) {
            if (entry.is_directory() && entry.path().filename() != "meta.yaml") {
                int dirNumber = std::stoi(entry.path().filename().string());
                maxDir = std::max(maxDir, dirNumber);
            }
        }


        std::ostringstream lidarTargetDirName;
        lidarTargetDirName << baseTargetDir << std::setw(8) << std::setfill('0') << maxDir << "/lidar_00000000/" ;
        lidarTargetDir = lidarTargetDirName.str();
        std::cout << "*******Lidar dir: " << lidarTargetDir << "*********" << std::endl;
       
    }

    std::string getNextDir() {
        std::ostringstream nextDirName;
        nextDirName << lidarTargetDir << std::setw(8) << std::setfill('0') << nextDirNr << "/" ;
        return nextDirName.str();
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidardatasaver");
    ros::NodeHandle nh;
    LidarDatasaver saver(nh);
    ros::spin();
    return 0;
}
