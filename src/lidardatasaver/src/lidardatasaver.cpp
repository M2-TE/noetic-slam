#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <chrono>

namespace fs = std::filesystem;

class LidarDatasaver {
public:
    LidarDatasaver(ros::NodeHandle nh) {
        std::cout << "********************************************" << std::endl;
        std::cout << "Lidar datasaver node was started." << std::endl;
        std::cout << "********************************************" << std::endl;
        
        setLidarTargetDir();

        // Set first time stamp; new data should be saved every 0.5sek
        const auto currentTime = std::chrono::system_clock::now();
        nextTimeStamp = (std::round(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() * 2) / 2.0) * 10 + 5;

        sub_pcl = nh.subscribe("/ouster/points", 10, &LidarDatasaver::callbackPointCloud, this);
    }

    // Read point cloud data and save it to seperate files
    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Create new files every 0.5 seconds
        if (nextTimeStamp > std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() * 10) {
            return;
        }

        // Create new directory with meta.yaml
        nextDirName.str("");
        nextDirName << lidarTargetDir << nextTimeStamp << "/" ;
        fs::create_directories(nextDirName.str());
        createMetaYAML();

        nextTimeStamp += 5;

        // Get data of sensor
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Create .data file for intensities
        std::ofstream dataFileIntensities(nextDirName.str() + "intensities.data", std::ios::binary);
        for (const auto& point : cloud.points) {
            float intensities = point.intensity;
            dataFileIntensities.write(reinterpret_cast<const char*>(&intensities), sizeof(float));
        }
        dataFileIntensities.close();

        // Create yaml for intensities
        createYAML("intensities.yaml", "intensities", "channel", "float", "[59463419, 1]");
        ROS_INFO("Saved intensities data.");


        // TODO: weiter dateien erstellen -> welche Werte werden benötigt?
    }

private:
    ros::Subscriber sub_pcl;
    std::string lidarTargetDir;

    long long nextTimeStamp;
    std::ostringstream nextDirName;

    // create YAML file
    void createYAML(const std::string& filename, const std::string& name, const std::string& entity, const std::string& datatype, const std::string& shape) {
        YAML::Node yamlNode;
        yamlNode["entity"] = entity;
        yamlNode["data_type"] = datatype;
        yamlNode["type"] = "array";
        yamlNode["shape"] = YAML::Load(shape);
        yamlNode["name"] = name;

        std::ofstream outFile(nextDirName.str() + filename);
        if (outFile.is_open()) {
            outFile << yamlNode;
            outFile.close();
        } else {
            ROS_ERROR_STREAM("Failed to create " << filename);
        }
    }

    void createMetaYAML() {
        std::ofstream outFile(nextDirName.str() + "meta.yaml");
        if (outFile.is_open()) {
            outFile << "{fov: {phiIncrement: 0.02500000037252903, phiStart: 0, phiStop: 360, thetaIncrement: 0.02471923828125, thetaStart: 30, thetaStop: 130}, programName: 1200 kHz, shock_detected: false, shock_factor: 2.0385180015479287, start_time: -1, entity: sensor_data, type: scan, end_time: -1, num_points: 1665406919, pose_estimation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], transformation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}";
            outFile.close();
        } else {
            ROS_ERROR_STREAM("Failed to create meta.yaml");
        }
    }

    void setLidarTargetDir() {
        std::string baseTargetDir = fs::absolute(fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()).string() + "/sampledata/raw/";
    
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
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidardatasaver");
    ros::NodeHandle nh;
    LidarDatasaver saver(nh);
    ros::spin();
    return 0;
}
