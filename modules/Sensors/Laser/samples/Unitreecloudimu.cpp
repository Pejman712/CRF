/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <signal.h>
#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/UnitreeL1/UnitreeL1.hpp"  
#include "IMU/UnitreeL1IMU/UnitreeL1IMU.hpp"

namespace po = boost::program_options;

bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char** argv) {
    // --------------------------------------------------------------
    // Input management with Boost program_options
    // --------------------------------------------------------------
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("device", po::value<std::string>(), "Serial port name (e.g. /dev/ttyUSB0)")
        ("config", po::value<std::string>(), "UnitreeL1 configuration file");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        std::cout << "Error parsing command line arguments: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (!vm.count("device")) {
        std::cout << "Missing device name" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (!vm.count("config")) {
        std::cout << "Missing configuration file" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string device = vm["device"].as<std::string>();
    std::string configFile = vm["config"].as<std::string>();

    // Parse the configuration file
    std::ifstream config(configFile);
    if (!config.is_open()) {
        std::cout << "Unable to open configuration file: " << configFile << std::endl;
        return -1;
    }

    nlohmann::json laserJSON;
    config >> laserJSON;

    // Create UnitreeL1 laser object
    std::unique_ptr<crf::sensors::laser::UnitreeL1> laser {new crf::sensors::laser::UnitreeL1(
        crf::sensors::laser::LaserConnectionType::Serial, device, 2000000, laserJSON)};

    std::unique_ptr<crf::sensors::imu::UnitreeL1IMU> imu {new crf::sensors::imu::UnitreeL1IMU(
    crf::sensors::imu::LaserConnectionType::Serial, device, 2000000)};//, laserJSON)};

    // Set up signal handler for Ctrl+C
    signal(SIGINT, ctrlc);

    // Check connection to the Lidar
    
    // Initialize the Lidar
    if (!laser->initialize()) {
        std::cout << "Not possible to initialize the LiDAR" << std::endl;
        return -1;
    }
    if (!imu->initialize()) {
        std::cout << "Not possible to initialize the LiDAR" << std::endl;
        return -1;
    }
    std::cout << "Sensor initialized" << std::endl;

    // Output the frequency from the JSON configuration
    std::cout << "Frequency: " << laserJSON["Frequency"] << " Hz" << std::endl;

    // Create a buffer to store accumulated point clouds
    pcl::PointCloud<Point3D>::Ptr accumulatedCloud(new pcl::PointCloud<Point3D>);

    // Create a PCL viewer to visualize the point cloud
    pcl::visualization::CloudViewer viewer("Unitree Lidar Viewer");

    // Variables for time tracking and accumulation interval
    auto lastSaveTime = std::chrono::system_clock::now();
    const std::chrono::milliseconds accumulationInterval(10000); // 500 ms interval

    // Main loop to fetch point cloud data and accumulate
    while (!viewer.wasStopped()&& !ctrl_c_pressed) {
        auto start = std::chrono::system_clock::now();
        
        // Get the point cloud from the Lidar
        pcl::PointCloud<Point3D>::Ptr cloud = laser->getTargetPointCloud();
        
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - start;
        std::cout << "Acquisition time: " << elapsedSeconds.count() << " s" << std::endl;

        // Check if the cloud is empty
        if (cloud == nullptr || cloud->points.empty()) {
            std::cout << "Cloud Empty" << std::endl;
            break;
        }

        // Accumulate points from the current cloud into the accumulated cloud
        *accumulatedCloud += *cloud;

        // Check if 500 ms has passed to display the accumulated points
        auto currentTime = std::chrono::system_clock::now();
        if (currentTime - lastSaveTime >= accumulationInterval) {
            // Display the accumulated points
            pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            for (const auto& point : accumulatedCloud->points) {
                pcl::PointXYZ xyz_point;
                xyz_point.x = point.x;
                xyz_point.y = point.y;
                xyz_point.z = point.z;
                xyz_cloud->points.push_back(xyz_point);
            }

            // Show the accumulated cloud in the viewer
            viewer.showCloud(xyz_cloud);

            // Reset the lastSaveTime
            lastSaveTime = currentTime;
            accumulatedCloud->clear();
        }
        crf::sensors::imu::IMUSignals imuSignals = imu->crf::sensors::imu::UnitreeL1IMU::getSignal();
        // Print the IMU signals
        std::cout << "IMU Signals:" << std::endl;
        std::cout << "\tQuaternion: [" 
                << imuSignals.quaternion.value()[0] << ", " 
                << imuSignals.quaternion.value()[1] << ", " 
                << imuSignals.quaternion.value()[2] << ", " 
                << imuSignals.quaternion.value()[3] << "]" << std::endl;
        std::cout << "\tAngular Velocity: [" 
                << imuSignals.angularVelocity.value()[0] << ", " 
                << imuSignals.angularVelocity.value()[1] << ", " 
                << imuSignals.angularVelocity.value()[2] << "]" << std::endl;
        std::cout << "\tLinear Acceleration: [" 
                << imuSignals.linearAcceleration.value()[0] << ", " 
                << imuSignals.linearAcceleration.value()[1] << ", " 
                << imuSignals.linearAcceleration.value()[2] << "]" << std::endl;

        // Break the loop if Ctrl+C is pressed
        if (ctrl_c_pressed) {
            break;
        }
    }

    // Deinitialize the Lidar
    if (!laser->deinitialize()) {
        std::cout << "Not possible to deinitialize the LiDAR" << std::endl;
        return -1;
    }

    return 0;
}