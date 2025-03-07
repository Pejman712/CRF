/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <signal.h>
#include <fstream>

#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/program_options.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Laser/RPLiDAR/RPLiDAR.hpp"

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
        ("config", po::value<std::string>(), "RPLiDAR configuration file");

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

    // Create RPlidar laser object
    std::unique_ptr<crf::sensors::laser::RPLiDAR> laser {new crf::sensors::laser::RPLiDAR(
        crf::sensors::laser::LaserConnectionType::Serial, device, 256000, laserJSON)};

    // Set up signal handler for Ctrl+C
    signal(SIGINT, ctrlc);

    // Check connection to the Lidar
    if (!laser->checkConnection()) {
        std::cout << "Not possible to connect to the LiDAR" << std::endl;
        return -1;
    }
    std::cout << "LiDAR connected on device " << device << std::endl;

    // Initialize the Lidar
    if (!laser->initialize()) {
        std::cout << "Not possible to initialize the LiDAR" << std::endl;
        return -1;
    }
    std::cout << "Sensor initialized" << std::endl;

    // Output the frequency from the JSON configuration
    std::cout << "Frequency: " << laserJSON["Frequency"] << " Hz" << std::endl;
    pcl::visualization::CloudViewer viewer("Viewer");

    // Main loop to fetch point cloud data and visualize
    while (!viewer.wasStopped()) {
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

        // Display the cloud points in the terminal
        for (const auto& point : cloud->points) {
            std::cout << "Point: x=" << point.x << ", y=" << point.y << ", z=" << point.z << std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
        for (const auto& point : cloud->points) {
            pcl::PointXYZ xyz_point;
            xyz_point.x = point.x;
            xyz_point.y = point.y;
            xyz_point.z = point.z;
            xyz_cloud->points.push_back(xyz_point);
        }

        // Show the accumulated cloud in the viewer
        viewer.showCloud(xyz_cloud);

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
