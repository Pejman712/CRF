/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>  // Include for quaternion operations
#include <cmath>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include "VisionUtility/PointCloud/Gicp.hpp"  // Ensure this points to your corrected gicp.hpp
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "EventLogger/EventLogger.hpp"
#include "Sensors/Laser/UnitreeL1/UnitreeL1.hpp"  




int main(int argc, char** argv) {

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

    // Set up signal handler for Ctrl+C
    signal(SIGINT, ctrlc);

    // Check connection to the Lidar
    
    // Initialize the Lidar
    if (!laser->initialize()) {
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
    const std::chrono::milliseconds accumulationInterval(1000); // 500 ms interval


    double totalProcessingTime = 0.0;
    double totalXTranslation = 0.0;
    double totalYTranslation = 0.0;
    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    std::vector<std::pair<double, double>> translationPath;
    translationPath.emplace_back(0.0, 0.0);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

// Create a pointer to store the previous scan
pcl::PointCloud<pcl::PointXYZ>::Ptr previousCloud(new pcl::PointCloud<pcl::PointXYZ>());

while (!viewer.stopped()) {
    // Get the current point cloud from the Lidar
    pcl::PointCloud<Point3D>::Ptr cloud = laser->getTargetPointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert your Point3D points to pcl::PointXYZ points
    for (const auto& point : cloud->points) {
        pcl::PointXYZ xyz_point;
        xyz_point.x = point.x;
        xyz_point.y = point.y;
        xyz_point.z = point.z;
        xyz_cloud->points.push_back(xyz_point);
    }

    // Skip the comparison if this is the first scan (i.e., no previous cloud available)
    if (previousCloud->empty()) {
        *previousCloud = *xyz_cloud;  // Store the first scan as the previous cloud
        continue;  // Skip to the next iteration
    }

    // Filter the clouds based on the height
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 5.0);

    pass.setInputCloud(previousCloud);
    pass.filter(*cloud_filtered_source);

    pass.setInputCloud(xyz_cloud);
    pass.filter(*cloud_filtered_target);

    Eigen::Matrix4f transformationMatrix;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Use 'template' keyword to disambiguate the function template
    bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
        cloud_filtered_source, cloud_filtered_target, transformationMatrix);

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> processingTime = endTime - startTime;

    if (!gicpConverged) {
        std::cerr << "GICP did not converge for the current scan comparison" << std::endl;
        nonConvergenceCount++;
    } else {
        // Process the transformation as before
        Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
        Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);
        float yaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));

        Eigen::Matrix3f newRotationMatrix;
        newRotationMatrix << cos(yaw), -sin(yaw), 0.0f,
                             sin(yaw),  cos(yaw), 0.0f,
                                  0.0f,      0.0f, 1.0f;

        translationVector[2] = 0.0f;
        Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
        modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
        modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;
        transformationMatrix = modifiedTransformationMatrix;

        // Accumulate the transformation matrix
        cumulativeTransformation = cumulativeTransformation * transformationMatrix.inverse();
    }

    // Update the previous cloud for the next iteration
    *previousCloud = *xyz_cloud;  // Save the current scan as the previous scan

    viewer->spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (viewer->wasStopped()) {
        break;
    }



    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
