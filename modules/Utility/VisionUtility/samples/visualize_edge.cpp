/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <iostream>
#include <string>
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::milliseconds
#include <algorithm>  // For sorting sharpness values
#include "VisionUtility/PointCloud/Edge.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"
  // Assuming the provided sharpness functions are here

// Function to filter the top percentage of the sharpest points
pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr filterTopSharpness(
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud, float percent) {

    // Ensure percent is between 0 and 100
    if (percent <= 0.0f || percent > 100.0f) {
        throw std::invalid_argument("Percent must be between 0 and 100.");
    }

    // Get the sharpness values
    std::vector<float> sharpness_values;
    for (const auto& point : sharpness_cloud->points) {
        sharpness_values.push_back(point.s);
    }

    // Sort the sharpness values in descending order
    std::sort(sharpness_values.begin(), sharpness_values.end(), std::greater<float>());

    // Get the threshold for the top N% sharpness
    size_t top_percent_index = static_cast<size_t>(sharpness_values.size() * (1.0f - (percent / 100.0f)));
    float sharpness_threshold = sharpness_values[top_percent_index];

    // Filter points that have sharpness values greater than or equal to the threshold
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr filtered_cloud(
        new pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>());

    for (const auto& point : sharpness_cloud->points) {
        if (point.s >= sharpness_threshold) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

// Function to visualize the sharpness of the edges in a point cloud
void visualizeSharpness(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud) {
    // Create a PCL Visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Edge Sharpness Visualizer"));

    // Set background to black
    viewer->setBackgroundColor(1, 1, 1);

    // Add the original point cloud in white (optional, or remove if you want only sharp points)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 0, 0); // White
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");

    // Add the sharpness points (color intensity based on sharpness)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& point : sharpness_cloud->points) {
        pcl::PointXYZRGB color_point;
        color_point.x = point.x;
        color_point.y = point.y;
        color_point.z = point.z;

        // Map sharpness value to color
        uint8_t intensity = static_cast<uint8_t>(255 * point.s);
        color_point.r = intensity;
        color_point.g = 255- intensity;
        color_point.b = intensity;  

        color_cloud->points.push_back(color_point);
    }
    color_cloud->width = color_cloud->points.size();
    color_cloud->height = 1;

    viewer->addPointCloud<pcl::PointXYZRGB>(color_cloud, "sharpness");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sharpness");

    // Keep the visualization window open until user closes it
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 100ms
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file> <percent>" << std::endl;
        return -1;
    }

    std::string pcd_file = argv[1];
    float percent = std::stof(argv[2]);  // Convert the argument to a float

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
        std::cerr << "Failed to load PCD file: " << pcd_file << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    // Set parameters for sharpness computation
    unsigned int kNeighbour = 20;  // Number of neighbors for edge detection

    // Call the edge sharpness function from the provided code
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud =
        crf::utility::visionutility::pointcloud::edge::computeEigenSharpness<pcl::PointXYZ>(cloud, kNeighbour,100);

    if (!sharpness_cloud || sharpness_cloud->empty()) {
        std::cerr << "No sharpness data computed!" << std::endl;
        return -1;
    }

    std::cout << "Computed sharpness for " << sharpness_cloud->points.size() << " points." << std::endl;

    // Filter the sharpness to show only the top N%
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr filtered_sharpness_cloud =
        filterTopSharpness(sharpness_cloud, percent);

    std::cout << "Filtered point cloud to " << filtered_sharpness_cloud->points.size() << " points (top " << percent << "% by sharpness)." << std::endl;
    
    std::string output_pcd_file = "filtered_sharpness_cloud.pcd";
    if (!crf::utility::visionutility::pointcloud::communication::saveCloudinPcdFormat<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>(filtered_sharpness_cloud, output_pcd_file, false)) {
        std::cerr << "Failed to save sharpness cloud to PCD file: " << output_pcd_file << std::endl;
        return -1;
    }
    // Visualize the top N% sharpness points
    visualizeSharpness(cloud, filtered_sharpness_cloud);

    return 0;
}
