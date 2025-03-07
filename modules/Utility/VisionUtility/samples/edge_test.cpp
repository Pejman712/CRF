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
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem> 
#include <thread> 
#include "VisionUtility/PointCloud/Edge.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"

namespace fs = std::filesystem;


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
// probably can add them in evaluation.hpp for now they just stay here 
float calculateAverageDistanceBetweenClosestPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) {
    // Create a k-d tree for searching in cloud2
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    float total_distance = 0.0f;
    size_t valid_points = 0;

    // For each point in cloud1, find the closest point in cloud2
    for (const auto& point : cloud1->points) {
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);

        if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0) {
            total_distance += std::sqrt(nearest_distances[0]);
            valid_points++;
        }
    }

    // Calculate the average distance
    if (valid_points == 0) {
        return 0.0f;  // Avoid division by zero
    }

    return total_distance / valid_points;
}

// Function to calculate feature tracking consistency between scans using pcl::PointXYZ
float calculateTrackingConsistency(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2, float ransac_threshold) {
    // Estimate correspondences between the source and target clouds
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> correspondence_estimation;
    pcl::Correspondences correspondences;
    correspondence_estimation.setInputSource(cloud1);
    correspondence_estimation.setInputTarget(cloud2);
    correspondence_estimation.determineCorrespondences(correspondences);

    std::cout << "Initial correspondences: " << correspondences.size() << std::endl;

    // RANSAC-based correspondence rejection
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac_rejector;
    ransac_rejector.setInputSource(cloud1);   // Set the source cloud
    ransac_rejector.setInputTarget(cloud2);   // Set the target cloud
    ransac_rejector.setInlierThreshold(ransac_threshold);  // Set the RANSAC inlier threshold
    ransac_rejector.setInputCorrespondences(pcl::make_shared<pcl::Correspondences>(correspondences));  // Set the initial correspondences

    pcl::Correspondences inlier_correspondences;
    ransac_rejector.getCorrespondences(inlier_correspondences);  // Fetch the inliers after RANSAC

    std::cout << "RANSAC inliers: " << inlier_correspondences.size() << std::endl;

    // Calculate tracking consistency as a percentage of inliers relative to initial correspondences
    return (static_cast<float>(inlier_correspondences.size()) / correspondences.size()) * 100.0f;
}

// Function to extract pcl::PointXYZ from pcl::PointCloud<PointXYZS> the correlation doesnt like custom pcl types 
pcl::PointCloud<pcl::PointXYZ>::Ptr extractXYZ(
    pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : sharpness_cloud->points) {
        pcl::PointXYZ xyz_point;
        xyz_point.x = point.x;
        xyz_point.y = point.y;
        xyz_point.z = point.z;
        xyz_cloud->points.push_back(xyz_point);
    }

    xyz_cloud->width = xyz_cloud->points.size();
    xyz_cloud->height = 1;
    xyz_cloud->is_dense = true;

    return xyz_cloud;
}

// Function to visualize the sharpness of the edges in a point cloud just to see the difference 
void visualizeSharpness(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Edge Sharpness Visualizer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 255, 255); // White
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& point : sharpness_cloud->points) {
        pcl::PointXYZRGB color_point;
        color_point.x = point.x;
        color_point.y = point.y;
        color_point.z = point.z;

        uint8_t intensity = static_cast<uint8_t>(255 * point.s);
        color_point.r = intensity;
        color_point.g = intensity;
        color_point.b = 255 - intensity;

        color_cloud->points.push_back(color_point);
    }
    color_cloud->width = color_cloud->points.size();
    color_cloud->height = 1;

    viewer->addPointCloud<pcl::PointXYZRGB>(color_cloud, "sharpness");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sharpness");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 100ms
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-folder> <percent>" << std::endl;
        return -1;
    }

    std::string pcd_folder = argv[1];
    float percent = std::stof(argv[2]);  // Convert the argument to a float

    // Create output CSV file
    std::ofstream csv_file("processing_results.csv");
    csv_file << "PCD File,Number of Key Features,Processing Time (ms)\n"; // Updated header for CSV file

    // Get all PCD files in the folder
    std::vector<std::string> pcd_files;
    for (const auto& entry : fs::directory_iterator(pcd_folder)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().string());
        }
    }

    // Sort the PCD files to ensure proper sequential comparison
    std::sort(pcd_files.begin(), pcd_files.end());

    // Ensure there are at least two scans to compare
    if (pcd_files.empty()) {
        std::cerr << "No PCD files in the folder to process." << std::endl;
        return -1;
    }

    // Iterate through each scan
    for (size_t i = 0; i < pcd_files.size(); ++i) {
        std::cout << "Processing scan " << i << " (" << pcd_files[i] << ")." << std::endl;

        // Measure the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        // Load the scan
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files[i], *cloud) == -1) {
            std::cerr << "Failed to load PCD file: " << pcd_files[i] << std::endl;
            continue;
        }

        std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

        // Call the edge sharpness function for the scan
        unsigned int kNeighbour = 100;
        pcl::PointCloud<crf::utility::visionutility::pointcloud::pointtypes::PointXYZS>::Ptr sharpness_cloud =
            crf::utility::visionutility::pointcloud::edge::computeEigenSharpness<pcl::PointXYZ>(cloud, kNeighbour);

        if (!sharpness_cloud || sharpness_cloud->empty()) {
            std::cerr << "No sharpness data computed!" << std::endl;
            continue;
        }

        // Filter the sharpness to show only the top N%
        auto filtered_sharpness = filterTopSharpness(sharpness_cloud, percent);

        std::cout << "Filtered point cloud to top " << percent << "% by sharpness." << std::endl;

        // Count the number of sharp points in the filtered cloud
        size_t sharp_points_detected = filtered_sharpness->points.size();
        std::cout << "Number of sharp points detected: " << sharp_points_detected << std::endl;

        // Measure the end time and calculate processing duration in milliseconds
        auto end_time = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Processing time: " << processing_time.count() << " ms." << std::endl;

        // Write results to CSV
        csv_file << pcd_files[i] << "," << sharp_points_detected << "," << processing_time.count() << "\n";
    }

    // Close the CSV file
    csv_file.close();

    return 0;
}
