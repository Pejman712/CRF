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
#include <thread>  
#include <chrono>  
#include <filesystem>  // For reading the directory (C++17)
#include <fstream>     // For CSV file operations
#include "VisionUtility/PointCloud/Keypoint.hpp" 
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"
#include "VisionUtility/PointCloud/Filter.hpp" 

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-folder>" << std::endl;
        return -1;
    }

    std::string pcd_folder = argv[1];

    // Create output CSV file
    std::ofstream csv_file("processing_results.csv");
    csv_file << "PCD File,Number of Key Features,Processing Time (ms)\n";  // Updated header for CSV file

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

    // Process each PCD file
    for (const auto& pcd_file : pcd_files) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Load the PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
            std::cerr << "Failed to load PCD file: " << pcd_file << std::endl;
            continue;  // Skip this file and move to the next
        }

        std::cout << "Loaded point cloud with " << cloud->points.size() << " points from file: " << pcd_file << std::endl;

        // Start measuring the processing time
        auto start_time = std::chrono::high_resolution_clock::now();

        // Compute normals
        auto normals = crf::utility::visionutility::pointcloud::normal::computeNormals<pcl::PointXYZ>(cloud, 20);

        // Keypoint extraction parameters
        std::vector<float> scales = {0.02f, 0.04f, 0.08f}; // Example scale values
        float persistence_alpha = 1.5f; // Persistence threshold
        pcl::NormType norm_method = pcl::L2_SQR; // Use L2_SQRDIST as the normalization method

        // Call the keypoint extraction function
        auto keypoints_result = crf::utility::visionutility::pointcloud::keypoint::getFPFHKeypoints<pcl::PointXYZ>(
            cloud, normals, scales, persistence_alpha, norm_method);

        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = keypoints_result.first;

        if (!keypoints || keypoints->empty()) {
            std::cerr << "No keypoints extracted from file: " << pcd_file << std::endl;
            continue;  // Skip this file if no keypoints were found
        }

        // End measuring the processing time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto processing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << "Extracted " << keypoints->points.size() << " keypoints from file: " << pcd_file << " in " << processing_duration << " ms." << std::endl;

        // Save the results to the CSV file
        csv_file << pcd_file << "," << keypoints->points.size() << "," << processing_duration << "\n";
    }

    csv_file.close();
    std::cout << "Processing completed. Results saved to 'processing_results.csv'." << std::endl;

    return 0;
}
