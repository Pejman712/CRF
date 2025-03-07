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
#include <thread>  // Add this header for std::this_thread
#include <chrono>  // For std::chrono::milliseconds
#include "VisionUtility/PointCloud/Keypoint.hpp" 
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"
#include "VisionUtility/PointCloud/Filter.hpp"// Assuming the provided functions are here

void visualizeKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
    // Create a PCL Visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Keypoint Visualizer"));

    // Set background to black
    viewer->setBackgroundColor(1, 1, 1);

    // Add the original point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 0, 0); // White
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");

    // Add the keypoints with a different color (red)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color(keypoints, 0, 255, 0); // Red
    viewer->addPointCloud<pcl::PointXYZ>(keypoints, keypoints_color, "keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

    // Keep the visualization window open until user closes it
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Use std::this_thread::sleep_for with chrono
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file>" << std::endl;
        return -1;
    }

    std::string pcd_file = argv[1];

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
        std::cerr << "Failed to load PCD file: " << pcd_file << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    // " create a cloud with checkForArtifactsInPointcloud before calculating the normals"
    /*
    float passThroughMinLimit = 0.01f; // Example minimum distance limit
    auto artifactcloud = crf::utility::visionutility::pointcloud::filter::checkForArtifactsInPointcloud<pcl::PointXYZ>(cloud, passThroughMinLimit);
    
    if (!artifactcloud) {
        std::cerr << "Point cloud contains artifacts or too close objects, aborting." << std::endl;
        
        return -1;
    }
    
    //std::cout << "Loaded artifact point cloud with " << artifactcloud->points.size() << " points." << std::endl;
    //visualizeKeypoints(cloud, artifactcloud);

    // Step 2: Compute normals
    //auto normals = crf::utility::visionutility::pointcloud::normal::computeNormals<pcl::PointXYZ>(artifactcloud, 20); 

    //visualizeKeypoints(cloud, artifactcloud);

    */

    auto normals = crf::utility::visionutility::pointcloud::normal::computeNormals<pcl::PointXYZ>(cloud, 20); 
    // Keypoint extraction parameters
    std::vector<float> scales = {0.02f, 0.04f, 0.08f}; // Example scale values
    float persistence_alpha = 1.5f; // Persistence threshold
    pcl::NormType norm_method = pcl::L2_SQR; // Use L2_SQRDIST as the normalization method

    // Call the keypoint extraction function from the provided code
    auto keypoints_result = crf::utility::visionutility::pointcloud::keypoint::getFPFHKeypoints<pcl::PointXYZ>(
        cloud, normals, scales, persistence_alpha, norm_method);

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = keypoints_result.first;

    if (!keypoints || keypoints->empty()) {
        std::cerr << "No keypoints extracted!" << std::endl;
        return -1;
    }
    std::string output_pcd_file = "keypoints.pcd";
    if (!crf::utility::visionutility::pointcloud::communication::saveCloudinPcdFormat<pcl::PointXYZ>(keypoints, output_pcd_file, false)) {
        std::cerr << "Failed to save keypoints to PCD file: " << output_pcd_file << std::endl;
        return -1;
    }
    std::cout << "Extracted " << keypoints->points.size() << " keypoints." << std::endl;
    // Visualize the keypoints
    visualizeKeypoints(cloud, keypoints);

    return 0;
}

