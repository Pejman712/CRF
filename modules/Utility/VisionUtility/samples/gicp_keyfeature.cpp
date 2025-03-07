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
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "VisionUtility/PointCloud/Keypoint.hpp" 
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"
#include "VisionUtility/PointCloud/Filter.hpp"
#include <VisionUtility/PointCloud/Gicp.hpp>



void visualizeKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Keypoint Visualizer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 255, 255); // White
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color(keypoints, 255, 0, 0); // Red
    viewer->addPointCloud<pcl::PointXYZ>(keypoints, keypoints_color, "keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Function to calculate RMSE between aligned clouds
double calculateRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    double sum_squared_error = 0.0;
    for (size_t i = 0; i < source->size(); ++i) {
        Eigen::Vector3f diff = source->points[i].getVector3fMap() - target->points[i].getVector3fMap();
        sum_squared_error += diff.squaredNorm();
    }
    return sqrt(sum_squared_error / source->size());
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file-1> <path-to-pcd-file-2>" << std::endl;
        return -1;
    }

    std::string pcd_file_1 = argv[1];
    std::string pcd_file_2 = argv[2];

    // Load the two PCD files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_1, *cloud_1) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_2, *cloud_2) == -1) {
        std::cerr << "Failed to load one or both PCD files." << std::endl;
        return -1;
    }

    // Compute normals for both clouds
    auto normals_1 = crf::utility::visionutility::pointcloud::normal::computeNormals<pcl::PointXYZ>(cloud_1, 20); 
    auto normals_2 = crf::utility::visionutility::pointcloud::normal::computeNormals<pcl::PointXYZ>(cloud_2, 20); 

    // Keypoint extraction parameters
    std::vector<float> scales = {0.02f, 0.04f, 0.08f}; // Example scale values
    float persistence_alpha = 1.5f;
    pcl::NormType norm_method = pcl::L2_SQR;

    // Extract keypoints from both clouds
    auto keypoints_result_1 = crf::utility::visionutility::pointcloud::keypoint::getFPFHKeypoints<pcl::PointXYZ>(
        cloud_1, normals_1, scales, persistence_alpha, norm_method);

    auto keypoints_result_2 = crf::utility::visionutility::pointcloud::keypoint::getFPFHKeypoints<pcl::PointXYZ>(
        cloud_2, normals_2, scales, persistence_alpha, norm_method);

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1 = keypoints_result_1.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2 = keypoints_result_2.first;

    if (!keypoints_1 || keypoints_1->empty() || !keypoints_2 || keypoints_2->empty()) {
        std::cerr << "Keypoint extraction failed!" << std::endl;
        return -1;
    }

    // Perform GICP registration
    Eigen::Matrix4f transformationMatrix;
    try {
        transformationMatrix = crf::utility::visionutility::pointcloud::gicp::gicp<pcl::PointXYZ>(pcd_file_1, pcd_file_2);
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    std::cout << "GICP Transformation Matrix:\n" << transformationMatrix << std::endl;

    // Apply the transformation to keypoints_1
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keypoints_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*keypoints_1, *transformed_keypoints_1, transformationMatrix);

    // Calculate RMSE between the aligned keypoints
    double rmse = calculateRMSE(transformed_keypoints_1, keypoints_2);
    std::cout << "Tracking accuracy (RMSE): " << rmse << std::endl;

    // Visualize the aligned keypoints
    visualizeKeypoints(cloud_1, transformed_keypoints_1);
    visualizeKeypoints(cloud_2, keypoints_2);

    return 0;
}
