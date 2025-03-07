/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <complex>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include "VisionUtility/PointCloud/Gicp.hpp"  // Ensure this points to your corrected gicp.hpp

namespace fs = std::filesystem;

// Type Definitions
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using FPFHT = pcl::FPFHSignature33;
using FPFHCloud = pcl::PointCloud<FPFHT>;

// Constants
const float VOXEL_GRID_SIZE = 0.01f;
const double RADIUS_NORMAL = 0.2;  // Adjusted for scale
const double RADIUS_FEATURE = 0.5;  // Adjusted for scale
const int MAX_SACIA_ITERATION = 1000;
const double MIN_CORRESPONDENCE_DIST = 0.01;
const double MAX_CORRESPONDENCE_DIST = 1.0;

long extractNumber(const std::string& filename) {
    std::string::size_type start = filename.find_last_of('_') + 1;
    std::string::size_type end = filename.find_last_of('.');
    std::string numberStr = filename.substr(start, end - start);
    return std::stol(numberStr);
}

// Function to modify the transformation matrix
Eigen::Matrix4f modifyTransformationMatrix(const Eigen::Matrix4f& transformationMatrix) {
    // Extract rotation matrix and translation vector from transformation matrix
    Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3,3>(0,0);
    Eigen::Vector3f translationVector = transformationMatrix.block<3,1>(0,3);

    // Extract yaw angle from rotation matrix
    float yaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));

    // Reconstruct rotation matrix with only yaw rotation
    Eigen::Matrix3f newRotationMatrix;
    newRotationMatrix << cos(yaw), -sin(yaw), 0.0f,
                         sin(yaw),  cos(yaw), 0.0f,
                              0.0f,      0.0f, 1.0f;

    // Zero out the Z component of translation vector
    translationVector[2] = 0.0f;

    // Reconstruct the transformation matrix
    Eigen::Matrix4f modifiedTransformationMatrix = Eigen::Matrix4f::Identity();
    modifiedTransformationMatrix.block<3,3>(0,0) = newRotationMatrix;
    modifiedTransformationMatrix.block<3,1>(0,3) = translationVector;

    return modifiedTransformationMatrix;
}

// Function to apply voxel grid filter
void voxelFilter(PointCloud::Ptr& cloud_in, PointCloud::Ptr& cloud_out, float grid_size) {
    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setLeafSize(grid_size, grid_size, grid_size);
    vox_grid.setInputCloud(cloud_in);
    vox_grid.filter(*cloud_out);
}

// Function to compute normals
pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr cloud, double radius) {
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setRadiusSearch(radius);
    norm_est.compute(*normalsPtr);
    return normalsPtr;
}

// Function to compute FPFH features
FPFHCloud::Ptr getFeatures(PointCloud::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    FPFHCloud::Ptr features(new FPFHCloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::FPFHEstimation<PointT, pcl::Normal, FPFHT> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(radius);
    fpfh_est.compute(*features);
    return features;
}

// Function to perform SAC-IA alignment
Eigen::Matrix4f sacIaAlign(PointCloud::Ptr source,
                           PointCloud::Ptr target,
                           FPFHCloud::Ptr source_feature,
                           FPFHCloud::Ptr target_feature,
                           int max_sacia_iterations,
                           double min_correspondence_dist,
                           double max_correspondence_dist,
                           bool& has_converged) {
    pcl::SampleConsensusInitialAlignment<PointT, PointT, FPFHT> sac_ia;
    Eigen::Matrix4f final_transformation;

    sac_ia.setInputSource(target);
    sac_ia.setSourceFeatures(target_feature);
    sac_ia.setInputTarget(source);
    sac_ia.setTargetFeatures(source_feature);
    sac_ia.setMaximumIterations(max_sacia_iterations);
    sac_ia.setMinSampleDistance(min_correspondence_dist);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);

    PointCloud registration_output;
    sac_ia.align(registration_output);
    has_converged = sac_ia.hasConverged();

    if (has_converged) {
        final_transformation = sac_ia.getFinalTransformation();
    } else {
        final_transformation = Eigen::Matrix4f::Identity();
    }

    return final_transformation;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <directory_path> <method>" << std::endl;
        std::cerr << "Methods: 'only sac_ia', 'only gicp', 'both combined'" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::string method = argv[2];
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // Convert to lowercase

    if (method != "only sac_ia" && method != "only gicp" && method != "both combined") {
        std::cerr << "Invalid method. Choose 'only sac_ia', 'only gicp', or 'both combined'." << std::endl;
        return -1;
    }

    std::vector<std::string> pcdFiles;

    // Read all PCD files in the directory
    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.path().extension() == ".pcd") {
            pcdFiles.push_back(entry.path().string());
        }
    }

    // Sort the files based on the numeric part of the filename
    std::sort(pcdFiles.begin(), pcdFiles.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });

    if (pcdFiles.size() < 2) {
        std::cerr << "Not enough point clouds for comparison." << std::endl;
        return -1;
    }

    double totalProcessingTime = 0.0;
    int comparisonCount = 0;
    int nonConvergenceCount = 0;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();

    try {
        for (size_t i = 1; i < pcdFiles.size(); ++i) {
            std::string sourceFile = pcdFiles[i - 1];
            std::string targetFile = pcdFiles[i];

            // Load the point clouds
            PointCloud::Ptr sourceCloud(new PointCloud);
            PointCloud::Ptr targetCloud(new PointCloud);
            if (pcl::io::loadPCDFile<PointT>(sourceFile, *sourceCloud) == -1) {
                std::cerr << "Couldn't read source file " << sourceFile << std::endl;
                return -1;
            }
            if (pcl::io::loadPCDFile<PointT>(targetFile, *targetCloud) == -1) {
                std::cerr << "Couldn't read target file " << targetFile << std::endl;
                return -1;
            }

            // Remove NaNs
            std::vector<int> indices1;
            std::vector<int> indices2;
            pcl::removeNaNFromPointCloud(*sourceCloud, *sourceCloud, indices1);
            pcl::removeNaNFromPointCloud(*targetCloud, *targetCloud, indices2);

            // Filter the clouds based on the height
            PointCloud::Ptr cloud_filtered_source(new PointCloud);
            PointCloud::Ptr cloud_filtered_target(new PointCloud);

            pcl::PassThrough<PointT> pass;
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 5.0);

            pass.setInputCloud(sourceCloud);
            pass.filter(*cloud_filtered_source);

            pass.setInputCloud(targetCloud);
            pass.filter(*cloud_filtered_target);

            // Remove outliers (optional)
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setMeanK(50);
            sor.setStddevMulThresh(10.0);
            sor.setInputCloud(cloud_filtered_source);
            sor.filter(*cloud_filtered_source);

            sor.setInputCloud(cloud_filtered_target);
            sor.filter(*cloud_filtered_target);

            // Downsample the point clouds
            voxelFilter(cloud_filtered_source, cloud_filtered_source, VOXEL_GRID_SIZE);
            voxelFilter(cloud_filtered_target, cloud_filtered_target, VOXEL_GRID_SIZE);

            auto startTime = std::chrono::high_resolution_clock::now();

            bool gicpConverged = false;
            bool sacIAConverged = false;
            Eigen::Matrix4f transformationMatrixGICP = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f transformationMatrixSACIA = Eigen::Matrix4f::Identity();

            if (method == "only gicp" || method == "both combined") {
                // Apply GICP
                gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<PointT>(
                    cloud_filtered_source, cloud_filtered_target, transformationMatrixGICP);
            }

            if (method == "only sac_ia" || method == "both combined") {
                // Compute normals
                pcl::PointCloud<pcl::Normal>::Ptr source_normals = getNormals(cloud_filtered_source, RADIUS_NORMAL);
                pcl::PointCloud<pcl::Normal>::Ptr target_normals = getNormals(cloud_filtered_target, RADIUS_NORMAL);

                // Compute FPFH features
                FPFHCloud::Ptr source_features = getFeatures(cloud_filtered_source, source_normals, RADIUS_FEATURE);
                FPFHCloud::Ptr target_features = getFeatures(cloud_filtered_target, target_normals, RADIUS_FEATURE);

                // Perform SAC-IA alignment
                transformationMatrixSACIA = sacIaAlign(
                    cloud_filtered_source,
                    cloud_filtered_target,
                    source_features,
                    target_features,
                    MAX_SACIA_ITERATION,
                    MIN_CORRESPONDENCE_DIST,
                    MAX_CORRESPONDENCE_DIST,
                    sacIAConverged);
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            if ((method == "only gicp" && !gicpConverged) ||
                (method == "only sac_ia" && !sacIAConverged) ||
                ((method == "both combined") && !gicpConverged && !sacIAConverged)) {
                std::cerr << "Registration did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }

            // Apply modifications to the transformation matrices
            Eigen::Matrix4f modifiedTransformationMatrixGICP = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f modifiedTransformationMatrixSACIA = Eigen::Matrix4f::Identity();

            if (gicpConverged) {
                modifiedTransformationMatrixGICP = modifyTransformationMatrix(transformationMatrixGICP);
            }

            if (sacIAConverged) {
                modifiedTransformationMatrixSACIA = modifyTransformationMatrix(transformationMatrixSACIA);
            }

            // Extract yaw angles and translation vectors
            float yawGICP = 0.0f;
            Eigen::Vector3f translationVectorGICP = Eigen::Vector3f::Zero();

            float yawSACIA = 0.0f;
            Eigen::Vector3f translationVectorSACIA = Eigen::Vector3f::Zero();

            if (gicpConverged) {
                // Extract yaw and translation
                Eigen::Matrix3f rotationMatrixGICP = modifiedTransformationMatrixGICP.block<3,3>(0,0);
                yawGICP = atan2(rotationMatrixGICP(1,0), rotationMatrixGICP(0,0));
                translationVectorGICP = modifiedTransformationMatrixGICP.block<3,1>(0,3);
            }

            if (sacIAConverged) {
                // Extract yaw and translation
                Eigen::Matrix3f rotationMatrixSACIA = modifiedTransformationMatrixSACIA.block<3,3>(0,0);
                yawSACIA = atan2(rotationMatrixSACIA(1,0), rotationMatrixSACIA(0,0));
                translationVectorSACIA = modifiedTransformationMatrixSACIA.block<3,1>(0,3);
            }

            // Average yaw angles and translation vectors
            float yawAverage = 0.0f;
            Eigen::Vector3f translationVectorAverage = Eigen::Vector3f::Zero();

            if (method == "both combined") {
                if (gicpConverged && sacIAConverged) {
                    // Both converged, average the rotations and translations
                    // Average rotations properly using complex numbers
                    std::complex<float> rotationGICP(cos(yawGICP), sin(yawGICP));
                    std::complex<float> rotationSACIA(cos(yawSACIA), sin(yawSACIA));

                    std::complex<float> rotationAverage = rotationGICP + rotationSACIA;
                    rotationAverage /= std::abs(rotationAverage);
                    yawAverage = atan2(rotationAverage.imag(), rotationAverage.real());

                    // Average translations
                    translationVectorAverage = (translationVectorGICP + translationVectorSACIA) / 2.0f;

                } else if (gicpConverged) {
                    // Only GICP converged
                    std::cout << "Only GICP converged, using its result." << std::endl;
                    yawAverage = yawGICP;
                    translationVectorAverage = translationVectorGICP;
                } else if (sacIAConverged) {
                    // Only SAC-IA converged
                    std::cout << "Only SAC-IA converged, using its result." << std::endl;
                    yawAverage = yawSACIA;
                    translationVectorAverage = translationVectorSACIA;
                }
            } else if (method == "only gicp" && gicpConverged) {
                yawAverage = yawGICP;
                translationVectorAverage = translationVectorGICP;
            } else if (method == "only sac_ia" && sacIAConverged) {
                yawAverage = yawSACIA;
                translationVectorAverage = translationVectorSACIA;
            } else {
                // Should not reach here due to earlier checks
                std::cerr << "Unexpected error in registration." << std::endl;
                continue;
            }

            // Reconstruct the averaged transformation matrix
            Eigen::Matrix3f rotationMatrixAverage;
            rotationMatrixAverage << cos(yawAverage), -sin(yawAverage), 0.0f,
                                     sin(yawAverage),  cos(yawAverage), 0.0f,
                                              0.0f,           0.0f, 1.0f;

            Eigen::Matrix4f averagedTransformationMatrix = Eigen::Matrix4f::Identity();
            averagedTransformationMatrix.block<3,3>(0,0) = rotationMatrixAverage;
            averagedTransformationMatrix.block<3,1>(0,3) = translationVectorAverage;

            totalProcessingTime += processingTime.count();
            comparisonCount++;

            // Invert the transformation matrix to align target to source
            Eigen::Matrix4f transformationMatrixToAlignTargetToSource = averagedTransformationMatrix.inverse();

            // Accumulate the transformation matrix
            cumulativeTransformation = cumulativeTransformation * transformationMatrixToAlignTargetToSource;

            std::cout << "Cumulative Transformation Matrix:\n" << cumulativeTransformation << std::endl;

            // Transform the target cloud
            PointCloud::Ptr transformedTargetCloud(new PointCloud);
            pcl::transformPointCloud(*targetCloud, *transformedTargetCloud, cumulativeTransformation);

            std::string cloudName = "transformedCloud_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> transformedColor(transformedTargetCloud, 255, 255, 255);
            viewer->addPointCloud<PointT>(transformedTargetCloud, transformedColor, cloudName);

            viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (viewer->wasStopped()) {
                break;
            }
        }

        if (comparisonCount > 0) {
            double averageProcessingTime = totalProcessingTime / comparisonCount;
            std::cout << "Average processing time: " << averageProcessingTime << " seconds" << std::endl;
        } else {
            std::cerr << "No valid comparisons made." << std::endl;
        }

        std::cout << "Number of non-converged registrations: " << nonConvergenceCount << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
