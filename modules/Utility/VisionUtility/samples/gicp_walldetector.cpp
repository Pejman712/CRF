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
#include <pcl/filters/statistical_outlier_removal.h>  // Added for SOR
#include <complex>  // Added for complex numbers
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>  // Added for RANSAC
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/ndt.h>  // Added for NDT
#include "VisionUtility/PointCloud/Gicp.hpp"  // Ensure this points to your corrected gicp.hpp

namespace fs = std::filesystem;

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

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <directory_path> <method>" << std::endl;
        std::cerr << "Methods: 'only gicp', 'only ndt', 'only plane detector', 'gicp + plane detector', 'ndt + plane detector'" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::string method = argv[2];
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // Convert to lowercase

    if (method != "only gicp" && method != "only ndt" && method != "only plane detector" &&
        method != "gicp + plane detector" && method != "ndt + plane detector") {
        std::cerr << "Invalid method. Choose 'only gicp', 'only ndt', 'only plane detector', 'gicp + plane detector', or 'ndt + plane detector'." << std::endl;
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
    std::vector<std::pair<double, double>> translationPath;
    translationPath.emplace_back(0.0, 0.0);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

    try {
        for (size_t i = 1; i < pcdFiles.size(); ++i) {
            std::string sourceFile = pcdFiles[i - 1];
            std::string targetFile = pcdFiles[i];

            // Load the point clouds
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(sourceFile, *sourceCloud) == -1) {
                std::cerr << "Couldn't read source file " << sourceFile << std::endl;
                return -1;
            }
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(targetFile, *targetCloud) == -1) {
                std::cerr << "Couldn't read target file " << targetFile << std::endl;
                return -1;
            }

            // Filter the clouds based on the height
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 2.5);  // Filter between 0 and 2.5 meters in Z

            pass.setInputCloud(sourceCloud);
            pass.filter(*cloud_filtered_source);

            pass.setInputCloud(targetCloud);
            pass.filter(*cloud_filtered_target);

            auto startTime = std::chrono::high_resolution_clock::now();

            bool registrationConverged = false;
            Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();

            // Registration methods
            if (method == "only gicp" || method == "gicp + plane detector") {
                // Apply GICP
                registrationConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
                    cloud_filtered_source, cloud_filtered_target, transformationMatrix);
            } else if (method == "only ndt" || method == "ndt + plane detector") {
                // Apply NDT
                pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
                ndt.setTransformationEpsilon(1e-8);
                ndt.setStepSize(0.1);
                ndt.setResolution(1.0);
                ndt.setMaximumIterations(500);

                // Set input clouds
                ndt.setInputSource(cloud_filtered_source);
                ndt.setInputTarget(cloud_filtered_target);

                // Perform alignment
                pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
                ndt.align(*outputCloud);
                registrationConverged = ndt.hasConverged();
                transformationMatrix = ndt.getFinalTransformation();
            } else if (method == "only plane detector") {
                // Skip initial registration
                registrationConverged = true;
                transformationMatrix = Eigen::Matrix4f::Identity();
            }

            if (!registrationConverged) {
                std::cerr << "Initial registration did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }

            // Now proceed with plane detection if required
            if (method == "only plane detector" || method == "gicp + plane detector" || method == "ndt + plane detector") {
                // Transform the source cloud using the initial transformation matrix
                pcl::PointCloud<pcl::PointXYZ>::Ptr alignedSourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*cloud_filtered_source, *alignedSourceCloud, transformationMatrix);

                // Apply RANSAC for plane detection on the source cloud
                pcl::ModelCoefficients::Ptr coefficientsSource(new pcl::ModelCoefficients());
                pcl::PointIndices::Ptr inliersSource(new pcl::PointIndices());
                pcl::SACSegmentation<pcl::PointXYZ> segSource;
                segSource.setOptimizeCoefficients(true);
                segSource.setModelType(pcl::SACMODEL_PLANE);
                segSource.setMethodType(pcl::SAC_RANSAC);
                segSource.setDistanceThreshold(0.05); // adjust as needed
                segSource.setInputCloud(alignedSourceCloud);
                segSource.segment(*inliersSource, *coefficientsSource);

                if (inliersSource->indices.size() == 0) {
                    PCL_ERROR("Could not estimate a plane model for the source dataset.");
                    nonConvergenceCount++;
                    continue;
                }

                // Extract plane points from source cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePlanePoints(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::ExtractIndices<pcl::PointXYZ> extractSource;
                extractSource.setInputCloud(alignedSourceCloud);
                extractSource.setIndices(inliersSource);
                extractSource.setNegative(false);
                extractSource.filter(*sourcePlanePoints);

                // Apply RANSAC for plane detection on the target cloud
                pcl::ModelCoefficients::Ptr coefficientsTarget(new pcl::ModelCoefficients());
                pcl::PointIndices::Ptr inliersTarget(new pcl::PointIndices());
                pcl::SACSegmentation<pcl::PointXYZ> segTarget;
                segTarget.setOptimizeCoefficients(true);
                segTarget.setModelType(pcl::SACMODEL_PLANE);
                segTarget.setMethodType(pcl::SAC_RANSAC);
                segTarget.setDistanceThreshold(0.05); // adjust as needed
                segTarget.setInputCloud(cloud_filtered_target);
                segTarget.segment(*inliersTarget, *coefficientsTarget);

                if (inliersTarget->indices.size() == 0) {
                    PCL_ERROR("Could not estimate a plane model for the target dataset.");
                    nonConvergenceCount++;
                    continue;
                }

                // Extract plane points from target cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr targetPlanePoints(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::ExtractIndices<pcl::PointXYZ> extractTarget;
                extractTarget.setInputCloud(cloud_filtered_target);
                extractTarget.setIndices(inliersTarget);
                extractTarget.setNegative(false);
                extractTarget.filter(*targetPlanePoints);

                // Perform NDT registration between the plane points
                Eigen::Matrix4f planeTransformationMatrix = Eigen::Matrix4f::Identity();
                bool planeRegistrationConverged = false;

                pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndtPlane;
                ndtPlane.setTransformationEpsilon(1e-8);
                ndtPlane.setStepSize(10);
                ndtPlane.setResolution(1.0);
                ndtPlane.setMaximumIterations(500);

                // Set input clouds
                ndtPlane.setInputSource(sourcePlanePoints);
                ndtPlane.setInputTarget(targetPlanePoints);

                // Perform alignment
                pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloudPlane(new pcl::PointCloud<pcl::PointXYZ>());
                ndtPlane.align(*outputCloudPlane);
                planeRegistrationConverged = ndtPlane.hasConverged();
                planeTransformationMatrix = ndtPlane.getFinalTransformation();

                if (!planeRegistrationConverged) {
                    std::cerr << "Plane-based NDT registration did not converge." << std::endl;
                    nonConvergenceCount++;
                    continue;
                }

                // Compute the total transformation
                transformationMatrix = planeTransformationMatrix * transformationMatrix;
            }

            // Modify the transformation matrix
            Eigen::Matrix4f modifiedTransformationMatrix = modifyTransformationMatrix(transformationMatrix);

            // Extract yaw and translation
            Eigen::Matrix3f rotationMatrix = modifiedTransformationMatrix.block<3,3>(0,0);
            float yaw = atan2(rotationMatrix(1,0), rotationMatrix(0,0));
            Eigen::Vector3f translationVector = modifiedTransformationMatrix.block<3,1>(0,3);

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            totalProcessingTime += processingTime.count();
            comparisonCount++;

            // Invert the transformation matrix to align target to source
            Eigen::Matrix4f transformationMatrixToAlignTargetToSource = modifiedTransformationMatrix.inverse();

            // Accumulate the transformation matrix
            cumulativeTransformation = cumulativeTransformation * transformationMatrixToAlignTargetToSource;

            std::cout << "Cumulative Transformation Matrix:\n" << cumulativeTransformation << std::endl;

            // Transform the target cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*targetCloud, *transformedTargetCloud, cumulativeTransformation);

            transformedClouds.push_back(transformedTargetCloud);

            std::string cloudName = "transformedCloud_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedColor(transformedTargetCloud, 255, 255, 255);
            viewer->addPointCloud<pcl::PointXYZ>(transformedTargetCloud, transformedColor, cloudName);

            viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (viewer->wasStopped()) {
                break;
            }
        }

        if (comparisonCount > 0) {
            double averageProcessingTime = totalProcessingTime / comparisonCount;
            std::cout << "Average processing time: " << averageProcessingTime << " seconds" << std::endl;

            std::cout << "Final path taken (X, Y):" << std::endl;
            for (const auto& point : translationPath) {
                std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
            }
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
