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
#include <pcl/registration/ndt.h>  // Added for NDT
#include <pcl/filters/statistical_outlier_removal.h>  // Added for SOR
#include <complex>  // Added for complex numbers
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
        std::cerr << "Methods: 'only ndt', 'only gicp', 'both combined'" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
    std::string method = argv[2];
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // Convert to lowercase

    if (method != "only ndt" && method != "only gicp" && method != "both combined") {
        std::cerr << "Invalid method. Choose 'only ndt', 'only gicp', or 'both combined'." << std::endl;
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
            pass.setFilterLimits(0, 5.0);

            pass.setInputCloud(sourceCloud);
            pass.filter(*cloud_filtered_source);

            pass.setInputCloud(targetCloud);
            pass.filter(*cloud_filtered_target);

            auto startTime = std::chrono::high_resolution_clock::now();

            bool gicpConverged = false;
            bool ndtConverged = false;
            Eigen::Matrix4f transformationMatrixGICP = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f transformationMatrixNDT = Eigen::Matrix4f::Identity();

            if (method == "only gicp" || method == "both combined") {
                // Apply GICP
                gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
                    cloud_filtered_source, cloud_filtered_target, transformationMatrixGICP);
            }

            if (method == "only ndt" || method == "both combined") {
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
                ndtConverged = ndt.hasConverged();
                transformationMatrixNDT = ndt.getFinalTransformation();
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            if ((method == "only gicp" && !gicpConverged) ||
                (method == "only ndt" && !ndtConverged) ||
                ((method == "both combined") && !gicpConverged && !ndtConverged)) {
                std::cerr << "Registration did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }

            // Apply modifications to the transformation matrices
            Eigen::Matrix4f modifiedTransformationMatrixGICP = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f modifiedTransformationMatrixNDT = Eigen::Matrix4f::Identity();

            if (gicpConverged) {
                modifiedTransformationMatrixGICP = modifyTransformationMatrix(transformationMatrixGICP);
            }

            if (ndtConverged) {
                modifiedTransformationMatrixNDT = modifyTransformationMatrix(transformationMatrixNDT);
            }

            // Extract yaw angles and translation vectors
            float yawGICP = 0.0f;
            Eigen::Vector3f translationVectorGICP = Eigen::Vector3f::Zero();

            float yawNDT = 0.0f;
            Eigen::Vector3f translationVectorNDT = Eigen::Vector3f::Zero();

            if (gicpConverged) {
                // Extract yaw and translation
                Eigen::Matrix3f rotationMatrixGICP = modifiedTransformationMatrixGICP.block<3,3>(0,0);
                yawGICP = atan2(rotationMatrixGICP(1,0), rotationMatrixGICP(0,0));
                translationVectorGICP = modifiedTransformationMatrixGICP.block<3,1>(0,3);
            }

            if (ndtConverged) {
                // Extract yaw and translation
                Eigen::Matrix3f rotationMatrixNDT = modifiedTransformationMatrixNDT.block<3,3>(0,0);
                yawNDT = atan2(rotationMatrixNDT(1,0), rotationMatrixNDT(0,0));
                translationVectorNDT = modifiedTransformationMatrixNDT.block<3,1>(0,3);
            }

            // Average yaw angles and translation vectors
            float yawAverage = 0.0f;
            Eigen::Vector3f translationVectorAverage = Eigen::Vector3f::Zero();

            if (method == "both combined") {
                if (gicpConverged && ndtConverged) {
                    // Both converged, average the rotations and translations
                    // Average rotations properly using complex numbers
                    std::complex<float> rotationGICP(cos(yawGICP), sin(yawGICP));
                    std::complex<float> rotationNDT(cos(yawNDT), sin(yawNDT));

                    std::complex<float> rotationAverage = rotationGICP + rotationNDT;
                    rotationAverage /= std::abs(rotationAverage);
                    yawAverage = atan2(rotationAverage.imag(), rotationAverage.real());

                    // Average translations
                    //translationVectorAverage = (translationVectorGICP + translationVectorNDT) / 2.0f;
                    translationVectorAverage = translationVectorGICP;

                } else if (gicpConverged) {
                    // Only GICP converged
                    std::cout << "Only GICP converged, using its result." << std::endl;
                    yawAverage = yawGICP;
                    translationVectorAverage = translationVectorGICP;
                } else if (ndtConverged) {
                    // Only NDT converged
                    std::cout << "Only NDT converged, using its result." << std::endl;
                    yawAverage = yawNDT;
                    translationVectorAverage = translationVectorNDT;
                }
            } else if (method == "only gicp" && gicpConverged) {
                yawAverage = yawGICP;
                translationVectorAverage = translationVectorGICP;
            } else if (method == "only ndt" && ndtConverged) {
                yawAverage = yawNDT;
                translationVectorAverage = translationVectorNDT;
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
