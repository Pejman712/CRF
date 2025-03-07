#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include "VisionUtility/PointCloud/Gicp.hpp"  // Ensure this points to your corrected gicp.hpp

namespace fs = std::filesystem;

long extractNumber(const std::string& filename) {
    std::string::size_type start = filename.find_last_of('_') + 1;
    std::string::size_type end = filename.find_last_of('.');
    std::string numberStr = filename.substr(start, end - start);
    return std::stol(numberStr);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];
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
    double totalXTranslation = 0.0;
    double totalYTranslation = 0.0;
    int comparisonCount = 0;
    int nonConvergenceCount = 0;
    std::vector<std::pair<double, double>> translationPath;
    translationPath.emplace_back(0.0, 0.0);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    Eigen::Matrix4f cumulativeTransformation = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> cumulativeTransformations;
    cumulativeTransformations.push_back(cumulativeTransformation);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;
    transformedClouds.reserve(pcdFiles.size());

    std::vector<Eigen::Vector3f> positions;
    positions.push_back(cumulativeTransformation.block<3,1>(0,3));

    // Load the initial point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr initialCloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFiles[0], *initialCloud) == -1) {
        std::cerr << "Couldn't read initial file " << pcdFiles[0] << std::endl;
        return -1;
    }

    transformedClouds.push_back(initialCloud);

    // Loop closure parameters
    float loopClosureThreshold = 2f; // Distance threshold for loop closure detection (in meters)
    bool loopClosurePerformed = false;

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

            Eigen::Matrix4f transformationMatrix;
            auto startTime = std::chrono::high_resolution_clock::now();

            // Use 'template' keyword to disambiguate the function template
            bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
                cloud_filtered_source, cloud_filtered_target, transformationMatrix);

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            if (!gicpConverged) {
                std::cerr << "GICP did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }

            // --- Only rotation around Z-axis and translation in X and Y ---
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

            totalProcessingTime += processingTime.count();
            comparisonCount++;

            // Invert the transformation matrix to align target to source
            Eigen::Matrix4f transformationMatrixToAlignTargetToSource = transformationMatrix.inverse();

            // Accumulate the transformation matrix
            cumulativeTransformation = cumulativeTransformation * transformationMatrixToAlignTargetToSource;
            cumulativeTransformations.push_back(cumulativeTransformation);

            // Extract and store position
            Eigen::Vector3f position = cumulativeTransformation.block<3,1>(0,3);
            positions.push_back(position);

            std::cout << "Cumulative Transformation Matrix:\n" << cumulativeTransformation << std::endl;

            // Transform the target cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*targetCloud, *transformedTargetCloud, cumulativeTransformation);

            transformedClouds.push_back(transformedTargetCloud);

            // Loop closure detection (only if not already performed)
            if (!loopClosurePerformed && transformedClouds.size() > 500) { // Wait until at least 10 frames
                float distanceToStart = (position - positions[0]).norm();

                if (distanceToStart < loopClosureThreshold) {
                    std::cout << "Loop closure detected at frame " << transformedClouds.size() - 1 << std::endl;

                    // Perform alignment between current cloud and initial cloud
                    Eigen::Matrix4f loopClosureTransformation;
                    bool loopClosureConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZ>(
                        transformedTargetCloud, transformedClouds[0], loopClosureTransformation);

                    if (loopClosureConverged) {
                        // Compute correction transformation
                        Eigen::Matrix4f correction = loopClosureTransformation;

                        // Decompose correction into rotation and translation
                        Eigen::Matrix3f correctionRotation = correction.block<3,3>(0,0);
                        Eigen::Vector3f correctionTranslation = correction.block<3,1>(0,3);

                        // Convert rotation matrices to quaternions
                        Eigen::Quaternionf q_identity(Eigen::Matrix3f::Identity());
                        Eigen::Quaternionf q_correction(correctionRotation);

                        size_t numTransforms = cumulativeTransformations.size();

                        // Apply correction to each previous transformation
                        for (size_t k = 1; k < numTransforms; ++k) {
                            float alpha = static_cast<float>(k) / static_cast<float>(numTransforms - 1);

                            // Interpolate rotations using SLERP
                            Eigen::Quaternionf q_interpolated = q_identity.slerp(alpha, q_correction);

                            // Interpolate translations linearly
                            Eigen::Vector3f t_interpolated = alpha * correctionTranslation;

                            // Construct incremental correction transformation
                            Eigen::Matrix4f incrementalCorrection = Eigen::Matrix4f::Identity();
                            incrementalCorrection.block<3,3>(0,0) = q_interpolated.toRotationMatrix();
                            incrementalCorrection.block<3,1>(0,3) = t_interpolated;

                            // Apply incremental correction
                            cumulativeTransformations[k] = incrementalCorrection * cumulativeTransformations[k];

                            // Update positions
                            positions[k] = cumulativeTransformations[k].block<3,1>(0,3);

                            // Update transformed clouds
                            pcl::transformPointCloud(*transformedClouds[k], *transformedClouds[k], incrementalCorrection);
                        }

                        loopClosurePerformed = true;

                        std::cout << "Loop closure correction applied." << std::endl;
                    } else {
                        std::cout << "Loop closure alignment did not converge." << std::endl;
                    }
                }
            }

            // Visualization
            viewer->removeAllPointClouds();
            for (size_t idx = 0; idx < transformedClouds.size(); ++idx) {
                std::string cloudName = "transformedCloud_" + std::to_string(idx);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedColor(transformedClouds[idx], 255, 255, 255);
                viewer->addPointCloud<pcl::PointXYZ>(transformedClouds[idx], transformedColor, cloudName);
            }

            viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (viewer->wasStopped()) {
                break;
            }
        }

        if (comparisonCount > 0) {
            double averageProcessingTime = totalProcessingTime / comparisonCount;
            std::cout << "Average processing time: " << averageProcessingTime << " seconds" << std::endl;
            std::cout << "Total accumulated X translation: " << totalXTranslation << std::endl;
            std::cout << "Total accumulated Y translation: " << totalYTranslation << std::endl;

            std::cout << "Final path taken (X, Y):" << std::endl;
            for (const auto& pos : positions) {
                std::cout << "(" << pos[0] << ", " << pos[1] << ")" << std::endl;
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
