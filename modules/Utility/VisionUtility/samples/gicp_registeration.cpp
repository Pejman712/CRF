#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

// Include the edge detection header
#include "VisionUtility/PointCloud/Edge.hpp"  // Assuming this is where computeEigenSharpness is defined
#include "VisionUtility/PointCloud/Gicp.hpp"

namespace fs = std::filesystem;
using namespace crf::utility::visionutility::pointcloud;

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

    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.path().extension() == ".pcd") {
            pcdFiles.push_back(entry.path().string());
        }
    }

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
    Eigen::Matrix4f previousTransformationMatrix = Eigen::Matrix4f::Identity();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> transformedClouds;

    try {
        for (size_t i = 1; i < pcdFiles.size(); ++i) {
            std::string sourceFile = pcdFiles[i - 1];
            std::string targetFile = pcdFiles[i];

            pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZI>());
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(sourceFile, *sourceCloud) == -1) {
                std::cerr << "Couldn't read source file " << sourceFile << std::endl;
                return -1;
            }
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(targetFile, *targetCloud) == -1) {
                std::cerr << "Couldn't read target file " << targetFile << std::endl;
                return -1;
            }

            // Apply Statistical Outlier Removal (SOR) filter
            pcl::PointCloud<pcl::PointXYZI>::Ptr sor_filtered_source(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr sor_filtered_target(new pcl::PointCloud<pcl::PointXYZI>());

            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            sor.setMeanK(50);  // Number of neighbors to analyze
            sor.setStddevMulThresh(20);  // Threshold for identifying outliers

            sor.setInputCloud(sourceCloud);
            sor.filter(*sor_filtered_source);

            sor.setInputCloud(targetCloud);
            sor.filter(*sor_filtered_target);

            // Replace the original clouds with the SOR filtered clouds
            sourceCloud = sor_filtered_source;
            targetCloud = sor_filtered_target;

            // Apply PassThrough filter on z-axis
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_source(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_target(new pcl::PointCloud<pcl::PointXYZI>());

            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 3.0);

            pass.setInputCloud(sourceCloud);
            pass.filter(*cloud_filtered_source);

            pass.setInputCloud(targetCloud);
            pass.filter(*cloud_filtered_target);

            // === Intensity filtering ===
            auto intensityFilter = [](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                if (cloud->points.size() > 0) {
                    // Extract intensity values
                    std::vector<float> intensities;
                    intensities.reserve(cloud->points.size());
                    for (const auto& point : cloud->points) {
                        intensities.push_back(point.intensity);
                    }

                    // Sort intensity values
                    std::sort(intensities.begin(), intensities.end());

                    // Compute 10th and 90th percentiles
                    size_t idx_10th = static_cast<size_t>(intensities.size() * 0.05);
                    size_t idx_90th = static_cast<size_t>(intensities.size() * 0.95);
                    float intensity_10th = intensities[idx_10th];
                    float intensity_90th = intensities[idx_90th];

                    // Apply intensity filter
                    pcl::PassThrough<pcl::PointXYZI> intensityPass;
                    intensityPass.setFilterFieldName("intensity");
                    intensityPass.setFilterLimits(intensity_10th, intensity_90th);
                    intensityPass.setInputCloud(cloud);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                    intensityPass.filter(*intensity_filtered);

                    return intensity_filtered;
                } else {
                    std::cerr << "Cloud is empty after z-filtering." << std::endl;
                    return pcl::PointCloud<pcl::PointXYZI>::Ptr();
                }
            };

            // Filter source and target clouds based on intensity
            cloud_filtered_source = intensityFilter(cloud_filtered_source);
            cloud_filtered_target = intensityFilter(cloud_filtered_target);

            if (!cloud_filtered_source || !cloud_filtered_target) {
                std::cerr << "Error in intensity filtering." << std::endl;
                continue;
            }

            // === Edge Detection and Sharpness Filtering ===
            // Function to compute sharpness and filter points
            auto edgeFilter = [](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterPercentage) {
                // Convert PointXYZI to PointXYZ
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::copyPointCloud(*cloud, *cloudXYZ);

                // Compute sharpness
                unsigned int kNeighbour = 10;  // You can adjust this value
                pcl::PointCloud<pointtypes::PointXYZS>::Ptr sharpnessCloud = crf::utility::visionutility::pointcloud::edge::computeEigenSharpness<pcl::PointXYZ>(cloudXYZ, kNeighbour, filterPercentage);

                if (!sharpnessCloud) {
                    std::cerr << "Error computing sharpness." << std::endl;
                    return pcl::PointCloud<pcl::PointXYZI>::Ptr();
                }

                // Convert back to PointXYZI and retain only the sharp points
                pcl::PointCloud<pcl::PointXYZI>::Ptr sharpIntensityCloud(new pcl::PointCloud<pcl::PointXYZI>());

                // Create a map from XYZ to intensity for the original cloud
                std::map<std::tuple<float, float, float>, float> intensityMap;
                for (const auto& point : cloud->points) {
                    intensityMap[std::make_tuple(point.x, point.y, point.z)] = point.intensity;
                }

                for (const auto& point : sharpnessCloud->points) {
                    pcl::PointXYZI pointXYZI;
                    pointXYZI.x = point.x;
                    pointXYZI.y = point.y;
                    pointXYZI.z = point.z;
                    pointXYZI.intensity = intensityMap[std::make_tuple(point.x, point.y, point.z)];
                    sharpIntensityCloud->points.push_back(pointXYZI);
                }

                sharpIntensityCloud->width = sharpIntensityCloud->points.size();
                sharpIntensityCloud->height = 1;
                sharpIntensityCloud->is_dense = true;

                return sharpIntensityCloud;
            };

            // Apply edge detection and sharpness filtering
            float edgeFilterPercentage = 99.0f;  // Keep top 10% sharpness points
            cloud_filtered_source = edgeFilter(cloud_filtered_source, edgeFilterPercentage);
            cloud_filtered_target = edgeFilter(cloud_filtered_target, edgeFilterPercentage);

            if (!cloud_filtered_source || !cloud_filtered_target) {
                std::cerr << "Error in edge filtering." << std::endl;
                return -1;
            }

            Eigen::Matrix4f transformationMatrix;

            auto startTime = std::chrono::high_resolution_clock::now();

            // Use the filtered clouds for GICP
            bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::template gicp<pcl::PointXYZI>(
                cloud_filtered_source, cloud_filtered_target, transformationMatrix);

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            if (!gicpConverged) {
                std::cerr << "GICP did not converge for " << sourceFile << " and " << targetFile << ". Using previous transformation matrix." << std::endl;
                nonConvergenceCount++;
                transformationMatrix = previousTransformationMatrix;
            }

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
            previousTransformationMatrix = transformationMatrix;

            totalProcessingTime += processingTime.count();
            comparisonCount++;

            Eigen::Matrix4f transformationMatrixToAlignTargetToSource = transformationMatrix.inverse();

            cumulativeTransformation = cumulativeTransformation * transformationMatrixToAlignTargetToSource;

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud_filtered_target, *transformedTargetCloud, cumulativeTransformation);

            transformedClouds.push_back(transformedTargetCloud);

            std::string cloudName = "transformedCloud_" + std::to_string(i);
            
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensityColor(transformedTargetCloud, "intensity");
            if (intensityColor.isCapable()) {
                viewer->addPointCloud<pcl::PointXYZI>(transformedTargetCloud, intensityColor, cloudName);
            } else {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> defaultColor(transformedTargetCloud, 255, 255, 255);
                viewer->addPointCloud<pcl::PointXYZI>(transformedTargetCloud, defaultColor, cloudName);
            }

            viewer->spinOnce();
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
