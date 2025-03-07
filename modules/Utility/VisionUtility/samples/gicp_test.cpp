#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>
#include <filesystem> 
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>
#include <pcl/common/centroid.h>
#include <VisionUtility/PointCloud/Gicp.hpp>  // Ensure this points to your gicp.hpp

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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformedClouds;

    try {
        for (size_t i = 1; i < pcdFiles.size(); ++i) {
            std::string sourceFile = pcdFiles[i - 1];
            std::string targetFile = pcdFiles[i];

            Eigen::Matrix4f transformationMatrix;
            auto startTime = std::chrono::high_resolution_clock::now();

            // Call the GICP function (with internal SOR)
            bool gicpConverged = crf::utility::visionutility::pointcloud::gicp::gicp<pcl::PointXYZ>(
                sourceFile, targetFile, transformationMatrix);

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            if (!gicpConverged) {
                std::cerr << "GICP did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }

            totalProcessingTime += processingTime.count();
            comparisonCount++;

            float x_translation = transformationMatrix(0, 3);
            float y_translation = transformationMatrix(1, 3);
            totalXTranslation += x_translation;
            totalYTranslation += y_translation;

            translationPath.emplace_back(totalXTranslation, totalYTranslation);

            std::cout << "Comparing " << sourceFile << " with " << targetFile << std::endl;
            std::cout << "Translation in x: " << x_translation << std::endl;
            std::cout << "Translation in y: " << y_translation << std::endl;
            std::cout << "Processing time: " << processingTime.count() << " seconds" << std::endl << std::endl;

            // Accumulate the transformation matrix
            cumulativeTransformation = transformationMatrix * cumulativeTransformation;

            // Load and transform the source and target clouds
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::io::loadPCDFile<pcl::PointXYZ>(sourceFile, *sourceCloud);
            pcl::io::loadPCDFile<pcl::PointXYZ>(targetFile, *targetCloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*sourceCloud, *transformedSourceCloud, transformationMatrix);

            // Set colors for the point clouds
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sourceColor(sourceCloud, 0, 255, 0);  // Green for source
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> targetColor(targetCloud, 255, 0, 0);  // Red for target
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedColor(transformedSourceCloud, 255, 255, 255);  // White for transformed

            // Remove previous point clouds
            viewer->removePointCloud("sourceCloud");
            viewer->removePointCloud("targetCloud");
            viewer->removePointCloud("transformedCloud");

            // Add the current point clouds
            viewer->addPointCloud<pcl::PointXYZ>(sourceCloud, sourceColor, "sourceCloud");
            viewer->addPointCloud<pcl::PointXYZ>(targetCloud, targetColor, "targetCloud");
            viewer->addPointCloud<pcl::PointXYZ>(transformedSourceCloud, transformedColor, "transformedCloud");

            // Set point cloud size for better visualization
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sourceCloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targetCloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transformedCloud");

            viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
