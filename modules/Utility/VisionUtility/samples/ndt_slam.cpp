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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>  // Include NDT header

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

            // Load the point clouds
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::io::loadPCDFile<pcl::PointXYZ>(sourceFile, *sourceCloud);
            pcl::io::loadPCDFile<pcl::PointXYZ>(targetFile, *targetCloud);

            // Perform Statistical Outlier Removal (Optional)
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSource(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredTarget(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);

            sor.setInputCloud(sourceCloud);
            sor.filter(*filteredSource);

            sor.setInputCloud(targetCloud);
            sor.filter(*filteredTarget);

            // Initialize Normal Distributions Transform (NDT)
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

            // Setting NDT parameters
            ndt.setTransformationEpsilon(1e-8);  // Maximum allowed difference between two consecutive transformations
            ndt.setStepSize(0.1);                // Maximum step size for More-Thuente line search
            ndt.setResolution(1.0);              // Resolution of NDT grid structure (VoxelGridCovariance)
            ndt.setMaximumIterations(500);        // Maximum number of registration iterations

            // Setting point clouds to ndt
            ndt.setInputSource(filteredSource);
            ndt.setInputTarget(filteredTarget);

            // Initial alignment estimate (identity)
            Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

            // Perform alignment
            pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
            auto startTime = std::chrono::high_resolution_clock::now();
            ndt.align(*alignedCloud, init_guess);
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processingTime = endTime - startTime;

            // Get the transformation matrix
            Eigen::Matrix4f transformationMatrix = ndt.getFinalTransformation();

            // Check if the alignment has converged
            bool ndtConverged = ndt.hasConverged();

            if (!ndtConverged) {
                std::cerr << "NDT did not converge for " << sourceFile << " and " << targetFile << std::endl;
                nonConvergenceCount++;
                continue;
            }
            std::cout << "Transformation Matrix:\n" << transformationMatrix << std::endl;

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
            cumulativeTransformation =  cumulativeTransformation * transformationMatrix.inverse();

            // Transform the target cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*targetCloud, *transformedCloud, cumulativeTransformation);

            transformedClouds.push_back(transformedCloud);

            std::string cloudName = "transformedCloud_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedColor(transformedCloud, 255, 255, 255);
            viewer->addPointCloud<pcl::PointXYZ>(transformedCloud, transformedColor, cloudName);
            
            viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
