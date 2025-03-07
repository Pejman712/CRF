#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include <thread>       // For std::this_thread
#include <chrono>       // For std::chrono::milliseconds

namespace fs = std::filesystem;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Type Definitions
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// Function to load a point cloud from file
bool loadPointCloud(const std::string& filename, PointCloudT::Ptr& cloud) {
    fs::path filepath(filename);
    std::string extension = filepath.extension().string();

    if (extension == ".pcd") {
        if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) {
            PCL_ERROR("Couldn't read PCD file %s \n", filename.c_str());
            return false;
        }
    }
    else if (extension == ".ply") {
        if (pcl::io::loadPLYFile<PointT>(filename, *cloud) == -1) {
            PCL_ERROR("Couldn't read PLY file %s \n", filename.c_str());
            return false;
        }
    }
    else {
        std::cerr << "Unsupported file format: " << extension << std::endl;
        return false;
    }
    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;
    return true;
}

// Function to estimate normals
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(PointCloudT::Ptr& cloud, int k = 10) {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setKSearch(k);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*normals);
    return normals;
}

// Function to detect edge points based on normal differences and confidence
pcl::PointCloud<PointT>::Ptr detectEdges(
    PointCloudT::Ptr& cloud,
    pcl::PointCloud<pcl::Normal>::Ptr& normals,
    float angle_threshold_deg = 30.0f,
    int k =100,
    float confidence_threshold = 0.50f) // 90% confidence
{
    pcl::PointCloud<PointT>::Ptr edge_cloud(new pcl::PointCloud<PointT>());
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    const float DEG2RAD = M_PI / 180.0f;
    float angle_threshold_rad = angle_threshold_deg * DEG2RAD;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> indices;
        std::vector<float> distances;
        if (tree->nearestKSearch(i, k, indices, distances) > 0) {
            int differing_normals = 0;
            int total_valid = 0;

            for (size_t j = 1; j < indices.size(); ++j) { // Start from 1 to skip the point itself
                // Ensure neighbor has a valid normal
                if (!std::isfinite(normals->points[indices[j]].normal_x) ||
                    !std::isfinite(normals->points[indices[j]].normal_y) ||
                    !std::isfinite(normals->points[indices[j]].normal_z)) {
                    continue;
                }

                float dot_product = normals->points[i].normal_x * normals->points[indices[j]].normal_x +
                                    normals->points[i].normal_y * normals->points[indices[j]].normal_y +
                                    normals->points[i].normal_z * normals->points[indices[j]].normal_z;
                // Clamp dot_product to avoid numerical issues
                dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
                float angle = std::acos(dot_product);
                if (angle > angle_threshold_rad) {
                    differing_normals++;
                }
                total_valid++;
            }

            // Calculate confidence
            if (total_valid == 0) continue; // Avoid division by zero
            float confidence = static_cast<float>(differing_normals) / static_cast<float>(total_valid);

            if (confidence >= confidence_threshold) {
                edge_cloud->points.push_back(cloud->points[i]);
            }
        }
    }
    edge_cloud->width = edge_cloud->points.size();
    edge_cloud->height = 1;
    edge_cloud->is_dense = true;
    std::cout << "Detected " << edge_cloud->points.size() << " edge points with confidence >= " << (confidence_threshold * 100) << "%" << std::endl;
    return edge_cloud;
}

// Function to downsample the point cloud
PointCloudT::Ptr downsampleCloud(PointCloudT::Ptr& cloud, float leaf_size = 0.01f) {
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after downsampling: " << cloud_filtered->points.size() << " points." << std::endl;
    return cloud_filtered;
}

int main(int argc, char** argv) {
    // Check for directory argument
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_directory>" << std::endl;
        return -1;
    }

    std::string directory_path = argv[1];

    // Check if directory exists
    if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
        std::cerr << "Invalid directory: " << directory_path << std::endl;
        return -1;
    }

    // Parameters
    const float angle_threshold_deg = 30.0f;      // Angle threshold in degrees
    const int k_neighbors = 10;                   // Number of neighbors
    const float confidence_threshold = 0.9f;      // 90% confidence

    // Iterate through the directory
    for (const auto& entry : fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
            std::string file_path = entry.path().string();
            std::string extension = entry.path().extension().string();
            // Check for PCD or PLY files
            if (extension != ".pcd" && extension != ".ply") {
                continue;
            }

            // Load point cloud
            PointCloudT::Ptr cloud(new PointCloudT());
            if (!loadPointCloud(file_path, cloud)) {
                continue;
            }

            // Optional: Downsample the cloud for faster processing
            PointCloudT::Ptr cloud_filtered = downsampleCloud(cloud, 0.005f); // Adjust leaf size as needed

            // Estimate normals
            pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(cloud_filtered, k_neighbors);

            // Detect edges with confidence
            pcl::PointCloud<PointT>::Ptr edge_cloud = detectEdges(
                cloud_filtered,
                normals,
                angle_threshold_deg,
                k_neighbors,
                confidence_threshold
            );

            // Visualization
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Edge Detection"));
            viewer->setBackgroundColor(0, 0, 0);

            // Original point cloud in white
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud_filtered, 255, 255, 255);
            viewer->addPointCloud<PointT>(cloud_filtered, cloud_color_handler, "original_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

            // Edge points in red
            pcl::visualization::PointCloudColorHandlerCustom<PointT> edge_color_handler(edge_cloud, 255, 0, 0);
            viewer->addPointCloud<PointT>(edge_cloud, edge_color_handler, "edge_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge_cloud");

            // Add coordinate system
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();

            std::cout << "Visualizing edges for " << entry.path().filename().string() << std::endl;

            // Display until the user closes the window
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Valid after including <thread> and <chrono>
            }

            // Clear viewer for next point cloud
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
        }
    }

    return 0;
}
