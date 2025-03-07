#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <functional>
#include <random>

namespace fs = boost::filesystem;

using Eigen::MatrixXf;
using Eigen::VectorXf;

// Function to perform k-means clustering using Eigen
std::vector<int> kMeansClustering(const MatrixXf& data, int num_clusters, int max_iterations = 100, float tolerance = 1e-4) {
    int num_points = data.rows();
    int dimensions = data.cols();

    // Randomly initialize cluster centroids
    MatrixXf centroids = MatrixXf::Random(num_clusters, dimensions);

    std::vector<int> labels(num_points, 0);
    MatrixXf new_centroids = MatrixXf::Zero(num_clusters, dimensions);
    std::vector<int> cluster_sizes(num_clusters, 0);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Assign points to the nearest centroid
        for (int i = 0; i < num_points; ++i) {
            float min_distance = std::numeric_limits<float>::max();
            for (int k = 0; k < num_clusters; ++k) {
                float distance = (data.row(i) - centroids.row(k)).squaredNorm();
                if (distance < min_distance) {
                    min_distance = distance;
                    labels[i] = k;
                }
            }
        }

        // Reset new centroids and cluster sizes
        new_centroids.setZero();
        std::fill(cluster_sizes.begin(), cluster_sizes.end(), 0);

        // Compute new centroids
        for (int i = 0; i < num_points; ++i) {
            new_centroids.row(labels[i]) += data.row(i);
            cluster_sizes[labels[i]]++;
        }

        for (int k = 0; k < num_clusters; ++k) {
            if (cluster_sizes[k] > 0) {
                new_centroids.row(k) /= cluster_sizes[k];
            }
        }

        // Check for convergence
        float centroid_shift = (centroids - new_centroids).norm();
        centroids = new_centroids;
        if (centroid_shift < tolerance) {
            break;
        }
    }

    return labels;
}

int main() {
    // Load the point cloud file
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string input_file = "/home/robotics/data/BA6I5/cloud_1734016904.pcd"; // Replace with your actual file path
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *filtered_point_cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }
    std::cout << "Loaded " << filtered_point_cloud->points.size() << " points from " << input_file << std::endl;

    // Convert point cloud to Eigen matrix
    int num_points = filtered_point_cloud->points.size();
    MatrixXf data(num_points, 3);
    for (size_t i = 0; i < filtered_point_cloud->points.size(); ++i) {
        data(i, 0) = filtered_point_cloud->points[i].x;
        data(i, 1) = filtered_point_cloud->points[i].y;
        data(i, 2) = filtered_point_cloud->points[i].z;
    }

    // Perform k-means clustering
    int num_clusters = 5; // Example cluster count
    std::vector<int> labels = kMeansClustering(data, num_clusters);

    // Separate points into clusters based on labels
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(num_clusters);
    for (int i = 0; i < num_clusters; ++i) {
        clusters[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    for (size_t i = 0; i < filtered_point_cloud->points.size(); ++i) {
        int cluster_idx = labels[i];
        clusters[cluster_idx]->points.push_back(filtered_point_cloud->points[i]);
    }

    // Dynahull logic: Create convex hulls for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> hulls(num_clusters);
    for (int i = 0; i < num_clusters; ++i) {
        hulls[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(clusters[i]);
        chull.reconstruct(*hulls[i]);

        // Ensure hull has correct metadata
        hulls[i]->width = hulls[i]->points.size();
        hulls[i]->height = 1;
        hulls[i]->is_dense = true;

        std::string hull_filename = "hull_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(hull_filename, *hulls[i]);
        std::cout << "Saved convex hull " << hull_filename << " with " << hulls[i]->points.size() << " points." << std::endl;
    }

    // Save or process clusters
    for (int i = 0; i < num_clusters; ++i) {
        clusters[i]->width = clusters[i]->points.size();
        clusters[i]->height = 1;
        clusters[i]->is_dense = true;

        std::string filename = "cluster_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *clusters[i]);
        std::cout << "Saved " << filename << " with " << clusters[i]->points.size() << " points." << std::endl;
    }

    return 0;
}
