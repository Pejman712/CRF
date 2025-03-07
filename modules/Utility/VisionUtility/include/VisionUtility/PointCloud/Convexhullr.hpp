#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>

// Include the edge detection and registration headers
//#include "VisionUtility/PointCloud/Edge.hpp"  // Assuming this is where computeEigenSharpness is defined
#include "VisionUtility/PointCloud/Gicp.hpp"

namespace fs = std::filesystem;
using namespace crf::utility::visionutility::pointcloud;

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace convexhullr {

pcl::PointCloud<pcl::PointXYZI>::Ptr rotateTargetCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud) {

    // Apply Statistical Outlier Removal (SOR) filter on the source cloud
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setMeanK(50);
    sor.setStddevMulThresh(20);
    sor.setInputCloud(sourceCloud);
    sor.filter(*sourceCloud);

    // Compute convex hull of the source cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr hull(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(sourceCloud);
    chull.setDimension(3);
    chull.setComputeAreaVolume(true);
    chull.reconstruct(*hull);

    if (hull->empty()) {
        std::cerr << "Convex hull is empty. Skipping transformation." << std::endl;
        return targetCloud; // Return unchanged target cloud
    }

    // Compute Centroid of Convex Hull
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull, centroid);

    // Compute Principal Component for Normal
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*hull, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0); // Smallest eigenvalue direction

    // Define Rotation: 10 Degrees Counterclockwise
    float angle = 0.0 * M_PI / 180.0; // Convert to radians
    Eigen::AngleAxisf rotation(angle, normal.normalized()); // Rotate around computed normal
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

    // Apply Transformation to the Target Cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedTargetCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*targetCloud, *transformedTargetCloud, transform);

    return transformedTargetCloud;
}

} // namespace convexhullr
} // namespace pointcloud
} // namespace visionutility
} // namespace utility
} // namespace crf
