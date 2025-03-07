#pragma once

// Include necessary headers
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <algorithm>
#include <set>

// Include the point type definitions
#include "Laser/Point3D.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace gradient {

/**
 * @brief Computes the sharpness of each point in the input point cloud using the eigenvalues
 *        of the covariance matrix of neighboring points.
 *
 * @param cloud      The input point cloud (with Point3D type).
 * @param kNeighbour The number of neighboring points to consider.
 */
inline void computeSharpness(
    pcl::PointCloud<Point3D>::Ptr cloud,
    unsigned int kNeighbour)
{
    if (!cloud || cloud->empty())
        return;

    // Build k-d tree for neighbor search
    pcl::search::KdTree<Point3D>::Ptr kdtree(new pcl::search::KdTree<Point3D>);
    kdtree->setInputCloud(cloud);

    // Prepare variables
    std::vector<int> pointIdxNKNSearch(kNeighbour);
    std::vector<float> pointNKNSquaredDistance(kNeighbour);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const Point3D& query_point = cloud->points[i];

        if (kdtree->nearestKSearch(query_point, kNeighbour, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            // Compute covariance matrix and centroid
            Eigen::Matrix3f covariance_matrix;
            Eigen::Vector4f centroid;

            pcl::computeMeanAndCovarianceMatrix(*cloud, pointIdxNKNSearch, covariance_matrix, centroid);

            // Compute eigenvalues
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
            Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

            // Compute sharpness as the ratio of the smallest eigenvalue to the sum of eigenvalues
            float sum_eigenvalues = eigenvalues.sum();
            if (sum_eigenvalues > 0)
            {
                float sharpness = eigenvalues[0] / sum_eigenvalues;
                cloud->points[i].sharpness = sharpness;
            }
            else
            {
                cloud->points[i].sharpness = 0;
            }
        }
        else
        {
            cloud->points[i].sharpness = 0;
        }
    }
}

/**
 * @brief Selects the points within the top M% for both intensity and sharpness.
 *
 * @param cloud    The input point cloud (with Point3D type).
 * @param mPercent The percentage (M) to select from top values.
 * @return A new point cloud containing the selected points.
 */
inline pcl::PointCloud<Point3D>::Ptr selectTopMPercentPoints(
    pcl::PointCloud<Point3D>::Ptr cloud,
    float mPercent)
{
    if (!cloud || cloud->empty())
        return pcl::PointCloud<Point3D>::Ptr(new pcl::PointCloud<Point3D>);

    size_t numPoints = cloud->points.size();
    size_t numSelected = static_cast<size_t>(numPoints * mPercent / 100.0f);

    if (numSelected == 0)
        return pcl::PointCloud<Point3D>::Ptr(new pcl::PointCloud<Point3D>);

    // Create vectors to hold intensity and sharpness values with indices
    std::vector<std::pair<float, size_t>> intensityValues;
    std::vector<std::pair<float, size_t>> sharpnessValues;

    intensityValues.reserve(numPoints);
    sharpnessValues.reserve(numPoints);

    for (size_t i = 0; i < numPoints; ++i)
    {
        intensityValues.emplace_back(cloud->points[i].intensity, i);
        sharpnessValues.emplace_back(cloud->points[i].sharpness, i);
    }

    // Sort in descending order
    auto comparator = [](const std::pair<float, size_t>& a, const std::pair<float, size_t>& b)
    {
        return a.first > b.first;
    };

    std::sort(intensityValues.begin(), intensityValues.end(), comparator);
    std::sort(sharpnessValues.begin(), sharpnessValues.end(), comparator);

    // Select top M% indices
    std::set<size_t> topIntensityIndices;
    std::set<size_t> topSharpnessIndices;

    for (size_t i = 0; i < numSelected; ++i)
    {
        topIntensityIndices.insert(intensityValues[i].second);
        topSharpnessIndices.insert(sharpnessValues[i].second);
    }

    // Find intersection of indices
    std::vector<size_t> selectedIndices;
    std::set_intersection(
        topIntensityIndices.begin(), topIntensityIndices.end(),
        topSharpnessIndices.begin(), topSharpnessIndices.end(),
        std::back_inserter(selectedIndices)
    );

    // Create new point cloud with selected points
    pcl::PointCloud<Point3D>::Ptr selectedCloud(new pcl::PointCloud<Point3D>);
    for (size_t idx : selectedIndices)
    {
        selectedCloud->points.push_back(cloud->points[idx]);
    }

    return selectedCloud;
}

/**
 * @brief Computes the transformation between the source and target point clouds using ICP.
 *
 * @param source The source point cloud (with Point3D type).
 * @param target The target point cloud (with Point3D type).
 * @return The transformation matrix aligning the source to the target.
 */
inline Eigen::Matrix4f findTransformation(
    pcl::PointCloud<Point3D>::Ptr source,
    pcl::PointCloud<Point3D>::Ptr target)
{
    pcl::IterativeClosestPoint<Point3D, Point3D> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);

    // Set ICP parameters if necessary
    // icp.setMaximumIterations(50);
    // icp.setMaxCorrespondenceDistance(0.05);

    pcl::PointCloud<Point3D> Final;
    icp.align(Final);

    if (icp.hasConverged())
    {
        return icp.getFinalTransformation();
    }
    else
    {
        return Eigen::Matrix4f::Identity();
    }
}

}  // namespace gradient
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
