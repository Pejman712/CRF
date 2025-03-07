/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thanapong Chuangyanyond CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */
#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>
#include <sys/time.h>
#include <cmath>  // For std::ceil

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/PointTypes.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace edge {

/**
 * @brief      Compute the sharpness of each point in the input point cloud by using the
 *             eigenvalues of the covariance matrix of neighbouring points and filter the
 *             points based on the top "filterPercentage" of sharpness values.
 *
 * @param[in]  inputCloud       The input cloud pointer.
 * @param[in]  kNeighbour       The number of neighbour points that will be used to compute the
 *                              sharpness value.
 * @param[in]  filterPercentage The percentage of points to retain based on sharpness.
 *
 * @tparam     PointT           Whatever PC type form pcl.
 *
 * @return     A Point cloud pointer with spatial positions of the input and
 *             newly computed sharpness values (s field), containing only the top
 *             "filterPercentage" of points based on sharpness.
 */
template <typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
pcl::PointCloud<pointtypes::PointXYZS>::Ptr computeEigenSharpness(
    typename pcl::PointCloud<PointT>::Ptr inputCloud, unsigned int kNeighbour, float filterPercentage) {
    crf::utility::logger::EventLogger logger("computeEigenSharpness");

    if (inputCloud == nullptr || inputCloud->size() < 1) {
        logger->warn("Input Point Cloud not valid");
        return nullptr;
    }

    if (inputCloud->points.size() < kNeighbour || !kNeighbour) {
        logger->warn("Invalid number of neighbour points.");
        return nullptr;
    }

    if (filterPercentage <= 0.0f || filterPercentage > 100.0f) {
        logger->warn("Filter percentage must be between 0 and 100.");
        return nullptr;
    }

    // Create a placeholder for the output point cloud
    pcl::PointCloud<pointtypes::PointXYZS>::Ptr output(new pcl::PointCloud<pointtypes::PointXYZS>);

    // Create KdTree from xyz point cloud
    typename pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    // Create a point cloud with only position and copy input to output
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*inputCloud, *inputCloudXYZ);
    pcl::copyPointCloud(*inputCloudXYZ, *output);
    kdTree.setInputCloud(inputCloudXYZ);

    std::vector<int> neighbourKdTree(kNeighbour);
    std::vector<float> neighbourKdTreeEuclideanDist(kNeighbour);
    pcl::PointXYZ searchPoint;

    // Loop for all points in point cloud
    for (std::size_t i = 0; i < inputCloud->points.size(); i++) {
        searchPoint = inputCloudXYZ->points[i];
        kdTree.nearestKSearch(searchPoint, kNeighbour, neighbourKdTree,
            neighbourKdTreeEuclideanDist);

        float xSum = 0.0f; float ySum = 0.0f; float zSum = 0.0f;
        float xMean = 0.0f; float yMean = 0.0f; float zMean = 0.0f;
        float xxSum = 0.0f; float yySum = 0.0f; float zzSum = 0.0f;
        float xySum = 0.0f; float xzSum = 0.0f; float yzSum = 0.0f;

        // First compute the neighbour cloud's mean.
        for (std::size_t j = 0; j < neighbourKdTree.size(); j++) {
            pcl::PointXYZ neighbourPoint = inputCloudXYZ->points[neighbourKdTree[j]];
            xSum += neighbourPoint.x;
            ySum += neighbourPoint.y;
            zSum += neighbourPoint.z;
        }

        xMean = xSum / neighbourKdTree.size();
        yMean = ySum / neighbourKdTree.size();
        zMean = zSum / neighbourKdTree.size();

        // Compute the covariance matrix
        for (std::size_t j = 0; j < neighbourKdTree.size(); j++) {
            pcl::PointXYZ neighbourPoint = inputCloudXYZ->points[neighbourKdTree[j]];

            // Compute variance of X, Y, and Z
            xxSum += (neighbourPoint.x - xMean) * (neighbourPoint.x - xMean);
            yySum += (neighbourPoint.y - yMean) * (neighbourPoint.y - yMean);
            zzSum += (neighbourPoint.z - zMean) * (neighbourPoint.z - zMean);

            // Compute covariance of XY, XZ, and YZ
            xySum += (neighbourPoint.x - xMean) * (neighbourPoint.y - yMean);
            xzSum += (neighbourPoint.x - xMean) * (neighbourPoint.z - zMean);
            yzSum += (neighbourPoint.y - yMean) * (neighbourPoint.z - zMean);
        }

        float varX = xxSum / neighbourKdTree.size();
        float varY = yySum / neighbourKdTree.size();
        float varZ = zzSum / neighbourKdTree.size();
        float covXY = xySum / neighbourKdTree.size();
        float covXZ = xzSum / neighbourKdTree.size();
        float covYZ = yzSum / neighbourKdTree.size();

        // Construct covariance matrix.
        Eigen::Matrix3f covarianceMatrix;
        covarianceMatrix << varX, covXY, covXZ,
                            covXY, varY, covYZ,
                            covXZ, covYZ, varZ;

        // Compute the eigenvalues
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covarianceMatrix);
        if (eigensolver.info() != Eigen::Success) {
            output->clear();
            return nullptr;
        }

        // Sort the eigenvalues
        std::vector<float> eigenvals = {eigensolver.eigenvalues()[0], eigensolver.eigenvalues()[1],
                                        eigensolver.eigenvalues()[2]};
        std::sort(eigenvals.begin(), eigenvals.end());

        output->points[i].s = eigenvals[0] / (eigenvals[0] + eigenvals[1] + eigenvals[2]);
    }

    // Sort points based on sharpness in descending order
    std::sort(output->points.begin(), output->points.end(),
        [](const pointtypes::PointXYZS& a, const pointtypes::PointXYZS& b) {
            return a.s > b.s;
        });

    // Calculate the number of points to keep based on filterPercentage
    size_t numPointsToKeep = static_cast<size_t>(std::ceil(filterPercentage * output->points.size() / 100.0f));
    if (numPointsToKeep > output->points.size()) {
        numPointsToKeep = output->points.size();
    }

    // Create a new point cloud with the top sharpness values
    pcl::PointCloud<pointtypes::PointXYZS>::Ptr filteredOutput(new pcl::PointCloud<pointtypes::PointXYZS>);
    filteredOutput->points.assign(output->points.begin(), output->points.begin() + numPointsToKeep);

    // Set the appropriate metadata
    filteredOutput->width = static_cast<uint32_t>(filteredOutput->points.size());
    filteredOutput->height = 1;
    filteredOutput->is_dense = output->is_dense;

    return filteredOutput;
}

}  // namespace edge
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
