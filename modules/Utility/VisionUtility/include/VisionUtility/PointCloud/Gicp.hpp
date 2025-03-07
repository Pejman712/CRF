/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <small_gicp/pcl/pcl_registration_impl.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <pcl/filters/statistical_outlier_removal.h>  // Added for SOR filtering
#include <iostream>
#include <Eigen/Core>
#include <pcl/filters/impl/filter_indices.hpp>
//#include <Laser/Point3D.hpp>



namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace gicp {

/**
 * @brief      Performs GICP (Generalized Iterative Closest Point) point cloud registration between source and target point clouds.
 *
 * @param[in]  sourceCloud    The source point cloud
 * @param[in]  targetCloud    The target point cloud
 * @param[out] transformationMatrix The transformation matrix if registration converges
 *
 * @tparam     PointT        Point type (should be compatible with pcl::PointXYZ)
 *
 * @return     True if GICP converges, false otherwise
 */
template <typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
bool gicp(typename pcl::PointCloud<PointT>::Ptr& sourceCloud,
          typename pcl::PointCloud<PointT>::Ptr& targetCloud,
          Eigen::Matrix4f& transformationMatrix) {
    std::cout << "Starting GICP point cloud registration" << std::endl;

    // Ensure the clouds do not contain NaNs
    //std::vector<int> indices;
    //typename pcl::PointCloud<PointT>::Ptr sourceCloudFiltered(new pcl::PointCloud<PointT>);
    //typename pcl::PointCloud<PointT>::Ptr targetCloudFiltered(new pcl::PointCloud<PointT>);

    //pcl::removeNaNFromPointCloud(*sourceCloud, *sourceCloudFiltered, indices);
    //pcl::removeNaNFromPointCloud(*targetCloud, *targetCloudFiltered, indices);

    // Create the registration object

    small_gicp::RegistrationPCL<PointT, PointT> reg;

    // Set the registration parameters
    reg.setNumThreads(4);
    reg.setCorrespondenceRandomness(20.0);
    reg.setMaxCorrespondenceDistance(20.0);
    reg.setVoxelResolution(0.0001);
    reg.setRegistrationType("GICP");

    // Set input source and target clouds
    reg.setInputSource(sourceCloud);  // Use filtered source cloud
    reg.setInputTarget(targetCloud);  // Use filtered target cloud

    // Create an output point cloud for storing aligned results
    typename pcl::PointCloud<PointT>::Ptr alignedCloud(new pcl::PointCloud<PointT>);

    // Perform the point cloud registration
    reg.align(*alignedCloud);

    // Check if registration has converged
    if (!reg.hasConverged()) {
        std::cerr << "GICP registration did not converge" << std::endl;
        return false;  // Return false if GICP did not converge
    }

    // Get the transformation matrix
    transformationMatrix = reg.getFinalTransformation();

    std::cout << "GICP registration completed successfully" << std::endl;
    //std::cout << "Transformation Matrix:\n" << transformationMatrix << std::endl;

    // Return true if GICP has converged
    return true;
}

}  // namespace gicp
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
