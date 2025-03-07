#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace crf {
namespace applications {
namespace graphoptimization {

class IGraphOptimization{
 public:
    /* 
     * Constructor
     * GraphOptimization(const std::string& mapperConfigFile);
     */
    virtual ~IGraphOptimization() = default;
    /* 
     * Returns:
     *  - True if the configFile is good
     *  - False if there is any kind of error in the configFile
     */
    virtual bool parse(const std::string& configFileName) = 0;
    /* 
     * Returns:
     *  - True if the vertex has been successfully pushed in the graph
     *  - False if parse hasn't been applied or other kind of error
     *  Function usage:
     *  - For organized and unorganized point clouds
     *  - The graphics results are optional
     */
    virtual bool addVertex(
        const Eigen::Matrix4f &realMotion,
        const Eigen::Matrix4f &previousPosition,
        boost::optional <const Eigen::Vector3f> boostFixPose,
        boost::optional <const Eigen::Vector3f> odometryPosition,
        boost::optional <const Eigen::Vector3f> groundTruthPosition) = 0;
    /* 
     * Returns:
     *  - Real position of the new point cloud respecting to the origin
     *  - False upon failure
     *  Function usage:
     *  - This function is though to compare the current point cloud with the previous one
     *  - If you need to make comparations between different point clouds, you must use the
     *    function 'comparePointClouds' of the mapper3d
     */
    virtual boost::optional<Eigen::Matrix4f> comparePointClouds(
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &sourcePointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &targetPointCloud) = 0;
    /* 
     * Returns:
     *  - True if a vector of point clouds has been successfully pulled from the graph
     *  - False upon failure
     */
    virtual boost::optional <std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>>
        getNodePointclouds() = 0;
    /* 
     * Returns:
     *  - True if a vector of vertices' positions has been successfully pulled from the graph
     *  - False upon failure
     */
    virtual boost::optional <std::vector <Eigen::Matrix4f, Eigen::aligned_allocator
        <Eigen::Matrix4f>>> getNodePositions() = 0;
    /* 
     * Returns:
     *  - True if the graph has been sucessfully reseted
     *  - False if there has been a problem
     */
    virtual bool resetGraph() = 0;
};

}  // namespace graphoptimization
}  // namespace applications
}  // namespace crf
