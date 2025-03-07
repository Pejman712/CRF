#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "Mapper3d/NormalsColorOctree.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <string>

namespace crf {
namespace applications {
namespace mapper3d {
class IMapper3d {
 public:
    virtual ~IMapper3d() = default;
    virtual boost::optional<Eigen::Matrix4f> updateMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        pointCloud, const Eigen::Matrix4f &sensorOrigin) = 0;
    virtual boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> getPointCloudMap() = 0;
    virtual boost::optional<NormalsColorOctree> getOctreeMap() = 0;
    virtual bool clearMap() = 0;
    virtual int getPointCloudMapSize() = 0;
    virtual int getOctreeMapSize() = 0;
    virtual bool savePointCloudToDisk(const std::string& savingPathWithoutExtension,
        bool plyflag) = 0;
    virtual bool saveOctreeToDisk(const std::string& savingPathWithoutExtension) = 0;
    virtual boost::optional<Eigen::Matrix4f> comparePointClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin) = 0;
    virtual boost::optional<Eigen::Matrix4f> comparePointClouds(
        bool organizedPointCloud,
        bool srcNormalCalculated,
        bool tgtNormalCalculated,
        const Eigen::Matrix4f &sourceSensorOrigin,
        const Eigen::Matrix4f &targetSensorOrigin,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointCloud,
        pcl::PointCloud<pcl::Normal>::Ptr sourceNormals,
        pcl::PointCloud<pcl::Normal>::Ptr targetNormals) = 0;
};
}  // namespace mapper3d
}  // namespace applications
}  // namespace crf
