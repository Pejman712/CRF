#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <memory>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>

#include "CommonInterfaces/IInitializable.hpp"
#include "Laser/LaserConfiguration.hpp"
#include "Point3D.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace crf {
namespace sensors {
namespace laser {

enum class LaserConnectionType {
    Serial = 0,
    Ethernet = 1
};


class ILaser : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~ILaser() = default;
    virtual pcl::PointCloud<Point3D>::Ptr getPointCloud() = 0;
    virtual pcl::PointCloud<Point3D>::Ptr getTargetPointCloud() = 0;
    virtual std::shared_ptr<crf::sensors::laser::LaserConfiguration> getConfiguration() = 0;
};
}  // namespace laser
}  // namespace sensors
}  // namespace crf