#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility>
#include <vector>

#include "Types/TaskTypes/TaskPose.hpp"

namespace crf {
namespace applications {
namespace walldetector {

struct WallParameter {
    float length;  // length [m]
    float distance;  // distance [m]
    float theta;  // angle [rad]
};

class IWallDetector {
 public:
    virtual ~IWallDetector() = default;
    virtual std::vector<WallParameter> detectWall(
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
      const utility::types::TaskPose& cameraPose) = 0;
};

}  // namespace walldetector
}  // namespace applications
}  // namespace crf
