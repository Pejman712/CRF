/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "WallDetector/IWallDetector.hpp"

namespace crf {
namespace applications {
namespace walldetector {

class WallDetectorMock : public IWallDetector {
 public:
  MOCK_METHOD2(detectWall,
      std::vector<WallParameter>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudData,
        const utility::types::TaskPose& cameraPose));
};

}  // namespace walldetector
}  // namespace applications
}  // namespace crf
