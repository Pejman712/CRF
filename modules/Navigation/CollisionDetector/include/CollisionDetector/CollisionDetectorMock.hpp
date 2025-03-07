/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <vector>

#include "CollisionDetector/ICollisionDetector.hpp"

namespace crf::navigation::collisiondetector {

class CollisionDetectorMock : public ICollisionDetector {
 public:
    MOCK_METHOD1(checkState,
        bool(const std::vector<float> &state));
    MOCK_METHOD2(checkMotion,
        bool(const std::vector<float> &initialState, const std::vector<float> &finalState));
    MOCK_METHOD1(updateMap,
        bool(const octomap::OcTree &tree));
    MOCK_METHOD1(clearance,
        boost::optional<float>(const std::vector<float> &state));
    // MOCK_METHOD1(updateMap,
    //     bool(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud));
};

}  // namespace crf::navigation::collisiondetector
