/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include <gmock/gmock.h>

#include "PathPlanner/IPathPlanner.hpp"

namespace crf::navigation::pathplanner {

class PathPlannerMock : public IPathPlanner {
 public:
    MOCK_METHOD(crf::expected<std::vector<std::vector<double>>>, computePath,
        (const std::vector<double>& start, const std::vector<double>& goal), (override));
};

}  // namespace crf::navigation::pathplanner
