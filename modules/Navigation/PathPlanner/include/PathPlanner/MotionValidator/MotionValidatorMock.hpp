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

#include "PathPlanner/MotionValidator/IMotionValidator.hpp"

namespace crf::navigation::pathplanner {

class MotionValidatorMock : public IMotionValidator {
 public:
    MOCK_METHOD(bool, isMotionValid,
        (const std::vector<double>& s1, const std::vector<double>& s2), (const override));
};

}  // namespace crf::navigation::pathplanner
