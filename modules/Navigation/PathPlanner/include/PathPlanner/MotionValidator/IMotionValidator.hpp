/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_motion_validator
 * @brief Interface to ensure the motion between two states is valid. Each
 * implementation can check this in their preferred way.
 *
 */
class IMotionValidator {
 public:
    ~IMotionValidator() = default;

    /**
     * @brief Check if the motion between two states is valid
     *
     * @param s1 First state
     * @param s2 Second state
     * @return true if the motion is valid
     * @return false otherwise
     */
    virtual bool isMotionValid(
        const std::vector<double>& s1, const std::vector<double>& s2) const = 0;
};

}  // namespace crf::navigation::pathplanner
