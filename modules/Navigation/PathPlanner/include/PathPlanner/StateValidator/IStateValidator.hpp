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
 * @ingroup group_state_validator
 * @brief Interface to ensure the a certain state is valid. Each
 * implementation can check this in their preferred way.
 *
 */
class IStateValidator {
 public:
    ~IStateValidator() = default;

    /**
     * @brief Check if the provided state is valid
     *
     * @param state State to check
     * @return true if valid
     * @return false otherwise
     */
    virtual bool isStateValid(const std::vector<double>& state) const = 0;
};

}  // namespace crf::navigation::pathplanner
