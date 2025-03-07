/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include "crf/expected.hpp"

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_path_planner
 * @brief Pure virtual class to represent all path planners
 *
 */
class IPathPlanner {
 public:
    virtual ~IPathPlanner() = default;

    /**
     * @brief Calculate a path that goes from start state to a goal state. Every time the method
     * is called a new path is calculated.
     *
     * @param start Starting position in N dimensions
     * @param goal Goal position in N dimensions
     * @return crf::expected<std::vector<double>> Final path calculated or error code indicating
     * why it failed
     */
    virtual crf::expected<std::vector<std::vector<double>>> computePath(
        const std::vector<double>& start, const std::vector<double>& goal) = 0;
};

}  // namespace crf::navigation::pathplanner
