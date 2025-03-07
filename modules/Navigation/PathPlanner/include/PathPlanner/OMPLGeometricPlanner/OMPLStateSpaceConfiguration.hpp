/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

// State Spaces
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

using ompl::base::CompoundStateSpace;
using ompl::base::RealVectorStateSpace;
using ompl::base::RealVectorBounds;
using ompl::base::SO2StateSpace;
using ompl::base::SO3StateSpace;
using ompl::base::SE2StateSpace;
using ompl::base::SE3StateSpace;

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_ompl_state_space
 * @brief Class to generate an state space from a JSON file. The generated space
 * can is a compound state space with the subspaces created
 *
 */
class OMPLStateSpaceConfiguration : public CompoundStateSpace {
 public:
    explicit OMPLStateSpaceConfiguration(const nlohmann::json& config);
    ~OMPLStateSpaceConfiguration() override = default;
};

}  // namespace crf::navigation::pathplanner
