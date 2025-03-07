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

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

#include "PathPlanner/StateValidator/IStateValidator.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLStateSpaceConfiguration.hpp"

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_ompl_state_validator_bridge
 * @brief OMPL uses other types to represent states in space. So we need a bridge to
 * pass from those states to doubles and check if they are valid
 *
 */
class OMPLStateValidatorBridge : public ompl::base::StateValidityChecker {
 public:
    OMPLStateValidatorBridge(
        std::shared_ptr<IStateValidator> stateValidator,
        std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace);
    ~OMPLStateValidatorBridge() = default;

    /**
     * @brief Check if a certain state is valid
     *
     * @param state State to check
     * @return true if valid
     * @return false otherwise
     */
    bool isValid(const ompl::base::State *state) const override;

 private:
    std::shared_ptr<IStateValidator> stateValidator_;
    std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace_;
};

}  // namespace crf::navigation::pathplanner
