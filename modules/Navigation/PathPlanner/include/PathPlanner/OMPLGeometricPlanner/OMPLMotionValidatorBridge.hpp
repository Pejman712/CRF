/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <utility>
#include <memory>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

#include "PathPlanner/MotionValidator/IMotionValidator.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLStateSpaceConfiguration.hpp"

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_ompl_motion_validator_bridge
 * @brief OMPL uses other types to represent states in space. So we need a bridge to
 * pass from those states to doubles and check the motion between them.
 *
 */
class OMPLMotionValidatorBridge : public ompl::base::MotionValidator {
 public:
    OMPLMotionValidatorBridge(
        std::shared_ptr<IMotionValidator> motionValidator,
        std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace);
    ~OMPLMotionValidatorBridge() = default;

    /**
     * @brief Check the motion between two states taking into account the
     * last valid state
     *
     * @param s1 First state
     * @param s2 Second state
     * @param lastValid Last valid point in the motion
     * @return true if the motion is valid
     * @return false potherwise
     */
    bool checkMotion(
        const ompl::base::State *s1,
        const ompl::base::State *s2,
        std::pair<ompl::base::State*, double> &lastValid) const override;

    /**
     * @brief Check the motion between two states
     *
     * @param s1 First state
     * @param s2 Second state
     * @return true if the motion is valid
     * @return false potherwise
     */
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

 private:
    std::shared_ptr<IMotionValidator> motionValidator_;
    std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace_;
};

}  // namespace crf::navigation::pathplanner
