/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <vector>
#include <utility>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/State.h>

#include "PathPlanner/OMPLGeometricPlanner/OMPLMotionValidatorBridge.hpp"

namespace crf::navigation::pathplanner {

OMPLMotionValidatorBridge::OMPLMotionValidatorBridge(
    std::shared_ptr<IMotionValidator> motionValidator,
    std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace):
    ompl::base::MotionValidator(std::make_shared<ompl::base::SpaceInformation>(stateSpace)),
    motionValidator_(motionValidator),
    stateSpace_(stateSpace) {
}

bool OMPLMotionValidatorBridge::checkMotion(
    const ompl::base::State *s1,
    const ompl::base::State *s2,
    std::pair<ompl::base::State*, double> &lastValid) const {
    return checkMotion(s1, s2);
}

bool OMPLMotionValidatorBridge::checkMotion(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
    std::vector<double> realsS1;
    std::vector<double> realsS2;
    stateSpace_->copyToReals(realsS1, s1);
    stateSpace_->copyToReals(realsS2, s2);
    return motionValidator_->isMotionValid(realsS1, realsS2);
}

}  // namespace crf::navigation::pathplanner
