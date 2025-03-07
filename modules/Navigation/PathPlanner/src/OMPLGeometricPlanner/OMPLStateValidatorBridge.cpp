/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>

#include "PathPlanner/OMPLGeometricPlanner/OMPLStateValidatorBridge.hpp"

namespace crf::navigation::pathplanner {

OMPLStateValidatorBridge::OMPLStateValidatorBridge(
    std::shared_ptr<IStateValidator> stateValidator,
    std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace):
    ompl::base::StateValidityChecker(std::make_shared<ompl::base::SpaceInformation>(stateSpace)),
    stateValidator_(stateValidator),
    stateSpace_(stateSpace) {
}

bool OMPLStateValidatorBridge::isValid(const ompl::base::State *state) const {
    std::vector<double> reals;
    stateSpace_->copyToReals(reals, state);
    return stateValidator_->isStateValid(reals);
}

}  // namespace crf::navigation::pathplanner
