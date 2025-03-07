/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

// Standard Libraries
#include <memory>
#include <vector>

// CRF Libraries
#include "MotionController/Teleoperation/InputShaperJointsController.hpp"

namespace crf::control::motioncontroller {

InputShaperJointsController::InputShaperJointsController(
    const JointVelocities& startVelocity,
    const JointAccelerations& maxAcceleration):
    dimensions_(startVelocity.size()),
    logger_("InputShaperJointsController") {
    logger_->debug("CTor");
    if (dimensions_ != maxAcceleration.size()) {
        throw std::logic_error(
            "InputShaperJointsController - CTor - Arguments dimensions don't match");
    }
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_.push_back(CubicPolynomialShaper(0, maxAcceleration[i]));
    }
    lastVelocity_ = startVelocity;
}

void InputShaperJointsController::setReference(const JointVelocities& reference) {
    if (dimensions_ != reference.size()) {
        throw std::invalid_argument(
            "InputShaperJointsController - setReference - Arguments dimensions don't match");
    }
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].setReference(reference[i]);
    }
}

void InputShaperJointsController::setAcceleration(const JointAccelerations& acceleration) {
    if (dimensions_ != acceleration.size()) {
        throw std::invalid_argument(
            "InputShaperJointsController - setAcceleration - Arguments dimensions don't match");
    }
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].setResponsivenessFactor(maxAcceleration_[i]/acceleration[i]);
    }
}

JointVelocities InputShaperJointsController::getVelocity(const double& evaluationPoint) {
    for (uint32_t i = 0; i < dimensions_; i++) {
        lastVelocity_[i] = inputShapers_[i].getInputPoint(evaluationPoint);
    }
    return lastVelocity_;
}

void InputShaperJointsController::reset() {
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].reset();
    }
}

}  // namespace crf::control::motioncontroller
