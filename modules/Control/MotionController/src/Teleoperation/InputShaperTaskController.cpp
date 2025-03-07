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
#include "MotionController/Teleoperation/InputShaperTaskController.hpp"

namespace crf::control::motioncontroller {

InputShaperTaskController::InputShaperTaskController(
    const TaskVelocity& startVelocity,
    const TaskAcceleration& maxAcceleration):
    dimensions_(6),
    logger_("InputShaperTaskController") {
    logger_->debug("CTor");
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_.push_back(CubicPolynomialShaper(0, maxAcceleration[i]));
    }
    lastVelocity_ = startVelocity;
}

void InputShaperTaskController::setReference(const TaskVelocity& reference) {
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].setReference(reference[i]);
    }
}

void InputShaperTaskController::setAcceleration(const TaskAcceleration& acceleration) {
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].setResponsivenessFactor(maxAcceleration_[i]/acceleration[i]);
    }
}

TaskVelocity InputShaperTaskController::getVelocity(const double& evaluationPoint) {
    TaskVelocity velocity;
    for (uint32_t i = 0; i < dimensions_; i++) {
        velocity[i] = inputShapers_[i].getInputPoint(evaluationPoint);
    }
    return velocity;
}

void InputShaperTaskController::reset() {
    for (uint32_t i = 0; i < dimensions_; i++) {
        inputShapers_[i].reset();
    }
}

}  // namespace crf::control::motioncontroller
