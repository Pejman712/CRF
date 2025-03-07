/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

// Standard Libraries
#include <vector>

// CRF Libraries
#include "InputShaper/CubicPolynomialShaper/CubicPolynomialShaper.hpp"
#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

// Using
using crf::math::inputshaper::CubicPolynomialShaper;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::logger::EventLogger;

namespace crf::control::motioncontroller {

/**
 * @brief Small class to group all the task input shapers and create a
 * simple change into joint signals.
 *
 * It uses Cubic polynomials as input shapers
 *
 */
class InputShaperTaskController {
 public:
    InputShaperTaskController(
        const TaskVelocity& startVelocity,
        const TaskAcceleration& maxAcceleration);
    ~InputShaperTaskController() = default;

    void setReference(const TaskVelocity& reference);
    void setAcceleration(const TaskAcceleration& acceleration);
    TaskVelocity getVelocity(const double& evaluationPoint);
    void reset();

 private:
    TaskAcceleration maxAcceleration_;
    std::vector<CubicPolynomialShaper> inputShapers_;

    uint32_t dimensions_;
    TaskVelocity lastVelocity_;

    EventLogger logger_;
};

}  // namespace crf::control::motioncontroller
