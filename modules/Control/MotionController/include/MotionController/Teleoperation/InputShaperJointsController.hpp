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
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::logger::EventLogger;

namespace crf::control::motioncontroller {

/**
 * @brief Small class to group all the joint input shapers and create a
 * simple change into joint signals.
 *
 * It uses Cubic polynomials as input shapers
 *
 */
class InputShaperJointsController {
 public:
    InputShaperJointsController(
        const JointVelocities& startVelocity,
        const JointAccelerations& maxAcceleration);
    ~InputShaperJointsController() = default;

    void setReference(const JointVelocities& reference);
    void setAcceleration(const JointAccelerations& acceleration);
    JointVelocities getVelocity(const double& evaluationPoint);
    void reset();

 private:
    JointAccelerations maxAcceleration_;
    std::vector<CubicPolynomialShaper> inputShapers_;

    uint32_t dimensions_;
    JointVelocities lastVelocity_;

    EventLogger logger_;
};

}  // namespace crf::control::motioncontroller
