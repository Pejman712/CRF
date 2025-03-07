/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

namespace crf::control::controller {

/**
 * @ingroup group_controller
 * @brief Struct to contain the result of the PID. It contains the output from the PID and the
 *        error so that it can be stored and later placed as input.
 */
struct PIDResult {
    explicit PIDResult(const uint32_t& size):
        output(size),
        error(size, 0) {
    }
    std::vector<double> output;
    std::vector<double> error;
};

/**
 * @ingroup group_controller
 * @brief Function to calculate a PID in several dimensions.
 *
 * @param ref Reference point.
 * @param feedback Feed-back from the closed loop.
 * @param previousError Error from the previous iteration if there is (otherwise 0).
 * @param Kp Constant for the proportional part.
 * @param Ki Constant for the integral part.
 * @param Kd Constant for the derivative part.
 * @param Ts Time period specified.
 * @return PIDResult Struct with the output and error.
 */
PIDResult PID(
    const std::vector<double>& ref,
    const std::vector<double>& feedback,
    const std::vector<double>& previousError,
    const std::vector<double>& Kp,
    const std::vector<double>& Ki,
    const std::vector<double>& Kd,
    const double& Ts);

}  // namespace crf::control::controller
