/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <algorithm>
#include <chrono>
#include <exception>
#include <vector>
#include <iostream>

#include "Controller/PIDController.hpp"

namespace crf::control::controller {

PIDResult PID(const std::vector<double>& ref, const std::vector<double>& feedback,
    const std::vector<double>& previousError, const std::vector<double>& Kp,
    const std::vector<double>& Ki, const std::vector<double>& Kd, const double& Ts) {
    uint32_t size = ref.size();
    if (size != feedback.size() || size != previousError.size() ||
        size != Kp.size() || size != Ki.size() || size != Kd.size() || Ts < 0) {
        throw std::invalid_argument("Wrong dimensions of PID parameters");
    }
    std::vector<double> integral(size);
    std::vector<double> derivative(size);
    PIDResult res(size);
    for (size_t i = 0; i < size; i++) {
        res.error[i] = ref[i] - feedback[i];
        integral[i] += res.error[i] * Ts;
        derivative[i] = (res.error[i] - previousError[i]) / Ts;
        res.output[i] = Kp[i] * res.error[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];
    }
    return res;
}

}  // namespace crf::control::controller
