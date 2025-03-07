/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <algorithm>
#include <chrono>
#include <exception>
#include <vector>

#include "ClosedLoopController/PIDController.hpp"

namespace crf::control::closedloopcontroller {

PIDController::PIDController():
    logger_("PIDController"),
    proportionalGain_(0),
    integrativeGain_(0),
    derivativeGain_(0),
    lastSetPointTime_(),
    size_(),
    previousError_(),
    integral_() {
        logger_->debug("CTor");
}

PIDController::PIDController(const std::vector<double>& Kp, const std::vector<double>& Ki,
    const std::vector<double>& Kd):
    logger_("PIDController"),
    proportionalGain_(Kp),
    integrativeGain_(Ki),
    derivativeGain_(Kd),
    lastSetPointTime_(),
    size_(Kp.size()),
    previousError_(size_),
    integral_(size_) {
    logger_->debug("CTor");
    if (!(Kp.size() == Ki.size() && Ki.size() == Kd.size()) || Kp.size() == 0) {
        logger_->error("Wrong dimensions of gain parameters: [{}, {}, {}]",
            Kp.size(), Ki.size(), Kd.size());
        throw std::invalid_argument("Wrong dimensions of gain parameters");
    }
}

boost::optional<std::vector<double>> PIDController::calculate(
    const std::vector<double>& setpoint, const std::vector<double>& feedbackValue) {
    if ((setpoint.size() != size_) || (feedbackValue.size() != size_)) {
        logger_->error("Wrong dimensions of arguments: [{}, {}]",
            setpoint.size(), feedbackValue.size());
        return boost::none;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        lastSetPointTime_.time_since_epoch()).count() == 0) {
        lastSetPointTime_ = std::chrono::high_resolution_clock::now();
        // return zero output if it is first time controller is called
        return std::vector<double>(size_);
    }
    // TODO(pptaszni): what if the controller will be called long time after it was created?
    auto currentSetPointTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = currentSetPointTime - lastSetPointTime_;
    std::vector<double> error(size_);
    std::vector<double> derivative(size_);
    std::vector<double> output(size_);
    for (size_t i = 0; i < size_; i++) {
        error[i] = setpoint[i] - feedbackValue[i];
        integral_[i] += (error[i] * elapsed.count());
        derivative[i] = (error[i] - previousError_[i]) / elapsed.count();
        output[i] = proportionalGain_[i] * error[i]
            + integrativeGain_[i] * integral_[i]
            + derivativeGain_[i] * derivative[i];
    }
    previousError_ = error;
    lastSetPointTime_ = currentSetPointTime;
    return output;
}

void PIDController::reset() {
    logger_->debug("reset()");
    // assigning epoch to lastSetPointTime_
    lastSetPointTime_ = decltype(lastSetPointTime_)();
    std::fill(previousError_.begin(), previousError_.end(), 0.0);
    std::fill(integral_.begin(), integral_.end(), 0.0);
}

}  // namespace crf::control::closedloopcontroller
