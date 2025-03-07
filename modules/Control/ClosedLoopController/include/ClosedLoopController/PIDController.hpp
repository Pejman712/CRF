/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <boost/optional.hpp>
#include <vector>

#include "ClosedLoopController/IClosedLoopController.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::closedloopcontroller {

class PIDController : public IClosedLoopController {
 public:
    PIDController();
    PIDController(const std::vector<double>& Kp, const std::vector<double>& Ki,
        const std::vector<double>& Kd);
    ~PIDController() override = default;
    boost::optional<std::vector<double>> calculate(
        const std::vector<double>& setpoint, const std::vector<double>& feedbackValue) override;
    void reset() override;

 private:
    utility::logger::EventLogger logger_;
    std::vector<double> proportionalGain_;
    std::vector<double> integrativeGain_;
    std::vector<double> derivativeGain_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastSetPointTime_;
    size_t size_;
    std::vector<double> previousError_;
    std::vector<double> integral_;
};

}  // namespace crf::control::closedloopcontroller
