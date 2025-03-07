/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <vector>

namespace crf::control::closedloopcontroller {

class IClosedLoopController {
 public:
    virtual ~IClosedLoopController() = default;
    /*
     * Calculates the control output according to the desired state (setpoint)
     * and a feedback from the system.
     *
     * Returns:
     *  - boost::none in case of failure (e.g. wrong size of arguments)
     *  - VectorXf representing the calculated control that can be used on the system
     *
     * Typical application:
     *
     *     std::shared_ptr<IClosedLoopController> controller;
     *     // initialize controller
     *     while (!stopCondition) {
     *         auto goal = desiredGoal();
     *         auto systemState = system->getState();
     *         system->applyControl(controller->calculate(goal, systemState));
     *     }
     *
     */
    virtual boost::optional<std::vector<double>> calculate(
        const std::vector<double>& setpoint, const std::vector<double>& feedbackValue) = 0;

    /*
     * @brief Restarts the 0 state of the controller (clears the output, the feedbacks and the history)
     */
    virtual void reset() = 0;
};

}  // namespace crf::control::closedloopcontroller
