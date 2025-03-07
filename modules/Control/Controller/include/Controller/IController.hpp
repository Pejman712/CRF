/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include "crf/expected.hpp"
#include "Types/Signals.hpp"

using crf::utility::types::JointSignals;
using crf::utility::types::TaskSignals;
using crf::utility::types::Signals;

namespace crf::control::controller {

/**
 * @ingroup group_controller
 * @brief IController interface class. Base for all the controllers.
 *
 */
class IController {
 public:
    virtual ~IController() = default;

    /**
     * @brief Class to calculate the output from the chosen controller. This method
     * takes a desired joint signal and the feedback values from the robot and calcualtes
     * a desired output for the joints
     *
     * @param desired Reference JointSignals for the controller.
     * @param actualJoints Feedback from the robot received in JointSignals.
     * @param actualTask Feedback from the robot received in TaskSignals.
     * @return Signals output calculated from the controller.
     */
    virtual Signals calculate(const JointSignals& desired, const JointSignals& actualJoints,
        const TaskSignals& actualTask) = 0;

    /**
     * @brief Class to calculate the output from the chosen controller. This method
     * takes a desired task signal and the feedback values from the robot and calcualtes
     * a desired output for the joints
     *
     * @param desired Reference TaskSignals for the controller.
     * @param actualJoints Feedback from the robot received in JointSignals.
     * @param actualTask Feedback from the robot received in TaskSignals.
     * @return Signals output calculated from the controller.
     */
    virtual Signals calculate(const TaskSignals& desired, const JointSignals& actualJoints,
        const TaskSignals& actualTask) = 0;
    /**
     * @brief It says if the controller is requestiong a stop of the device being controlled do to
     *        an internal failure.
     *
     * @return true if it has to stop.
     * @return false if everything is ok.
     */
    virtual bool checkStopCondition() = 0;
    /**
     * @brief Depending on the implemenation, each controller will have a different
     * set of parameters to be tuned or modified. This method allows a JSON object to
     * be sent to the controller and interpreted as to change this values.
     *
     * @param parameters JSON with all the objects to be changed
     * @return true If the change was succesfull
     * @return false otherwise
     */
    virtual crf::expected<bool> setParameters(const nlohmann::json& parameters) = 0;
    /**
     * @brief Get the Current Parameters. It returns the current values of the parameters
     * set by the user in a JSON format. If the user has not modified them it will return
     * the default values.
     *
     * @return nlohmann::json with the fields and values
     */
    virtual nlohmann::json getParameters() = 0;
    /**
     * @brief Resets the controller: e.g. reset the IK, the PIDs, etc...
     *
     */
    virtual void reset() = 0;
};

}  // namespace crf::control::controller
