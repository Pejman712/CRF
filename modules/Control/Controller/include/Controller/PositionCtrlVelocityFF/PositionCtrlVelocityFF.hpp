/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include "Controller/IController.hpp"
#include "Controller/PIDController.hpp"

#include "InverseKinematics/IClosedLoopInverseKinematics.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::control::inversekinematics::IKinematicObjectiveFunction;

namespace crf::control::controller {

/**
 * @ingroup group_position_ctrl_velocity_ff
 * @brief Class designed to do a PID between the current position in time (t) and the expected
 * one at the same time (t). After, we add the desired velocity for the next interval (t+Ts)
 * into one calculated by the PID
 *
 */
class PositionCtrlVelocityFF: public IController {
 public:
    explicit PositionCtrlVelocityFF(
        const std::vector<double>& Kp,
        const std::vector<double>& Ki,
        const std::vector<double>& Kd,
        const double& Ts,
        std::shared_ptr<IClosedLoopInverseKinematics> inverseKinematics,
        const std::vector<std::shared_ptr<IKinematicObjectiveFunction>>& objectiveFunctions =
            std::vector<std::shared_ptr<IKinematicObjectiveFunction>>(0));
    ~PositionCtrlVelocityFF() = default;

    /**
     * @brief Function to do a PID between the current position in time (t) and the expected
     * one at the same time (t). After we add the desired velocity into the calculated one.
     *
     * @param desired a JointSignals with the desired joint values.
     * @param actualJoints Feedback from the robot received in JointSignals.
     * @param actualTask Feedback from the robot received in TaskSignals.
     * @return Signals output calculated from the controller.
     */
    Signals calculate(const JointSignals& desired,
        const JointSignals& actualJoints, const TaskSignals& actualTask) override;

    /**
     * @brief Function to do a PID between the current position in time (t) and the expected
     * one at the same time (t). After we add the desired velocity into the calculated one.
     *
     * @param desired a TaskSignals with the desired joint values.
     * @param actualJoints Feedback from the robot received in JointSignals.
     * @param actualTask Feedback from the robot received in TaskSignals.
     * @return Signals output calculated from the controller.
     */
    Signals calculate(const TaskSignals& desired,
        const JointSignals& actualJoints, const TaskSignals& actualTask) override;

    bool checkStopCondition() override;
    crf::expected<bool> setParameters(const nlohmann::json& parameters) override;
    nlohmann::json getParameters() override;
    void reset() override;

 private:
    std::vector<double> Kp_;
    std::vector<double> Ki_;
    std::vector<double> Kd_;
    double Ts_;
    std::shared_ptr<IClosedLoopInverseKinematics> inverseKinematics_;
    std::vector<std::shared_ptr<IKinematicObjectiveFunction>> objectiveFunctions_;

    PIDResult pidResult_;
    Signals result_;
    uint64_t dimensions_;
    JointPositions qAttraction_;
    crf::control::inversekinematics::ResultsIK resultsIK_;
    std::atomic<bool> stopCondition_;
    std::atomic<bool> stopOnLowManipulability_;
    std::atomic<bool> stopOnToleranceViolation_;
    std::atomic<bool> stopOnComputationTimeViolation_;
    std::atomic<bool> stopOnWorkspaceViolation_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::control::controller
