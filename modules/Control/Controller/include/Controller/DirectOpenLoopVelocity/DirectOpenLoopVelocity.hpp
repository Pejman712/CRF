/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *          Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include "Controller/IController.hpp"
#include "Types/Signals.hpp"
#include "InverseKinematics/IOpenLoopInverseKinematics.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::control::inversekinematics::IOpenLoopInverseKinematics;
using crf::control::inversekinematics::IKinematicObjectiveFunction;

namespace crf::control::controller {

/**
 * @ingroup group_direct_open_loop_velocity
 * @brief
 *
 */
class DirectOpenLoopVelocity: public IController {
 public:
    explicit DirectOpenLoopVelocity(
        const uint64_t& dimensions,
        std::shared_ptr<IOpenLoopInverseKinematics> inverseKinematics,
        const std::vector<std::shared_ptr<IKinematicObjectiveFunction>>& objectiveFunctions =
            std::vector<std::shared_ptr<IKinematicObjectiveFunction>>(0));
    ~DirectOpenLoopVelocity() = default;

    Signals calculate(const JointSignals& desired, const JointSignals& actualJoints,
        const TaskSignals& actualTask) override;
    Signals calculate(const TaskSignals& desired, const JointSignals& actualJoints,
        const TaskSignals& actualTask) override;
    bool checkStopCondition() override;
    crf::expected<bool> setParameters(const nlohmann::json& parameters) override;
    nlohmann::json getParameters() override;
    void reset() override;

 private:
    uint64_t dimensions_;
    std::shared_ptr<IOpenLoopInverseKinematics> inverseKinematics_;
    std::vector<std::shared_ptr<IKinematicObjectiveFunction>> objectiveFunctions_;

    Signals result_;
    JointPositions qAttraction_;
    TaskPose emptyDesired0thDer_;
    crf::control::inversekinematics::ResultsIK resultsIK_;
    std::atomic<bool> stopCondition_;
    std::atomic<bool> stopOnLowManipulability_;
    std::atomic<bool> stopOnToleranceViolation_;
    std::atomic<bool> stopOnComputationTimeViolation_;
    std::atomic<bool> stopOnWorkspaceViolation_;

    crf::utility::logger::EventLogger logger_;

    JointSignals inverseKinematics(const TaskSignals& desired);
};

}  // namespace crf::control::controller
