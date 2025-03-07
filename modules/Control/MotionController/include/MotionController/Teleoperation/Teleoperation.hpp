/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *          Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

// Standard Libraries
#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <set>

// CRF Libraries
#include "MotionController/AbstractMotionController.hpp"
#include "Robot/IRobot.hpp"
#include "Controller/IController.hpp"
#include "MotionController/Teleoperation/InputShaperJointsController.hpp"
#include "MotionController/Teleoperation/InputShaperTaskController.hpp"

// Utility
#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

// Using
using crf::actuators::robot::IRobot;
using crf::actuators::robot::RobotConfiguration;
using crf::control::controller::IController;

namespace crf::control::motioncontroller {

/**
 * @ingroup group_teleoperation
 * @brief Implementaion of IMotionController. It's main point is teleoperation for
 * the control of the robots. It includes input shaping for a smooth response from
 * the user input.
 * @details This class is thread safe
 */
class Teleoperation: public AbstractMotionController {
 public:
    Teleoperation(
        std::shared_ptr<IRobot> robot,
        std::shared_ptr<IController> controller,
        const std::chrono::milliseconds& velocityTimeInterval = std::chrono::milliseconds(100));
    ~Teleoperation() override;

    crf::expected<bool> appendPath(const std::vector<JointPositions>& path) override;
    crf::expected<bool> appendPath(
        const std::vector<TaskPose>& path,
        const TrajectoryExecutionMethod& method,
        const PointReferenceFrame& frame) override;

    crf::expected<bool> setVelocity(const JointVelocities& velocity) override;
    crf::expected<bool> setVelocity(
        const TaskVelocity& velocity,
        const PointReferenceFrame& frame) override;

    crf::expected<bool> setTorque(const JointForceTorques& torque) override;
    crf::expected<bool> setTorque(
        const TaskForceTorque& torque,
        const PointReferenceFrame& frame) override;

    crf::expected<bool> setProfileVelocity(const JointVelocities& velocity) override;
    crf::expected<bool> setProfileVelocity(const TaskVelocity& velocity) override;

    crf::expected<bool> setProfileAcceleration(const JointAccelerations& acceleration) override;
    crf::expected<bool> setProfileAcceleration(const TaskAcceleration& acceleration) override;

    crf::expected<bool> setParameters(const nlohmann::json& params) override;
    nlohmann::json getCurrentParameters() override;
    nlohmann::json getParametersDefinition() const override;

    crf::expected<bool> isTrajectoryRunning() override;

    void softStop() override;
    void hardStop() override;

 private:
    std::shared_ptr<IRobot> robot_;
    std::shared_ptr<IController> controller_;

    std::shared_ptr<InputShaperJointsController> shaperJoints_;
    std::shared_ptr<InputShaperTaskController> shaperTask_;

    std::shared_ptr<RobotConfiguration> robotConfig_;

    crf::utility::logger::EventLogger logger_;

    Signals jointControl() override;
    Signals taskControl() override;
    void commandTimeout() override;
};

}  // namespace crf::control::motioncontroller
