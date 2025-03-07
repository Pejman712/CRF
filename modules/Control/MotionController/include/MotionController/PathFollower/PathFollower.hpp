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
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <set>

#include "Controller/IController.hpp"
#include "Robot/IRobot.hpp"
#include "TrajectoryGenerator/IJointTrajectoryGenerator.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "ForwardKinematics/IForwardKinematics.hpp"
#include "Types/Types.hpp"
#include "MotionController/AbstractMotionController.hpp"


#include "EventLogger/EventLogger.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointSignals;
using crf::utility::types::TaskSignals;
using crf::utility::types::Signals;
using crf::actuators::robot::RobotConfiguration;
using crf::control::inversekinematics::ResultFlags;
using crf::control::controller::IController;
using crf::actuators::robot::IRobot;
using crf::control::trajectorygenerator::IJointTrajectoryGenerator;
using crf::control::trajectorygenerator::ITaskTrajectoryGenerator;
using crf::control::forwardkinematics::IForwardKinematics;

namespace crf::control::motioncontroller {

/**
 * @ingroup group_path_follower
 * @brief Motion controller implementation that generates a trajectory between several points for
 * the robot to follow.
 *
 */
class PathFollower: public AbstractMotionController {
 public:
    PathFollower(
        std::shared_ptr<IRobot> robot,
        std::shared_ptr<IController> controller,
        std::shared_ptr<IJointTrajectoryGenerator> jointTrajGenerator,
        std::shared_ptr<ITaskTrajectoryGenerator> taskTrajGenerator);
    ~PathFollower() override;

    bool initialize() override;

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

 private:
    std::shared_ptr<IRobot> robot_;
    std::shared_ptr<IController> controller_;
    std::shared_ptr<IJointTrajectoryGenerator> jointTrajGenerator_;
    std::shared_ptr<ITaskTrajectoryGenerator> taskTrajGenerator_;

    std::shared_ptr<RobotConfiguration> robotConfig_;

    crf::utility::logger::EventLogger logger_;

    Signals jointControl() override;
    Signals taskControl() override;
};

}  // namespace crf::control::motioncontroller
