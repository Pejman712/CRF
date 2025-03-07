/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <future>
#include <optional>

#include "RobotBaseController/IRobotBaseController.hpp"
#include "TrajectoryPointGenerator/ITrajectoryPointGenerator.hpp"
#include "Types/TaskTypes/TaskSignals.hpp"  // There is one in TrajectoryGenerator and another in Types (??)

#include "RobotBase/IRobotBase.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotbasecontroller {

class RobotBaseVelocityController : public IRobotBaseController {
 public:
    RobotBaseVelocityController() = delete;
    RobotBaseVelocityController(
        std::shared_ptr<crf::actuators::robotbase::IRobotBase>);
    RobotBaseVelocityController(const RobotBaseVelocityController& other) = delete;
    RobotBaseVelocityController(RobotBaseVelocityController&& other) = delete;
    ~RobotBaseVelocityController() override;

    bool initialize() override;
    bool deinitialize() override;

    std::future<bool> setPosition(
        const crf::utility::types::TaskPose& targetPosition) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& targetPosition) override;

    bool setVelocity(
        const crf::utility::types::TaskVelocity& targetVelocity) override;
    bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& targetVelocity) override;

    bool interruptTrajectory() override;

    crf::utility::types::TaskPose getPosition() override;
    crf::utility::types::TaskVelocity getVelocity() override;

    bool setMaximumVelocity(
        const utility::types::TaskVelocity& maxVelocity) override;
    bool setMaximumAcceleration(
        const utility::types::TaskAcceleration& maxAcceleration) override;

 private:
    std::shared_ptr<crf::actuators::robotbase::IRobotBase> base_;
    std::shared_ptr<crf::actuators::robotbase::RobotBaseConfiguration> configuration_;
    std::shared_ptr<control::trajectorypointgenerator::ITrajectoryPointGenerator> trajGenerator_;

    crf::utility::types::TaskTrajectoryData currentState_;
    crf::utility::types::TaskTrajectoryData targetState_;
    crf::utility::types::TaskTrajectoryData taskLimit_;

    bool stopControlLoop_;
    std::atomic<bool> interruptTrajectory_;
    std::atomic<bool> baseMoving_;
    bool initialized_;
    std::vector<bool> dimSelectionVec_;
    const std::chrono::microseconds rtLoopTime_;
    crf::control::trajectorypointgenerator::ControlMode trajGeneratorMode_;

    std::thread controlLoopThread_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastCommandTime_;

    std::mutex m_;
    std::condition_variable cv_;

    mutable std::mutex currentStateMutex_;
    mutable std::mutex targetStateMutex_;
    utility::logger::EventLogger logger_;

    const float baseMovingVelocityThreshold_ = 0.01;
    const int velocityCommandInterval_ = 2000;
    const float proximityThreshold_ = 0.05;

    void controlLoop();
    bool updateBaseStatus();
    bool updateTrajGenerator(const crf::control::trajectorypointgenerator::ControlMode& mode);
    bool taskTrajectoryExecution(
        const std::vector<utility::types::TaskPose>& positions,
        const std::vector<utility::types::TaskVelocity>& velocities);
    bool inProximityOfTargetPosition(
        const utility::types::TaskPose& position);
};

}  // namespace crf::control::robotbasecontroller
