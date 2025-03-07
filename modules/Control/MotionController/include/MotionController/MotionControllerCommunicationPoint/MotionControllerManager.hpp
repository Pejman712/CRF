/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include "Types/JsonConverters.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "MotionController/IMotionController.hpp"

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller_communication_point
 * @brief Unique class that manages the calls to the object. This class has to make sure the access to
 * the object is thread safe if the class does not guarantee it.
 *
 */
class MotionControllerManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    MotionControllerManager() = delete;
    MotionControllerManager(std::shared_ptr<crf::control::motioncontroller::IMotionController> controller,  // NOLINT
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    MotionControllerManager(const MotionControllerManager& other) = delete;
    MotionControllerManager(MotionControllerManager&& other) = delete;
    ~MotionControllerManager();

    crf::expected<bool> startJointsControlLoop(const int& priority);
    crf::expected<bool> startTaskControlLoop(const int& priority);

    crf::expected<bool> appendPath(
        const int priority,
        const std::vector<JointPositions>& path);
    crf::expected<bool> appendPath(
        const int priority,
        const std::vector<TaskPose>& path,
        const TrajectoryExecutionMethod& method,
        const PointReferenceFrame& frame);

    crf::expected<bool> setVelocity(
        const int priority,
        const JointVelocities& velocity);
    crf::expected<bool> setVelocity(
        const int& priority,
        const TaskVelocity& velocity,
        const PointReferenceFrame& frame);

    crf::expected<bool> setTorque(
        const int priority,
        const JointForceTorques& torque);
    crf::expected<bool> setTorque(
        const int priority,
        const TaskForceTorque& torque,
        const PointReferenceFrame& frame);

    crf::expected<bool> setProfileVelocity(
        const int priority,
        const JointVelocities& velocity);
    crf::expected<bool> setProfileVelocity(
        const int& priority,
        const TaskVelocity& velocity);

    crf::expected<bool> setProfileAcceleration(
        const int priority,
        const JointAccelerations& acceleration);

    crf::expected<bool> setProfileAcceleration(
        const int& priority,
        const TaskAcceleration& acceleration);

    crf::expected<bool> softStop(const int& priority);
    crf::expected<bool> hardStop(const int& priority);

    crf::expected<bool> setParameters(
        const int& priority,
        const nlohmann::json& params);
    nlohmann::json getCurrentParameters(
        const int& priority);
    nlohmann::json getParametersDefinition(
        const int& priority);

    nlohmann::json getStatus();
    nlohmann::json getConfiguration();

 private:
    std::shared_ptr<crf::control::motioncontroller::IMotionController> controller_;
};

}  // namespace crf::control::motioncontroller
