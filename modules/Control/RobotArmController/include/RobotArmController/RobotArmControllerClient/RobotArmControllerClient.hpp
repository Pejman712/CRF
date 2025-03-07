/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "RobotArmController/IRobotArmController.hpp"
#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "RobotArmController/ControllerMode.hpp"
#include "RobotArmController/TrajectoryExecutionMethod.hpp"
#include "RobotArmController/PointReferenceFrame.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/JsonConverters.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::control::robotarmcontroller {

/*
 * @brief
 */
class RobotArmControllerClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public IRobotArmController {
 public:
    explicit RobotArmControllerClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    ~RobotArmControllerClient();

    bool initialize() override;
    bool deinitialize() override;

    std::future<bool> setPosition(
        const crf::utility::types::JointPositions& position) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::JointPositions>& positions) override;
    std::future<bool> setPosition(
        const crf::utility::types::TaskPose& position,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& positions,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool setVelocity(
        const crf::utility::types::JointVelocities& velocity) override;
    bool setVelocity(
        const std::vector<crf::utility::types::JointVelocities>& velocities) override;
    bool setVelocity(
        const crf::utility::types::TaskVelocity& velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& velocities,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool setAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) override;
    bool setAcceleration(
        const std::vector<crf::utility::types::JointAccelerations>& accelerations) override;
    bool setAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    bool setAcceleration(
        const std::vector<crf::utility::types::TaskAcceleration>& accelerations,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool interruptTrajectory() override;

    crf::utility::types::JointPositions getJointPositions() override;
    crf::utility::types::TaskPose getTaskPose() override;
    crf::utility::types::JointVelocities getJointVelocities() override;
    crf::utility::types::TaskVelocity getTaskVelocity() override;
    crf::utility::types::JointAccelerations getJointAccelerations() override;
    crf::utility::types::TaskAcceleration getTaskAcceleration() override;
    crf::utility::types::JointForceTorques getJointForceTorques() override;


    bool setJointsMaximumVelocity(
        const crf::utility::types::JointVelocities& velocity) override;
    bool setTaskMaximumVelocity(
        const crf::utility::types::TaskVelocity& velocity) override;
    bool setJointsMaximumAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) override;
    bool setTaskMaximumAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration) override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::chrono::milliseconds serverReplyTimeout_;
    int numberOfJoints_;
    float frequency_;
    int priority_;

    Receiver<bool> receiverSetMode_;
    Receiver<bool> receiverSetPos_;
    Receiver<bool> receiverTrajectoryResult_;
    Receiver<bool> receiverSetVel_;
    Receiver<bool> receiverSetAcc_;
    Receiver<bool> receiverInterrupt_;
    Receiver<bool> receiverSetJoiMaxVel_;
    Receiver<bool> receiverSetJoiMaxAcc_;
    Receiver<bool> receiverSetTaskMaxVel_;
    Receiver<bool> receiverSetTaskMaxAcc_;
    Receiver<bool> receiverSetGripPos_;
    Receiver<bool> receiverSetGripVel_;

    crf::utility::types::JointPositions statusJointPositions_;
    crf::utility::types::JointVelocities statusJointVelocities_;
    crf::utility::types::JointAccelerations statusJointAccelerations_;
    crf::utility::types::JointForceTorques statusJointForceTorques_;
    crf::utility::types::TaskPose statusTaskPose_;
    crf::utility::types::TaskVelocity statusTaskVelocity_;
    crf::utility::types::TaskAcceleration statusTaskAcceleration_;

    utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;

    bool setMode(crf::control::robotarmcontroller::ControllerMode mode);
    bool waitJointsTrajResult();
    bool waitTaskTrajResult();

    bool sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace crf::control::robotarmcontroller
