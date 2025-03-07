/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playan Garai BE/CEM/MRO 2023
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <vector>
#include <memory>
#include <set>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "MotionController/IMotionController.hpp"

#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

#include "Types/JsonConverters.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"
#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace control {
namespace motioncontroller {

class MotionControllerClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public IMotionController {
 public:
    explicit MotionControllerClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    MotionControllerClient() = delete;
    MotionControllerClient(const MotionControllerClient&) = delete;
    MotionControllerClient(MotionControllerClient&&) = delete;
    ~MotionControllerClient() override;

    bool initialize() override;
    bool deinitialize() override;

    void startJointsControlLoop() override;
    void startTaskControlLoop() override;

    crf::expected<bool> appendPath(const std::vector<JointPositions>& path) override;
    crf::expected<bool> appendPath(
        const std::vector<TaskPose>& path,
        const TrajectoryExecutionMethod& method,
        const PointReferenceFrame& frame) override;
    crf::expected<bool> setVelocity(
        const JointVelocities& velocity) override;
    crf::expected<bool> setVelocity(
        const TaskVelocity& velocity,
        const PointReferenceFrame& frame) override;
    crf::expected<bool> setTorque(
        const JointForceTorques& torque) override;
    crf::expected<bool> setTorque(
        const TaskForceTorque& torque,
        const PointReferenceFrame& frame) override;

    crf::expected<bool> setProfileVelocity(const JointVelocities& velocity) override;
    crf::expected<bool> setProfileVelocity(const TaskVelocity& velocity) override;
    crf::expected<bool> setProfileAcceleration(const JointAccelerations& acceleration) override;
    crf::expected<bool> setProfileAcceleration(const TaskAcceleration& acceleration) override;

    void softStop() override;
    void hardStop() override;
    crf::expected<bool> setParameters(const nlohmann::json& params) override;
    nlohmann::json getCurrentParameters() override;
    nlohmann::json getParametersDefinition() const override;
    crf::expected<bool> isTrajectoryRunning() override;
    Signals getSignals() override;
    std::set<crf::Code> getStatus() override;
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const uint32_t priority_;

    Receiver<crf::expected<bool>> receiverStartJointsControlLoop_;
    Receiver<crf::expected<bool>> receiverStartTaskControlLoop_;
    Receiver<crf::expected<bool>> receiverAppendPath_;
    Receiver<crf::expected<bool>> receiverSetVelocity_;
    Receiver<crf::expected<bool>> receiverSetTorque_;
    Receiver<crf::expected<bool>> receiverSetProfileVelocity_;
    Receiver<crf::expected<bool>> receiverSetProfileAcceleration_;
    Receiver<crf::expected<bool>> receiverSoftStop_;
    Receiver<crf::expected<bool>> receiverHardStop_;
    Receiver<crf::expected<bool>> receiverSetParameters_;

    std::atomic<uint32_t> priorityUnderControl_;
    crf::expected<bool> isTrajectoryRunning_;
    std::set<crf::Code> controllerStatus_;
    Signals robotSignals_;

    crf::utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    crf::expected<bool> sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<crf::expected<bool>>& receiver);
};

}  // namespace motioncontroller
}  // namespace control
}  // namespace crf
