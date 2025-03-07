/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
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

#include "RobotBaseController/IRobotBaseController.hpp"
#include "RobotBaseController/ControllerMode.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/JsonConverters.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::control::robotbasecontroller {

/*
 * @brief 
 */
class RobotBaseControllerClient: public IRobotBaseController {
 public:
    explicit RobotBaseControllerClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout,
        const float& frequency,
        const uint32_t& priority);
    ~RobotBaseControllerClient();

    bool initialize() override;
    bool deinitialize() override;

    std::future<bool> setPosition(
        const crf::utility::types::TaskPose& position) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& positions) override;

    bool setVelocity(
        const crf::utility::types::TaskVelocity& velocity) override;
    bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& velocities) override;

    bool interruptTrajectory() override;

    crf::utility::types::TaskPose getPosition() override;
    crf::utility::types::TaskVelocity getVelocity() override;

    bool setMaximumVelocity(
        const crf::utility::types::TaskVelocity& velocity) override;
    bool setMaximumAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration) override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const float frequency_;
    const int priority_;

    bool initialized_;
    std::atomic<bool> stopThread_;
    std::thread grabberThread_;
    nlohmann::json error_;
    std::mutex mtx_;
    std::condition_variable jsonCV_;

    std::mutex socketMutex_;

    std::mutex lockControlMutex_;
    bool lockControlResult_;
    std::condition_variable lockControlCV_;

    std::mutex unlockControlMutex_;
    bool unlockControlResult_;
    std::condition_variable unlockControlCV_;

    std::mutex activateMutex_;
    bool activateResult_;
    std::condition_variable activateCV_;

    std::mutex trajectoryMutex_;
    bool trajectoryResult_;
    std::condition_variable trajectoryCV_;

    std::mutex statusMutex_;
    bool statusResult_;
    std::condition_variable statusCV_;

    std::atomic<uint64_t> actionRequested_;

    crf::utility::types::TaskPose statusPosition_;
    crf::utility::types::TaskVelocity statusVelocity_;

    utility::logger::EventLogger logger_;

    void grabber();
    bool lockControl();
    bool unlockControl();
    bool sendPosition(const std::vector<crf::utility::types::TaskPose>& positions);
    bool startStream(const int& freq);
    bool stopStream();
};

}  // namespace crf::control::robotbasecontroller
