/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
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

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerClient/StatusStreamerClient.hpp"
#include "MissionManager/IMissionManager.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"

#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::applications::missionmanager {

class MissionManagerClient:
    public crf::utility::devicemanager::StatusStreamerClient,
    public IMissionManager {
 public:
    explicit MissionManagerClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket>,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency = 0);
    ~MissionManagerClient();

    bool start() override;
    bool next() override;
    bool stop() override;
    bool pause() override;
    bool resume() override;
    bool goHome() override;
    bool recharge() override;
    bool setStatus(const nlohmann::json& json) override;
    nlohmann::json getStatus() override;
    bool emergency() override;
    bool rearm() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;

    Receiver<bool> receiverStart_;
    Receiver<bool> receiverNext_;
    Receiver<bool> receiverStop_;
    Receiver<bool> receiverPause_;
    Receiver<bool> receiverResume_;
    Receiver<bool> receiverGoHome_;
    Receiver<bool> receiverRecharge_;
    Receiver<bool> receiverSetStatus_;
    Receiver<bool> receiverEmergency_;
    Receiver<bool> receiverRearm_;

    nlohmann::json statusUpdate_;

    utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    bool sendPacket(const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace crf::applications::missionmanager
