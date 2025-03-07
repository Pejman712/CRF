/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerClient/StatusStreamerClient.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/**
 * @ingroup group_priority_access_client
 * @brief Client that can communicate with a Device Manager Priority Access
 *        communication point.
 */
class PriorityAccessClient: public StatusStreamerClient {
 public:
    PriorityAccessClient() = delete;
    PriorityAccessClient(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout,
        const float& frequency,
        const uint32_t& priority);
    ~PriorityAccessClient() override;

 private:
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const int priority_;

    Receiver<bool> receiverLock_;
    Receiver<bool> receiverUnlock_;

    crf::utility::logger::EventLogger logger_;

 protected:
    bool lockControl();
    bool unlockControl();
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
