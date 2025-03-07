/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <string>
#include <functional>

#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

PriorityAccessClient::PriorityAccessClient(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout,
    const float& frequency,
    const uint32_t& priority):
    StatusStreamerClient(socket, serverReplyTimeout, frequency),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    priority_(priority),
    logger_("PriorityAccessClient") {
        logger_->debug("CTor");

        receivers_.insert({"lockControl",  &receiverLock_});
        receivers_.insert({"unlockControl", &receiverUnlock_});
}

PriorityAccessClient::~PriorityAccessClient() {
    logger_->debug("DTor");
    if (initialized_) StatusStreamerClient::deinitialize();
}

// Protected
bool PriorityAccessClient::lockControl() {
    logger_->debug("lockControl");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "lockControl";
    json.data["priority"] = priority_;

    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), true);
    socketLock.unlock();

    std::optional<bool> result = receiverLock_.waitFor(serverReplyTimeout_);
    if (!result) return false;
    if (!result.value()) {
        logger_->error("Not able to get control");
        return false;
    }
    return true;
}

bool PriorityAccessClient::unlockControl() {
    logger_->debug("unlockControl");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "unlockControl";
    json.data["priority"] = priority_;

    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), true);
    socketLock.unlock();

    std::optional<bool> result = receiverUnlock_.waitFor(serverReplyTimeout_);
    if (!result) return false;
    if (!result.value()) {
        logger_->error("Not able to release the control");
        return false;
    }
    return true;
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
