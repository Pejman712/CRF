/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "TimeSynchronization/TimeSynchronizationServer.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace communication {
namespace timesynchronizationserver {

TimeSynchronizationServer::TimeSynchronizationServer(
    const std::shared_ptr<IPC>& ipc) :
        logger_("TimeSynchronizationServer"),
        ipc_(ipc),
        stopThread_(false),
        runThread_() {
            logger_->debug("CTor");
}

TimeSynchronizationServer::~TimeSynchronizationServer() {
    logger_->debug("DTor");
    deinitialize();
}

bool TimeSynchronizationServer::initialize() {
    logger_->debug("initialize()");
    if (runThread_.joinable()) {
        logger_->warn("Alerady initialized");
        return false;
    }
    stopThread_ = false;
    runThread_ = std::thread(&TimeSynchronizationServer::run, this);
    return true;
}

bool TimeSynchronizationServer::deinitialize() {
    logger_->debug("deinitialize()");
    if (!runThread_.joinable()) {
        logger_->warn("Not initialized");
        return false;
    }
    stopThread_ = true;
    runThread_.join();
    return true;
}

void TimeSynchronizationServer::run() {
    logger_->debug("run()");
    Packets::PacketHeader header;
    std::string buffer;
    while (!stopThread_) {
        if (!ipc_->read(buffer, header)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
            continue;
        }
        auto now = std::chrono::high_resolution_clock::now();
        communication::datapackets::JSONPacket json;
        if (!json.deserialize(buffer)) {
            continue;
        }
        logger_->info("Received sync packet");
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        json.data_["time"] = ms.count();
        if (!ipc_->write(json.serialize(), json.getHeader())) {
            logger_->warn("Could not write message");
            continue;
        }
    }
}

}  // namespace timesynchronizationserver
}  // namespace communication
}  // namespace crf
