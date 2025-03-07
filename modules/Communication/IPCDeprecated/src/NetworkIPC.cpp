/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Giacomo Lunghi CERN EN/SMM/MRO,
 *  ==================================================================================================
 */
#include <memory>
#include <string>

#include "IPC/NetworkIPC.hpp"

namespace crf {
namespace communication {
namespace ipc {

NetworkIPC::NetworkIPC(
    const std::shared_ptr<networkserver::INetworkServer>& server) :
    server_(server),
    logger_("NetworkIPC"),
    isOpen_(false) {
        logger_->debug("CTor");
}

NetworkIPC::~NetworkIPC() {
    logger_->debug("DTor");
    close();
}

bool NetworkIPC::open() {
    logger_->debug("open()");
    if (isOpen_) {
        logger_->warn("Already open");
        return false;
    }
    isOpen_ = server_->acceptConnection();
    return isOpen_;
}

bool NetworkIPC::close() {
    logger_->debug("close");
    if (!isOpen_) {
        logger_->warn("Already closed");
        return false;
    }
    server_->disconnect();
    isOpen_ = false;
    return true;
}

bool NetworkIPC::write(const std::string& bytes, const Packets::PacketHeader& header) {
    logger_->debug("write");
    if (!isOpen_) {
        logger_->warn("IPC not open");
        return false;
    }
    if (isAutoreconnecting()) {
        logger_->info("Ongoing autoreconnection with the client ... ");
        return false;
    }
    if (!server_->isConnected()) {
        logger_->warn("NetworkServer is not connected, connection with client probably lost");
        invokeAutoReconnection();
        return false;
    }
    if (!server_->send(header, bytes)) {
        logger_->warn("NetworkServer could not send data");
        return false;
    }
    return true;
}

bool NetworkIPC::read(std::string& bytes, Packets::PacketHeader& header) {
    logger_->debug("read");
    if (!isOpen_) {
        logger_->warn("IPC not open");
        return false;
    }
    if (isAutoreconnecting()) {
        logger_->info("Ongoing autoreconnection with the client ... ");
        return false;
    }
    if (!server_->isConnected()) {
        logger_->warn("NetworkServer is not connected, connection with client probably lost");
        invokeAutoReconnection();
        return false;
    }
    if (!server_->receive(&header, &bytes)) {
        logger_->warn("NetworkServer could not receive data");
        return false;
    }
    return true;
}

bool NetworkIPC::invokeAutoReconnection() {
    logger_->info("invokeAutoReconnection");
    if (autoReconnection_.valid()) {
        logger_->error("Some impl fuckup, future should be empty at this point");
        return false;
    }
    autoReconnection_ = std::async(std::launch::async,
        [this] () {
            return server_->acceptConnection();
        });
    return true;
}
bool NetworkIPC::isAutoreconnecting() {
    if (!autoReconnection_.valid()) {
        return false;
    }
    if (autoReconnection_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        logger_->info("Autoreconnection finished with result = {}", autoReconnection_.get());
        return false;
    }
    return true;
}

}  // namespace ipc
}  // namespace communication
}  // namespace crf
