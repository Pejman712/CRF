#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Giacomo Lunghi CERN EN/SMM/MRO,
 *  ==================================================================================================
 */

#include <condition_variable>
#include <memory>
#include <mutex>
#include <future>
#include <string>

#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkServer/INetworkServer.hpp"

namespace crf {
namespace communication {
namespace ipc {

class NetworkIPC : public IPC {
 public:
    NetworkIPC() = delete;
    explicit NetworkIPC(const std::shared_ptr<networkserver::INetworkServer>&);
    ~NetworkIPC() override;

    bool open() override;
    bool close() override;

    bool write(const std::string& bytes, const Packets::PacketHeader& header) override;
    bool read(std::string& bytes, Packets::PacketHeader& header) override;
 private:
    std::shared_ptr<networkserver::INetworkServer> server_;
    utility::logger::EventLogger logger_;
    bool isOpen_;
    std::future<bool> autoReconnection_;
    bool invokeAutoReconnection();
    bool isAutoreconnecting();
};

}  // namespace ipc
}  // namespace communication
}  // namespace crf
