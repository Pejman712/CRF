/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>
#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocketServer.hpp"

namespace crf {
namespace communication {
namespace sockets {

class UnixSocketServer : public ISocketServer {
 public:
    UnixSocketServer() = delete;
    explicit UnixSocketServer(const std::string& name);
    UnixSocketServer(const UnixSocketServer&) = delete;
    UnixSocketServer(UnixSocketServer&&) = delete;
    ~UnixSocketServer() override;

    bool open() override;
    bool close() override;

    bool isOpen() override;

    std::optional<std::shared_ptr<ISocket> > acceptConnection() override;

 private:
    int getSocketError();
    bool isInTimedWait();
    utility::logger::EventLogger logger_;

    std::string name_;
    int serverFd_;

    bool isOpen_;
    std::vector<std::shared_ptr<ISocket> > connectedClients_;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
