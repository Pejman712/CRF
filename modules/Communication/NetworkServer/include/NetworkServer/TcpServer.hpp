#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/ISocketInterface.hpp"
#include "NetworkServer/INetworkServer.hpp"

namespace crf {
namespace communication {
namespace networkserver {

class TcpServer: public INetworkServer {
 public:
    TcpServer() = delete;
    explicit TcpServer(int port,
        std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    TcpServer(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    ~TcpServer() override;

    bool acceptConnection(bool blocking) override;
    bool disconnect() override;
    bool send(const Packets::PacketHeader& header, const std::string& buffer) override;
    bool receive(Packets::PacketHeader* header, std::string* buffer) override;
    bool isConnected() const override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface_;
    int serverSocket_;
    int connectedSocket_;
};

}  // namespace networkserver
}  // namespace communication
}  // namespace crf
