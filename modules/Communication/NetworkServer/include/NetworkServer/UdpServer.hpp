#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/ISocketInterface.hpp"
#include "NetworkServer/INetworkServer.hpp"

namespace crf {
namespace communication {
namespace networkserver {

class UdpServer : public INetworkServer {
 public:
    UdpServer() = delete;
    explicit UdpServer(int port,
     std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    UdpServer(const UdpServer&) = delete;
    UdpServer(UdpServer&&) = delete;
    ~UdpServer() override;

    // Unlike TCP, UDP has no real connect and disconnect,
    // here we only modify data memebers of the class and send recieve connection messages
    bool acceptConnection(bool blocking) override;
    bool disconnect() override;

    bool send(const Packets::PacketHeader& header, const std::string& buffer) override;
    bool receive(Packets::PacketHeader* header, std::string* buffer) override;
    bool isConnected() const override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface_;
    int serverSocket_;
    struct sockaddr_in clientAddress_;
    bool isConnected_;
};

}  // namespace networkserver
}  // namespace communication
}  // namespace crf
