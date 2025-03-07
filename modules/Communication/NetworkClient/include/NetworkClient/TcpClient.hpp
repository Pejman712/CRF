#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <netdb.h>

#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/ISocketInterface.hpp"
#include "NetworkClient/INetworkClient.hpp"

namespace crf {
namespace communication {
namespace networkclient {

class TcpClient: public INetworkClient {
 public:
    explicit TcpClient(
        std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    TcpClient(const std::string& address, int port,
        std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    TcpClient(const TcpClient&) = delete;
    TcpClient(TcpClient&&) = delete;
    ~TcpClient() override;

    bool connect(bool blocking) override;
    bool disconnect() override;
    bool send(const Packets::PacketHeader& header, const std::string& buffer) override;
    bool receive(Packets::PacketHeader* header, std::string* buffer) override;
    bool isConnected() const override;
 private:
    utility::logger::EventLogger logger_;
 protected:
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface_;
    int socket_;
    struct addrinfo* addrinfoPtr_;
};

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
