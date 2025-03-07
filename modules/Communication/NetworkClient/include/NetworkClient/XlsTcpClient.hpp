#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/ISocketInterface.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/TcpClient.hpp"

namespace crf {
namespace communication {
namespace networkclient {

class XlsTcpClient: public TcpClient {
 public:
    XlsTcpClient() = delete;
    /**
     * Parameters:
     *  - address: IP address of the XLS device (TODO: what is the typical address?)
     *  - port: TODO: what is the typical port?
     */
    XlsTcpClient(const std::string& address, int port,
        std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    XlsTcpClient(const XlsTcpClient&) = delete;
    XlsTcpClient(XlsTcpClient&&) = delete;
    ~XlsTcpClient() override;
    bool send(const Packets::PacketHeader& header, const std::string& buffer) override;
    bool receive(Packets::PacketHeader* header, std::string* buffer) override;

 private:
    utility::logger::EventLogger logger_;
};

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
