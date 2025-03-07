#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>

/*
 * FWDs
 */
namespace Packets {
class PacketHeader;
}  // namespace Packets

namespace crf {
namespace communication {
namespace networkclient {

class INetworkClient {
 public:
    virtual ~INetworkClient() = default;

    /*
     * Reminder: default arg only works when dereferencing the base class, where it was declared
     */
    virtual bool connect(bool blocking = true) = 0;
    virtual bool disconnect() = 0;
    virtual bool send(const Packets::PacketHeader& header, const std::string& buffer) = 0;
    virtual bool receive(Packets::PacketHeader* header, std::string* buffer) = 0;
    virtual bool isConnected() const = 0;
};

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
