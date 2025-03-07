#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/StreamWriter.hpp"
#include "CommUtility/StreamReader.hpp"

#include <string>

namespace Packets {

class UDPConnectionPacket : public Packet {
 public:
    enum UDP_CONNECTION_STATE {
        CONNECT = 1,
        DISCONNECT = 0
    };

    static const unsigned short type = UDP_CONNECTION_PACKET_TYPE;  // NOLINT
    static const unsigned int size = 4;

    uint32_t action;

    virtual std::string serialize() const {
        Packets::StreamWriter writer;
        writer.write(action);
        return writer.toString();
    }

    virtual bool deserialize(const std::string& buffer) {
        Packets::StreamReader reader(buffer);
        reader.read(&action);
        return true;
    }

    virtual PacketHeader getHeader() const {
        PacketHeader header;
        header.type = this->type;
        header.length = this->size;
        return header;
    }
};

/** UDP Connection packet actions:
    1: connect;
    2: disconnect;
**/
}  // namespace Packets
