#pragma once

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"

namespace Packets {

  class GSMInfoStatusPacket : public Packet {
  public:
    static const unsigned short type = GSM_INFO_STATU_PACKET_TYPE;

    std::string cellName;
    std::string connectionType;
    float LTErsrq;
    float LTEsinr;
    float LTErsrp;
    float WCDMAecic;
    float WCDMArscp;
    float GSMSignalStrength;
    long GSMBytesReceived;
    long GSMBytesSent;

    virtual std::string serialize() const{
        int size = cellName.length() + 2 + connectionType.length() + 2 + 4*6 + 8*2;    
        char buf[size];

        short cellNameLength = cellName.length();
        int iterator = 0;
        memcpy(buf + iterator, &cellNameLength, 2);
        iterator += 2;
        memcpy(buf+iterator, cellName.c_str(), cellName.length());
        iterator += cellName.length();

        short connTypeLength = connectionType.length();
        memcpy(buf+iterator, &connTypeLength, 2);
        iterator += 2;
        memcpy(buf+iterator, connectionType.c_str(), connectionType.length());
        iterator += connectionType.length();

        memcpy(buf+iterator, &LTErsrq, 4);
        iterator += 4;
        memcpy(buf+iterator, &LTEsinr, 4);
        iterator += 4;
        memcpy(buf+iterator, &LTErsrp, 4);
        iterator += 4;
        memcpy(buf+iterator, &WCDMAecic, 4);
        iterator += 4;
        memcpy(buf+iterator, &WCDMArscp, 4);
        iterator += 4;
        memcpy(buf+iterator, &GSMSignalStrength, 4);
        iterator += 4;
        memcpy(buf+iterator, &GSMBytesReceived, 8);
        iterator += 8;
        memcpy(buf+iterator, &GSMBytesSent, 8);

        return std::string(buf, size);
    }
    
    virtual bool deserialize(const std::string& buffer) {
        short cellNameLength;
        int iterator = 0;
        memcpy(&cellNameLength, buffer.c_str(), 2);
        iterator += 2;

        cellName = std::string (buffer.c_str()+iterator, cellNameLength);
        iterator += cellNameLength;

        short connTypeLength;
        memcpy(&connTypeLength, buffer.c_str()+iterator, 2);
        iterator += 2;

        connectionType = std::string(buffer.c_str()+iterator, connTypeLength);
        iterator += connTypeLength;

        memcpy(&LTErsrq, buffer.c_str()+iterator,  4);
        iterator += 4;
        memcpy(&LTEsinr, buffer.c_str()+iterator, 4);
        iterator += 4;
        memcpy(&LTErsrp, buffer.c_str()+iterator,  4);
        iterator += 4;
        memcpy(&WCDMAecic, buffer.c_str()+iterator,  4);
        iterator += 4;
        memcpy(&WCDMArscp, buffer.c_str()+iterator, 4);
        iterator += 4;
        memcpy(&GSMSignalStrength, buffer.c_str()+iterator, 4);
        iterator += 4;
        memcpy(&GSMBytesReceived, buffer.c_str()+iterator, 8);
        iterator += 8;
        memcpy(&GSMBytesSent, buffer.c_str()+iterator, 8);


        return true;
    }

    virtual PacketHeader getHeader() const {
      PacketHeader header;
      header.type = this->type;
      header.length = cellName.length() + 2 + connectionType.length() + 2 + 4*6 + 8*2;
      return header;
    }
  };
}
