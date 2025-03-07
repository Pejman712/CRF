#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
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
#include "NetworkClient/TcpClient.hpp"

namespace crf {
namespace communication {
namespace networkclient {

/**
 * Class for communication with Siemens PLC controllers using S5 FetchWrite protocol
 * over TCP/IP
 *
 * Basic features:
 *  - connect/disconnect to/from PLC on the given IP and port; usually FETCH operations
 *      are executed on port 2000, and WRITE operations on port 2001
 *  - FETCH data of the given Length from the given DataBlock and Address
 *  - WRITE data of the given Length to the given DataBlock and Address
 *
 * Arguments for both FETCH and WRITE operations are specified in the FetchWritePacket,
 * therefore valid FetchWritePacket should always be passed to both send and receive operations
 *
 * Note: Siemens PLC endianness (Big Endian) is different than endianness of most modern PC
 * stations (Little Endian). Take it into consideration when interpreting numerical values
 * that are bigger than 1 byte.
 */
class FetchWriteClient: public TcpClient {
 public:
    FetchWriteClient() = delete;
    /**
     * Parameters:
     *  - address: IP address of the PLC, e.g. "192.168.0.69"
     *  - port: port for FETCH or WRITE operations, default ports for
     *      Siemens CPU 315 are 2000 (FETCH) and 2001 (WRITE)
     */
    FetchWriteClient(const std::string& address, int port,
        std::shared_ptr<utility::commutility::ISocketInterface> socketInterface = nullptr);
    FetchWriteClient(const FetchWriteClient&) = delete;
    FetchWriteClient(FetchWriteClient&&) = delete;
    ~FetchWriteClient() override;
    /**
     * Executes WRITE operation according to S5 FetchWrite Siemens protocol, i.e.
     * sends the 16-bytes telegram + Length bytes of data, receives 16-bytes ACK
     * The following parameters:
     *  - dataBlockNumber_
     *  - startAddress_
     *  - dataLength_
     * must be specified in the FetchWritePacket which must be serialized to string
     * and passed as a `send` method argument
     *
     * Parameters:
     *  - header: PacketHeader returned by FetchWritePacket::getHeader method
     *  - buffer: string returned by FetchWritePacket::serialize method
     *
     * Return value:
     *  - true upon success
     *  - false upon failure, which includes: client not connected, wrong packet type,
     *      failure to deserialize the packet, unsupported dataLength
     *
     * Note: Only dataLength_ equal to 1 or any even number greater than 0 are accepted
     * This limitation is due to the fact that Siemens PLC unit size is 2 bytes (16 bits);
     */
    bool send(const Packets::PacketHeader& header, const std::string& buffer) override;
    /**
     * Executes FETCH operation according to S5 FetchWrite Siemens protocol, i.e.
     * sends the 16-bytes telegram, receives 16-bytes ACK, receives Length bytes of data
     * The following parameters:
     *  - dataBlockNumber_
     *  - startAddress_
     *  - dataLength_
     * must be specified in the FetchWritePacket which must be serialized to string
     * and passed as a `receive` method argument
     *
     * Parameters:
     *  - header: pointer to PacketHeader returned by FetchWritePacket::getHeader method
     *  - buffer: pointer to string returned by FetchWritePacket::serialize method
     *
     * Return value:
     *  - true upon success
     *  - false upon failure, which includes: client not connected, wrong packet type,
     *      failure to deserialize the packet, unsupported dataLength
     *
     * Note: Only dataLength_ equal to 1 or any even number greater than 0 are accepted
     * This limitation is due to the fact that Siemens PLC unit size is 2 bytes (16 bits);
     *
     * Note: Second argument `buffer` will be overwritten with the data received from the PLC
     * To get the received data user should use the `buffer` to FetchWritePacket::deserialize
     * and read FetchWritePacket::data_ field.
     */
    bool receive(Packets::PacketHeader* header, std::string* buffer) override;

 private:
    bool sendTelegram(const Packets::PacketHeader& header,
        const std::string& buffer, uint8_t opCode);
    bool receiveAck();
    utility::logger::EventLogger logger_;
};

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
