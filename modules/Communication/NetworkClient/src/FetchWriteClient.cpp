/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <errno.h>
#include <array>
#include <stdexcept>
#include <string>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/PacketTypes.hpp"
#include "NetworkClient/FetchWritePacket.hpp"
#include "NetworkClient/FetchWriteClient.hpp"

#define SYSTEM_ID_H 'S'
#define SYSTEM_ID_L '5'
#define TELEGRAM_LENGTH 16
#define OP_CODE_ID 1
#define OP_CODE_LENGTH 3
#define OP_CODE_WRITE 3
#define OP_CODE_FETCH 5
#define ORG_FIELD 3
#define ORG_FIELD_LENGTH 8
#define ORG_ID_DB 1
#define EMPTY_FIELD 0xff
#define EMPTY_FIELD_LENGTH 2
#define ACK_LENGTH 16

namespace crf {
namespace communication {
namespace networkclient {

FetchWriteClient::FetchWriteClient(const std::string& address, int port,
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    TcpClient(address, port, socketInterface),
    logger_("FetchWriteClient") {
    logger_->debug("CTor");
}

FetchWriteClient::~FetchWriteClient() {
    logger_->debug("DTor");
}

bool FetchWriteClient::send(const Packets::PacketHeader& header, const std::string& buffer) {
    logger_->info("send");
    if (!sendTelegram(header, buffer, OP_CODE_WRITE)) {
        return false;
    }
    Packets::FetchWritePacket packet;
    packet.deserialize(buffer);
    std::size_t bytesSent = socketInterface_->send(socket_, packet.data_.data(),
        packet.dataLength_, 0);
    /*
     * Continuation of HAX for Siemens unit length. In case of 1 byte types
     * we send additional dummy byte equal to 0;
     */
    if (packet.dataLength_ == 1) {
        uint8_t zero(0);
        bytesSent = socketInterface_->send(socket_, &zero, 1, 0);
    }
    if (bytesSent < 0) {
        logger_->warn("Failed to send request: {}", strerror(errno));
        return false;
    }
    logger_->debug("bytesSent: {}", bytesSent);
    if (!receiveAck()) {
        return false;
    }
    return true;
}

bool FetchWriteClient::receive(Packets::PacketHeader* header, std::string* buffer) {
    logger_->info("receive");
    if (!sendTelegram(*header, *buffer, OP_CODE_FETCH)) {
        return false;
    }
    if (!receiveAck()) {
        return false;
    }
    Packets::FetchWritePacket packet;
    packet.deserialize(*buffer);
    auto data = std::make_unique<uint8_t[]>(packet.dataLength_);
    if (socketInterface_->recv(socket_, data.get(), packet.dataLength_, 0) < packet.dataLength_) {
        logger_->error("Received too few bytes");
        return false;
    }
    packet.data_.clear();
    for (int i = 0; i < packet.dataLength_; i++) {
        packet.data_.push_back(data[i]);
    }
    *buffer = packet.serialize();
    return true;
}

bool FetchWriteClient::sendTelegram(const Packets::PacketHeader& header,
    const std::string& buffer, uint8_t opCode) {
    if (header.type() != Packets::FETCHWRITE_PACKET_TYPE) {
        logger_->error("Trying to send telegram for unsupported packet type: {}", header.type);
        return false;
    }
    if (!isConnected()) {
        logger_->warn("Cannot execute operation, because not connected");
        return false;
    }
    Packets::FetchWritePacket packet;
    if (!packet.deserialize(buffer)) {
        logger_->warn("Failed to deserialize packet");
        return false;
    }
    std::array<uint8_t, TELEGRAM_LENGTH> req = {
        SYSTEM_ID_H, SYSTEM_ID_L, TELEGRAM_LENGTH, OP_CODE_ID, OP_CODE_LENGTH, opCode, ORG_FIELD,
        ORG_FIELD_LENGTH, ORG_ID_DB, 0, 0, 0, 0, 0, EMPTY_FIELD, EMPTY_FIELD_LENGTH
    };
    req[9] = packet.dataBlockNumber_;
    // Siemens PLC has different endianness
    req[0xa] = (reinterpret_cast<uint8_t*>(&packet.startAddress_))[1];
    req[0xb] = (reinterpret_cast<uint8_t*>(&packet.startAddress_))[0];
    if ((packet.dataLength_ > 1 && packet.dataLength_ % 2) || packet.dataLength_ == 0) {
        logger_->warn("Weird uneven dataLength_: {}. Aborting procedure.", packet.dataLength_);
        return false;
    }
    /*
     * This is HAX for Siemens unit length equal to 16 bits (instead of 8 for our machines)
     */
    uint16_t convertedDataLength = (packet.dataLength_ == 1) ? 1 : packet.dataLength_/2;
    req[0xc] = (reinterpret_cast<uint8_t*>(&convertedDataLength))[1];
    req[0xd] = (reinterpret_cast<uint8_t*>(&convertedDataLength))[0];
    logger_->debug("Byte9 (DB): {}", static_cast<int>(req[9]));
    logger_->debug("Byte10 StartAddr[H]: {}", static_cast<int>(req[10]));
    logger_->debug("Byte11 StartAddr[L]: {}", static_cast<int>(req[11]));
    logger_->debug("Byte12 SourceLength[H]: {}", static_cast<int>(req[12]));
    logger_->debug("Byte13 SourceLength[L]: {}", static_cast<int>(req[13]));
    int bytesSent = 0;
    bytesSent = socketInterface_->send(socket_, req.data(), req.size(), 0);
    logger_->debug("bytesSent: {}", bytesSent);
    if (bytesSent < 0) {
        logger_->warn("Failed to send request: {}", strerror(errno));
        return false;
    }
    return true;
}

bool FetchWriteClient::receiveAck() {
    int bytesReceived = 0;
    std::array<uint8_t, ACK_LENGTH> ack;
    logger_->info("Going to receive ACK");
    if (bytesReceived = socketInterface_->recv(
        socket_, ack.data(), ack.size(), 0) < ack.size()) {
        logger_->error("Received too few bytes: {}", bytesReceived);
        return false;
    }
    if (ack[8]) {
        logger_->error("Got error {} from PLC!", ack[8]);
        return false;
    }
    return true;
}

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
