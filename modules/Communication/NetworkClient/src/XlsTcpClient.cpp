/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <errno.h>

#include "CommUtility/PacketTypes.hpp"
#include "NetworkClient/XlsTcpClient.hpp"

#define STANDARD_HEADER_LENGTH 12

namespace crf {
namespace communication {
namespace networkclient {

XlsTcpClient::XlsTcpClient(const std::string& address, int port,
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    TcpClient(address, port, socketInterface),
    logger_("XlsTcpClient") {
    logger_->debug("CTor");
}

XlsTcpClient::~XlsTcpClient() {
    logger_->debug("DTor");
}

bool XlsTcpClient::send(const Packets::PacketHeader& header, const std::string& buffer) {
    logger_->debug("send");
    if (!isConnected()) {
        logger_->warn("Cannot send, because not connected");\
        return false;
    }
    if (header.type() != Packets::XLS_ADAPTER_PACKET_TYPE) {
        logger_->warn("Packet type {} is not supported by this client", header.type);
        return false;
    }
    if (header.length != buffer.size()) {
        logger_->warn("Requested data length {} is not equal to buffer size {}",
            header.length, buffer.size());
        return false;
    }
    if (header.length < STANDARD_HEADER_LENGTH) {
        logger_->error("Created Packet length Incorrect");
        return false;
    }
    int bytesSent = socketInterface_->send(socket_, buffer.c_str(), buffer.size(), 0);
    if (bytesSent < 0) {
        logger_->warn("Failed to send data: {}", strerror(errno));
        return false;
    }
    return true;
}

bool XlsTcpClient::receive(Packets::PacketHeader* header, std::string* buffer) {
    logger_->debug("receive");
    if (!isConnected()) {
        logger_->warn("Cannot receive, because not connected");\
        return false;
    }
    if (header->type != Packets::XLS_ADAPTER_PACKET_TYPE) {
        logger_->warn("Packet type {} is not supported by this client", header->type);
        return false;
    }
    char* buff = buff = new char[header->length];
    int bytesReceived = socketInterface_->recv(socket_, buff, header->length, 0);
    logger_->debug("Received {} bytes", bytesReceived);
    if (bytesReceived < 0) {
        logger_->warn("Failed to receive data: {}", strerror(errno));
        delete[] buff;
        return false;
    }
    if (bytesReceived != header->length) {
        logger_->warn("Number of bytes received {} is not equal to number of bytes requested {}",
            bytesReceived, header->length);
    }
    buffer->assign(buff, bytesReceived);
    delete[] buff;
    return true;
}

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
