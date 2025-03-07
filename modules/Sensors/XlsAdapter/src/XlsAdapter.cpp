/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "XlsAdapter/XlsAdapter.hpp"

#define STANDARD_HEADER_LENGTH 12
#define ONE_WIRE_MODE 0x00000001
#define TWO_WIRES_MODE 0x00000003

using crf::communication::networkclient::INetworkClient;

namespace crf {
namespace sensors {
namespace xlsadapter {

XlsAdapter::XlsAdapter(std::shared_ptr<INetworkClient> tcpCommPoint):
    logger_("XlsAdapter"),
    tcpCommPoint_(tcpCommPoint),
    initialized_(false) {
    logger_->debug("CTor");
}

XlsAdapter::~XlsAdapter() {
    logger_->debug("DTor");
    deinitialize();
}

bool XlsAdapter::initialize() {
    logger_->info("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!tcpCommPoint_->connect()) {
        logger_->warn("Failed to connect");
        return false;
    }
    initialized_ = true;
    return true;
}

bool XlsAdapter::deinitialize() {
    logger_->info("deinitialize");
    if (!initialized_) {
        logger_->warn("Not previously initialized");
        return false;
    }
    if (!tcpCommPoint_->disconnect()) {
        logger_->warn("Failed to disconnect");
        return false;
    }
    initialized_ = false;
    return true;
}

std::vector<float> XlsAdapter::getData() {
    logger_->info("getData");
    if (!initialized_) {
        logger_->warn("Device not initialized");
        return std::vector<float>();
    }
    XlsMessage message;
    std::string buffer = message.createSubscribeMessage();
    Packets::PacketHeader header = message.getXlsPacketHeader();
    header.length = buffer.size();
    if (!tcpCommPoint_->send(header, buffer)) {
        logger_->error("Failed to send packet");
        return std::vector<float>();
    }
    XlsMessage receivedMessage;
    header = receivedMessage.getXlsPacketHeader();
    int bytesRequested = STANDARD_HEADER_LENGTH;
    buffer.clear();
    std::string buff;
    do {
        header.length = bytesRequested;
        if (!tcpCommPoint_->receive(&header, &buffer)) {
            logger_->error("Failed to receive reply");
            return std::vector<float>();
        }
        buff.append(buffer);
        bytesRequested = message.getBytesToBeRequested(buff);
    } while (bytesRequested > 0);
    std::vector<float> positions = receivedMessage.interpretData(buff);
    return positions;
}

bool XlsAdapter::changeMode(Mode m) {
    logger_->info("changeMode");
    if (!initialized_) {
        logger_->warn("Device not initialized");
        return false;
    }
    XlsMessage message;
    std::string buffer;
    switch (m) {
        case OneWire:
            buffer = message.createSetModeMessage(ONE_WIRE_MODE);
        break;
        case TwoWires:
            buffer = message.createSetModeMessage(TWO_WIRES_MODE);
        break;
    }
    Packets::PacketHeader header = message.getXlsPacketHeader();
    header.length = buffer.size();
    if (!tcpCommPoint_->send(header, buffer)) {
        logger_->error("Failed to send packet");
        return false;
    }
    XlsMessage receivedMessage;
    header = receivedMessage.getXlsPacketHeader();
    int bytesRequested = STANDARD_HEADER_LENGTH;
    buffer.clear();
    std::string buff;
    do {
        header.length = bytesRequested;
        if (!tcpCommPoint_->receive(&header, &buffer)) {
            logger_->error("Failed to receive reply");
            return false;
        }
        buff.append(buffer);
        bytesRequested = message.getBytesToBeRequested(buff);
    } while (bytesRequested > 0);
    return receivedMessage.checkReply(buff);
}

}  // namespace xlsadapter
}  // namespace sensors
}  // namespace crf
