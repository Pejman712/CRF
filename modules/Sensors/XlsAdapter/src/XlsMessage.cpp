/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "XlsAdapter/XlsMessage.hpp"

#define STANDARD_HEADER_LENGTH 12
#define MID_SUBSCRIPTION 0xff000001
#define MID_SET_CODE 0x01012F02
#define DATA_SUBSCRIPTION 0x01012000
#define HDR_VERSION 0x01
#define HDR_SIZE_SUBSCRIPTION 0x0010
#define HDR_SIZE_SET_CODE 0x001C
#define PACKET_EXISTS_FLAG 0x40
#define PACKET_NOT_EXISTS_FLAG 0x00
#define REFERENCE 0x0001
#define DEFINITION_BLOCK_LENGTH 4
#define ONE_BLOCK 0x0001
#define FOUR_BYTES_BLOCK 0x0004
#define EIGHT_BYTES_BLOCK 0x0008
#define CODE_SET_MODE 0x0000005A
#define CODE_SETTINGS_TABLE 0x00000003
#define UNIT_MILLIMETERS 0x00000001
#define VALUE_ONLY 0x00000000
#define UNUSED 0x0000
#define GLOBAL_ERROR_STATUS_BYTE 8
#define XPOSITION 22
#define YPOSITION 23
#define XDISTANCE 24
#define YDISTANCE 25

namespace crf {
namespace sensors {
namespace xlsadapter {

XlsMessage::XlsMessage():
    logger_("XlsMessage"),
    MID_(),
    hdrSize_(),
    hdrVersion_(HDR_VERSION),
    packInfo_(PACKET_NOT_EXISTS_FLAG),
    reference_(REFERENCE),
    unused_(UNUSED),
    numBlock_(),
    blockSize_(),
    customHeader_(),
    dataBlock_() {
    logger_->debug("CTor");
}

Packets::PacketHeader XlsMessage::getXlsPacketHeader() {
    Packets::PacketHeader header{};
    header.type = Packets::XLS_ADAPTER_PACKET_TYPE;
    return header;
}

std::string XlsMessage::createSubscribeMessage() {
    MID_ = changeEndianness(MID_SUBSCRIPTION);
    hdrSize_ = HDR_SIZE_SUBSCRIPTION;
    packInfo_ = PACKET_EXISTS_FLAG;
    numBlock_ = ONE_BLOCK;
    blockSize_ = FOUR_BYTES_BLOCK;
    uint32_t subscribeCode = DATA_SUBSCRIPTION;
    for (int i=0; i < sizeof subscribeCode; i++) {
        dataBlock_.push_back((reinterpret_cast<uint8_t*>(&subscribeCode))[i]);
    }
    return serialize();
}

std::string XlsMessage::createSetModeMessage(uint32_t mode) {
    MID_ = changeEndianness(MID_SET_CODE);
    hdrSize_ = HDR_SIZE_SET_CODE;
    packInfo_ = PACKET_EXISTS_FLAG;
    numBlock_ = ONE_BLOCK;
    blockSize_ = EIGHT_BYTES_BLOCK;
    // Unit, Table (3=S), Info
    std::vector<uint32_t> customHeader = {UNIT_MILLIMETERS, CODE_SETTINGS_TABLE, VALUE_ONLY};
    for (int j=0; j < customHeader.size(); j++) {
        for (int i = 0 ; i < sizeof(decltype(customHeader)::value_type); i++) {
            customHeader_.push_back((reinterpret_cast<uint8_t*>(&customHeader[j]))[i]);
        }
    }
    uint32_t code = CODE_SET_MODE;
    for (int i=0; i < sizeof code; i++) {
        dataBlock_.push_back((reinterpret_cast<uint8_t*>(&code))[i]);
    }
    for (int i=0; i < sizeof mode; i++) {
        dataBlock_.push_back((reinterpret_cast<uint8_t*>(&mode))[i]);
    }
    return serialize();
}

int XlsMessage::getBytesToBeRequested(const std::string& buff) {
    if (!deserialize(buff)) {
        logger_->info("Error while deserializing message received, buffer size {}", buff.size());
        return 0;
    }
    if (buff.size() == STANDARD_HEADER_LENGTH) {
        return static_cast<int>(hdrSize_-STANDARD_HEADER_LENGTH);
    } else {
        if (packInfo_ == PACKET_EXISTS_FLAG && (buff.size() != hdrSize_ + \
            static_cast<int>(numBlock_) * static_cast<int>(blockSize_))) {
            return static_cast<int>(numBlock_) * static_cast<int>(blockSize_);
        }
    }
    return 0;
}

std::vector<float> XlsMessage::interpretData(const std::string& buff) {
    if (!deserialize(buff)) {
        logger_->info("Error while deserializing message received, buffer size {}", buff.size());
        return std::vector<float>();
    }
    if ((static_cast<int>(hdrSize_)+(static_cast<int>(numBlock_)*static_cast<int>(blockSize_))) !=\
        buff.size()) {
        logger_->info("Not all bytes have been requested, expected: {}, got: {}",
            static_cast<int>(hdrSize_)+(static_cast<int>(numBlock_)*static_cast<int>(blockSize_)),
            buff.size());
        return std::vector<float>();
    }
    logger_->info("Start Interpreting");
    int numberAverage = static_cast<int>(customHeader_.data())[4];
    float dataValue;
    std::vector<float> positions;
    for (int i=0; i < numBlock_; i++) {
        float sum = 0.0;
        int code;
        std::memcpy(&code, dataBlock_.data() + blockSize_*i, sizeof code);
        int offset = sizeof code;
        for (int k=0; k < numberAverage; k++) {
            std::memcpy(&dataValue, dataBlock_.data()+i*blockSize_+offset, sizeof dataValue);
            sum += dataValue;
            offset += sizeof dataValue;
        }
        if (static_cast<int>(code) == XPOSITION | YPOSITION | XDISTANCE | YDISTANCE) {
            positions.push_back((sum/numberAverage));
        }
    }
    return positions;
}

bool XlsMessage::checkReply(const std::string& buff) {
    if (!deserialize(buff)) {
        logger_->info("Error while deserializing message received, buffer size {}", buff.size());
        return false;
    }
    if ((static_cast<int>(hdrSize_)+(static_cast<int>(numBlock_)*static_cast<int>(blockSize_))) !=\
        buff.size()) {
        logger_->info("Not all bytes have been requested, expected: {}, got: {}",
            static_cast<int>(hdrSize_) + (static_cast<int>(numBlock_) * \
            static_cast<int>(blockSize_)), buff.size());
        return false;
    }
    if (static_cast<int>(numBlock_) != 0) {
        int errorCode = static_cast<int>(customHeader_)[GLOBAL_ERROR_STATUS_BYTE];
        interpretErrorCode(errorCode);
        return false;
    }
    return true;
}

bool XlsMessage::deserialize(const std::string& buff) {
    std::size_t offset = 0;
    if (buff.length() < (sizeof MID_ + sizeof hdrSize_ + sizeof hdrVersion_
            + sizeof packInfo_ + sizeof reference_ + sizeof unused_)) {
        return false;
    }
    std::memcpy(&MID_, buff.c_str() + offset, sizeof MID_);
    offset += sizeof MID_;
    std::memcpy(&hdrSize_, buff.c_str() + offset, sizeof hdrSize_);
    offset += sizeof hdrSize_;
    std::memcpy(&hdrVersion_, buff.c_str() + offset, sizeof hdrVersion_);
    offset += sizeof hdrVersion_;
    std::memcpy(&packInfo_, buff.c_str() + offset, sizeof packInfo_);
    offset += sizeof packInfo_;
    std::memcpy(&reference_, buff.c_str() + offset, sizeof reference_);
    offset += sizeof reference_;
    std::memcpy(&unused_, buff.c_str() + offset, sizeof unused_);
    offset += sizeof unused_;
    if (packInfo_ == PACKET_EXISTS_FLAG && buff.length() > offset) {
        std::memcpy(&numBlock_, buff.c_str() + offset, sizeof numBlock_);
        offset += sizeof numBlock_;
        std::memcpy(&blockSize_, buff.c_str() + offset, sizeof blockSize_);
        offset += sizeof blockSize_;
    }
    if (static_cast<int>(hdrSize_) > offset && buff.length() > offset) {
        std::copy(buff.begin() + offset, buff.begin() + static_cast<int>(hdrSize_),
            std::back_inserter(customHeader_));
        offset += sizeof(decltype(customHeader_)::value_type) * customHeader_.size();
    }
    if (packInfo_ == PACKET_EXISTS_FLAG && buff.length() > offset) {
        std::copy(buff.begin() + offset, buff.end(), std::back_inserter(dataBlock_));
    }
    return true;
}
std::string XlsMessage::serialize() {
    char buf[getSize()];  // NOLINT
    std::size_t offset = 0;
    std::memcpy(buf + offset, &MID_, sizeof MID_);
    offset += sizeof MID_;
    std::memcpy(buf + offset, &hdrSize_, sizeof hdrSize_);
    offset += sizeof hdrSize_;
    std::memcpy(buf + offset, &hdrVersion_, sizeof hdrVersion_);
    offset += sizeof hdrVersion_;
    std::memcpy(buf + offset, &packInfo_, sizeof packInfo_);
    offset += sizeof packInfo_;
    std::memcpy(buf + offset, &reference_, sizeof reference_);
    offset += sizeof reference_;
    std::memcpy(buf + offset, &unused_, sizeof unused_);
    offset += sizeof unused_;
    if (packInfo_ == PACKET_EXISTS_FLAG) {
        std::memcpy(buf + offset, &numBlock_, sizeof numBlock_);
        offset += sizeof numBlock_;
        std::memcpy(buf + offset, &blockSize_, sizeof blockSize_);
        offset += sizeof blockSize_;
    }
    if (static_cast<int>(hdrSize_) > offset) {
        std::memcpy(buf + offset, customHeader_.data(),
            sizeof(decltype(customHeader_)::value_type) *customHeader_.size());
        offset += sizeof(decltype(customHeader_)::value_type) * customHeader_.size();
    }
    if (packInfo_ == PACKET_EXISTS_FLAG) {
        std::memcpy(buf + offset, dataBlock_.data(),
            sizeof(decltype(dataBlock_)::value_type) * dataBlock_.size());
        offset += sizeof(decltype(dataBlock_)::value_type) * dataBlock_.size();
    }
    return std::string(buf, offset);
}

std::size_t XlsMessage::getSize() const {
    return sizeof MID_ + sizeof hdrSize_ + sizeof hdrVersion_ + sizeof packInfo_
        + sizeof reference_ + sizeof unused_+ sizeof numBlock_ + sizeof blockSize_
        + sizeof(decltype(customHeader_)::value_type) * customHeader_.size()
        + sizeof(decltype(dataBlock_)::value_type) * dataBlock_.size();
}

bool XlsMessage::interpretErrorCode(int errorCode) {
    if (errorCode >= 8) {
        logger_->error("Non-existent code number");
        errorCode -= 8;
    }
    if (errorCode >= 4) {
        logger_->error("Wrong code programming value");
        logger_->info("A valid value recommended by the system is found in the Return Value Code");
        errorCode -= 4;
    }
    if (errorCode >= 2) {
        logger_->error("Wrong code table parameter value");
        errorCode -= 2;
    }
    if (errorCode == 1) {
        logger_->error("Wrong unit parameter value");
    }
    return true;
}

uint32_t changeEndianness(uint32_t swapMID) {
    return ((swapMID >> 24)&0xff) | ((swapMID << 8)&0xff0000)
        | ((swapMID >> 8)&0xff00) | ((swapMID << 24)&0xff000000);
}

}  // namespace xlsadapter
}  // namespace sensors
}  // namespace crf
