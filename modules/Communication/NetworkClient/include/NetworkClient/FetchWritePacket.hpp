#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <cstdint>
#include <cstring>
#include <exception>
#include <ostream>
#include <string>
#include <vector>
#include <algorithm>

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"

namespace Packets {

class FetchWritePacket: public Packet {
 public:
    FetchWritePacket(): dataBlockNumber_(0), startAddress_(0), dataLength_(0), data_() {}
    std::string serialize() const override {
        char buf[getSize()];  // NOLINT
        std::size_t offset = 0;
        std::memcpy(buf + offset, &dataBlockNumber_, sizeof dataBlockNumber_);
        offset += sizeof dataBlockNumber_;
        std::memcpy(buf + offset, &startAddress_, sizeof startAddress_);
        offset += sizeof startAddress_;
        std::memcpy(buf + offset, &dataLength_, sizeof dataLength_);
        offset += sizeof dataLength_;
        std::memcpy(buf + offset, data_.data(),
            sizeof(decltype(data_)::value_type) * data_.size());
        return std::string(buf, getSize());
    }
    bool deserialize(const std::string& buff) override {
        data_.clear();
        if (buff.length() < getSize()) {
            return false;
        }
        std::size_t offset = 0;
        std::memcpy(&dataBlockNumber_, buff.c_str() + offset, sizeof dataBlockNumber_);
        offset += sizeof dataBlockNumber_;
        std::memcpy(&startAddress_, buff.c_str() + offset, sizeof startAddress_);
        offset += sizeof startAddress_;
        std::memcpy(&dataLength_, buff.c_str() + offset, sizeof dataLength_);
        offset += sizeof dataLength_;
        std::copy(buff.begin() + offset, buff.end(), std::back_inserter(data_));
        return true;
    }
    PacketHeader getHeader() const override {
        PacketHeader header{};
        header.type = FETCHWRITE_PACKET_TYPE;
        header.length = getSize();
        return header;
    }
    std::string toJSONString() const override {
        std::string jsonString("{\"dataBlockNumber_\": " + std::to_string(dataBlockNumber_)
            + ", \"startAddress_\": " + std::to_string(startAddress_)
            + ", \"dataLength_\": " + std::to_string(dataLength_)
            + ", \"data_\": []}");
        if (data_.empty()) {
            return jsonString;
        }
        std::string arrayContent = std::to_string(static_cast<int>(data_[0]));
        for (int i = 1; i < data_.size(); i++) {
            arrayContent += ", ";
            arrayContent += std::to_string(static_cast<int>(data_[i]));
        }
        jsonString.insert(jsonString.find_last_of("]"), arrayContent);
        return jsonString;
    }
    uint8_t dataBlockNumber_;
    uint16_t startAddress_;
    uint16_t dataLength_;
    std::vector<uint8_t> data_;

 private:
    std::size_t getSize() const {
        return sizeof dataBlockNumber_ + sizeof startAddress_ + sizeof dataLength_ +
            sizeof(decltype(data_)::value_type) * data_.size();
    }
};

inline bool operator==(const FetchWritePacket& lhs, const FetchWritePacket& rhs) {
    return lhs.dataBlockNumber_ == rhs.dataBlockNumber_
        && lhs.startAddress_ == rhs.startAddress_
        && lhs.dataLength_ == rhs.dataLength_
        && lhs.data_ == rhs.data_;
}

inline std::ostream& operator<<(std::ostream& os, const FetchWritePacket& packet) {
    return os << packet.toJSONString();
}

}  // namespace Packets
