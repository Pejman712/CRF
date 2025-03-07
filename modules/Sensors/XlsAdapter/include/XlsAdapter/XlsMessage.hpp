#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "CommUtility/CommunicationPacket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace xlsadapter {

class XlsMessage {
 public:
    XlsMessage();
    /*
     * Content of the Message:
     * Header:
     *
     *                         Header Standard                    |      Def Block        |   Custom Header   |
     * MID | HdrSize | HdrVersion | PackInfo | Reference | Unused |  NumBlock | BlockSize | n Custom Blocks   |
     *  4  |    2    |      1     |    1     |     2     |   2    |     2     |     2     |         4n        |
     *
     * if Def Block and Data Block exists defined by PackInfo == 0x40
     *
     * Data:
     *   Data Block   |
     * DATA (n bytes) |
     *   BlockSize    |
     *
     * For more info see 5.3.1 section in the Xls Communication manual
     */
    Packets::PacketHeader getXlsPacketHeader();
    std::string createSubscribeMessage();
    std::string createSetModeMessage(uint32_t mode);
    int getBytesToBeRequested(const std::string& buff);
    std::vector<float> interpretData(const std::string& buff);
    bool checkReply(const std::string& buff);

 private:
    utility::logger::EventLogger logger_;
    uint32_t MID_;
    uint16_t hdrSize_;
    uint8_t hdrVersion_;
    uint8_t packInfo_;
    uint16_t reference_;
    uint16_t unused_;
    uint16_t numBlock_;
    uint16_t blockSize_;
    std::vector<uint8_t> customHeader_;
    std::vector<uint8_t> dataBlock_;
    std::string serialize();
    bool deserialize(const std::string& buff);
    std::size_t getSize() const;
    bool interpretErrorCode(int errorCode);
};

uint32_t changeEndianness(uint32_t swapMID);

}  // namespace xlsadapter
}  // namespace sensors
}  // namespace crf
