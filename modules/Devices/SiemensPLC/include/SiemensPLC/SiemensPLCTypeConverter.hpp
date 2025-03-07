/* Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

namespace crf {
namespace devices {
namespace siemensplc {

class SiemensPLCTypeConverter {
 public:
    SiemensPLCTypeConverter() = delete;

    static bool getBit(const std::string&, unsigned int bitNumber, unsigned int offset = 0);
    static uint8_t getByte(const std::string&, unsigned int offset = 0);
    static int8_t  getSInt(const std::string&, unsigned int offset = 0);
    static uint16_t getWord(const std::string&, unsigned int offset = 0);
    static int16_t getShort(const std::string&, unsigned int offset = 0);
    static uint16_t getUShort(const std::string&, unsigned int offset = 0);
    static uint32_t getDWord(const std::string&, unsigned int offset = 0);
    static uint32_t getUInt(const std::string&, unsigned int offset = 0);
    static int32_t getInt(const std::string&, unsigned int offset = 0);
    static int64_t getLWord(const std::string&, unsigned int offset = 0);
    static uint64_t getULong(const std::string&, unsigned int offset = 0);
    static int64_t getLong(const std::string&, unsigned int offset = 0);
    static float getFloat(const std::string&, unsigned int offset = 0);
    static double getDouble(const std::string&, unsigned int offset = 0);
};

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
