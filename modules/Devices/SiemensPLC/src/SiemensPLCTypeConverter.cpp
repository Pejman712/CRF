/* Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <stdexcept>

#include <Snap7/s7.hpp>

#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf {
namespace devices {
namespace siemensplc {

bool SiemensPLCTypeConverter::getBit(const std::string& buffer, unsigned int bitNumber,
    unsigned int offset) {
    if (buffer.size() < offset + 1) {
        throw std::invalid_argument("Bad buffer size");
    }
    if (bitNumber > 7) {
        throw std::invalid_argument("Bit number must be between 0 and 7");
    }
    return S7_GetBitAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset, bitNumber);
}

uint8_t SiemensPLCTypeConverter::getByte(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint8_t)) {
        throw std::invalid_argument("Bad buffer size in getByte");
    }
    return S7_GetByteAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

int8_t  SiemensPLCTypeConverter::getSInt(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(int8_t)) {
        throw std::invalid_argument("Bad buffer size in getSInt");
    }
    return S7_GetSIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

uint16_t SiemensPLCTypeConverter::getWord(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint16_t)) {
        throw std::invalid_argument("Bad buffer size in getWord");
    }
    return S7_GetWordAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

int16_t SiemensPLCTypeConverter::getShort(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(int16_t)) {
        throw std::invalid_argument("Bad buffer size in getShort");
    }
    return S7_GetIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

uint16_t SiemensPLCTypeConverter::getUShort(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint16_t)) {
        throw std::invalid_argument("Bad buffer size in getUShort");
    }
    return S7_GetUIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

uint32_t SiemensPLCTypeConverter::getDWord(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint32_t)) {
        throw std::invalid_argument("Bad buffer size in getDWord");
    }
    return S7_GetDWordAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

uint32_t SiemensPLCTypeConverter::getUInt(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint32_t)) {
        throw std::invalid_argument("Bad buffer size in getUInt");
    }
    return S7_GetUDIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

int32_t SiemensPLCTypeConverter::getInt(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(int32_t)) {
        throw std::invalid_argument("Bad buffer size in getInt");
    }
    return S7_GetDIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

int64_t SiemensPLCTypeConverter::getLWord(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(int64_t)) {
        throw std::invalid_argument("Bad buffer size in getLWord");
    }
    return S7_GetLWordAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

uint64_t SiemensPLCTypeConverter::getULong(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(uint64_t)) {
        throw std::invalid_argument("Bad buffer size in getULong");
    }
    return S7_GetULIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

int64_t SiemensPLCTypeConverter::getLong(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(int64_t))
            throw std::invalid_argument("Bad buffer size in getLong");
    return S7_GetLIntAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

float SiemensPLCTypeConverter::getFloat(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(float)) {
        throw std::invalid_argument("Bad buffer size in getFloat");
    }
    return S7_GetRealAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

double SiemensPLCTypeConverter::getDouble(const std::string& buffer, unsigned int offset) {
    if (buffer.size() < offset + sizeof(double)) {
        throw std::invalid_argument("Bad buffer size in getDouble");
    }
    return S7_GetLRealAt(reinterpret_cast<unsigned char*>(
        const_cast<char*>(buffer.c_str())), offset);
}

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
