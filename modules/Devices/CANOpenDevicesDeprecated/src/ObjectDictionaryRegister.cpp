/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "CANOpenDevices/ObjectDictionaryRegister.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

ObjectDictionaryRegister::ObjectDictionaryRegister(const std::string& name, uint8_t size,
    uint16_t index, uint8_t subindex) :
    name_(name),
    size_(size),
    index_(index),
    subindex_(subindex),
    lastUpdate_(),
    value_() {
    if ((size != 1) && (size != 2) && (size != 4)) {
        throw std::runtime_error("Not possible to create a register with this size");
    }
    value_.resize(size);
    for (int i=0; i < size_; i++) {
        value_[i] = 0x00;
    }
}

std::string ObjectDictionaryRegister::getName() const {
    return name_;
}

uint8_t ObjectDictionaryRegister::getSize() const {
    return size_;
}

uint16_t ObjectDictionaryRegister::getIndex() const {
    return index_;
}

uint8_t ObjectDictionaryRegister::getSubindex() const {
    return subindex_;
}

template<typename T>
bool ObjectDictionaryRegister::setValue(T value) {
    if (sizeof(T) != size_) {
        return false;
    }
    char buffer[4];
    std::memcpy(buffer, &value, size_);
    value_ = std::string(buffer, size_);
    lastUpdate_ = std::chrono::high_resolution_clock::now();
    return true;
}

bool ObjectDictionaryRegister::setValue(const std::string& buf) {
    if (buf.length() != size_) {
        return false;
    }
    value_ = buf;
    lastUpdate_ = std::chrono::high_resolution_clock::now();
    return true;
}

template<typename T>
T ObjectDictionaryRegister::getValue() const {
    if (sizeof(T) != size_) {
        throw std::runtime_error("Requested value size different than buffer size for register" +
            name_ + ": requested size "+ std::to_string(sizeof(T)) +
            ", register size " + std::to_string(size_));
    }
    T value;
    std::memcpy(&value, value_.c_str(), sizeof(T));
    return value;
}

// Definitiion of all the templated functions only for the following types
template uint8_t ObjectDictionaryRegister::getValue<uint8_t>() const;
template int8_t ObjectDictionaryRegister::getValue<int8_t>() const;
template uint16_t ObjectDictionaryRegister::getValue<uint16_t>() const;
template int16_t ObjectDictionaryRegister::getValue<int16_t>() const;
template uint32_t ObjectDictionaryRegister::getValue<uint32_t>() const;
template int32_t ObjectDictionaryRegister::getValue<int32_t>() const;

template bool ObjectDictionaryRegister::setValue<uint8_t>(uint8_t);
template bool ObjectDictionaryRegister::setValue<int8_t>(int8_t);
template bool ObjectDictionaryRegister::setValue<uint16_t>(uint16_t);
template bool ObjectDictionaryRegister::setValue<int16_t>(int16_t);
template bool ObjectDictionaryRegister::setValue<uint32_t>(uint32_t);
template bool ObjectDictionaryRegister::setValue<int32_t>(int32_t);

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
