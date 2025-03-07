/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "CANOpenDevices/CANOpenSdoManager.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

CANOpenSdoManager::CANOpenSdoManager(uint8_t id,
    std::shared_ptr<communication::cansocket::ICANSocket> socket,
    std::shared_ptr<ObjectDictionary> dictionary,
    const std::chrono::milliseconds& sdoResponseTimeout) :
        logger_("CANOpenSdoManager-" + std::to_string(id)),
        id_(id),
        socket_(socket),
        dictionary_(dictionary),
        sdoResponseTimeout_(sdoResponseTimeout) {}

template<typename T>
std::optional<T> CANOpenSdoManager::readRegister(const std::string& name) {
    auto reg = dictionary_->getRegister(name);
    if (!reg) {
        logger_->warn("Object dictionary does not contain the node ID register {}", name);
        return std::nullopt;
    }

    auto frame = buildSdoReadFrame(reg.get());
    if (!socket_->write(&frame)) {
        logger_->warn("Failed to write sdo read frame {}", name);
        return std::nullopt;
    }

    auto retval = dictionary_->waitForSdoResponse(reg.get(), sdoResponseTimeout_);
    if (!retval) {
        logger_->warn("Timeout in waiting for SDO response {}", name);
        return std::nullopt;
    }

    if ((retval->first.getName() == name) && (retval->second)) {
        return retval->first.getValue<T>();
    } else {
        return std::nullopt;
    }
}

template<typename T>
std::optional<T> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex) {
    auto reg = dictionary_->getRegister(index, subindex);
    if (!reg) {
        logger_->warn("Object dictionary does not contain the node ID register {}", index);
        return std::nullopt;
    }

    auto frame = buildSdoReadFrame(reg.get());
    if (!socket_->write(&frame)) {
        logger_->warn("Failed to write sdo read frame {}", reg->getName());
        return std::nullopt;
    }

    auto retval = dictionary_->waitForSdoResponse(reg.get(), sdoResponseTimeout_);
    if (!retval) {
        logger_->warn("Timeout in waiting for SDO response {}", reg->getName());
        return std::nullopt;
    }

    if ((retval->first.getIndex() == index) &&
        (retval->first.getSubindex() == subindex) &&
        (retval->second)) {
        return retval->first.getValue<T>();
    } else {
        return std::nullopt;
    }
}

template<typename T>
bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, T value) {
    auto reg = dictionary_->getRegister(index, subindex);
    if (!reg) {
        logger_->warn("Object dictionary does not contain the node ID register {}", index);
        return false;
    }

    auto frame = buildSdoWriteFrame(reg.get(), value);
    if (!socket_->write(&frame)) {
        logger_->warn("Failed to write sdo read frame {}", reg->getName());
        return false;
    }

    auto retval = dictionary_->waitForSdoResponse(reg.get(), sdoResponseTimeout_);
    if (!retval) {
        logger_->warn("Timeout in waiting for SDO response {}", reg->getName());
        return false;
    }

    if ((retval->first.getIndex() == index) &&
        (retval->first.getSubindex() == subindex) &&
        (retval->second)) {
            retval->first.setValue(value);
            return true;
    } else {
        return false;
    }
}

template<typename T>
bool CANOpenSdoManager::writeRegister(const std::string& name, T value) {
    auto reg = dictionary_->getRegister(name);
    if (!reg) {
        logger_->warn("Object dictionary does not contain the node ID register {}", name);
        return false;
    }

    auto frame = buildSdoWriteFrame(reg.get(), value);
    if (!socket_->write(&frame)) {
        logger_->warn("Failed to write sdo read frame {}", name);
        return false;
    }

    auto retval = dictionary_->waitForSdoResponse(reg.get(), sdoResponseTimeout_);
    if (!retval) {
        logger_->warn("Timeout in waiting for SDO response {}", name);
        return false;
    }

    if ((retval->first.getName() == name) && (retval->second)) {
        retval->first.setValue(value);
        return true;
    } else {
        logger_->warn("Erroneus response with name \"{}\" and value \"{}\"",
            retval->first.getName(), retval->second);
        return false;
    }
}

can_frame CANOpenSdoManager::buildSdoReadFrame(const ObjectDictionaryRegister& reg) {
    can_frame frame = {};
    frame.can_id = 0x600+ id_;
    frame.can_dlc = 0x08;
    frame.data[0] = 0x40;
    frame.data[1] = reg.getIndex() & 0xFF;
    frame.data[2] = (reg.getIndex() >> 8) & 0xFF;
    frame.data[3] = reg.getSubindex();
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return frame;
}

template<typename T>
can_frame CANOpenSdoManager::buildSdoWriteFrame(const ObjectDictionaryRegister& reg, T value) {
    can_frame frame = {};
    frame.can_id = 0x600+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = 0x2F;
    frame.data[1] = reg.getIndex() & 0xFF;
    frame.data[2] = (reg.getIndex() >> 8) & 0xFF;
    frame.data[3] = reg.getSubindex();

    if (reg.getSize() == 1) {
        frame.data[0] = 0x2F;
        frame.data[4] = value & 0xFF;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    } else if (reg.getSize() == 2) {
        frame.data[0] = 0x2B;
        frame.data[4] = value & 0xFF;
        frame.data[5] = (value >> 8) & 0xFF;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    } else if (reg.getSize() == 4) {
        frame.data[0] = 0x23;
        frame.data[4] = value & 0xFF;
        frame.data[5] = (value >> 8) & 0xFF;
        frame.data[6] = (value >> 16) & 0xFF;
        frame.data[7] = (value >> 24) & 0xFF;
    }

    return frame;
}

template std::optional<uint8_t> CANOpenSdoManager::readRegister(const std::string& name);
template std::optional<uint16_t> CANOpenSdoManager::readRegister(const std::string& name);
template std::optional<uint32_t> CANOpenSdoManager::readRegister(const std::string& name);
template std::optional<int8_t> CANOpenSdoManager::readRegister(const std::string& name);
template std::optional<int16_t> CANOpenSdoManager::readRegister(const std::string& name);
template std::optional<int32_t> CANOpenSdoManager::readRegister(const std::string& name);

template std::optional<uint8_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);
template std::optional<uint16_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);
template std::optional<uint32_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);
template std::optional<int8_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);
template std::optional<int16_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);
template std::optional<int32_t> CANOpenSdoManager::readRegister(uint16_t index, uint8_t subindex);

template bool CANOpenSdoManager::writeRegister(const std::string& name, uint8_t value);
template bool CANOpenSdoManager::writeRegister(const std::string& name, uint16_t value);
template bool CANOpenSdoManager::writeRegister(const std::string& name, uint32_t value);
template bool CANOpenSdoManager::writeRegister(const std::string& name, int8_t value);
template bool CANOpenSdoManager::writeRegister(const std::string& name, int16_t value);
template bool CANOpenSdoManager::writeRegister(const std::string& name, int32_t value);

template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, uint8_t value);
template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, uint16_t value);
template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, uint32_t value);
template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, int8_t value);
template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, int16_t value);
template bool CANOpenSdoManager::writeRegister(uint16_t index, uint8_t subindex, int32_t value);

template can_frame CANOpenSdoManager::buildSdoWriteFrame<uint8_t>(
    const ObjectDictionaryRegister& reg, uint8_t value);
template can_frame CANOpenSdoManager::buildSdoWriteFrame<uint16_t>(
    const ObjectDictionaryRegister& reg, uint16_t value);
template can_frame CANOpenSdoManager::buildSdoWriteFrame<uint32_t>(
    const ObjectDictionaryRegister& reg, uint32_t value);
template can_frame CANOpenSdoManager::buildSdoWriteFrame<int8_t>(
    const ObjectDictionaryRegister& reg, int8_t value);
template can_frame CANOpenSdoManager::buildSdoWriteFrame<int16_t>(
    const ObjectDictionaryRegister& reg, int16_t value);
template can_frame CANOpenSdoManager::buildSdoWriteFrame<int32_t>(
    const ObjectDictionaryRegister& reg, int32_t value);

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
