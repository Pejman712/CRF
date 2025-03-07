/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <memory>
#include <string>
#include <utility>

#include "CANOpenDevices/CANOpenIOs/CANOpenIOModule.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

CANOpenIOModule::CANOpenIOModule(uint8_t id,
    std::shared_ptr<communication::cansocket::ICANSocket> socket) :
    logger_("CANOpenIOModule-"+ std::to_string(id)),
    initialized_(false),
    heartbeatTime_(250),
    heartbeatConsumerId_(0x25),
    id_(id),
    socket_(socket),
    defaultSdoResponseTimeout_(std::chrono::milliseconds(100)),
    nmtIsAliveThreshold_(std::chrono::seconds(1)) {
    logger_->debug("CTor");
    std::string directory = __FILE__;
    directory = directory.substr(0, directory.find("CANOpenDevicesDeprecated"));
    directory += "CANOpenDevicesDeprecated/configurations/CiA401.json";
    objectDictionary_.reset(new ObjectDictionary(directory));
    sdoManager_.reset(new CANOpenSdoManager(id_, socket_, objectDictionary_,
        defaultSdoResponseTimeout_));
}

CANOpenIOModule::~CANOpenIOModule() {
    logger_->debug("DTor");
    if (initialized_)
        deinitialize();
}

bool CANOpenIOModule::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    setNMTState(0x80);

    logger_->info("Setup producer and consumer heartbeat");
    if (!sdoManager_->writeRegister<uint32_t>("consumerHeartbeatTime/consumer1",
        ((heartbeatConsumerId_ << 16) | (heartbeatTime_+5)))) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("producerHeartbeatTime", heartbeatTime_)) {
        return false;
    }
    if (!sdoManager_->readRegister<uint32_t>("deviceType")) {
        return false;
    }

    // We don't check the retval of these functions because some IO devices
    // don't implement these registers
    sdoManager_->readRegister<uint8_t>("input8bit/count");
    if (auto value = sdoManager_->readRegister<uint8_t>("output8bit/count")) {
        // It reinitialize the outputs
        auto outputDOIndex = objectDictionary_->getRegister("output8bit/count").value().getIndex();
        for (uint8_t i = 0; i < value.value(); i++) {
            sdoManager_->writeRegister(outputDOIndex, i+1, 0);
        }
    }

    setNMTState(0x01);
    initialized_ = true;
    return true;
}

bool CANOpenIOModule::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    initialized_ = false;
    return true;
}

int CANOpenIOModule::getCANID() {
    logger_->debug("getCANID");
    return id_;
}

bool CANOpenIOModule::isAlive() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return ((std::chrono::high_resolution_clock::now() -
        objectDictionary_->getNMTState().second) < nmtIsAliveThreshold_);
}

std::shared_ptr<ObjectDictionary> CANOpenIOModule::getObjectDictionary() {
    logger_->debug("getObjectDictionary");
    return objectDictionary_;
}

bool CANOpenIOModule::hasDigitalInputs() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::bitset<32> set(objectDictionary_->getRegister("deviceType").value().getValue<uint32_t>());
    return set[16];
}

uint16_t CANOpenIOModule::getDigitalInputCount() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    return objectDictionary_->getRegister("input8bit/count").value().getValue<uint8_t>()*8;
}

std::optional<bool> CANOpenIOModule::getDigitalInputState(uint16_t index) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (index > getDigitalInputCount()) {
        logger_->warn("Requested index higher than amount of digital outputs");
        return std::nullopt;
    }
    auto reg = objectDictionary_->getRegister("input8bit/count");
    uint8_t subindex = index/8 + 1;
    auto value = sdoManager_->readRegister<uint8_t>(reg->getIndex(), subindex);
    if (!value) {
        return std::nullopt;
    }
    uint8_t offset = index%8;
    std::bitset<8> set(value.value());
    return static_cast<bool>(set[offset]);
}

std::optional<bool> CANOpenIOModule::getDigitalInputPolarity(uint16_t index) {
    return std::nullopt;
}

bool CANOpenIOModule::setDigitalInputPolarity(uint16_t index, bool polarity) {
    return false;
}

bool CANOpenIOModule::hasDigitalOutputs() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::bitset<32> set(objectDictionary_->getRegister("deviceType").value().getValue<uint32_t>());
    return set[17];
}

uint16_t CANOpenIOModule::getDigitalOutputCount() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    return objectDictionary_->getRegister("output8bit/count").value().getValue<uint8_t>()*8;
}

bool CANOpenIOModule::setDigitalOutputState(uint16_t index, bool state) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (index > getDigitalOutputCount()) {
        logger_->warn("Requested index higher than amount of digital outputs");
        return false;
    }
    auto reg = objectDictionary_->getRegister("output8bit/count");
    uint8_t subindex = index/8 + 1;
    auto value = sdoManager_->readRegister<uint8_t>(reg->getIndex(), subindex);
    if (!value) {
        return false;
    }

    uint8_t offset = index%8;
    uint8_t newValue = value.value();
    if (state) {
        newValue |= (0x01 << offset);
    } else {
        newValue &= ~(0x01 << offset);
    }

    return sdoManager_->writeRegister<uint8_t>(reg->getIndex(), subindex, newValue);
}

std::optional<bool> CANOpenIOModule::getDigitalOutputState(uint16_t index) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (index > getDigitalOutputCount()) {
        logger_->warn("Requested index higher than amount of digital outputs");
        return std::nullopt;
    }
    auto reg = objectDictionary_->getRegister("output8bit/count");
    uint8_t subindex = index/8 + 1;
    auto value = sdoManager_->readRegister<uint8_t>(reg->getIndex(), subindex);
    if (!value) {
        return std::nullopt;
    }
    uint8_t offset = index%8;
    std::bitset<8> set(value.value());
    return static_cast<bool>(set[offset]);
}

bool CANOpenIOModule::setDigitalOutputPolarity(uint16_t index, bool polarity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (index > getDigitalOutputCount()) {
        logger_->warn("Requested index higher than amount of digital outputs");
        return false;
    }
    auto reg = objectDictionary_->getRegister("changeOutputPolarity8bit/count");
    uint8_t subindex = index/8 + 1;
    auto value = sdoManager_->readRegister<uint8_t>(reg->getIndex(), subindex);
    if (!value) {
        return false;
    }
    uint8_t offset = index%8;
    uint8_t newValue = value.value();

    if (polarity) {
        newValue |= (0x01 << offset);
    } else {
        newValue &= ~(0x01 << offset);
    }

    return sdoManager_->writeRegister<uint8_t>(reg->getIndex(), subindex, newValue);
}

std::optional<bool> CANOpenIOModule::getDigitalOutputPolarity(uint16_t index) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (index > getDigitalOutputCount()) {
        logger_->warn("Requested index higher than amount of digital outputs");
        return std::nullopt;
    }
    auto reg = objectDictionary_->getRegister("changeOutputPolarity8bit/count");
    uint8_t subindex = index/8 + 1;
    auto value = sdoManager_->readRegister<uint8_t>(reg->getIndex(), subindex);
    if (!value) {
        logger_->warn("Failed to read changeOutputPolarity8bit");
        return std::nullopt;
    }

    uint8_t offset = index%8;
    std::bitset<8> set(value.value());
    return static_cast<bool>(set[offset]);
}

bool CANOpenIOModule::hasAnalogueInputs() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::bitset<32> set(objectDictionary_->getRegister("deviceType").value().getValue<uint32_t>());
    return set[18];
}

uint8_t CANOpenIOModule::getAnalogueInputCount() {
    return 0;
}

std::optional<int32_t> CANOpenIOModule::getAnalogueInput(uint8_t index) {
    return std::nullopt;
}

bool CANOpenIOModule::hasAnalogueOutputs() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::bitset<32> set(objectDictionary_->getRegister("deviceType").value().getValue<uint32_t>());
    return set[19];
}

uint8_t CANOpenIOModule::getAnalogueOutputCount() {
    return 0;
}

bool CANOpenIOModule::setAnalogueOutput(uint8_t index, int32_t value) {
    return false;
}

std::optional<int32_t> CANOpenIOModule::getAnalogueOutput(uint8_t index) {
    return std::nullopt;
}

bool CANOpenIOModule::setNMTState(uint8_t state) {
    can_frame frame = {};
    frame.can_id = 0x00;
    frame.can_dlc = 0x02;
    frame.data[0] = state;
    frame.data[1] = id_;
    return socket_->write(&frame) == sizeof(frame);
}

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
