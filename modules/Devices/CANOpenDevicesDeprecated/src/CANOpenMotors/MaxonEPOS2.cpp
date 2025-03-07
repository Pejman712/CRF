/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <boost/optional.hpp>
#include <thread>

#include "CANOpenDevices/CANOpenDefinitions.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS2.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

MaxonEPOS2::MaxonEPOS2(uint8_t id, std::shared_ptr<communication::cansocket::ICANSocket> socket) :
    logger_("MaxonEPOS2-"+ std::to_string(id)),
    initialized_(false),
    heartbeatTime_(250),
    heartbeatConsumerId_(0x25),
    id_(id),
    socket_(socket),
    defaultSdoResponseTimeout_(std::chrono::milliseconds(100)),
    nmtIsAliveThreshold_(std::chrono::seconds(1)),
    quickStopActive_(false) {
    logger_->debug("CTor");
    std::string directory = __FILE__;
    directory = directory.substr(0, directory.find("CANOpenDevicesDeprecated"));
    directory += "CANOpenDevicesDeprecated/configurations/CiA402.json";
    objectDictionary_.reset(new ObjectDictionary(directory));
    sdoManager_.reset(new CANOpenSdoManager(id_, socket_, objectDictionary_,
        defaultSdoResponseTimeout_));
}

MaxonEPOS2::~MaxonEPOS2() {
    logger_->debug("DTor");
    deinitialize();
}

bool MaxonEPOS2::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    // setNMTState(0x81);
    // Allow time for node reset
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    setNMTState(0x80);

    // Check if the motor is responding properly
    auto nodeId = sdoManager_->readRegister<uint8_t>("nodeID");
    if (!nodeId) {
        logger_->warn("Failed to read nodeID");
        return false;
    }

    if (nodeId.value() != id_) {
        logger_->warn("The motor responded with a different can id: {}/{}", nodeId.value(), id_);
        return false;
    }

    // Setup producer and consumer heartbeat
    logger_->info("Setup producer and consumer heartbeat");
    if (!sdoManager_->writeRegister<uint32_t>("consumerHeartbeatTime/consumer1",
        ((heartbeatConsumerId_ << 16) | (heartbeatTime_+5)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("producerHeartbeatTime", heartbeatTime_)) {
        return false;
    }
    if (!sdoManager_->readRegister<uint16_t>("statusWord") ||
        !sdoManager_->readRegister<uint16_t>("controlWord") ||
        !sdoManager_->readRegister<uint8_t>("modesOfOperationDisplay")) {
            logger_->error("Failed to read initial status");
            return false;
    }
    if (!setupPDOs()) {
        logger_->error("Failed to setup the PDOs");
        return false;
    }
    setNMTState(0x01);
    initialized_ = true;
    return true;
}

bool MaxonEPOS2::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    disableOperation();
    initialized_ = false;
    return true;
}

int MaxonEPOS2::getCANID() {
    return id_;
}

bool MaxonEPOS2::isAlive() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return ((std::chrono::high_resolution_clock::now() -
        objectDictionary_->getNMTState().second) < nmtIsAliveThreshold_);
}

bool MaxonEPOS2::inFault() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & 0x0108) == 0x0108;
}

bool MaxonEPOS2::inQuickStop()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & 0x0117) == 0x0117;
}

bool MaxonEPOS2::isEnabled()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & 0x0137) == 0x0137;
}

bool MaxonEPOS2::isReadyToSwitchOn()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & 0x0121) == 0x0121;
}

std::optional<uint8_t> MaxonEPOS2::getNMTState() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getNMTState().first;
}

std::optional<int32_t> MaxonEPOS2::getVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getRegister("actualVelocity").value().getValue<int32_t>();
}

std::optional<int32_t> MaxonEPOS2::getPosition() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getRegister("actualPosition").value().getValue<int32_t>();
}

std::optional<int32_t> MaxonEPOS2::getCurrent() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return static_cast<int32_t>(
        objectDictionary_->getRegister("currentActualValueAveraged").value().getValue<int16_t>());
}

std::optional<uint16_t> MaxonEPOS2::getStatusWord() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getRegister("statusWord").value().getValue<uint16_t>();
}

std::optional<uint8_t> MaxonEPOS2::getModeOfOperation() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getRegister("modesOfOperationDisplay").value().getValue<uint8_t>();
}

std::optional<uint32_t> MaxonEPOS2::getDigitalInput() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return static_cast<uint32_t>(
        objectDictionary_->getRegister("digitalInputs/state").value().getValue<uint16_t>());
}

bool MaxonEPOS2::setDigitalOutput(uint32_t) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return false;
}

bool MaxonEPOS2::resetDigitalOutput(uint32_t) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return false;
}

bool MaxonEPOS2::enableOperation() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
    if (!isReadyToSwitchOn()) {
        logger_->error("Not ready to switch on: statusword {}",
            objectDictionary_->getRegister("statusWord").value().getValue<uint16_t>());
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", 0x000F)) return false;
    if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
    do {
        if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
        if (inFault()) {
            logger_->error("Motor did not enable");
            return false;
        }
    } while (!isEnabled());
    quickStopActive_ = false;
    return true;
}

bool MaxonEPOS2::disableOperation() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", 0x0007)) {
        return false;
    }
    do {
        if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
        if (inFault()) {
            logger_->error("Motor did not disable");
            return false;
        }
    } while (isEnabled());
    return true;
}

bool MaxonEPOS2::quickStop() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::ProfileVelocityMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfileVelocityMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }
    can_frame frame = {};
    frame.can_id = 0x300+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x02;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    quickStopActive_ = true;
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::stop() {
    return setVelocity(0);
}

bool MaxonEPOS2::shutdown() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", 0x0006)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
    if (!isReadyToSwitchOn()) {
        logger_->error("Not ready to switch on");
        return false;
    }
    return true;
}

bool MaxonEPOS2::faultReset() {
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", 0x0000)) return false;
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", 0x0080)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!sdoManager_->readRegister<uint16_t>("statusWord")) return false;
    if (inFault()) {
        logger_->error("Could not remove motor fault");
        return false;
    }
    if (!shutdown()) return false;
    return true;
}

bool MaxonEPOS2::setPosition(int32_t position, bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::ProfilePositionMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfilePositionMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }
    can_frame frame = {};
    frame.can_id = 0x200+id_;
    frame.can_dlc = 0x06;
    frame.data[0] = position & 0xFF;
    frame.data[1] = (position >> 8) & 0xFF;
    frame.data[2] = (position >> 16) & 0xFF;
    frame.data[3] = (position >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x6F : 0x2F);
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (socket_->write(&frame) != sizeof(frame)) {
        logger_->error("Failed to set position");
        return false;
    }
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x7F : 0x3F);
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setPosition(int32_t position, uint32_t velocity, bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::ProfilePositionMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfilePositionMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }
    uint32_t currentVelocity =
        objectDictionary_->getRegister("profileVelocity").value().getValue<uint32_t>();
    if (currentVelocity != velocity) {
        if (!sdoManager_->writeRegister("profileVelocity", velocity))
            return false;
    }

    can_frame frame = {};
    frame.can_id = 0x200+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = position & 0xFF;
    frame.data[1] = (position >> 8) & 0xFF;
    frame.data[2] = (position >> 16) & 0xFF;
    frame.data[3] = (position >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x6F : 0x2F);
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    if (socket_->write(&frame) != sizeof(frame)) {
        logger_->error("Failed to set position");
        return false;
    }
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x7F : 0x3F);
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
    bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    uint32_t currentVelocity =
        objectDictionary_->getRegister("profileVelocity").value().getValue<uint32_t>();
    uint32_t currentAcceleration =
        objectDictionary_->getRegister("profileAcceleration").value().getValue<uint32_t>();
    uint32_t currentDeceleration =
        objectDictionary_->getRegister("profileDeceleration").value().getValue<uint32_t>();

    if (currentVelocity != velocity) {
        if (!sdoManager_->writeRegister("profileVelocity", velocity))
            return false;
    }
    if (currentAcceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileAcceleration", acceleration))
            return false;
    }
    if (currentDeceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileDeceleration", acceleration))
            return false;
    }

    if (getModeOfOperation().value() != ModesOfOperation::ProfilePositionMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfilePositionMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }

    can_frame frame = {};
    frame.can_id = 0x200+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = position & 0xFF;
    frame.data[1] = (position >> 8) & 0xFF;
    frame.data[2] = (position >> 16) & 0xFF;
    frame.data[3] = (position >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x6F : 0x2F);
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (socket_->write(&frame) != sizeof(frame)) {
        logger_->error("Failed to set position");
        return false;
    }
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x7F : 0x3F);
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
    uint32_t deceleration, bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    uint32_t currentVelocity =
        objectDictionary_->getRegister("profileVelocity").value().getValue<uint32_t>();
    uint32_t currentAcceleration =
        objectDictionary_->getRegister("profileAcceleration").value().getValue<uint32_t>();
    uint32_t currentDeceleration =
        objectDictionary_->getRegister("profileDeceleration").value().getValue<uint32_t>();

    if (currentVelocity != velocity) {
        if (!sdoManager_->writeRegister("profileVelocity", velocity))
            return false;
    }
    if (currentAcceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileAcceleration", acceleration))
            return false;
    }
    if (currentDeceleration != deceleration) {
        if (!sdoManager_->writeRegister("profileDeceleration", deceleration))
            return false;
    }

    if (getModeOfOperation().value() != ModesOfOperation::ProfilePositionMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfilePositionMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }

    can_frame frame = {};
    frame.can_id = 0x200+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = position & 0xFF;
    frame.data[1] = (position >> 8) & 0xFF;
    frame.data[2] = (position >> 16) & 0xFF;
    frame.data[3] = (position >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x6F : 0x2F);
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (socket_->write(&frame) != sizeof(frame)) {
        logger_->error("Failed to set position");
        return false;
    }
    frame.data[4] = quickStopActive_ ? 0x02 : (relative ? 0x7F : 0x3F);
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::positionReached() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    auto statusWord = getStatusWord();
    if (!statusWord) return false;
    return ((statusWord.value() >> 10) & 0x01) == 0x01;
}

bool MaxonEPOS2::setVelocity(int32_t velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::ProfileVelocityMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfileVelocityMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }
    can_frame frame = {};
    frame.can_id = 0x300+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = velocity & 0xFF;
    frame.data[1] = (velocity >> 8) & 0xFF;
    frame.data[2] = (velocity >> 16) & 0xFF;
    frame.data[3] = (velocity >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : 0x0F;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setVelocity(int32_t velocity, uint32_t acceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    uint32_t currentAcceleration =
        objectDictionary_->getRegister("profileAcceleration").value().getValue<uint32_t>();
    uint32_t currentDeceleration =
        objectDictionary_->getRegister("profileDeceleration").value().getValue<uint32_t>();

    if (currentAcceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileAcceleration", acceleration))
            return false;
    }
    if (currentDeceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileDeceleration", acceleration))
            return false;
    }

    if (getModeOfOperation().value() != ModesOfOperation::ProfileVelocityMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfileVelocityMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }

    can_frame frame = {};
    frame.can_id = 0x300+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = velocity & 0xFF;
    frame.data[1] = (velocity >> 8) & 0xFF;
    frame.data[2] = (velocity >> 16) & 0xFF;
    frame.data[3] = (velocity >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : 0x0F;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    uint32_t currentAcceleration =
        objectDictionary_->getRegister("profileAcceleration").value().getValue<uint32_t>();
    uint32_t currentDeceleration =
        objectDictionary_->getRegister("profileDeceleration").value().getValue<uint32_t>();

    if (currentAcceleration != acceleration) {
        if (!sdoManager_->writeRegister("profileAcceleration", acceleration))
            return false;
    }
    if (currentDeceleration != deceleration) {
        if (!sdoManager_->writeRegister("profileDeceleration", deceleration))
            return false;
    }

    if (getModeOfOperation().value() != ModesOfOperation::ProfileVelocityMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::ProfileVelocityMode)) {
                logger_->error("Could not change mode of operation");
                return false;
        }
    }

    can_frame frame = {};
    frame.can_id = 0x300+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = velocity & 0xFF;
    frame.data[1] = (velocity >> 8) & 0xFF;
    frame.data[2] = (velocity >> 16) & 0xFF;
    frame.data[3] = (velocity >> 24) & 0xFF;
    frame.data[4] = quickStopActive_ ? 0x02 : 0x0F;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setTorque(int16_t torque) {
    logger_->warn("EPOS2 does not implement torque control");
    return false;
}

bool MaxonEPOS2::setCurrent(int16_t current) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (getModeOfOperation().value() != ModesOfOperation::CurrentMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation",
            ModesOfOperation::CurrentMode)) {
            logger_->error("Could not change mode of operation");
            return false;
        }
    }

    can_frame frame = {};
    frame.can_id = 0x400+id_;
    frame.can_dlc = 0x04;
    frame.data[0] = current & 0xFF;
    frame.data[1] = (current >> 8) & 0xFF;
    frame.data[2] = quickStopActive_ ? 0x02 : 0x0F;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return socket_->write(&frame) == sizeof(frame);
}

bool MaxonEPOS2::setProfileVelocity(uint32_t velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("profileVelocity", velocity);
}

bool MaxonEPOS2::setProfileAcceleration(uint32_t acceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("profileAcceleration", acceleration);
}

bool MaxonEPOS2::setProfileDeceleration(uint32_t deceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("profileDeceleration", deceleration);
}

bool MaxonEPOS2::setQuickstopDeceleration(uint32_t deceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("quickStopDeceleration", deceleration);
}

bool MaxonEPOS2::setMaxAcceleration(uint32_t maxAcceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("maxAcceleration", maxAcceleration);
}

bool MaxonEPOS2::setPositionLimits(std::pair<int32_t, int32_t> limits) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("softwarePositionLimit/minimum", limits.first) &&
        sdoManager_->writeRegister("softwarePositionLimit/maximum", limits.second);
}

uint32_t MaxonEPOS2::getProfileVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("profileVelocity")) {
        return 0;
    }
    return objectDictionary_-> getRegister("profileVelocity").value().getValue<uint32_t>();
}

uint32_t MaxonEPOS2::getProfileAcceleration() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("profileAcceleration")) {
        return 0;
    }
    return objectDictionary_->getRegister("profileAcceleration").value().getValue<uint32_t>();
}

uint32_t MaxonEPOS2::getProfileDeceleration() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("profileDeceleration")) {
        return 0;
    }
    return objectDictionary_->getRegister("profileDeceleration").value().getValue<uint32_t>();
}

uint32_t MaxonEPOS2::getQuickstopDeceleration() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("quickStopDeceleration")) {
        return 0;
    }
    return objectDictionary_->getRegister("quickStopDeceleration").value().getValue<uint32_t>();
}

uint32_t MaxonEPOS2::getMaximumVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("maxProfileVelocity")) {
        return 0;
    }
    return objectDictionary_->getRegister("maxProfileVelocity").value().getValue<uint32_t>();
}

uint32_t MaxonEPOS2::getMaximumAcceleration() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("maxAcceleration")) {
        return 0;
    }
    return objectDictionary_->getRegister("maxAcceleration").value().getValue<uint32_t>();
}

std::pair<int32_t, int32_t> MaxonEPOS2::getPositionLimits() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::pair<int32_t, int32_t>(0, 0);
    }

    if (!sdoManager_->readRegister<int32_t>("softwarePositionLimit/minimum")) {
        return std::pair<int32_t, int32_t>(0, 0);
    }
    int32_t minimum = objectDictionary_->getRegister(
        "softwarePositionLimit/minimum").value().getValue<int32_t>();

    if (!sdoManager_->readRegister<int32_t>("softwarePositionLimit/maximum")) {
        return std::pair<int32_t, int32_t>(0, 0);
    }
    int32_t maximum = objectDictionary_->getRegister(
        "softwarePositionLimit/maximum").value().getValue<int32_t>();

    return std::pair<int32_t, int32_t>(minimum, maximum);
}

std::shared_ptr<ObjectDictionary> MaxonEPOS2::getObjectDictionary() {
    return objectDictionary_;
}


bool MaxonEPOS2::setupPDOs() {
    logger_->info("Setting up TPDO1");
    if (!sdoManager_->writeRegister<uint32_t>(
        "transmitPDO1Parameter/cobID", ((0x4000 << 16) | (0x0180 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Mapping/numberOfObjects", 0)) {
        return false;
    }
    std::vector<ObjectDictionaryRegister> pdo1Registers;
    auto reg = objectDictionary_->getRegister("actualVelocity").value();
    pdo1Registers.push_back(reg);
    uint32_t mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8); // NOLINT
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("actualPosition").value();
    pdo1Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Mapping/numberOfObjects", 2)) {
        return false;
    }
    objectDictionary_->addPDOMapping(3, pdo1Registers);

    logger_->info("Setting up TPDO2");
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Parameter/cobID",
        ((0x4000 << 16) | (0x0280 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Mapping/numberOfObjects", 0)) {
        return false;
    }
    std::vector<ObjectDictionaryRegister> pdo2Registers;
    reg = objectDictionary_->getRegister("currentActualValueAveraged").value();
    pdo2Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("digitalInputs/state").value();
    pdo2Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Mapping/numberOfObjects", 2)) {
        return false;
    }
    objectDictionary_->addPDOMapping(5, pdo2Registers);

    // Setting TPDO3
    logger_->info("Setting up TPDO3");
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Parameter/cobID",
        ((0x4000 << 16) | (0x0380 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Mapping/numberOfObjects", 0)) {
        return false;
    }
    std::vector<ObjectDictionaryRegister> pdo3Registers;
    reg = objectDictionary_->getRegister("statusWord").value();
    pdo3Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("modesOfOperationDisplay").value();
    pdo3Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Mapping/numberOfObjects", 2)) {
        return false;
    }
    objectDictionary_->addPDOMapping(7, pdo3Registers);

    // Disable TPDO4
    logger_->info("Disable TPDO4");
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO4Mapping/numberOfObjects", 0)) {
        return false;
    }

    // Setting RPDO1
    logger_->info("Setting RPDO1");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Parameter/cobID",
        ((0x4000 << 16) | (0x0200 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Mapping/numberOfObjects", 0)) {
        return false;
    }
    reg = objectDictionary_->getRegister("targetPosition").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("controlWord").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Mapping/numberOfObjects", 2)) {
        return false;
    }

    // Setting RPDO2
    logger_->info("Setting RPDO2");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Parameter/cobID",
        ((0x4000 << 16) | (0x0300 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Mapping/numberOfObjects", 0)) {
        return false;
    }
    reg = objectDictionary_->getRegister("targetVelocity").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("controlWord").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Mapping/numberOfObjects", 2)) {
        return false;
    }

    // Setting RPDO3
    logger_->info("Setting RPDO3");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO3Parameter/cobID",
        ((0x4000 << 16) | (0x0400 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO3Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO3Mapping/numberOfObjects", 0)) {
        return false;
    }
    reg = objectDictionary_->getRegister("currentModeSettingValue").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO3Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("controlWord").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO3Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO3Mapping/numberOfObjects", 2)) {
        return false;
    }

    // Setting RPDO4
    logger_->info("Setting RPDO4");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO4Parameter/cobID",
        ((0x4000 << 16) | (0x0500 + id_)))) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO4Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO4Mapping/numberOfObjects", 0)) {
        return false;
    }
    reg = objectDictionary_->getRegister("modesOfOperation").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO4Mapping/mappedObject1",
        mappedObjectValue)) {
            return false;
    }
    reg = objectDictionary_->getRegister("controlWord").value();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO4Mapping/mappedObject2",
        mappedObjectValue)) {
            return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO4Mapping/numberOfObjects", 2)) {
        return false;
    }

    return true;
}

bool MaxonEPOS2::setNMTState(uint8_t state) {
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
