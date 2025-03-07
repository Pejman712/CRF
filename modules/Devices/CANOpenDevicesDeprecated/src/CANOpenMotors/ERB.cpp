/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thomas Breant CERN EN/SMM/MRO
 * Contributor: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <bitset>
#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <thread>

#include "CANOpenDevices/CANOpenMotors/ERB.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

ERB::ERB(const uint8_t& id, std::shared_ptr<communication::cansocket::ICANSocket> socket,
    const uint8_t& mode):
    id_(id),
    socket_(socket),
    positionMode_(mode),
    quickStopActive_(false),
    initialized_(false),
    logger_("ERB-"+ std::to_string(id)) {
        logger_->debug("CTor");
        std::string directory = __FILE__;
        directory = directory.substr(0, directory.find("CANOpenDevicesDeprecated"));
        directory += "CANOpenDevicesDeprecated/configurations/ERB145-115.json";
        objectDictionary_.reset(new ObjectDictionary(directory));
        sdoManager_.reset(new CANOpenSdoManager(id_, socket_, objectDictionary_,
            defaultSdoResponseTimeout_));
}

ERB::~ERB() {
    logger_->debug("DTor");
    deinitialize();
}

bool ERB::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (!setNMTState(NMT_RESETNODE)) {
        logger_->error("failed setting NMT state : NMT_RESETNODE");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!setNMTState(NMT_ENTERPREOPERATIONAL)) {
        logger_->error("failed setting NMT state : NMT_ENTERPREOPERATIONAL");
        return false;
    }
    if (!setModeOfOperation(positionMode_)) {
        logger_->error("failed setting mode of operation to {}", positionMode_);
        return false;
    }
    if (!sdoManager_->readRegister<uint8_t>("serveurSDO2Parameter/nodeIDofSDOclient")) {
        logger_->warn("Failed to read nodeID");
        return false;
    }
    if (objectDictionary_->getRegister("serveurSDO2Parameter/nodeIDofSDOclient").get().getValue<uint8_t>() != id_) {  // NOLINT
        logger_->warn("The motor responded with a different can id: {}/{}",
            objectDictionary_->getRegister("serveurSDO2Parameter/nodeIDofSDOclient").get().getValue<uint8_t>(),  // NOLINT
            id_);
        return false;
    }

    // Setup heartbeat
    logger_->info("Setup producer and consumer heartbeat");
    if (!sdoManager_->writeRegister<uint32_t>("consumerHeartbeatTime/consumer1", ((heartbeatConsumerId_ << 16) | (heartbeatTime_+5)))) {  // NOLINT
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
    if (!setNMTState(NMT_STARTREMOTENODE)) {
        logger_->error("failed setting NMT state : NMT_STARTREMOTENODE");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!setupPDOs()) {
        logger_->error("Failed to setup the PDOs");
        return false;
    }
    initialized_ = true;

    // check if the node isn't in fault
    if (inFault()) faultReset();
    return true;
}

bool ERB::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!setModeOfOperation(positionMode_)) {
        return false;
    }
    if (inFault()) faultReset();
    disableOperation();
    initialized_ = false;
    return true;
}

int ERB::getCANID() {
    return id_;
}

bool ERB::isAlive() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return ((std::chrono::high_resolution_clock::now() - objectDictionary_->getNMTState().second) < nmtIsAliveThreshold_); // NOLINT
}

bool ERB::inFault() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & FAULT) == FAULT;
}

bool ERB::inQuickStop() {
        if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return ((getStatusWord().value() & IN_QUICK_STOP) == IN_QUICK_STOP && !isEnabled());
}

bool ERB::isSwitchOnDisabled() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & SWITCHED_ON_DISABLED) == SWITCHED_ON_DISABLED;
}

bool ERB::isSwitchedOn() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & SWITCHED_ON) == SWITCHED_ON;
}

bool ERB::isEnabled() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & OPERATION_ENABLED) == OPERATION_ENABLED;
}

bool ERB::isReadyToSwitchOn() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return (getStatusWord().value() & READY_TO_SWITCH_ON) == READY_TO_SWITCH_ON;
}

std::optional<uint8_t> ERB::getNMTState() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    return objectDictionary_->getNMTState().first;
}

std::optional<int32_t> ERB::getVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<int32_t>("actualVelocity")) return std::nullopt;
    return objectDictionary_->getRegister("actualVelocity").get().getValue<int32_t>();
}

std::optional<int32_t> ERB::getPosition() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<int32_t>("actualPosition")) return std::nullopt;
    return objectDictionary_->getRegister("actualPosition").get().getValue<int32_t>();
}

std::optional<int32_t> ERB::getCurrent() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<int16_t>("actualCurrent")) return std::nullopt;
    return objectDictionary_->getRegister("actualCurrent").get().getValue<int16_t>();
}

std::optional<int32_t> ERB::getTorque() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<int16_t>("actualTorque")) return std::nullopt;
    return objectDictionary_->getRegister("actualTorque").get().getValue<int16_t>();
}

std::optional<uint16_t> ERB::getStatusWord() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<uint16_t>("statusWord")) return std::nullopt;
    return objectDictionary_->getRegister("statusWord").get().getValue<uint16_t>();
}

std::optional<uint8_t> ERB::getModeOfOperation() {
    return objectDictionary_->getRegister("modesOfOperationDisplay").get().getValue<uint8_t>();
}

std::optional<uint32_t> ERB::getDigitalInput() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!sdoManager_->readRegister<uint32_t>("digitalInputs")) return std::nullopt;
    return objectDictionary_->getRegister("digitalInputs").get().getValue<uint32_t>();
}

bool ERB::setDigitalOutput(uint32_t) {
    logger_->debug("setDigitalOutput");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setDigitalOutput Not implemented");
    return false;
}

bool ERB::resetDigitalOutput(uint32_t) {
    logger_->debug("resetDigitalOutput");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("resetDigitalOutput not implemented");
    return false;
}

bool ERB::enableOperation() {
    logger_->debug("enableOperation");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->info("Enabling operation");
    if (!isReadyToSwitchOn()) {
        if (isSwitchOnDisabled()) {
            if (!sdoManager_->writeRegister<uint16_t>("controlWord", SHUTDOWN)) return false;
            while (!isReadyToSwitchOn()) {
                if (inFault()) {
                    logger_->error("Motor did not enable");
                    return false;
                }
            }
            logger_->debug("Ready to switch on : statusword {}", getStatusWord().value());
        }
    }

    if (!sdoManager_->writeRegister<uint16_t>("controlWord", SWITCH_ON)) return false;
    while (!isSwitchedOn()) {
        if (inFault()) {
            logger_->error("Motor did not enable");
            return false;
        }
    }

    logger_->debug("Switched on : statusword {}", getStatusWord().value());
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", ENABLE_OPERATION)) return false;
    while (!isEnabled()) {
        if (inFault()) {
            logger_->error("Motor did not enable");
            return false;
        }
    }
    logger_->debug("Operation enabled : statusword {}", getStatusWord().value());
    quickStopActive_ = false;
    return true;
}

bool ERB::disableOperation() {
    logger_->debug("disableOperation");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!isEnabled()) {
        logger_->warn("Operation already disabled");
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", DISABLE_OPERATION)) return false;
    while (isEnabled()) {
        if (inFault()) {
            logger_->error("Motor did not disable");
            return false;
        }
    }
    logger_->debug("Operation disabled : statusword {}", getStatusWord().value());
    return true;
}

bool ERB::stop() {
    logger_->debug("stop");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return setVelocity(0, 0, maxDeceleration_);
}

bool ERB::quickStop() {
    logger_->debug("quickStop");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::VelocityMode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation", ModesOfOperation::VelocityMode)) {  // NOLINT
            logger_->error("Could not change mode of operation");
            return false;
        }
    }

    // send PDO with the control word to stop
    can_frame frame = {};
    frame.can_id = 0x300+id_;
    frame.can_dlc = 0x08;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = QUICK_STOP;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    quickStopActive_ = true;
    return socket_->write(&frame) == sizeof(frame);
}

bool ERB::shutdown() {
    logger_->debug("shutdown");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", SHUTDOWN)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!isReadyToSwitchOn()) {
        logger_->error("Not ready to switch on");
        return false;
    }
    return true;
}

bool ERB::faultReset() {
    logger_->debug("faultReset");
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", FAULT_RESET1)) return false;
    if (!sdoManager_->writeRegister<uint16_t>("controlWord", FAULT_RESET2)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (inFault()) {
        logger_->error("Could not remove motor fault");
        return false;
    }
    if (!shutdown()) return false;
    return true;
}

bool ERB::setPosition(int32_t position, bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!isEnabled()) {
        logger_->error("Operation is not enabled");
        return false;
    }
    if (positionMode_ == ModesOfOperation::ProfilePositionMode) {
        if (getModeOfOperation().value() != ModesOfOperation::ProfilePositionMode) {
            if (!setModeOfOperation(ModesOfOperation::ProfilePositionMode)) {
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
    logger_->warn("setPosition not implemented in mode {}", positionMode_);
    return false;
}

bool ERB::setPosition(int32_t position, uint32_t velocity, bool relative) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getProfileVelocity() != velocity) {
        if (!sdoManager_->writeRegister("profileVelocity", velocity)) return false;
    }
    return setPosition(position, relative);
}

bool ERB::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration, bool relative) {  // NOLINT
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getProfileAcceleration() != acceleration) {
        if (!sdoManager_->writeRegister("profileAcceleration", acceleration)) return false;
    }
    return setPosition(position, velocity, relative);
}

bool ERB::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, bool relative) {  // NOLINT
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (getProfileDeceleration() != deceleration) {
        if (!sdoManager_->writeRegister("profileDeceleration", deceleration)) return false;
    }
    return setPosition(position, velocity, acceleration, relative);
}

bool ERB::positionReached() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    std::optional<uint16_t> statusWord = getStatusWord();
    if (!statusWord) return false;
    return ((statusWord.value() >> 10) & 0x01) == 0x01;
}

bool ERB::setVelocity(int32_t velocity) {
    logger_->debug("setVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!isEnabled()) {
        logger_->error("Operation is not enabled");
        return false;
    }
    if (getModeOfOperation().value() != ModesOfOperation::VelocityMode) {
        if (!setModeOfOperation(ModesOfOperation::VelocityMode)) return false;
    }
    can_frame frame = {};
    frame.can_id = 0x300 + id_;
    frame.can_dlc = 0x04;
    frame.data[0] = velocity & 0xFF;
    frame.data[1] = (velocity >> 8) & 0xFF;
    frame.data[2] = quickStopActive_ ? 0x02 : 0x7F;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return socket_->write(&frame) == sizeof(frame);
}

bool ERB::setVelocity(int32_t velocity, uint32_t acceleration) {
    logger_->debug("setVelocity with acceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->readRegister<uint32_t>("vlVelocityAcceleration/DeltaSpeed")) return false;
    uint32_t currentAcceleration = objectDictionary_->getRegister("vlVelocityAcceleration/DeltaSpeed").get().getValue<uint32_t>(); // NOLINT
    if (currentAcceleration != acceleration) {
        if (!sdoManager_->writeRegister<uint32_t>("vlVelocityAcceleration/DeltaSpeed", acceleration)) return false;  // NOLINT
    }
    return setVelocity(velocity);
}

bool ERB::setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) {
    logger_->debug("setVelocity with acceleration and deceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->readRegister<uint32_t>("vlVelocityDeceleration/DeltaSpeed")) return false;
    uint32_t currentDeceleration = objectDictionary_->getRegister("vlVelocityDeceleration/DeltaSpeed").get().getValue<uint32_t>();  // NOLINT
    if (currentDeceleration != deceleration) {
        if (!sdoManager_->writeRegister<uint32_t>("vlVelocityDeceleration/DeltaSpeed", deceleration)) return false;  // NOLINT
    }
    return setVelocity(velocity, acceleration);
}

bool ERB::setTorque(int16_t torque) {
    logger_->debug("setTorque");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setTorque not implemented");
    return false;
}

bool ERB::setCurrent(int16_t) {
    logger_->debug("setCurrent");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setCurrent Not supported");
    return false;
}

bool ERB::setProfileVelocity(uint32_t velocity) {
    logger_->debug("setProfileVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("profileVelocity", velocity);
}

bool ERB::setProfileAcceleration(uint32_t acceleration) {
    logger_->debug("setProfileAcceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    return sdoManager_->writeRegister("profileAcceleration", acceleration);
}

bool ERB::setProfileDeceleration(uint32_t) {
    logger_->debug("setProfileDeceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setProfileDeceleration Not supported");
    return false;
}

bool ERB::setQuickstopDeceleration(uint32_t) {
    logger_->debug("setQuickstopDeceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setQuickstopDeceleration Not supported");
    return false;
}

bool ERB::setMaxAcceleration(uint32_t) {
    logger_->debug("setMaxAcceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    logger_->warn("setMaxAcceleration Not supported");
    return false;
}

bool ERB::setPositionLimits(std::pair<int32_t, int32_t> limits) {
    logger_->debug("setPositionLimits");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!sdoManager_->writeRegister("softwarePositionLimit/minimum", limits.first)) return false;
    return sdoManager_->writeRegister("softwarePositionLimit/maximum", limits.second);
}

uint32_t ERB::getProfileVelocity() {
    logger_->debug("getProfileVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("profileVelocity")) return 0;
    return objectDictionary_->getRegister("profileVelocity").get().getValue<uint32_t>();
}

uint32_t ERB::getProfileAcceleration() {
    logger_->debug("getProfileAcceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("profileAcceleration")) return 0;
    return objectDictionary_->getRegister("profileAcceleration").get().getValue<uint32_t>();
}

uint32_t ERB::getProfileDeceleration() {
    logger_->debug("getProfileDeceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    // profile deceleration only usable in the velocity mode
    if (getModeOfOperation().value() == ModesOfOperation::VelocityMode) {
        if (!sdoManager_->readRegister<uint32_t>("vlVelocityDeceleration/DeltaSpeed")) return 0;
        uint32_t deltaSpeed = objectDictionary_->getRegister("vlVelocityDeceleration/DeltaSpeed").get().getValue<uint32_t>();  // NOLINT
        if (!sdoManager_->readRegister<uint16_t>("vlVelocityDeceleration/DeltaTime")) return 0;
        uint16_t deltaTime = objectDictionary_->getRegister("vlVelocityDeceleration/DeltaTime").get().getValue<uint16_t>();  // NOLINT
        return deltaSpeed/deltaTime;
    }
    logger_->warn("getProfileDeceleration Not supported for this mode");
    return 0;
}

uint32_t ERB::getQuickstopDeceleration() {
    logger_->debug("getQuickstopDeceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    logger_->warn("getQuickstopDeceleration Not supported");
    return 0;
}

uint32_t ERB::getMaximumVelocity() {
    logger_->debug("getMaximumVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    if (!sdoManager_->readRegister<uint32_t>("vlVelocityMinMaxAmount/MaxAmount")) return 0;
    return objectDictionary_->getRegister("vlVelocityMinMaxAmount/MaxAmount").get().getValue<uint32_t>();  // NOLINT
}

uint32_t ERB::getMaximumAcceleration() {
    logger_->debug("getMaximumAcceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return 0;
    }
    logger_->warn("getMaximumAcceleration Not supported");
    return 0;
}

std::pair<int32_t, int32_t> ERB::getPositionLimits() {
    logger_->debug("getPositionLimits");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::pair<int32_t, int32_t>(0, 0);
    }
    if (!sdoManager_->readRegister<int32_t>("softwarePositionLimit/minimum")) {
        return std::pair<int32_t, int32_t>(0, 0);
    }
    int32_t minimum =
        objectDictionary_->getRegister("softwarePositionLimit/minimum").get().getValue<int32_t>();

    if (!sdoManager_->readRegister<int32_t>("softwarePositionLimit/maximum")) {
        return std::pair<int32_t, int32_t>(0, 0);
    }
    int32_t maximum =
        objectDictionary_->getRegister("softwarePositionLimit/maximum").get().getValue<int32_t>();
    return std::pair<int32_t, int32_t>(minimum, maximum);
}

std::shared_ptr<ObjectDictionary> ERB::getObjectDictionary() {
    return objectDictionary_;
}

bool ERB::setupPDOs() {
    logger_->debug("setupPDOs");
    // Setting TPDO1
    logger_->info("Setting up TPDO1");
    // set COBID to 0x83010000 to clear the register and disable TPDO1 for configuration
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Parameter/cobID", 0x83010000 )) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Mapping/numberOfObjects", 0)) {
        return false;
    }

    std::vector<ObjectDictionaryRegister> pdo1Registers;

    // Map actualVelocity
    auto reg = objectDictionary_->getRegister("actualVelocity").get();
    pdo1Registers.push_back(reg);
    // compute the address of the register to map (= 0xindex subindex size)
    uint32_t mappedObjectValue =
        (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Mapping/mappedObject1", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Map actualPosition
    reg = objectDictionary_->getRegister("actualPosition").get();
    pdo1Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Mapping/mappedObject2", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Enable TPD01
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO1Mapping/numberOfObjects", 2)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Parameter/cobID", ((0x4000 << 16) | (0x0180 + id_)))) { // NOLINT
        return false;
    }
    objectDictionary_->addPDOMapping(3, pdo1Registers);

    // Setting TPDO2
    logger_->info("Setting up TPDO2");
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Parameter/cobID", 0x83010000 )) { // NOLINT
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Mapping/numberOfObjects", 0)) {
        return false;
    }

    std::vector<ObjectDictionaryRegister> pdo2Registers;

    // Map actualTorque
    reg = objectDictionary_->getRegister("actualTorque").get();
    pdo2Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Mapping/mappedObject1", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Map statusWord
    reg = objectDictionary_->getRegister("statusWord").get();
    pdo2Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Mapping/mappedObject2", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Enable TPD02
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO2Mapping/numberOfObjects", 2)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO2Parameter/cobID", ((0x4000 << 16) | (0x0280 + id_)))) { // NOLINT
        return false;
    }
    objectDictionary_->addPDOMapping(5, pdo2Registers);

    // Setting TPDO3
    logger_->info("Setting up TPDO3");
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO1Parameter/cobID", 0x83010000 )) { // NOLINT
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Parameter/transmissionType", 1)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Mapping/numberOfObjects", 0)) {
        return false;
    }

    std::vector<ObjectDictionaryRegister> pdo3Registers;

    // Map digitalInputs
    reg = objectDictionary_->getRegister("digitalInputs").get();
    pdo3Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Mapping/mappedObject1", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Map modesOfOperationDisplay
    reg = objectDictionary_->getRegister("modesOfOperationDisplay").get();
    pdo3Registers.push_back(reg);
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Mapping/mappedObject2", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Enable TPD03
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO3Mapping/numberOfObjects", 2)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint32_t>("transmitPDO3Parameter/cobID", ((0x4000 << 16) | (0x0380 + id_)))) { // NOLINT
        return false;
    }
    objectDictionary_->addPDOMapping(7, pdo3Registers);

    // Disable TPDO4
    logger_->info("Disabling TPDO4");
    if (!sdoManager_->writeRegister<uint8_t>("transmitPDO4Mapping/numberOfObjects", 0)) {
        return false;
    }

    // Setting RPDO1
    logger_->info("Setting RPDO1");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Parameter/cobID", 0x83010000 )) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Mapping/numberOfObjects", 0)) {
        return false;
    }

    // Map targetPosition
    reg = objectDictionary_->getRegister("targetPosition").get();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8);
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Mapping/mappedObject1", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Map controlWord
    reg = objectDictionary_->getRegister("controlWord").get();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8); // NOLINT
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Mapping/mappedObject2", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Enable RPD01
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO1Mapping/numberOfObjects", 2)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO1Parameter/cobID", ((0x4000 << 16) | (0x0200 + id_)))) { // NOLINT
        return false;
    }

    // Setting RPDO2
    logger_->info("Setting RPDO2");
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Parameter/cobID", 0x83010000 )) { // NOLINT
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Parameter/transmissionType", 255)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Mapping/numberOfObjects", 0)) {
        return false;
    }

    // Map vlTargetVelocity
    reg = objectDictionary_->getRegister("vlTargetVelocity").get();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8); // NOLINT
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Mapping/mappedObject1", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Map controlWord
    reg = objectDictionary_->getRegister("controlWord").get();
    mappedObjectValue = (reg.getIndex() << 16) | (reg.getSubindex() << 8) | (reg.getSize()*8); // NOLINT
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Mapping/mappedObject2", mappedObjectValue)) { // NOLINT
        return false;
    }

    // Enable RPD02
    if (!sdoManager_->writeRegister<uint8_t>("receivePDO2Mapping/numberOfObjects", 2)) {
        return false;
    }
    if (!sdoManager_->writeRegister<uint32_t>("receivePDO2Parameter/cobID", ((0x4000 << 16) | (0x0300 + id_)))) { // NOLINT
        return false;
    }
    return true;
}

bool ERB::setNMTState(uint8_t state) {
    logger_->debug("setNMTState");
    can_frame frame = {};
    frame.can_id = NMT_ID;
    frame.can_dlc = 0x02;
    frame.data[0] = state;
    frame.data[1] = id_;
    logger_->info("Setting NMT state {}", state);
    return socket_->write(&frame) == sizeof(frame);
}

bool ERB::setModeOfOperation(const uint8_t mode) {
    logger_->debug("setModeOfOperation {}", mode);
    if (!sdoManager_->readRegister<uint8_t>("modesOfOperationDisplay")) {
        return false;
    }
    while (getModeOfOperation().value() != mode) {
        if (!sdoManager_->writeRegister<uint8_t>("modesOfOperation", mode)) {
            logger_->error("Could not change mode of operation");
            return false;
        }
        if (!sdoManager_->readRegister<uint8_t>("modesOfOperationDisplay")) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    logger_->info("Mode set : {}", getModeOfOperation().value());
    return true;
}

bool ERB::setRights(std::string rights) {
    logger_->debug("setRights");
    int address = 0x2008;
    int index_low = address & 0xFF;
    int index_high = (address >> 8) & 0xFF;
    can_frame frame = {};
    if (rights == "diag") {
        // User rigths DIAG : password = Diag (ASCII)
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0x23;
        frame.data[1] = index_low;
        frame.data[2] = index_high;
        frame.data[3] = 0x00;
        frame.data[7] = 0x67;
        frame.data[6] = 0x61;
        frame.data[5] = 0x69;
        frame.data[4] = 0x44;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        logger_->info("Access rights set to diag");
    } else if (rights == "profi") {
        // User rigths Profi : password = Schunk
        // first frame of fragmentation : contains the size of the data
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0b00100001;
        frame.data[1] = index_low;
        frame.data[2] = index_high;
        frame.data[3] = 0x00;
        frame.data[4] = 0x06;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // second frame of fragmentation : contains the data
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0b00000011;
        frame.data[1] = 0x53;
        frame.data[2] = 0x63;
        frame.data[3] = 0x68;
        frame.data[4] = 0x75;
        frame.data[5] = 0x6e;
        frame.data[6] = 0x6b;
        frame.data[7] = 0x00;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        logger_->info("Access rights set to profi");
    } else if (rights == "advanced") {
        // Access rigths advanced : password = ?SCHUNK!
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0b00100001;
        frame.data[1] = index_low;
        frame.data[2] = index_high;
        frame.data[3] = 0x00;
        frame.data[4] = 0x08;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0b00000000;
        frame.data[1] = 0x3f;
        frame.data[2] = 0x53;
        frame.data[3] = 0x43;
        frame.data[4] = 0x48;
        frame.data[5] = 0x55;
        frame.data[6] = 0x4e;
        frame.data[7] = 0x4b;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0b00011101;
        frame.data[1] = 0x21;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        logger_->info("Access rights set to advanced");

    } else {
        // User rigths GUEST : invalid password reset the access level to guest
        frame.can_id = 0x600 + id_;
        frame.can_dlc = 0x08;
        frame.data[0] = 0x23;
        frame.data[1] = index_low;
        frame.data[2] = index_high;
        frame.data[3] = 0x00;
        frame.data[7] = 0x00;
        frame.data[6] = 0x00;
        frame.data[5] = 0x00;
        frame.data[4] = 0x00;
        if (!socket_->write(&frame)) {
            logger_->error("Failed to set access rights : {}", rights);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        logger_->info("Incorrect parameter : access rights set to guest");
        return false;
    }
    return true;
}

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
