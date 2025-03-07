/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <utility>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "EtherCATDevices/EtherCATMotor.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

EtherCATMotor::EtherCATMotor(uint16_t id, std::shared_ptr<EtherCATManager> ECM) :
    motorOut_(NULL),
    motorIn_(NULL),
    pdoLinked_(false),
    logger_("EtherCATMotor " + std::to_string(id)),
    initialized_(false),
    id_(id),
    ECManager_(ECM) {
    logger_->debug("CTor");
}

EtherCATMotor::~EtherCATMotor() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool EtherCATMotor::initialize() {
    logger_->debug("initialize");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!ECManager_->checkPreOp(id_)) {
        logger_->error("Not in Pre-Operation Mode. PDO configuration can't be done");
        return false;
    }
    if (!ECManager_->writeSDO<int16_t>(id_, 0x605A, 0x00, 6)) {
        logger_->error("Command Registers can't be configured");
        return false;
    }
    logger_->info("Command Registers configured");

    if (!ECManager_->writeSDO<uint16_t>(id_, 0x1C12, 0x00, 0x0000) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C12, 0x01, 0x1605) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C12, 0x02, 0x161D) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C12, 0x00, 0x0002)) {
        logger_->error("Receive PDO can't be configured");
        return false;
    }
    logger_->info("Receive PDO configured");

    if (!ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x00, 0x0000) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x01, 0x1A03) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x02, 0x1A0B) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x03, 0x1A13) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x04, 0x1A1F) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x05, 0x1A1D) ||
        !ECManager_->writeSDO<uint16_t>(id_, 0x1C13, 0x00, 0x0005)) {
        logger_->error("Transmit PDO can't be configured");
        return false;
    }
    logger_->info("Trasmit PDO configured");

    initialized_ = true;
    return true;
}

bool EtherCATMotor::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }
    if (!pdoLinked_) {
        std::optional<uint16_t> controlIn = ECManager_->readSDO<uint16_t>(id_, 0x6040, 0x00);
        if (!controlIn) {
            logger_->error("Can't retrieve controlWord via SDO to disable voltage of the motor");
            return false;
        }

        uint16_t controlOut = controlIn.value();
        controlOut = controlOut & controlword::DisableVoltage_AND;
        controlOut = controlOut | controlword::DisableVoltage_OR;
        if (!ECManager_->writeSDO<uint16_t>(id_, 0x6040, 0x00, controlOut)) {
            logger_->error("Can't write controlWord via SDO to disable voltage of the motor");
            return false;
        }
    } else {
        if (!disableVoltage()) {
            logger_->error("Can't write controlWord via PDO to disable voltage of the motor");
            return false;
        }
    }
    initialized_ = false;
    pdoLinked_ = false;
    return true;
}

int EtherCATMotor::getID() {
    return id_;
}

bool EtherCATMotor::bindPDOs() {
    if (!initialized_) {
        logger_->error("Motor not initialized");
        pdoLinked_ = false;
        return false;
    }
    if (!ECManager_->ioMapConfigured()) {
        logger_->error("IOMap is not configured");
        pdoLinked_ = false;
        return false;
    }

    std::optional<uint8_t*> in = ECManager_->retrieveInputs(id_);
    std::optional<uint8_t*> out = ECManager_->retrieveOutputs(id_);
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        pdoLinked_ = false;
        return false;
    }

    motorIn_ = reinterpret_cast<EC_input*>(in.value());
    motorOut_ = reinterpret_cast<EC_output*>(out.value());
    pdoLinked_ = true;
    logger_->info("PDO Binded");
    return true;
}

std::optional<uint16_t> EtherCATMotor::getEtherCatState() {
    if (!initialized_) {
        logger_->error("Motor not initialized");
        return std::nullopt;
    }
    return ECManager_->getSlaveState(id_);
}

bool EtherCATMotor::isAlive() {
    if (!initialized_) {
        logger_->error("Motor not initialized");
        return false;
    }
    return ECManager_->slaveCommunicationCheck(id_);
}


// Device Internal State Machine

std::optional<bool> EtherCATMotor::inFault() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::InFault_Mask) != statusword::InFault_Value) {
        return false;
    }

    std::optional<uint16_t> res = ECManager_->readSDO<uint16_t>(id_, 0x603F, 0x00);
    if (!res) {
        logger_->warn("Cannot retrieve the ERROR CODE");
        logger_->warn("Cannot retrieve the ERROR CODE");
    } else {
        logger_->warn("ERROR CODE: {}", res.value());
        logger_->warn("ERROR CODE: {}", res.value());
    }
    return true;
}

std::optional<bool> EtherCATMotor::inQuickStop() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::InQuickStop_Mask) != statusword::InQuickStop_Value) {
        return false;
    }
    return true;
}

std::optional<bool> EtherCATMotor::isEnabled() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::IsEnabled_Mask) != statusword::IsEnabled_Value) {
        return false;
    }
    return true;
}

std::optional<bool> EtherCATMotor::isReadyToSwitchOn() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::IsReadyToSwitchOn_Mask) != statusword::IsReadyToSwitchOn_Value) {  // NOLINT
        return false;
    }
    return true;
}

std::optional<bool> EtherCATMotor::isSwitchOnDisabled() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::IsSwitchOnDisabled_Mask) != statusword::IsSwitchOnDisabled_Value) {  // NOLINT
        return false;
    }
    return true;
}

std::optional<bool> EtherCATMotor::isSwitchedOn() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::IsSwitchedOn_Mask) != statusword::IsSwitchedOn_Value) {
        return false;
    }
    return true;
}

bool EtherCATMotor::enableOperation() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    uint16_t var = motorOut_->controlWord;
    var = var & controlword::EnableOperation_AND;
    var = var | controlword::EnableOperation_OR;
    motorOut_->controlWord = var;
    return timedCheck_function([this](){return isEnabled();});
}

bool EtherCATMotor::disableOperation() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    uint16_t var = motorOut_->controlWord;
    var &= controlword::DisableOperation_AND;
    var |= controlword::DisableOperation_OR;
    motorOut_->controlWord = var;
    return timedCheck_function([this](){return isSwitchedOn();});
}

bool EtherCATMotor::disableVoltage() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    uint16_t var = motorOut_->controlWord;
    var = var & controlword::DisableVoltage_AND;
    var = var | controlword::DisableVoltage_OR;
    motorOut_->controlWord = var;
    return timedCheck_function([this](){return isSwitchOnDisabled();});
}

bool EtherCATMotor::stop() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    motorOut_->controlWord = motorOut_->controlWord | controlword::bit_halt;
    return timedCheck_withMask(motorIn_->statusWord, statusword::bit_targetReached);
}

bool EtherCATMotor::quickStop() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    auto SO = isSwitchedOn();
    if (!SO) {
        logger_->error("Failed to get switchedOn");
        return false;
    }

    uint16_t var = motorOut_->controlWord;
    var &= controlword::QuickStop_AND;
    var |= controlword::QuickStop_OR;
    motorOut_->controlWord = var;
    if (SO.value()) {
        return timedCheck_function([this](){return isSwitchOnDisabled();});
    }
    return timedCheck_function([this](){return inQuickStop();});
}

bool EtherCATMotor::shutdown() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    uint16_t var = motorOut_->controlWord;
    var = var & controlword::ShutDown_AND;
    var = var | controlword::ShutDown_OR;
    motorOut_->controlWord = var;
    return timedCheck_function([this](){return isReadyToSwitchOn();});
}

bool EtherCATMotor::faultReset() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    auto F = inFault();
    if (!F) {
        logger_->error("Communication problem with the Motor");
        return false;
    }
    if (!F.value()) {
        logger_->error("Motor not in correct state. Can't perform fault reset");
        return false;
    }
    uint16_t var = motorOut_->controlWord;
    var |= controlword::FaultReset_OR;
    motorOut_->controlWord = var;
    return timedCheck_function([this](){return isSwitchOnDisabled();});
}

std::optional<bool> EtherCATMotor::targetReached() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::bit_targetReached) != statusword::bit_targetReached) {
        return false;
    }
    return true;
}

std::optional<bool> EtherCATMotor::internalLimitActive() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    if ((motorIn_->statusWord & statusword::bit_internalLimitActive) !=
    statusword::bit_internalLimitActive) {
        return false;
    }
    return true;
}


// Motor Data - PDOs

std::optional<int32_t> EtherCATMotor::getVelocity() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->velocityActualValue;
}

std::optional<int32_t> EtherCATMotor::getPosition() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->positionActualValue;
}

std::optional<int32_t> EtherCATMotor::getCurrent() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->currentActualValue;
}

std::optional<uint16_t> EtherCATMotor::getStatusWord() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->statusWord;
}

std::optional<int8_t> EtherCATMotor::getModeOfOperation() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->modeOfOperationDisplay;
}

std::optional<bool> EtherCATMotor::getDigitalInput(uint32_t bit) {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    int val = (motorIn_->digitalInputs >> (15 + bit)) & 1U;
    return val == 1;
}

std::optional<int16_t> EtherCATMotor::getTorque() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->torqueActualValue;
}

std::optional<int16_t> EtherCATMotor::getAnalogInput() {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return motorIn_->analogInput;
}

bool EtherCATMotor::setModeOfOperation(int8_t mo) {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    motorOut_->modeOfOperation = mo;
    return timedCheck(motorIn_->modeOfOperationDisplay, motorOut_->modeOfOperation);
}

bool EtherCATMotor::setDigitalOutput(uint32_t bit) {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    motorOut_->digitalOutputs |= 1UL << (15 + bit);
    return true;
}

bool EtherCATMotor::resetDigitalOutput(uint32_t bit) {
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    motorOut_->digitalOutputs &= ~(1UL << (15 + bit));
    return true;
}

bool EtherCATMotor::setPosition(int32_t pos, bool relative) {
    logger_->debug("setPosition 1");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t>  mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfilePositionMode) {
        logger_->warn("Motor not in Profile Position Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfilePositionMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Position Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Position Mode");
    }

    motorOut_->controlWord &= ~(controlword::bit_halt);  // bit 8 = 0 halt
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    if ((motorIn_->statusWord & statusword::bit_setNewPointAck) != 0x0000) {
        return false;
    }
    if (relative) {
        motorOut_->controlWord |= controlword::bit_relative;  // bit 6 = 1 relative
    } else {
        motorOut_->controlWord &= ~(controlword::bit_relative);  // bit 6 = 0 relative
    }

    motorOut_->targetPosition = pos;
    motorOut_->controlWord |= controlword::bit_newSetPoint;  // bit 4 = 1 new setpoint
    if (!timedCheck_withMask(motorIn_->statusWord, statusword::bit_setNewPointAck)) {
        logger_->error("New setpoint not acknowledged by the motor");
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    return true;
}

bool EtherCATMotor::setPosition(int32_t pos, uint32_t vel, bool relative) {
    logger_->debug("setPosition 2");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfilePositionMode) {
        logger_->warn("Motor not in Profile Position Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfilePositionMode)) {
            logger_->debug("Can't change the Mode of Operation to Profile Position Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Position Mode");
    }

    if (!setProfileVelocity(vel)) {
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_halt);  // bit 8 = 0 halt
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    if ((motorIn_->statusWord & statusword::bit_setNewPointAck) != 0x0000) {
        return false;
    }
    if (relative) {
        motorOut_->controlWord |= controlword::bit_relative;  // bit 6 = 1 relative
    } else {
        motorOut_->controlWord &= ~(controlword::bit_relative);  // bit 6 = 0 relative
    }

    motorOut_->targetPosition = pos;
    motorOut_->controlWord |= controlword::bit_newSetPoint;  // bit 4 = 1 new setpoint
    if (!timedCheck_withMask(motorIn_->statusWord, statusword::bit_setNewPointAck)) {
        logger_->error("New setpoint not acknowledged by the motor");
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    return true;
}


bool EtherCATMotor::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
    bool relative) {
    logger_->debug("setPosition(pos: {}, vel: {}, acc: {})", position, velocity, acceleration);
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfilePositionMode) {
        logger_->warn("Motor not in Profile Position Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfilePositionMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Position Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Position Mode");
    }

    if (!setProfileVelocity(velocity) || !setProfileAcceleration(acceleration)) {
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_halt);  // bit 8 = 0 halt
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    if ((motorIn_->statusWord & statusword::bit_setNewPointAck) != 0x0000) {
        return false;
    }
    if (relative) {
        motorOut_->controlWord |= controlword::bit_relative;  // bit 6 = 1 relative
    } else {
        motorOut_->controlWord &= ~(controlword::bit_relative);  // bit 6 = 0 relative
    }

    motorOut_->targetPosition = position;
    motorOut_->controlWord |= controlword::bit_newSetPoint;  // bit 4 = 1 new setpoint
    if (!timedCheck_withMask(motorIn_->statusWord, statusword::bit_setNewPointAck)) {
        logger_->error("New setpoint not acknowledged by the motor");
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    return true;
}

bool EtherCATMotor::setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
    uint32_t deceleration, bool relative) {
    logger_->debug("setPosition(pos: {}, vel: {}, acc: {}, dec: {})",
        position, velocity, acceleration, deceleration);
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfilePositionMode) {
        logger_->warn("Motor not in Profile Position Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfilePositionMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Position Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Position Mode");
    }

    if (!setProfileVelocity(velocity)) {
        return false;
    }
    if (!setProfileAcceleration(acceleration) || !setProfileDeceleration(deceleration)) {
        return false;
    }

    motorOut_->controlWord &= ~(controlword::bit_halt);  // bit 8 = 0 halt
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    if ((motorIn_->statusWord & statusword::bit_setNewPointAck) != 0x0000) {
        return false;
    }
    if (relative) {
        motorOut_->controlWord |= controlword::bit_relative;  // bit 6 = 1 relative
    } else {
        motorOut_->controlWord &= ~(controlword::bit_relative);  // bit 6 = 0 relative
    }

    motorOut_->targetPosition = position;
    motorOut_->controlWord |= controlword::bit_newSetPoint;  // bit 4 = 1 new setpoint
    if (!timedCheck_withMask(motorIn_->statusWord, statusword::bit_setNewPointAck)) {
        logger_->error("New setpoint not acknowledged by the motor");
        return false;
    }
    motorOut_->controlWord &= ~(controlword::bit_newSetPoint);  // bit 4 = 0 new setpoint
    return true;
}

bool EtherCATMotor::setVelocity(int32_t vel) {
    logger_->debug("setVelocity 1");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
            logger_->error("Motor not initialized or not alive.");
            return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfileVelocityMode) {
        logger_->warn("Motor not in Profile Velocity Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfileVelocityMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Velocity Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Velocity Mode");
    }
    motorOut_->controlWord = motorOut_->controlWord & 0xFEFF;  // bit 8 = 0 halt
    motorOut_->targetVelocity = vel;
    return true;
}

bool EtherCATMotor::setVelocity(int32_t velocity, uint32_t acceleration) {
    logger_->debug("setVelocity 2");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfileVelocityMode) {
        logger_->warn("Motor not in Profile Velocity Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfileVelocityMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Velocity Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Velocity Mode");
    }
    if (!setProfileAcceleration(acceleration)) {
        return false;
    }
    motorOut_->controlWord = motorOut_->controlWord & 0xFEFF;  // bit 8 = 0 halt
    motorOut_->targetVelocity = velocity;
    return true;
}

bool EtherCATMotor::setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) {
    logger_->debug("setVelocity 3");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfileVelocityMode) {
        logger_->warn("Motor not in Profile Velocity Mode: {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfileVelocityMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Velocity Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Velocity Mode");
    }
    if (!setProfileAcceleration(acceleration) || !setProfileDeceleration(deceleration)) {
        return false;
    }
    motorOut_->controlWord = motorOut_->controlWord & 0xFEFF;  // bit 8 = 0 halt
    motorOut_->targetVelocity = velocity;
    return true;
}

bool EtherCATMotor::setTorque(int16_t tor) {
    logger_->debug("setTorque");
    if (!initialized_ || !pdoLinked_ || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    std::optional<int8_t> mo = getModeOfOperation();
    if (!mo) {
        logger_->error("Communication problem.");
        return false;
    }
    if (mo.value() != modesofoperation::ProfileTorqueMode) {
        logger_->error("Motor not in Profile Torque Mode : {}", mo.value());
        if (!setModeOfOperation(modesofoperation::ProfileTorqueMode)) {
            logger_->error("Can't change the Mode of Operation to Profile Torque Mode");
            return false;
        }
        logger_->info("Mode of Operation changed to Profile Torque Mode");
    }
    motorOut_->targetTorque = tor;
    return true;
}

bool EtherCATMotor::setMaxTorque(uint16_t maxtorque) {
    if (!initialized_ || !pdoLinked_  || !isAlive()) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    motorOut_->maxTorque = maxtorque;
    return true;
}


// Motor Parameters - SDO

bool EtherCATMotor::setProfileVelocity(uint32_t vel) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6081, 0x00, vel);
}

bool EtherCATMotor::setProfileAcceleration(uint32_t acc) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6083, 0x00, acc);
}

bool EtherCATMotor::setProfileDeceleration(uint32_t dec) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6084, 0x00, dec);
}

bool EtherCATMotor::setMaxVelocity(uint32_t maxVel) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x607F, 0x00, maxVel);
}

bool EtherCATMotor::setMaxAcceleration(uint32_t maxAcc) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x60C5, 0x00, maxAcc);
}

bool EtherCATMotor::setMaxDeceleration(uint32_t maxDec) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x60C6, 0x00, maxDec);
}

bool EtherCATMotor::setQuickstopDeceleration(uint32_t quickDec) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6085, 0x00, quickDec);
}

bool EtherCATMotor::setPositionLimits(std::pair<int32_t, int32_t> limits) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return (ECManager_->writeSDO<int32_t>(id_, 0x607D, 0x01, limits.first) &&
        ECManager_->writeSDO<int32_t>(id_, 0x607D, 0x02, limits.second));
}

bool EtherCATMotor::setMaxCurrent(uint16_t maxcurrent) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6073, 0x00, maxcurrent);
}

bool EtherCATMotor::setMotorRatedCurrent(uint32_t motorcurrent) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6075, 0x00, motorcurrent);
}

bool EtherCATMotor::setMotorRatedTorque(uint32_t motortorque) {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->writeSDO<uint32_t>(id_, 0x6076, 0x00, motortorque);
}

std::optional<uint32_t> EtherCATMotor::getProfileVelocity() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6081, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getProfileAcceleration() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6083, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getProfileDeceleration() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6084, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getQuickstopDeceleration() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6085, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getMaxVelocity() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x607F, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getMaxAcceleration() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x60C5, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getMaxDeceleration() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x60C6, 0x00);
}

std::optional<std::pair<int32_t, int32_t>> EtherCATMotor::getPositionLimits() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return std::nullopt;
    }
    std::optional<int32_t> min = ECManager_->readSDO<int32_t>(id_, 0x607D, 0x01);
    std::optional<int32_t> max = ECManager_->readSDO<int32_t>(id_, 0x607D, 0x02);
    if (!min || !max) {
        return std::nullopt;
    }
    return std::pair<int32_t, int32_t>(min.value(), max.value());
}

std::optional<uint32_t> EtherCATMotor::getMotorRatedCurrent() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6075, 0x00);
}

std::optional<uint32_t> EtherCATMotor::getMotorRatedTorque() {
    if (!initialized_) {
        logger_->error("Motor not initialized or not alive.");
        return false;
    }
    return ECManager_->readSDO<uint32_t>(id_, 0x6076, 0x00);
}

bool EtherCATMotor::timedCheck_function(std::function<std::optional<bool>()> func) {
    std::optional<bool> state;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(10*constants::cycleTime_timedCheck));
        state = func();
        if (!state) {
            return false;
        }
        if (state.value()) {
            return true;
        }
        end = std::chrono::steady_clock::now();
    } while (std::chrono::duration_cast<std::chrono::milliseconds>
    (end - start).count() <= constants::totalTime_timedCheck);
    return false;
}

template<typename T>
bool EtherCATMotor::timedCheck_withMask(const T& valueFrom, const T& valueCheck, int timeTot,
    int timeCycle) {
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(timeCycle));
        if ((valueFrom & valueCheck) == valueCheck) {
            return true;
        }
        end = std::chrono::steady_clock::now();
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() <= timeTot);
    return false;
}

template<typename T>
bool EtherCATMotor::timedCheck(const T& value1, const T& value2, int timeTot, int timeCycle) {
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(timeCycle));
        if (value1 == value2) {
            return true;
        }
        end = std::chrono::steady_clock::now();
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() <= timeTot);
    return false;
}

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
