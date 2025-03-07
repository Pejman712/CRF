/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <map>
#include <utility>

#include "TIM/TIMStatus.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::tim {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMStatus::TIMStatus() :
    logger_("TIMStatus"),
    timPosition_(0),
    timVelocity_(0),
    targetReached_(false),
    timStopped_(false),
    charging_(false),
    chargingCurrent_(0),
    batteryVoltage_(0),
    batteryCurrent_(0),
    economyMode_(false),
    chargingArmConnected_(false),
    chargingArmRetracted_(false),
    frontWarningField_(false),
    backWarningField_(false),
    timHeartbeat_(0),
    mainMotorOn_(false),
    safeToMove_(false) {
    logger_->debug("CTor");
}

TIMStatus::TIMStatus(const TIMStatus& input) :
    logger_("TIMStatus") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    timPosition_ = input.timPosition();
    timVelocity_ = input.timVelocity();
    targetReached_ = input.targetReached();
    timStopped_ = input.timStopped();
    charging_ = input.charging();
    chargingCurrent_ = input.chargingCurrent();
    batteryVoltage_ = input.batteryVoltage();
    batteryCurrent_ = input.batteryCurrent();
    economyMode_ = input.economyMode();
    chargingArmConnected_ = input.chargingArmConnected();
    chargingArmRetracted_ = input.chargingArmRetracted();
    frontWarningField_ = input.frontWarningField();
    backWarningField_ = input.backWarningField();
    timHeartbeat_ = input.timHeartbeat();
    mainMotorOn_ = input.mainMotorOn();
    safeToMove_ = input.safeToMove();
}

bool TIMStatus::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer");
    try {
        timPosition_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["TIMPosition"][0]);
        timVelocity_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["TIMVelocity"][0]);
        targetReached_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["TargetReached"][1],
            variablesDBLocation["TargetReached"][0]);
        timStopped_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["TIMStopped"][1],
            variablesDBLocation["TIMStopped"][0]);
        charging_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["Charging"][1],
            variablesDBLocation["Charging"][0]);
        chargingCurrent_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["ChargingCurrent"][0]);
        batteryVoltage_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["BatteryVoltage"][0]);
        batteryCurrent_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["BatteryCurrent"][0]);
        economyMode_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["EconomyMode"][1],
            variablesDBLocation["EconomyMode"][0]);
        chargingArmConnected_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ChargingArmConnected"][1],
            variablesDBLocation["ChargingArmConnected"][0]);
        chargingArmRetracted_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ChargingArmRetracted"][1],
            variablesDBLocation["ChargingArmRetracted"][0]);
        frontWarningField_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["FrontWarningField"][1],
            variablesDBLocation["FrontWarningField"][0]);
        backWarningField_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BackWarningField"][1],
            variablesDBLocation["BackWarningField"][0]);
        timHeartbeat_ = SiemensPLCTypeConverter::getShort(buffer,
            variablesDBLocation["TIMHeartbeat"][0]);
        mainMotorOn_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MainMotorOn"][1],
            variablesDBLocation["MainMotorOn"][0]);
        safeToMove_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["SafeToMove"][1],
            variablesDBLocation["SafeToMove"][0]);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM status - {}", e.what());
        return false;
    }
    isEmpty_ = false;
    return true;
}

void TIMStatus::clear() {
    isEmpty_ = true;
    timPosition_ = 0;
    timVelocity_ = 0;
    targetReached_ = false;
    timStopped_ = false;
    charging_ = false;
    chargingCurrent_ = 0;
    batteryVoltage_ = 0;
    batteryCurrent_ = 0;
    economyMode_ = false;
    chargingArmConnected_ = false;
    chargingArmRetracted_ = false;
    frontWarningField_ = false;
    backWarningField_ = false;
    timHeartbeat_ = 0;
    mainMotorOn_ = false;
    safeToMove_ = false;
    return;
}

bool TIMStatus::isEmpty() const {
    return isEmpty_;
}

float TIMStatus::timPosition() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return timPosition_;
}

void TIMStatus::timPosition(float input) {
    isEmpty_ = false;
    timPosition_ = input;
}

float TIMStatus::timVelocity() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return timVelocity_;
}

void TIMStatus::timVelocity(float input) {
    isEmpty_ = false;
    timVelocity_ = input;
}

bool TIMStatus::targetReached() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return targetReached_;
}

void TIMStatus::targetReached(bool input) {
    isEmpty_ = false;
    targetReached_ = input;
}

bool TIMStatus::timStopped() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return timStopped_;
}

void TIMStatus::timStopped(bool input) {
    isEmpty_ = false;
    timStopped_ = input;
}

bool TIMStatus::charging() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return charging_;
}

void TIMStatus::charging(bool input) {
    isEmpty_ = false;
    charging_ = input;
}

float TIMStatus::chargingCurrent() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return chargingCurrent_;
}

void TIMStatus::chargingCurrent(float input) {
    isEmpty_ = false;
    chargingCurrent_ = input;
}

float TIMStatus::batteryVoltage() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return batteryVoltage_;
}

void TIMStatus::batteryVoltage(float input) {
    isEmpty_ = false;
    batteryVoltage_ = input;
}

float TIMStatus::batteryCurrent() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return batteryCurrent_;
}

void TIMStatus::batteryCurrent(float input) {
    isEmpty_ = false;
    batteryCurrent_ = input;
}

bool TIMStatus::economyMode() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return economyMode_;
}

void TIMStatus::economyMode(bool input) {
    isEmpty_ = false;
    economyMode_ = input;
}

bool TIMStatus::chargingArmConnected() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return chargingArmConnected_;
}

void TIMStatus::chargingArmConnected(bool input) {
    isEmpty_ = false;
    chargingArmConnected_ = input;
}

bool TIMStatus::chargingArmRetracted() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return chargingArmRetracted_;
}

void TIMStatus::chargingArmRetracted(bool input) {
    isEmpty_ = false;
    chargingArmRetracted_ = input;
}

bool TIMStatus::frontWarningField() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return frontWarningField_;
}

void TIMStatus::frontWarningField(bool input) {
    isEmpty_ = false;
    frontWarningField_ = input;
}

bool TIMStatus::backWarningField() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return backWarningField_;
}

void TIMStatus::backWarningField(bool input) {
    isEmpty_ = false;
    backWarningField_ = input;
}

int16_t TIMStatus::timHeartbeat() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return timHeartbeat_;
}

void TIMStatus::timHeartbeat(int16_t input) {
    isEmpty_ = false;
    timHeartbeat_ = input;
}

bool TIMStatus::mainMotorOn() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return mainMotorOn_;
}

void TIMStatus::mainMotorOn(bool input) {
    isEmpty_ = false;
    mainMotorOn_ = input;
}

bool TIMStatus::safeToMove() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Status is empty");
    }
    return safeToMove_;
}

void TIMStatus::safeToMove(bool input) {
    isEmpty_ = false;
    safeToMove_ = input;
}

}  // namespace crf::actuators::tim
