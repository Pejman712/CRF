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

#include "TIM/TIMCommands.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::tim {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMCommands::TIMCommands() :
    logger_("TIMCommands"),
    isEmpty_(true),
    localHeartbeat_(0),
    setPositionManually_(false),
    moveToTarget_(false),
    jogForward_(false),
    jogBackward_(false),
    stop_(false),
    emergencyStop_(false),
    chargingArmManualControl_(false),
    startCharging_(false),
    stopCharging_(false),
    extendChargingArm_(false),
    retractChargingArm_(false),
    enableEconomyMode_(false),
    disableEconomyMode_(false),
    rebootRobotArmWagon_(false),
    setObstacleMaximumVelocity_(false),
    devicesRetracted_(false),
    allowMovement_(false),
    acknowledgeAlarms_(false) {
    logger_->debug("CTor");
}

TIMCommands::TIMCommands(const TIMCommands& input) :
    logger_("TIMCommands") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    localHeartbeat_ = input.localHeartbeat();
    setPositionManually_ = input.setPositionManually();
    moveToTarget_ = input.moveToTarget();
    jogForward_ = input.jogForward();
    jogBackward_ = input.jogBackward();
    stop_ = input.stop();
    emergencyStop_ = input.emergencyStop();
    chargingArmManualControl_ = input.chargingArmManualControl();
    startCharging_ = input.startCharging();
    stopCharging_ = input.stopCharging();
    extendChargingArm_ = input.extendChargingArm();
    retractChargingArm_ = input.retractChargingArm();
    enableEconomyMode_ = input.enableEconomyMode();
    disableEconomyMode_ = input.disableEconomyMode();
    rebootRobotArmWagon_ = input.rebootRobotArmWagon();
    setObstacleMaximumVelocity_ = input.setObstacleMaximumVelocity();
    devicesRetracted_ = input.devicesRetracted();
    allowMovement_ = input.allowMovement();
    acknowledgeAlarms_ = input.acknowledgeAlarms();
}

bool TIMCommands::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer");
    try {
        localHeartbeat_ = SiemensPLCTypeConverter::getShort(buffer,
            variablesDBLocation["LocalHeartbeat"][0]);
        setPositionManually_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["SetPositionManually"][1],
            variablesDBLocation["SetPositionManually"][0]);
        moveToTarget_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MoveToTarget"][1],
            variablesDBLocation["MoveToTarget"][0]);
        jogForward_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["JogForward"][1],
            variablesDBLocation["JogForward"][0]);
        jogBackward_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["JogBackward"][1],
            variablesDBLocation["JogBackward"][0]);
        stop_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["Stop"][1],
            variablesDBLocation["Stop"][0]);
        emergencyStop_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["EmergencyStop"][1],
            variablesDBLocation["EmergencyStop"][0]);
        chargingArmManualControl_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ChargingArmManualControl"][1],
            variablesDBLocation["ChargingArmManualControl"][0]);
        startCharging_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["StartCharging"][1],
            variablesDBLocation["StartCharging"][0]);
        stopCharging_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["StopCharging"][1],
            variablesDBLocation["StopCharging"][0]);
        extendChargingArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ExtendChargingArm"][1],
            variablesDBLocation["ExtendChargingArm"][0]);
        retractChargingArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RetractChargingArm"][1],
            variablesDBLocation["RetractChargingArm"][0]);
        enableEconomyMode_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["EnableEconomyMode"][1],
            variablesDBLocation["EnableEconomyMode"][0]);
        disableEconomyMode_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["DisableEconomyMode"][1],
            variablesDBLocation["DisableEconomyMode"][0]);
        rebootRobotArmWagon_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RebootRobotArmWagon"][1],
            variablesDBLocation["RebootRobotArmWagon"][0]);
        devicesRetracted_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["DevicesRetracted"][1],
            variablesDBLocation["DevicesRetracted"][0]);
        allowMovement_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["AllowMovement"][1],
            variablesDBLocation["AllowMovement"][0]);
        acknowledgeAlarms_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["AcknowledgeAlarms"][1],
            variablesDBLocation["AcknowledgeAlarms"][0]);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM inputs - {}", e.what());
        return false;
    }
    try {
        setObstacleMaximumVelocity_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["SetObstacleMaximumVelocity"][1],
            variablesDBLocation["SetObstacleMaximumVelocity"][0]);
    } catch (std::invalid_argument& e) {
        logger_->debug("No permission to write obstacles - {}", e.what());
    }
    isEmpty_ = false;
    return true;
}

void TIMCommands::clear() {
    isEmpty_ = true;
    localHeartbeat_ = 0;
    setPositionManually_ = false;
    moveToTarget_ = false;
    jogForward_ = false;
    jogBackward_ = false;
    stop_ = false;
    emergencyStop_ = false;
    chargingArmManualControl_ = false;
    startCharging_ = false;
    stopCharging_ = false;
    extendChargingArm_ = false;
    retractChargingArm_ = false;
    enableEconomyMode_ = false;
    disableEconomyMode_ = false;
    rebootRobotArmWagon_ = false;
    setObstacleMaximumVelocity_ = false;
    devicesRetracted_ = false;
    allowMovement_ = false;
    acknowledgeAlarms_ = false;
    return;
}

bool TIMCommands::isEmpty() const {
    return isEmpty_;
}

int16_t TIMCommands::localHeartbeat() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return localHeartbeat_;
}

void TIMCommands::localHeartbeat(int16_t input) {
    isEmpty_ = false;
    localHeartbeat_ = input;
}

bool TIMCommands::setPositionManually() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return setPositionManually_;
}

void TIMCommands::setPositionManually(bool input) {
    isEmpty_ = false;
    setPositionManually_ = input;
}

bool TIMCommands::moveToTarget() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return moveToTarget_;
}

void TIMCommands::moveToTarget(bool input) {
    isEmpty_ = false;
    moveToTarget_ = input;
}

bool TIMCommands::jogForward() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return jogForward_;
}

void TIMCommands::jogForward(bool input) {
    isEmpty_ = false;
    jogForward_ = input;
}

bool TIMCommands::jogBackward() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return jogBackward_;
}

void TIMCommands::jogBackward(bool input) {
    isEmpty_ = false;
    jogBackward_ = input;
}

bool TIMCommands::stop() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return stop_;
}

void TIMCommands::stop(bool input) {
    isEmpty_ = false;
    stop_ = input;
}

bool TIMCommands::emergencyStop() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return emergencyStop_;
}

void TIMCommands::emergencyStop(bool input) {
    isEmpty_ = false;
    emergencyStop_ = input;
}

bool TIMCommands::chargingArmManualControl() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return chargingArmManualControl_;
}

void TIMCommands::chargingArmManualControl(bool input) {
    isEmpty_ = false;
    chargingArmManualControl_ = input;
}

bool TIMCommands::startCharging() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return startCharging_;
}

void TIMCommands::startCharging(bool input) {
    isEmpty_ = false;
    startCharging_ = input;
}

bool TIMCommands::stopCharging() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return stopCharging_;
}

void TIMCommands::stopCharging(bool input) {
    isEmpty_ = false;
    stopCharging_ = input;
}

bool TIMCommands::extendChargingArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return extendChargingArm_;
}

void TIMCommands::extendChargingArm(bool input) {
    isEmpty_ = false;
    extendChargingArm_ = input;
}

bool TIMCommands::retractChargingArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return retractChargingArm_;
}

void TIMCommands::retractChargingArm(bool input) {
    isEmpty_ = false;
    retractChargingArm_ = input;
}

bool TIMCommands::enableEconomyMode() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return enableEconomyMode_;
}

void TIMCommands::enableEconomyMode(bool input) {
    isEmpty_ = false;
    enableEconomyMode_ = input;
}

bool TIMCommands::disableEconomyMode() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return disableEconomyMode_;
}

void TIMCommands::disableEconomyMode(bool input) {
    isEmpty_ = false;
    disableEconomyMode_ = input;
}

bool TIMCommands::rebootRobotArmWagon() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return rebootRobotArmWagon_;
}

void TIMCommands::rebootRobotArmWagon(bool input) {
    isEmpty_ = false;
    rebootRobotArmWagon_ = input;
}

bool TIMCommands::setObstacleMaximumVelocity() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return setObstacleMaximumVelocity_;
}

void TIMCommands::setObstacleMaximumVelocity(bool input) {
    isEmpty_ = false;
    setObstacleMaximumVelocity_ = input;
}

bool TIMCommands::devicesRetracted() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return devicesRetracted_;
}

void TIMCommands::devicesRetracted(bool input) {
    isEmpty_ = false;
    devicesRetracted_ = input;
}

bool TIMCommands::allowMovement() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return allowMovement_;
}

void TIMCommands::allowMovement(bool input) {
    isEmpty_ = false;
    allowMovement_ = input;
}

bool TIMCommands::acknowledgeAlarms() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Commands is empty");
    }
    return acknowledgeAlarms_;
}

void TIMCommands::acknowledgeAlarms(bool input) {
    isEmpty_ = false;
    acknowledgeAlarms_ = input;
}

}  // namespace crf::actuators::tim
