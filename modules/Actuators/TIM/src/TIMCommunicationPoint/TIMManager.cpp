/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <optional>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>

#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "TIM/ITIM.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

TIMManager::TIMManager(std::shared_ptr<crf::actuators::tim::ITIM> tim,
    const std::chrono::milliseconds& inizializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        tim,
        inizializationTimeout,
        controlAccessTimeout),
    tim_(tim),
    logger_("TIMManager") {
    logger_->debug("CTor");
}

TIMManager::~TIMManager() {
    logger_->debug("DTor");
}

std::optional<bool> TIMManager::setCurrentPosition(const float &position,
    const uint32_t &priority) {
    logger_->debug("setCurrentPosition");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->setCurrentPosition(position);
}

std::optional<bool> TIMManager::setTargetPosition(const float &position,
    const uint32_t &priority) {
    logger_->debug("setTargetPosition");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->setTargetPosition(position);
}

std::optional<bool> TIMManager::setTargetVelocity(const float &velocity,
    const uint32_t &priority) {
    logger_->debug("setTargetVelocity");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->setTargetVelocity(velocity);
}

std::optional<bool> TIMManager::moveToTarget(const float &position, const float &velocity,
    const uint32_t &priority) {
    logger_->debug("moveToTarget");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->moveToTarget(position, velocity);
}

std::optional<bool> TIMManager::jog(const float &velocity, const uint32_t &priority) {
    logger_->debug("jog");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->jog(velocity);
}

std::optional<bool> TIMManager::stop(const uint32_t &priority) {
    logger_->debug("stop");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->stop();
}

std::optional<bool> TIMManager::emergencyStop(const uint32_t &priority) {
    logger_->debug("emergencyStop");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->emergencyStop();
}

std::optional<bool> TIMManager::extendChargingArm(const uint32_t &priority) {
    logger_->debug("extendChargingArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->extendChargingArm();
}

std::optional<bool> TIMManager::retractChargingArm(const uint32_t &priority) {
    logger_->debug("retractChargingArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->retractChargingArm();
}

std::optional<bool> TIMManager::startCharging(const uint32_t &priority) {
    logger_->debug("startCharging");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->startCharging();
}

std::optional<bool> TIMManager::stopCharging(const uint32_t &priority) {
    logger_->debug("stopCharging");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->stopCharging();
}

std::optional<bool> TIMManager::enableEconomyMode(const uint32_t &priority) {
    logger_->debug("enableEconomyMode");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->enableEconomyMode();
}

std::optional<bool> TIMManager::disableEconomyMode(const uint32_t &priority) {
    logger_->debug("disableEconomyMode");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->disableEconomyMode();
}

std::optional<bool> TIMManager::rebootRobotArmWagon(const uint32_t &priority) {
    logger_->debug("rebootRobotArmWagon");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->rebootRobotArmWagon();
}

std::optional<bool> TIMManager::setObstacleArea(const LHCObstacle &obstacle,
    const uint32_t &priority) {
    logger_->debug("getClosestObstacleAreas");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->setObstacleArea(obstacle);
}

std::optional<bool> TIMManager::devicesRetracted(bool deviceStatus, const uint32_t &priority) {
    logger_->debug("devicesRetracted");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->devicesRetracted(deviceStatus);
}

std::optional<bool> TIMManager::allowMovement(bool allow, const uint32_t &priority) {
    logger_->debug("allowMovement");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->allowMovement(allow);
}

std::optional<bool> TIMManager::acknowledgeAlarms(const uint32_t &priority) {
    logger_->debug("acknowledgeAlarms");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return tim_->acknowledgeAlarms();
}

nlohmann::json TIMManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);

    nlohmann::json statusJSON;
    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["priorityUnderControl"] = simpleAccessControl_.getHighestPriority();
    statusJSON["isConnected"] = tim_->isConnected();
    statusJSON["currentPosition"] = tim_->getCurrentPosition();
    statusJSON["targetPosition"] = tim_->getTargetPosition();
    statusJSON["currentVelocity"] = tim_->getCurrentVelocity();
    statusJSON["targetVelocity"] = tim_->getTargetVelocity();
    statusJSON["currentMaximumVelocity"] = tim_->getCurrentMaximumVelocity();
    statusJSON["isMoving"] = tim_->isMoving();
    statusJSON["isTargetReached"] = tim_->isTargetReached();
    statusJSON["isCharging"] = tim_->isCharging();
    statusJSON["chargingCurrent"] = tim_->getChargingCurrent();
    statusJSON["batteryVoltage"] = tim_->getBatteryVoltage();
    statusJSON["batteryCurrent"] = tim_->getBatteryCurrent();
    statusJSON["isInEconomy"] = tim_->isInEconomy();

    std::array<LHCObstacle, 2> obstacles = tim_->getClosestObstacleAreas();
    if (!obstacles[0].isEmpty()) {
        statusJSON["previousObstacleArea"]["identifier"] = obstacles[0].identifier();
        statusJSON["previousObstacleArea"]["type"] = obstacles[0].type();
        statusJSON["previousObstacleArea"]["startPosition"] = obstacles[0].startPosition();
        statusJSON["previousObstacleArea"]["endPosition"] = obstacles[0].endPosition();
        statusJSON["previousObstacleArea"]["maximumVelocity"] = obstacles[0].maximumVelocity();
        statusJSON["previousObstacleArea"]["mustRetractDevices"] = obstacles[0].mustRetractDevices();  // NOLINT
    } else {
        statusJSON["previousObstacleArea"] = nullptr;
    }
    if (!obstacles[1].isEmpty()) {
        statusJSON["nextObstacleArea"]["identifier"] = obstacles[1].identifier();
        statusJSON["nextObstacleArea"]["type"] = obstacles[1].type();
        statusJSON["nextObstacleArea"]["startPosition"] = obstacles[1].startPosition();
        statusJSON["nextObstacleArea"]["endPosition"] = obstacles[1].endPosition();
        statusJSON["nextObstacleArea"]["maximumVelocity"] = obstacles[1].maximumVelocity();
        statusJSON["nextObstacleArea"]["mustRetractDevices"] = obstacles[1].mustRetractDevices();
    } else {
        statusJSON["nextObstacleArea"] = nullptr;
    }

    LHCObstacle obstacle = tim_->getCurrentObstacleArea();
    if (!obstacle.isEmpty()) {
        statusJSON["currentObstacleArea"]["identifier"] = obstacle.identifier();
        statusJSON["currentObstacleArea"]["type"] = obstacle.type();
        statusJSON["currentObstacleArea"]["startPosition"] = obstacle.startPosition();
        statusJSON["currentObstacleArea"]["endPosition"] = obstacle.endPosition();
        statusJSON["currentObstacleArea"]["maximumVelocity"] = obstacle.maximumVelocity();
        statusJSON["currentObstacleArea"]["mustRetractDevices"] = obstacle.mustRetractDevices();
    } else {
        statusJSON["currentObstacleArea"] = nullptr;
    }

    statusJSON["devicesRetracted"] = tim_->devicesRetracted();
    statusJSON["isFrontWarningFieldActive"] = tim_->isFrontWarningFieldActive();
    statusJSON["isBackWarningFieldActive"] = tim_->isBackWarningFieldActive();
    statusJSON["isSafeToMove"] = tim_->isSafeToMove();

    crf::actuators::tim::TIMAlarms alarms = tim_->getAlarms();
    if (!alarms.isEmpty()) {
        statusJSON["alarms"]["barcodeReaderError"] = alarms.barcodeReaderError();
        statusJSON["alarms"]["batteryError"] = alarms.batteryError();
        statusJSON["alarms"]["chargingArmRequiresAcknowledgement"] = alarms.chargingArmRequiresAcknowledgement();  // NOLINT
        statusJSON["alarms"]["chargingArmMotorError"] = alarms.chargingArmMotorError();
        statusJSON["alarms"]["frontBumperPressed"] = alarms.frontBumperPressed();
        statusJSON["alarms"]["frontLaserScannerError"] = alarms.frontLaserScannerError();
        statusJSON["alarms"]["frontProtectiveFieldReading"] = alarms.frontProtectiveFieldReading();
        statusJSON["alarms"]["backBumperPressed"] = alarms.backBumperPressed();
        statusJSON["alarms"]["backLaserScannerError"] = alarms.backLaserScannerError();
        statusJSON["alarms"]["backProtectiveFieldReading"] = alarms.backProtectiveFieldReading();
        statusJSON["alarms"]["mainMotorRequiresAcknowledgement"] = alarms.mainMotorRequiresAcknowledgement();  // NOLINT
        statusJSON["alarms"]["mainMotorError"] = alarms.mainMotorError();
        statusJSON["alarms"]["positionEncoderError"] = alarms.positionEncoderError();
        statusJSON["alarms"]["positionEncoderReadingError"] = alarms.positionEncoderReadingError();
        statusJSON["alarms"]["velocityEncoderError"] = alarms.velocityEncoderError();
        statusJSON["alarms"]["velocityEncoderReadingError"] = alarms.velocityEncoderReadingError();
        statusJSON["alarms"]["emergencyStop"] = alarms.emergencyStop();
    } else {
        statusJSON["alarms"] = nullptr;
    }
    return statusJSON;
}

}  // namespace crf::actuators::tim
