/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
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

#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::timrpwagon {

TIMRPWagonManager::TIMRPWagonManager(std::shared_ptr<actuators::timrpwagon::ITIMRPWagon> timRPWagon,
    const std::chrono::milliseconds& inizializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    crf::utility::devicemanager::DeviceManagerWithPriorityAccess(timRPWagon, inizializationTimeout,
        controlAccessTimeout),
    timRPWagon_(timRPWagon),
    logger_("TIMRPWagonManager") {
    logger_->debug("CTor");
}

TIMRPWagonManager::~TIMRPWagonManager() {
    logger_->debug("DTor");
}

std::optional<bool> TIMRPWagonManager::retractRPArm(const uint32_t &priority) {
    logger_->debug("retractRPArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->retractRPArm();
}

std::optional<bool> TIMRPWagonManager::deployRPArm(const uint32_t &priority) {
    logger_->debug("deployRPArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->deployRPArm();
}

std::optional<bool> TIMRPWagonManager::moveRPArmUp(const uint32_t &priority) {
    logger_->debug("moveRPArmUp");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->moveRPArmUp();
}

std::optional<bool> TIMRPWagonManager::moveRPArmDown(const uint32_t &priority) {
    logger_->debug("moveRPArmDown");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->moveRPArmDown();
}

std::optional<bool> TIMRPWagonManager::stopRPArm(const uint32_t &priority) {
    logger_->debug("stopRPArm");
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->stopRPArm();
}

std::optional<bool> TIMRPWagonManager::lockRPArm(const uint32_t &priority) {
    logger_->debug("lockRPArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->lockRPArm();
}

std::optional<bool> TIMRPWagonManager::unlockRPArm(const uint32_t &priority) {
    logger_->debug("unlockRPArm");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->unlockRPArm();
}

std::optional<bool> TIMRPWagonManager::resetRPArmDriver(const uint32_t &priority) {
    logger_->debug("resetRPArmDriver");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->resetRPArmDriver();
}

std::optional<bool> TIMRPWagonManager::acknowledgeErrors(const uint32_t &priority) {
    logger_->debug("acknowledgeErrors");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) return std::nullopt;
    if (!initializeDevice()) return std::nullopt;
    return timRPWagon_->acknowledgeErrors();
}

nlohmann::json TIMRPWagonManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);

    nlohmann::json statusJSON;
    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["priorityUnderControl"] = simpleAccessControl_.getHighestPriority();
    statusJSON["isConnected"] = timRPWagon_->isConnected();
    statusJSON["rpArmPosition"] = timRPWagon_->getRPArmPosition();

    std::optional<bool> rpArmInError = timRPWagon_->isRPArmInError();
    if (rpArmInError) {
        statusJSON["isRPArmInError"] = rpArmInError.value();
    } else {
        statusJSON["isRPArmInError"] = nullptr;
    }
    return statusJSON;
}

}  // namespace crf::actuators::timrpwagon
