/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <chrono>

#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"

namespace crf::actuators::mechanicalstabilizer {

MechanicalStabilizerManager::MechanicalStabilizerManager(
    std::shared_ptr<IMechanicalStabilizer> stabilizer,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        stabilizer,
        initializationTimeout,
        controlAccessTimeout),
    stabilizer_(stabilizer),
    logger_("MechanicalStabilizerManager") {
    logger_->debug("CTor");
}

MechanicalStabilizerManager::~MechanicalStabilizerManager() {
    logger_->debug("DTor");
}

nlohmann::json MechanicalStabilizerManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);

    nlohmann::json statusJSON;
    uint32_t priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }
    std::optional<bool> active = stabilizer_->isActivated();
    if (active) {
        statusJSON["active"] = active.value();
    }
    std::optional<bool> inFault = stabilizer_->isInFault();
    if (inFault) {
        statusJSON["isInFault"] = inFault.value();
    }
    return statusJSON;
}

bool MechanicalStabilizerManager::activate(const uint32_t &priority) {
    logger_->debug("activate");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!stabilizer_->activate()) {
        logger_->error("Failed to activate stabilizer");
        return false;
    }
    return true;
}

bool MechanicalStabilizerManager::deactivate(const uint32_t &priority) {
    logger_->debug("deactivate");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!stabilizer_->deactivate()) {
        logger_->error("Failed to deactivate stabilizer");
        return false;
    }
    return true;
}

bool MechanicalStabilizerManager::resetFaultState(const uint32_t &priority) {
    logger_->debug("resetFaultState");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!stabilizer_->resetFaultState()) {
        logger_->error("Failed to resetFaultState shielding");
        return false;
    }
    return true;
}

}  // namespace crf::actuators::mechanicalstabilizer
