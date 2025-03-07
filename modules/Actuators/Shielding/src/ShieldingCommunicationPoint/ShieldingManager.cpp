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

#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"

namespace crf::actuators::shielding {

ShieldingManager::ShieldingManager(
    std::shared_ptr<IShielding> shielding,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        shielding,
        initializationTimeout,
        controlAccessTimeout),
    shielding_(shielding),
    logger_("ShieldingManager") {
    logger_->debug("CTor");
}

ShieldingManager::~ShieldingManager() {
    logger_->debug("DTor");
}

nlohmann::json ShieldingManager::getStatus() {
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

    std::optional<bool> open = shielding_->isOpen();
    std::optional<bool> close = shielding_->isClosed();
    if (open && close) {
        if (open.value()) {
            statusJSON["position"] = "open";
        } else if (close.value()) {
            statusJSON["position"] = "close";
        } else {
            statusJSON["position"] = "unknown";
        }
    } else {
        statusJSON["position"] = "unknown";
    }

    std::optional<bool> inFault = shielding_->isInFault();
    if (inFault) {
        statusJSON["isInFault"] = inFault.value();
    }

    return statusJSON;
}

bool ShieldingManager::open(const uint32_t &priority) {
    logger_->debug("open");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!shielding_->open()) {
        logger_->error("Failed to open shielding");
        return false;
    }
    return true;
}

bool ShieldingManager::close(const uint32_t &priority) {
    logger_->debug("close");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!shielding_->close()) {
        logger_->error("Failed to close shielding");
        return false;
    }
    return true;
}

bool ShieldingManager::resetFaultState(const uint32_t &priority) {
    logger_->debug("resetFaultState");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (!shielding_->resetFaultState()) {
        logger_->error("Failed to resetFaultState shielding");
        return false;
    }
    return true;
}

}  // namespace crf::actuators::shielding
