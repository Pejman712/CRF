/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "Tool/ActiveToolCommunicationPoint/ActiveToolManager.hpp"

namespace crf::devices::tool {

ActiveToolManager::ActiveToolManager(
    std::shared_ptr<crf::devices::tool::IActiveTool> activetool,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(activetool, initializationTimeout, controlAccessTimeout),
    activetool_(activetool) {
    logger_ = crf::utility::logger::EventLogger("ActiveToolManager");
    logger_->debug("CTor");
}

ActiveToolManager::~ActiveToolManager() {
    logger_->debug("DTor");
}

crf::expected<bool> ActiveToolManager::activate(
    const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return activetool_->activate();
}

crf::expected<bool> ActiveToolManager::deactivate(
    const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return activetool_->deactivate();
}

nlohmann::json ActiveToolManager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    uint32_t priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;

    statusJSON["isActive"] = activetool_->isActive();

    return statusJSON;
}

}   // namespace crf::devices::tool
