/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorManager.hpp"

namespace crf::actuators::linearactuator {

LinearActuatorManager::LinearActuatorManager(
    std::shared_ptr<crf::actuators::linearactuator::ILinearActuator> actuator,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(actuator, initializationTimeout, controlAccessTimeout),
    actuator_(actuator) {
    logger_ = crf::utility::logger::EventLogger("LinearActuatorManager");
    logger_->debug("CTor");
}

LinearActuatorManager::~LinearActuatorManager() {
    logger_->debug("DTor");
}

crf::expected<bool> LinearActuatorManager::setPosition(
    const uint32_t& priority,
    const double &position) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return actuator_->setPosition(position);
}

crf::expected<bool> LinearActuatorManager::setVelocity(
    const uint32_t& priority,
    const double& velocity) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return actuator_->setVelocity(velocity);
}

nlohmann::json LinearActuatorManager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["priorityUnderControl"] = simpleAccessControl_.getHighestPriority();

    statusJSON["position"] = actuator_->getPosition();
    statusJSON["velocity"] = actuator_->getVelocity();

    return statusJSON;
}

}  // namespace crf::actuators::linearactuator
