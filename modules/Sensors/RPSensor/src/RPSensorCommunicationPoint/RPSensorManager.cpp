/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "RPSensor/IRPSensor.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

RPSensorManager::RPSensorManager(
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> sensor,
    const std::chrono::milliseconds& inizializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(sensor, inizializationTimeout, controlAccessTimeout),
    sensor_(sensor) {
    logger_ = crf::utility::logger::EventLogger("RPSensorManager");
    logger_->debug("CTor");
    }

RPSensorManager::~RPSensorManager() {
    logger_->debug("DTor");
}

nlohmann::json RPSensorManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    uint32_t priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;

    std::optional<float> doseRate = sensor_->getDoseRate();
    std::optional<float> cumulativeDose = sensor_->getCumulativeDose();
    if (!doseRate || !cumulativeDose) {
        return statusJSON;
    }
    statusJSON["doseRate"] = doseRate.value();
    statusJSON["cumulativeDose"] = cumulativeDose.value();

    return statusJSON;
}

std::optional<bool> RPSensorManager::resetCumulativeDose(const uint32_t &priority) {
    logger_->debug("resetCumulativeDose");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    if (!checkCommandPriority(priority)) {
        return std::nullopt;
    }
    if (!initializeDevice()) {
        return std::nullopt;
    }
    return sensor_->resetCumulativeDose();
}

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
