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

#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorManager.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

EnvironmentalSensorManager::EnvironmentalSensorManager(
    std::shared_ptr<crf::sensors::environmentalsensors::IEnvironmentalSensor> sensor,
    const std::chrono::milliseconds& inizializationTimeout) :
    DeviceManagerWithAutoInitialization(sensor, inizializationTimeout),
    sensor_(sensor) {
    logger_ = crf::utility::logger::EventLogger("EnvironmentalSensorManager");
    logger_->debug("CTor");
    }

EnvironmentalSensorManager::~EnvironmentalSensorManager() {
    logger_->debug("DTor");
}

nlohmann::json EnvironmentalSensorManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    std::optional<float> measurement = sensor_->getMeasurement();
    if (measurement) {
        statusJSON["measurement"] = measurement.value();
    } else {
        statusJSON["measurement"] = nullptr;
    }

    return statusJSON;
}

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
