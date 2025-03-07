/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
#include "EnvironmentalSensors/IEnvironmentalSensor.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

/*
 * @brief
 */
class EnvironmentalSensorManager :
    public crf::utility::devicemanager::DeviceManagerWithAutoInitialization {
 public:
    EnvironmentalSensorManager(
        std::shared_ptr<crf::sensors::environmentalsensors::IEnvironmentalSensor> sensor,
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(60));
    EnvironmentalSensorManager(const EnvironmentalSensorManager& other) = delete;
    EnvironmentalSensorManager(EnvironmentalSensorManager&& other) = delete;
    EnvironmentalSensorManager() = delete;
    ~EnvironmentalSensorManager();

    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::sensors::environmentalsensors::IEnvironmentalSensor> sensor_;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
