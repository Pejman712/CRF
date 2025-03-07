/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "RPSensor/IRPSensor.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

/*
 * @brief
 */
class RPSensorManager : public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {
 public:
    RPSensorManager(
        std::shared_ptr<crf::sensors::rpsensor::IRPSensor> sensor,
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(15));
    RPSensorManager(const RPSensorManager& other) = delete;
    RPSensorManager(RPSensorManager&& other) = delete;
    RPSensorManager() = delete;
    ~RPSensorManager();

    nlohmann::json getStatus() override;

    std::optional<bool> resetCumulativeDose(const uint32_t &priority);

 private:
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> sensor_;
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
