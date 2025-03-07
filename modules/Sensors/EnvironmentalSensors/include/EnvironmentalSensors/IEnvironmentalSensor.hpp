/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <optional>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

class IEnvironmentalSensor : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IEnvironmentalSensor() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Function to retrieve the measurements from the specific sensor
     * @return std::nullopt if the sensor fails to retrieve data
     * @return a float value with the data if succesfull
     */
    virtual std::optional<float> getMeasurement() = 0;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
