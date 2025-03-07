/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <optional>

#include <gmock/gmock.h>

#include "EnvironmentalSensors/IEnvironmentalSensor.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

class EnvironmentalSensorMock : public IEnvironmentalSensor {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(std::optional<float>, getMeasurement, (), (override));
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
