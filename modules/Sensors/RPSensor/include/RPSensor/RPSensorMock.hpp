/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "RPSensor/IRPSensor.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class RPSensorMock : public IRPSensor {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(std::optional<float>, getDoseRate, (), (override));
    MOCK_METHOD(std::optional<float>, getCumulativeDose, (), (override));
    MOCK_METHOD(bool, resetCumulativeDose, (), (override));
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
