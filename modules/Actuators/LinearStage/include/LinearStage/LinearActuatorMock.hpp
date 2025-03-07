/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "LinearStage/ILinearActuator.hpp"

namespace crf::actuators::linearactuator {

class LinearActuatorMock : public ILinearActuator {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(crf::expected<bool>, setPosition, (const double& position), (override));
    MOCK_METHOD(crf::expected<bool>, setVelocity, (const double& velocity), (override));

    MOCK_METHOD(crf::expected<double>, getPosition, (), (const override));
    MOCK_METHOD(crf::expected<double>, getVelocity, (), (const override));
};

}  // namespace crf::actuators::linearactuator
