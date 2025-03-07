/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#pragma once

#include <optional>

#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"

namespace crf::actuators::mechanicalstabilizer {

class MechanicalStabilizerMock : public IMechanicalStabilizer {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(bool, activate, (), (override));
    MOCK_METHOD(bool, deactivate, (), (override));
    MOCK_METHOD(bool, resetFaultState, (), (override));
    MOCK_METHOD(std::optional<bool>, isInFault, (), (override));
    MOCK_METHOD(std::optional<bool>(), isActivated, (), (override));
    MOCK_METHOD(std::optional<bool>(), isDeactivated, (), (override));
};

}  // namespace crf::actuators::mechanicalstabilizer
