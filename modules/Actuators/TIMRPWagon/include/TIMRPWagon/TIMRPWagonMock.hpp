/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include "TIMRPWagon/ITIMRPWagon.hpp"

namespace crf::actuators::timrpwagon {

class TIMRPWagonMock : public ITIMRPWagon {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(bool, isConnected, (), (override));

    MOCK_METHOD(RPArmPosition, getRPArmPosition, (), (override));

    MOCK_METHOD(std::optional<bool>, retractRPArm, (), (override));
    MOCK_METHOD(std::optional<bool>, deployRPArm, (), (override));

    MOCK_METHOD(std::optional<bool>, moveRPArmUp, (), (override));
    MOCK_METHOD(std::optional<bool>, moveRPArmDown, (), (override));

    MOCK_METHOD(std::optional<bool>, stopRPArm, (), (override));

    MOCK_METHOD(std::optional<bool>, lockRPArm, (), (override));
    MOCK_METHOD(std::optional<bool>, unlockRPArm, (), (override));

    MOCK_METHOD(std::optional<bool>, isRPArmInError, (), (override));
    MOCK_METHOD(std::optional<bool>, resetRPArmDriver, (), (override));
    MOCK_METHOD(std::optional<bool>, acknowledgeErrors, (), (override));

    MOCK_METHOD(std::shared_ptr<TIMRPWagonConfiguration>, getConfiguration, (), (override));
};

}  // namespace crf::actuators::timrpwagon
