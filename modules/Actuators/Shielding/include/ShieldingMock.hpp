/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#pragma once

#include <optional>

#include "Shielding/IShielding.hpp"

namespace crf::actuators::shielding {

class ShieldingMock : public IShielding {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(bool, open, (), (override));
    MOCK_METHOD(bool, close, (), (override));
    MOCK_METHOD(bool, resetFaultState, (), (override));
    MOCK_METHOD(std::optional<bool>, isInFault, (), (override));
    MOCK_METHOD(std::optional<bool>(), isOpen, (), (override));
    MOCK_METHOD(std::optional<bool>(), isClosed, (), (override));
};

}  // namespace crf::actuators::shielding
