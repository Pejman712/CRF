#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <memory>
#include <utility>
#include <vector>

#include "Dynamixel/IDynamixel.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

class DynamixelMock : public IDynamixel {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_METHOD2(writeDynamixel,
        bool(uint8_t, uint16_t));
    MOCK_METHOD1(readCurrentPosition,
        int16_t(uint8_t));
    MOCK_METHOD0(getZeroPosition,
        std::vector<uint16_t>());
    MOCK_METHOD0(getConfiguration,
        DynamixelConfiguration());
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
