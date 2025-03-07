#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include "Dynamixel/IDynamixelSDK.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

class DynamixelSDKMock : public IDynamixelSDK {
 public:
    MOCK_METHOD1(getPortHandler, dynamixel::PortHandler *(const char *));
    MOCK_METHOD1(getPacketHandler, dynamixel::PacketHandler *(float));

    MOCK_METHOD1(openPort, bool(dynamixel::PortHandler *));
    MOCK_METHOD1(closePort, void(dynamixel::PortHandler *));
    MOCK_METHOD2(setBaudRate, bool(dynamixel::PortHandler *, const int));

    MOCK_METHOD6(write1ByteTxRx, int(dynamixel::PacketHandler *,
        dynamixel::PortHandler *, uint8_t, uint16_t, uint8_t, uint8_t *));
    MOCK_METHOD6(write2ByteTxRx, int(dynamixel::PacketHandler *,
        dynamixel::PortHandler *, uint8_t, uint16_t, uint16_t, uint8_t *));
    MOCK_METHOD6(read2ByteTxRx, int(dynamixel::PacketHandler *,
        dynamixel::PortHandler *, uint8_t, uint16_t, uint16_t *, uint8_t *));
    MOCK_METHOD2(getRxPacketError, const char *(dynamixel::PacketHandler *, uint8_t));
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
