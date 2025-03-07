#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <Dynamixel/IDynamixelSDK.hpp>

namespace crf {
namespace devices {
namespace dynamixelstepper {

class DynamixelSDK : public IDynamixelSDK {
 public:
    DynamixelSDK() = default;
    ~DynamixelSDK() = default;

    dynamixel::PortHandler *getPortHandler(const char *) override;
    dynamixel::PacketHandler *getPacketHandler(float) override;

    bool openPort(dynamixel::PortHandler *) override;
    void closePort(dynamixel::PortHandler *) override;
    bool setBaudRate(dynamixel::PortHandler *, const int) override;

    int write1ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t, uint16_t,
        uint8_t, uint8_t *) override;
    int write2ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t, uint16_t,
        uint16_t, uint8_t *) override;
    int read2ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t, uint16_t,
        uint16_t *, uint8_t *) override;
    const char *getRxPacketError(dynamixel::PacketHandler *, uint8_t) override;
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
