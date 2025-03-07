/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf {
namespace devices {
namespace ethercatdevices {

namespace constants {
    const int cycleTime_timedCheck = 2;  // milliseconds
    const int totalTime_timedCheck = 3000;  // milliseconds
}  // namespace constants

namespace modesofoperation {
    const int8_t ProfilePositionMode = 1;
    const int8_t ProfileVelocityMode = 3;
    const int8_t ProfileTorqueMode = 4;
    const int8_t HomingMode = 6;
    const int8_t CyclicSynchronousPositionMode = 8;
    const int8_t CyclicSynchronousVelocityMode = 9;
    const int8_t CyclicSynchronousTorqueMode = 10;
}  // namespace modesofoperation

namespace statusword {
    const uint16_t InFault_Mask = 0x004F;
    const uint16_t InFault_Value = 0x0008;
    const uint16_t InQuickStop_Mask = 0x006F;
    const uint16_t InQuickStop_Value = 0x0007;
    const uint16_t IsEnabled_Mask = 0x006F;
    const uint16_t IsEnabled_Value = 0x0027;
    const uint16_t IsReadyToSwitchOn_Mask = 0x006F;
    const uint16_t IsReadyToSwitchOn_Value = 0x0021;
    const uint16_t IsSwitchOnDisabled_Mask = 0x004F;
    const uint16_t IsSwitchOnDisabled_Value = 0x0040;
    const uint16_t IsSwitchedOn_Mask = 0x006F;
    const uint16_t IsSwitchedOn_Value = 0x0023;
    const uint16_t bit_setNewPointAck = 0x1000;
    const uint16_t bit_targetReached = 0x0400;
    const uint16_t bit_internalLimitActive = 0x0800;
}  // namespace statusword

namespace controlword {
    const uint16_t ShutDown_AND = 0xFF7E;
    const uint16_t ShutDown_OR = 0x0006;
    const uint16_t QuickStop_AND = 0xFF7B;
    const uint16_t QuickStop_OR = 0x0002;
    const uint16_t EnableOperation_AND = 0xFF7F;
    const uint16_t EnableOperation_OR = 0x000F;
    const uint16_t DisableOperation_AND = 0xFF77;
    const uint16_t DisableOperation_OR = 0x0007;
    const uint16_t DisableVoltage_AND = 0xFF7D;
    const uint16_t DisableVoltage_OR = 0x0000;
    const uint16_t FaultReset_AND = 0xFF7F;
    const uint16_t FaultReset_OR = 0x0080;
    const uint16_t bit_halt = 0x0100;
    const uint16_t bit_newSetPoint = 0x0010;
    const uint16_t bit_relative = 0x0040;
}  // namespace controlword

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
