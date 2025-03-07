/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::devices::canopendevices {

namespace ModesOfOperation {
    const uint8_t CurrentMode = 0xFD;
    const uint8_t ProfilePositionMode = 1;
    const uint8_t VelocityMode = 2;
    const uint8_t ProfileVelocityMode = 3;
    const uint8_t ProfileTorqueMode = 4;
    const uint8_t HomingMode = 6;
    const uint8_t InterpolatedPositionMode = 7;
    const uint8_t CyclicSynchronousPositionMode = 8;
    const uint8_t CyclicSynchronousVelocityMode = 9;
    const uint8_t CyclicSynchronousTorqueMode = 10;
}  // namespace ModesOfOperation

namespace FrameManagement {
    const u_int32_t FUNCTION_CODE_MASK = 0x00000780;
    const u_int32_t NODE_ID_MASK = 0x0000007F;
}  // namespace FrameManagement

namespace MessageFilters {
    const u_int8_t SDO_RESPONSE = 11;
    const u_int8_t SDO_REQUEST = 12;
    const u_int8_t PDO1_RX = 3;
    const u_int8_t PDO2_RX = 5;
    const u_int8_t PDO3_RX = 7;
    const u_int8_t PDO4_RX = 9;
    const u_int8_t PDO1_TX = 4;
    const u_int8_t PDO2_TX = 6;
    const u_int8_t PDO3_TX = 8;
    const u_int8_t NMT_HEARTBEAT = 14;
    const u_int8_t EMERGENCY = 1;
    const u_int8_t SYNC = 1;  // To distinguish if the can_frame is empty or there is an emergency.
}  // namespace MessageFilters

namespace NMTManagement {
    const u_int8_t NMTBoot = 0x00;
    const u_int8_t NMTStopped = 0x04;
    const u_int8_t NMTOperational = 0x05;
    const u_int8_t NMTPreOperational = 0x7F;
}  // namespace NMTManagement

enum statusWord {
    NOT_READY_TO_SWITCH_ON = 0x0000,
    READY_TO_SWITCH_ON = 0x0021,
    SWITCHED_ON = 0x0023,
    OPERATION_ENABLED = 0x0027,
    FAULT = 0x0008,
    IN_QUICK_STOP  = 0x0007,
    SWITCHED_ON_DISABLED = 0x0040,
    FAULT_REACTION_ACTIVE = 0x000F
};

enum controlWord {
    SHUTDOWN = 0x0006,
    SWITCH_ON = 0x0007,
    SWITCH_ON_AND_ENABLE_OPERATION = 0x000F,
    DISABLE_VOLTAGE = 0x0000,
    QUICK_STOP = 0x0002,
    DISABLE_OPERATION = 0x0007,
    ENABLE_OPERATION = 0x000F,
    FAULT_RESET1 = 0x0000,
    FAULT_RESET2 = 0x0080
};

}  // namespace crf::devices::canopendevices
