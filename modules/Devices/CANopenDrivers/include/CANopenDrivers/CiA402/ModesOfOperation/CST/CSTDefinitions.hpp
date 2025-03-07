/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstdint>

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cst
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in cyclic synchronous torque mode.
 * Situation where a certain bit is active is when the drive follows the
 * target value of the torque. (When the target torque is used as input to the control loop).
 *
 */
struct StatusWordCSTValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000,
    };
};

using StatusWordCST = StatusWordCSTValues::Type;

/**
 * @ingroup group_cst
 * @brief Enum of bit masks for the
 * StatusWordCST.
 *
 */
struct StatusWordCSTMaskValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000,
    };
};

using StatusWordCSTMask = StatusWordCSTMaskValues::Type;

/**
 * @ingroup group_cst
 * @brief Enum that explains the torque limitation options.
 * For more information check the CiA 402 profile.
 *
 */
struct TorqueLimitationOptionCodes {
    enum Type : int8_t {
        NoLimitVelocityAcceleration = 0x00,
        TorqueReducedUntilZero = 0x01,
        TorqueModifiedInsideRampLimits = 0x02,
        TorqueModifiedAccordingToSign = 0x03,
        TorqueModifiedInsideMaxTorqueRange = 0x04
    };
};

using TorqueLimitationOptionCode = TorqueLimitationOptionCodes::Type;

}  // namespace crf::devices::canopendrivers
