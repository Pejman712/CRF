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
 * @ingroup group_pvm
 * @brief Enum of values of the controlword for confinguring the profile
 * velocity mode.
 *
 */
struct ControlWordPVMValues {
    enum Type : uint16_t {
        Execute = 0x000F,
        Halt = 0X010F
    };
};

using ControlWordPVM = ControlWordPVMValues::Type;

/**
 * @ingroup group_pvm
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in profile velocity mode.
 * Situations where certain bits are active is when a target is reached,
 * when new setpoint is acknowledged and when there is an error of
 * following the reference position.
 *
 */
struct StatusWordPVMValues {
    enum Type : uint16_t {
        TargetReached = 0x0400,
        SpeedLimited = 0x0800,
        SpeedIsZero = 0x1000,
        MaxSlippageError = 0x2000
    };
};

using StatusWordPVM = StatusWordPVMValues::Type;

/**
 * @ingroup group_pvm
 * @brief Enum of bit masks for the
 * StatusWordPVM.
 *
 */
struct StatusWordPVMMaskValues {
    enum Type : uint16_t {
        TargetReached = 0x0400,
        SpeedLimited = 0x0800,
        SpeedIsZero = 0x1000,
        MaxSlippageError = 0x2000
    };
};

using StatusWordPVMMask = StatusWordPVMValues::Type;

}  // namespace crf::devices::canopendrivers
