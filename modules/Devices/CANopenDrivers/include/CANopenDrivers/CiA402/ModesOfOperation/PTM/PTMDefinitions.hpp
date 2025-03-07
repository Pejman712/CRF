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
 * @ingroup group_ptm
 * @brief Enum of values of the controlword for confinguring the profile
 * velocity mode.
 *
 */
struct ControlWordPTMValues {
    enum Type : uint16_t {
        Execute = 0x000F,
        Halt = 0X010F
    };
};

using ControlWordPTM = ControlWordPTMValues::Type;

/**
 * @ingroup group_ptm
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in torque mode.
 * Situations where a certain bit is active is when the target torque is reached and
 * when the torque exceeds the limit values.
 *
 */
struct StatusWordPTMValues {
    enum Type : uint16_t {
        TargetReached = 0x0400,
        TorqueLimited = 0x0800
    };
};

using StatusWordPTM = StatusWordPTMValues::Type;

/**
 * @ingroup group_ptm
 * @brief Enum of bit masks for the
 * StatusWordPTM.
 *
 */
struct StatusWordPTMMaskValues {
    enum Type : uint16_t {
        TargetReached = 0x0400,
        TorqueLimited = 0x0800
    };
};

using StatusWordPTMMask = StatusWordPTMMaskValues::Type;

/**
 * @ingroup group_ptm
 * @brief Enum for the type of profile the torque should follow
 *
 */
struct TorqueProfileTypePTMValues {
    enum Type : int16_t {
        Linear = 0x0000,
        Sin2 = 0X0001
    };
};

using TorqueProfileTypePTM = TorqueProfileTypePTMValues::Type;

}  // namespace crf::devices::canopendrivers

