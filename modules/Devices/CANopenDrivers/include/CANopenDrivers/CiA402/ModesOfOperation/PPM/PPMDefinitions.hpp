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
 * @ingroup group_ppm
 * @brief Enum of values of the controlword used for configuring
 * the profile position mode.
 *
 */
struct ControlWordPPMValues {
    enum Type : uint16_t {
        Execute = 0x000F,
        Halt = 0X010F,
        Relative = 0x004F,
        Absolute = 0x000F,
        AssumeTargetPosition = 0x001F,
        ActivateChangeSetPoint = 0x002F,
        DeactivateChangeSetPoint = 0x000F,
        ActivateLaterChangeSetPoint = 0x020F,
        DeactivateLaterChangeSetPoint = 0x000F,
        ActivateEndlessMovement = 0x800F,
        DeactivateEndlessMovement = 0x000F
    };
};

using ControlWordPPM = ControlWordPPMValues::Type;

/**
 * @ingroup group_ppm
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in profile position mode.
 * Situations where certain bits are active is when a target is reached,
 * when new setpoint is acknowledged and when there is an error of
 * following the reference position.
 *
 */
struct StatusWordPPMValues {
    enum Type : uint16_t {
        TargetReachedPPM = 0x0400,
        SetpointAcknowledgePPM = 0x1000,
        FollowingErrorPPM = 0x2000
    };
};

using StatusWordPPM = StatusWordPPMValues::Type;

/**
 * @ingroup group_ppm
 * @brief Enum of bit masks for the
 * StatusWordPPM.
 *
 */
struct StatusWordPPMMaskValues {
    enum Type : uint16_t {
        TargetReachedMaskPPM = 0x0400,
        SetpointAcknowledgeMaskPPM = 0x1000,
        FollowingErrorMaskPPM = 0x2000
    };
};

using StatusWordPPMMask = StatusWordPPMMaskValues::Type;

/**
 * @ingroup group_ppm
 * @brief Enum class for type of position reference
 * for the profile position mode.
 *
 */
enum class PositionReference {
    Absolute = 0,
    Relative = 1
};

}  // namespace crf::devices::canopendrivers
