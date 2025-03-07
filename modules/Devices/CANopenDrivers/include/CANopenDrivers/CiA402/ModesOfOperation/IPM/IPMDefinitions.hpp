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
 * @ingroup group_ipm
 * @brief Enum of values of the controlword used for configuring
 * the interpolated position mode.
 *
 */
struct ControlWordIPMValues {
    enum Type : uint16_t {
        Execute = 0x000F,
        Halt = 0X010F,
        EnableIPM = 0x001F,
        DisableIPM = 0X000F
    };
};

using ControlWordIPM = ControlWordIPMValues::Type;

/**
 * @ingroup group_ipm
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in interpolated position mode.
 * Situations where certain bits are active is when a target is reached,
 * when new setpoint is acknowledged and when there is an error of
 * following the reference position.
 *
 */
struct StatusWordIPMValues {
    enum Type : int16_t {
        TargetReached = 0x0400,
        IPMModeActive = 0x1000,
        FollowingError = 0x2000
    };
};

using StatusWordIPM = StatusWordIPMValues::Type;

/**
 * @ingroup group_ipm
 * @brief Enum of bit masks for the
 * StatusWordIPM.
 *
 */
struct StatusWordIPMMaskValues {
    enum Type : uint16_t {
        TargetReached = 0x0400,
        IPMModeActive = 0x1000,
        FollowingError = 0x2000
    };
};

using StatusWordIPMMask = StatusWordIPMMaskValues::Type;

/**
 * @brief
 *
 */
struct InterpolationSubmodeValues {
    enum Type : int16_t {
        LinearInterpolation = 0x0000
    };
};

using InterpolationSubmode = InterpolationSubmodeValues::Type;

/**
 * @ingroup group_ipm
 * @brief A struct for configuring the parameters for the buffer.
 * The object 60C4h -Interpolation data configuration provides the maximum buffer size, indicates the configured buffer
 * organization of interpolation data, and provides objects to define the size of the data record
 * and to clear the buffers. This object is used to enable the drive device to receive the needed
 * data in advance. It also is used to store the positions and further data sent by the control device.
*/
struct InterpolationDataConfiguration {
    uint32_t maximumBufferSize;
    uint32_t actualBufferSize;
    uint8_t bufferOrganization;
    uint16_t bufferPosition;
    uint8_t sizeOfDataRecord;
    uint8_t bufferClear;
};

}  // namespace crf::devices::canopendrivers
