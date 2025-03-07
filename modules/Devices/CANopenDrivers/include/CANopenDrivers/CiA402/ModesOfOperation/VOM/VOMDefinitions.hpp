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
 * @ingroup group_vom
 * @brief Enum of values of the controlword for confinguring the velocity mode.
 *
 */
struct ControlWordVOMValues {
    enum Type : uint16_t {
        Execute = 0x007F,
        Halt = 0X010F
    };
};

using ControlWordVOM = ControlWordVOMValues::Type;

/**
 * @ingroup group_vom
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in velocity mode.
 * Situation where a certain bit is active is when the limit
 * of the speed is exceeded.
 *
 */
struct StatusWordVOMValues {
    enum Type : uint16_t {
        SpeedLimited = 0x0800
    };
};

using StatusWordVOM = StatusWordVOMValues::Type;
/**
 * @ingroup group_vom
 * @brief Enum of bit masks for the
 *  StatusWordVOM.
 *
 */
struct StatusWordVOMMaskValues {
    enum Type : uint16_t {
        SpeedLimited = 0x0800
    };
};

using StatusWordVOMMask = StatusWordVOMMaskValues::Type;

/**
 * @ingroup group_vom
 * @brief Enum for the unit of the velocity.
 * The default unit is rpm. Configuring the unit is done as : numerator/denominator.
 *
 */
struct DimensionFactorVOM {
    int32_t numerator;
    int32_t denominator;
};

/**
 * @ingroup group_vom
 * @brief Struct for setting the parameters of the velocity
 * set-point factor. It is used to modify the resolution or direct range
 * of specified set-point.
 *
 */
struct VelocitySetPointFactorVOM {
    int16_t numerator;
    int16_t denominator;
};

/**
 * @ingroup group_vom
 * @brief Enum for defining limits of the velocity.
 *
 */
struct VelocityMinMaxAmountVOM {
    uint32_t min;
    uint32_t max;
};

/**
 * @ingroup group_vom
 * @brief Enum for defining the velocity quick stop ramp.
 * The ramp is calculated as quickstop = deltaSpeed/deltaTime.
 *
 */
struct VelocityQuickStopVOM {
    uint32_t deltaSpeed;
    uint16_t deltaTime;
};

}  // namespace crf::devices::canopendrivers
