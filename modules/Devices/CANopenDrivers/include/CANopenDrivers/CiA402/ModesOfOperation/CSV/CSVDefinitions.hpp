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
 * @ingroup group_csv
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in cyclic synchronous velocity mode.
 * Situation where a certain bit is active is when the drive follows the
 * target value of the velocity.
 *
 */
struct StatusWordCSVValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000
    };
};

using StatusWordCSV = StatusWordCSVValues::Type;

/**
 * @ingroup group_csv
 * @brief Enum of bit masks for the
 * StatusWordCSV.
 *
 */
struct StatusWordCSVMaskValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000
    };
};

using StatusWordCSVMask = StatusWordCSVMaskValues::Type;

}  // namespace crf::devices::canopendrivers

