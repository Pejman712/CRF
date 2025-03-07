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
 * @ingroup group_csp
 * @brief Enum of values of the statusword when a certain bit is
 * active when the drive is in cyclic synchronous position mode.
 * Situations where certain bits are active is when the drive follows the
 * target value of the position and when a following error occcurs aka when
 * the actual position and demand position of the drive exceed the following error value.
 *
 */
struct StatusWordCSPValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000,
        FollowingError = 0x2000
    };
};

using StatusWordCSP = StatusWordCSPValues::Type;

/**
 * @ingroup group_csp
 * @brief Enum of bit masks for the StatusWordCSP.
 *
 */
struct StatusWordCSPMaskValues {
    enum Type : uint16_t {
        DriveFollowsCommandValue = 0x1000,
        FollowingError = 0x2000
    };
};

using StatusWordCSPMask = StatusWordCSPMaskValues::Type;

}  // namespace crf::devices::canopendrivers
