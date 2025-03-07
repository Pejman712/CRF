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
 * @ingroup group_can_open_drivers
 * @brief Enum of types of values that can be in the
 * registers of the driver. The types go from boolean to
 * identitiy.
 *
 */
enum Type : uint16_t {
    BOOLEAN = 0x0001,
    INT_8 = 0x0002,
    INT_16 = 0x0003,
    INT_32 = 0x0004,
    INT_64 = 0x0015,
    UINT_8 = 0x0005,
    UINT_16 = 0x0006,
    UINT_32 = 0x0007,
    UINT_64 = 0x001B,
    VISIBLE_STRING = 0x009,
    OCTET_STRING = 0x000A,
    PDO_MAPPING = 0x0021,
    IDENTITIY = 0x0023
};

}  // namespace crf::devices::canopendrivers
