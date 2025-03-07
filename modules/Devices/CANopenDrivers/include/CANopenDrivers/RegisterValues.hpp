/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <any>
#include <cstdint>

#include "CANopenDrivers/TypeDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_can_open_drivers
 * @brief Struct to group a register (index and sub-index) with it's value and
 * it's value type
 *
 */
struct RegisterValues {
    RegisterValues() = default;
    RegisterValues(uint16_t ix, uint8_t subix, std::any val, Type ty):
        index(ix), subindex(subix), value(val), type(ty) {}
    uint16_t index;
    uint8_t subindex;
    std::any value;
    Type type;
};

}  // namespace crf::devices::canopendrivers
