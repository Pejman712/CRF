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
 * @brief Enum class that defines most of the sub-indexes present in the 301 CANOpen
 * profile (the communication profile)
 */
struct SubindexValues {
    enum Type : uint8_t {
        SUB0 = 0x00,
        SUB1 = 0x01,
        SUB2 = 0x02,
        SUB3 = 0x03,
        SUB4 = 0x04,
        SUB5 = 0x05,
        SUB6 = 0x06,
        SUB7 = 0x07,
        SUB8 = 0x08,
        SUB9 = 0x09,
        SUBA = 0x0A,
        SUBB = 0x0B,
        SUBC = 0x0C,
        SUBD = 0x0D,
        SUBE = 0x0E,
        SUBF = 0x0F,

        SUB10 = 0x10,
        SUB11 = 0x11,
        SUB12 = 0x12,
        SUB13 = 0x13,
        SUB14 = 0x14,
        SUB15 = 0x15,
        SUB16 = 0x16,
        SUB17 = 0x17,
        SUB18 = 0x18,
        SUB19 = 0x19,
        SUB1A = 0x1A,
        SUB1B = 0x1B,
        SUB1C = 0x1C,
        SUB1D = 0x1D,
        SUB1E = 0x1E,
        SUB1F = 0x1F,

        SUB20 = 0x20,
        SUB21 = 0x21,
        SUB22 = 0x22,
        SUB23 = 0x23,
        SUB24 = 0x24,
        SUB25 = 0x25,
        SUB26 = 0x26,
        SUB27 = 0x27,
        SUB28 = 0x28,
        SUB29 = 0x29,
        SUB2A = 0x2A,
        SUB2B = 0x2B,
        SUB2C = 0x2C,
        SUB2D = 0x2D,
        SUB2E = 0x2E,
        SUB2F = 0x2F,

        SUB30 = 0x30,
        SUB31 = 0x31,
        SUB32 = 0x32,
        SUB33 = 0x33,
        SUB34 = 0x34,
        SUB35 = 0x35,
        SUB36 = 0x36,
        SUB37 = 0x37,
        SUB38 = 0x38,
        SUB39 = 0x39,
        SUB3A = 0x3A,
        SUB3B = 0x3B,
        SUB3C = 0x3C,
        SUB3D = 0x3D,
        SUB3E = 0x3E,
        SUB3F = 0x3F,
        SUB40 = 0x40,
    };
};

using Subindex = SubindexValues::Type;

}  // namespace crf::devices::canopendrivers
