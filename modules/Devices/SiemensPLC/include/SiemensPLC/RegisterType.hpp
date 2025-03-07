/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf {
namespace devices {
namespace siemensplc {

/*
 * @brief It defines the variable type that will be sent to the daatablock of the PLC.
 */
enum class RegisterType {
    /*
     * @brief Not defined.
     */
    UNKNOWN = 0,
    /*
     * @brief Defined a cast to bool.
     */
    R_BOOL = 1,
    /*
     * @brief Defined a cast to uint8_t.
     */
    R_BYTE = 2,
    /*
     * @brief Defined a cast to int8_t.
     */
    R_SINT = 3,
    /*
     * @brief Defined a cast to uint16_t.
     */
    R_WORD = 4,
    /*
     * @brief Defined a cast to uint16_t.
     */
    R_UINT = 5,
    /*
     * @brief Defined a cast to int16_t.
     */
    R_INT = 6,
    /*
     * @brief Defined a cast to uint32_t.
     */
    R_DWORD = 7,
    /*
     * @brief Defined a cast to uint32_t.
     */
    R_UDINT = 8,
    /*
     * @brief Defined a cast to int32_t.
     */
    R_DINT = 9,
    /*
     * @brief Defined a cast to uint64_t.
     */
    R_LWORD = 10,
    /*
     * @brief Defined a cast to uint64_t.
     */
    R_ULINT = 11,
    /*
     * @brief Defined a cast to int64_t.
     */
    R_LINT = 12,
    /*
     * @brief Defined a cast to float.
     */
    R_REAL = 13,
    /*
     * @brief Defined a cast to double.
     */
    R_LREAL = 14
};

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
