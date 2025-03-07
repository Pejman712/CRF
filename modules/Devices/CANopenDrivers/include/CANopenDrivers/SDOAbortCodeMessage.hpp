/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <map>
#include <string>

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_can_open_drivers
 * @brief Map to translate SDO abort codes into understandable text
 *
 */
const std::map<uint32_t, std::string> SDOAbortCodeMessage = {
    {0x00000000, "No Error"},
    {0x05030000, "Toggle bit not alternated"},
    {0x05040000, "SDO protocol timed out"},
    {0x05040001, "Client/server command specifier not valid or unknown"},
    {0x05040002, "Invalid block size (block mode only)"},
    {0x05040003, "Invalid sequence number (block mode only)"},
    {0x05040004, "CRC error (block mode only)"},
    {0x05040005, "Out of memory"},
    {0x06010000, "Unsupported access to an object"},
    {0x06010001, "Attempt to read a write-only object"},
    {0x06010002, "Attempt to write a read-only object"},
    {0x06020000, "Object does not exist in the object dictionary"},
    {0x06040041, "Object cannot be mapped to the PDO"},
    {0x06040042, "Number and length of the objects to be mapped would exceed PDO length"},
    {0x06040043, "General parameter incompatibility"},
    {0x06040047, "General internal incompatibility in the device"},
    {0x06060000, "Access failed due to an hardware error"},
    {0x06070010, "Data type does not match, length of service parameter does not match"},
    {0x06070012, "Data type does not match, length of service parameter too high"},
    {0x06070013, "Data type does not match, length of service parameter too low"},
    {0x06090011, "Sub-index does not exist"},
    {0x06090030, "Value range of parameter exceeded"},
    {0x06090031, "Value of parameter written too high"},
    {0x06090032, "Value of parameter written too low"},
    {0x06090036, "Maximum value is less than minimum value"},
    {0x08000000, "General error"},
    {0x08000020, "Data cannot be transferred or stored to the application"},
    {0x08000021, "Data cannot be transferred or stored to the application because of local control"}, //  NOLINT
    {0x08000022, "Data cannot be transferred or stored to the application because of the present device state"}, // NOLINT
    {0x08000023, "Object dictionary dynamic generation fails or no object dictionary is present"},
    {0x08000024, "No data available"}
};

}  // namespace crf::devices::canopendrivers
