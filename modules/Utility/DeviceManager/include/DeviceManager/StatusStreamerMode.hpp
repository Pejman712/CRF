/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <nlohmann/json.hpp>

namespace crf {
namespace utility {
namespace devicemanager {

enum class StatusStreamerMode {
    /**
     * @brief No Mode has been declared
     *
     */
    NotDefined = 0,
    /**
     * @brief Frequency mode selected. An update will be sent following the specified
     * frequency
     *
     */
    Frequency = 1,
    /**
     * @brief Differential mode selected. An update will be sent whenever a new status
     * has been recorded. The frequency specifies the speed at which updates are checked
     *
     */
    Differential = 2
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
