/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/RegisterValues.hpp"
#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402Registers.hpp"
#include "CANopenDrivers/CiA402/ModesOfOperation/CSP/CSPConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_csp
 * @brief Class of Cyclic Synchronous Position. It returns the configuration registers of this
 * mode, the target registers, and compares any specific bit of the status word.
 *
 */
class CSP {
 public:
    explicit CSP(const nlohmann::json& config);
    explicit CSP(const std::string& config) = delete;
    ~CSP() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the cyclic synchronous position mode.
     * @param: none
     * @return: vector of RegisterValues
     *
     */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the cyclic sycnhronous position mode.
     * @param: positionOffset - optional aditive position value
     * @param: velocityOffset - optional velocity feed foward input
     * @param: torqueOffset - optional torque feed foward input
     * @return: vector of RegisterValues
     *
     */
    std::vector<RegisterValues> setTargetRegisters(
        const int32_t& positionOffset = 0x00000000,
        const int32_t& velocityOffset = 0x00000000,
        const int16_t& torqueOffset = 0x0000);

    /**
     * @brief Function that returns whether or not the driver is following the target value.
     * @param: statusWord - the current statusword
     * @return: true - driver is following the target value,
     * @return: false - driver isn't following the target value
     *
     */
    bool isDriveFollowingCommandValue(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not there is a following error
     * @param: statusWord - the current statusword
     * @return: true - a following error occured, the difference between
     * actual position and demand position exceeds the following error window
     * @return: false - a following error didn't occur
     *
     */
    bool isFollowingError(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    CSPConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 20;
    const uint64_t numberOfTargetRegisters_ = 3;
};

}  // namespace crf::devices::canopendrivers
