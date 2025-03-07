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
#include "CANopenDrivers/CiA402/ModesOfOperation/CST/CSTConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cst
 * @brief Class CST is the class for cyclic synchronous torque mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class CST {
 public:
    explicit CST(const nlohmann::json& config);
    explicit CST(const std::string& config) = delete;
    ~CST() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the cyclic synchronous torque mode.
     * @param: none
     * @return: vector of RegisterValues
     */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the cyclic sycnhronous torque mode.
     * @param: torqueOffset - optional torque feed foward input
     * @return: vector of RegisterValues
     */
    std::vector<RegisterValues> setTargetRegisters(const int16_t& torqueOffset = 0x0000);

    /**
     * @brief Function that returns whether or not the actual torque value  is
     * following the target torque value.
     * @param: statusWord - the current statusword
     * @return: bool value: true - actual value is following the target value,
     * false - actual value isn't following the target value
     */
    bool isDriveFollowingCommandValue(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    CSTConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 20;
    const uint64_t numberOfTargetRegisters_ = 1;
};

}  // namespace crf::devices::canopendrivers
