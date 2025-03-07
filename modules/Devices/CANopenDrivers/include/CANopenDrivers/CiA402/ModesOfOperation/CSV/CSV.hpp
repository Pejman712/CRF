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
#include "CANopenDrivers/CiA402/ModesOfOperation/CSV/CSVConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_csv
 * @brief Class CSV is the class for cyclic synchronous velocity mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class CSV {
 public:
    explicit CSV(const nlohmann::json& config);
    explicit CSV(const std::string& config) = delete;
    ~CSV() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the cyclic synchronous velocity mode.
     * @param: none
     * @return: vector of RegisterValues
     */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the cyclic sycnhronous velocity mode.
     * @param: velocityOffset - optional velocity feed foward input
     * @param: torqueOffset - optional torque feed foward input
     * @return: vector of RegisterValues
     */
    std::vector<RegisterValues> setTargetRegisters(
        const int32_t& velocityOffset = 0x00000000, const int16_t& torqueOffset = 0x0000);

    /**
     * @brief Function that returns whether or not the driver is following the target value.
     * @param: statusWord - the current statusword
     * @return: bool value: true - driver is following the target value,
     * false - driver isn't following the target value
     */
    bool isDriveFollowingCommandValue(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    CSVConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 20;
    const uint64_t numberOfTargetRegisters_ = 2;
};

}  // namespace crf::devices::canopendrivers
