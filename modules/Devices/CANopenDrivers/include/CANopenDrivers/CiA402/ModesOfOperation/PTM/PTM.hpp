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
#include "CANopenDrivers/CiA402/ModesOfOperation/PTM/PTMConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ptm
 * @brief Class PTM is the class for torque mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class PTM {
 public:
    explicit PTM(const nlohmann::json& config);
    explicit PTM(const std::string& config) = delete;
    ~PTM() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the torque mode.
     * @param: none
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the torque mode.
     * @param: targetTorque- target torque for the mode
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> setTargetRegisters(const int16_t& targetTorque);

    /**
     * @brief Function that returns the controlword for for starting
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPTM object
    */
    ControlWordPTM activateWord() const;

    /**
     * @brief Function that returns the controlword for stopping
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPTM object
    */
    ControlWordPTM deactivateWord() const;

    /**
     * @brief Function that returns whether or not the target has been reached.
     * @param: statusWord - the current statusword
     * @return: bool value: true - target is reached, false - target isn't reached
    */
    bool isTargetReached(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the limit is exceeded with the torque,
     * meaning if the torque exceeds the maximum torque.
     * @param: statusWord - the current statusword
     * @return: bool value: true - torque goes over the maximum limit, false - torque is whitin the limit.
    */
    bool isTorqueLimited(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    PTMConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 20;
    const uint64_t numberOfTargetRegisters_ = 1;
};

}  // namespace crf::devices::canopendrivers
