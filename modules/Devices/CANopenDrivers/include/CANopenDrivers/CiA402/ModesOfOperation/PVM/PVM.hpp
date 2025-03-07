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
#include "CANopenDrivers/CiA402/ModesOfOperation/PVM/PVMConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_pvm
 * @brief Class PVM is the class for profile velocity mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class PVM {
 public:
    explicit PVM(const nlohmann::json& config);
    explicit PVM(const std::string& config) = delete;
    ~PVM() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the  profile velocity mode.
     * @param: none
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the profile velocity mode.
     * @param: targetVelocity - value of the target velocity
     * @param: profileAcceleration - acceleration for the trapezoidal profile of velocity
     * @param: profileDeceleration - deceleration for the trapezoidal profile of velocity
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> setTargetRegisters(
        const uint32_t& profileAcceleration, const uint32_t& profileDeceleration);

    /**
     * @brief Function that returns the controlword for for starting
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPVM object
    */
    ControlWordPVM activateWord() const;

    /**
     * @brief Function that returns the controlword for stopping
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPVM object
    */
    ControlWordPVM deactivateWord() const;

    /**
     * @brief Function that returns whether or not the target has been reached.
     * @param: statusWord - the current statusword
     * @return: bool value: true - target is reached, false - target isn't reached
    */
    bool isTargetReached(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the speed of the motor reached one of the limits.
     * @param: statusWord - the current statusword
     * @return: bool value: true - speed of the motor went over the limit (under min or over  max),
     * false- the speed is within the limits.
    */
    bool isSpeedLimited(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the speed is zero of the motor.
     * @param: statusWord - the current statusword
     * @return: bool value: true - speed is zero, false - speed isn't zero
    */
    bool isSpeedZero(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the maximum slippage of a
     * asynchronous motor is reached.
     * @param: statusWord - the current statusword
     * @return: bool value: true - max slippage reached, false - max slippage not reached
    */
    bool isMaxSlippageReached(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    PVMConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 21;
    const uint64_t numberOfTargetRegisters_ = 2;
};

}  // namespace crf::devices::canopendrivers
