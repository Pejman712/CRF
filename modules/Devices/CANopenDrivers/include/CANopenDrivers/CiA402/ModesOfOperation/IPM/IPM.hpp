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
#include "CANopenDrivers/CiA402/ModesOfOperation/IPM/IPMConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ipm
 * @brief Class IPM is the class for interpolated position mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class IPM {
 public:
    explicit IPM(const nlohmann::json& config);
    explicit IPM(const std::string& config) = delete;
    ~IPM() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the interpolated position  mode.
     * @param: none
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the interpolated position mode.
     * @param: profileVelocity - maximum velocity of the profile type used in this mode
     * @param: profileAcceleration - acceleration of the motor
     * @param: profileDeceleration - deceleration of the motor
     * @param: endVelocity - velocity at which the motor reaches the target position
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> setTargetRegisters(const uint32_t& profileVelocity,
        const uint32_t& profileAcceleration, const uint32_t& profileDeceleration);

    /**
     * @brief Function that returns the controlword for setting
     * the IPM mode.
     * @param: none
     * @return: ControlWordIPM object
    */
    ControlWordIPM setIPM() const;

    /**
     * @brief Function that returns the controlword for starting
     * the movement of the motor.
     * @param: none
     * @return: ControlWordIPM object
    */
    ControlWordIPM activateWord() const;

    /**
     * @brief Function that returns the controlword for stopping
     * the movement of the motor.
     * @param: none
     * @return: ControlWordIPM object
    */
    ControlWordIPM deactivateWord() const;

    /**
     * @brief Function that returns the controlword for unsetting
     * the IPM mode.
     * @param: none
     * @return: ControlWordIPM object
    */
    ControlWordIPM unsetIPM() const;

    /**
     * @brief Function that returns whether or not the target has been reached.
     * @param: statusWord - the current statusword
     * @return: bool value: true - target is reached, false - target isn't reached
    */
    bool isTargetReached(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the IPM mode has been set.
     * @param: statusWord - the current statusword
     * @return: bool value: true - the IPM mode is set , false - the IPM mode isn't set
    */
    bool isIPMModeSet(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not there is a following error.
     * A following error occurs when the actual position isn't following the reference position.
     * @param: statusWord - the current statusword
     * @return: bool value: true - the actual position isn't following the reference position ,
     * false - the actual position is following the reference position
    */
    bool isFollowingError(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    IPMConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 20;
    const uint64_t numberOfTargetRegisters_ = 3;
};

}  // namespace crf::devices::canopendrivers
