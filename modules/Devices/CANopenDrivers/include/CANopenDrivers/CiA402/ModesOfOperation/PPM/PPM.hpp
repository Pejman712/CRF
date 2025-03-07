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
#include "CANopenDrivers/CiA402/ModesOfOperation/PPM/PPMConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ppm
 * @brief Class PPM is the class for profile position mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class PPM {
 public:
    explicit PPM(const nlohmann::json& config);
    explicit PPM(const std::string& config) = delete;
    ~PPM() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the profile position mode.
     * @param: none
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers that are used for
     * commanding the profile position mode.
     * @param: targetPosition - target position for the profile position mode
     * @param: reference - type of reference position: relative or absolute
     * @param: profileVelocity - maximum velocity of the profile type used in this mode
     * @param: profileAcceleration - acceleration of the motor
     * @param: profileDeceleration - deceleration of the motor
     * @param: endVelocity - velocity at which the motor reaches the target position
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> setTargetRegisters(
        const uint32_t& profileVelocity, const uint32_t& profileAcceleration,
        const uint32_t& profileDeceleration, const PositionReference& reference);

    /**
     * @brief Function that returns the controlword for for starting
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM activateWord() const;

    /**
     * @brief Function that returns the controlword for stopping
     * the movement of the motor.
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM deactivateWord() const;

    /**
     * @brief Function that returns the controlword for activating the
     * endless movement of the motor.
     * @param: none
     * @param: ControlWordPPM object
    */
    ControlWordPPM activateEndlessMovementWord() const;

    /**
     * @brief Function that returns the controlword for deactivating the
     * endless movement of the motor.
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM deactivateEndlessMovementWord() const;

    /**
     * @brief Function that returns the controlword for the motor
     * to assume to the new target position (both for relative and absolute reference).
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM assumeNewTargetPositonWord() const;

    /**
     * @brief Function that returns the controlword for the motor
     * to immediatly change target position (both for absolute and relative)
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM activateChangeSetImmediatelyWord() const;

    /**
     * @brief Function that returns the controlword for the motor
     * to stop the immediate change of the target position.
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM deactivateChangeSetImmediatelyWord() const;

    /**
     * @brief Function that returns the controlword for the motor
     * to change the set-point (target position) after the motor
     * raches the previous set-point (target position). The motor will
     * wait for the previous trajectory to finish before starting the
     * new one to the new target position.
     * (For more information check the CiA 402 documentation)
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM activateChangeSetPoint() const;

    /**
     * @brief Function that returns the controlword for the motor
     * to stop the change of the set-point (target position) after the motor
     * raches the previous set-point (target position).
     * @param: none
     * @return: ControlWordPPM object
    */
    ControlWordPPM deactivateChangeSetPoint() const;

    /**
     * @brief Function that returns whether or not the target has been reached.
     * @param: statusWord - the current statusword
     * @return: bool value: true - target is reached, false - target isn't reached
    */
    bool isTargetReached(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not the new setpoint has been acknowledged.
     * @param: statusWord - the current statusword
     * @return: bool value: true - setpoint is acknowledged, false - setpoint isn't acknowledged
    */
    bool isSetPointAcknowledged(const uint16_t& statusWord) const;

    /**
     * @brief Function that returns whether or not a following error occured.
     * @param: statusWord - the current statusword
     * @return: bool value: true - following error occured , false - following error didn't occur
    */
    bool isFollowingError(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    PPMConfiguration config_;
    PositionReference reference_;

    const uint64_t numberOfConfigRegisters_ = 10;
    const uint64_t numberOfTargetRegisters_ = 3;
};

}  // namespace crf::devices::canopendrivers
