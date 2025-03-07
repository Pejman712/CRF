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
#include "CANopenDrivers/CiA402/ModesOfOperation/VOM/VOMConfiguration.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_vom
 * @brief Class VOM is the class for velocity mode for the driver.
 * The constructor accepts a json file where it has the list or set of registers
 * for configuring the mode with the default values.
 *
 */
class VOM {
 public:
    explicit VOM(const nlohmann::json& config);
    explicit VOM(const std::string& config) = delete;
    ~VOM() = default;

    /**
     * @brief Function that returns a vector of registers that are used for
     * configuring the velocity mode.
     * @param: none
     * @return: vector of RegisterValues
    */
    std::vector<RegisterValues> getConfigRegisters() const;

    /**
     * @brief Function that returns a vector of registers for setting the velocity mode.
     * @param: deltaSpeedAcc - value of delta speed for acceleration ramp
     * @param: deltaTimeAcc -  value of time for acceleration ramp
     * @param: deltaSpeedDec - value of delta speed for deceleration ramp
     * @param: deltaTimeDec - value of delta time for deceleration ramp
     * @return: RegisterValues object - vector of registers
    */
    std::vector<RegisterValues> setTargetRegisters(
        const uint32_t& deltaSpeedAcc, const uint16_t& deltaTimeAcc,
        const uint32_t deltaSpeedDec, const uint16_t deltaTimeDec);

    /**
     * @brief Function that returns the controlword for for starting
     * the movement of the motor.
     * @param: none
     * @return: ControlWordVOM object
    */
    ControlWordVOM activateWord() const;

    /**
     * @brief Function that returns the controlword for stopping
     * the movement of the motor.
     * @param: none
     * @return: ControlWordVOM object
    */
    ControlWordVOM deactivateWord() const;

    /**
     * @brief Function that returns whether or not the speed of the motor reached one of the limits.
     * @param: statusWord - the current statusword
     * @return: bool value: true - speed of the motor went over the limit (under min or over  max),
     * false- the speed is within the limits.
    */
    bool isSpeedLimited(const uint16_t& statusWord) const;

 private:
    void parseConfigRegisters();

    std::vector<RegisterValues> configRegisters_;
    std::vector<RegisterValues> targetRegisters_;
    VOMConfiguration config_;

    const uint64_t numberOfConfigRegisters_ = 8;
    const uint64_t numberOfTargetRegisters_ = 4;
};

}  // namespace crf::devices::canopendrivers
