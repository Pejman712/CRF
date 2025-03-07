/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <string>
#include <optional>
#include <map>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"
#include "CANopenDrivers/CiA402/ModesOfOperation/CST/CSTDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cst
 * @brief Class CSTConfiguration is a class for configuring the registers of the
 * cyclic synchronous torque mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * CST class for writing values in the registers.
 *
 */
class CSTConfiguration {
 public:
    explicit CSTConfiguration(const nlohmann::json& json);
    explicit CSTConfiguration(const std::string& str) = delete;
    ~CSTConfiguration() = default;

    /**
     * @brief Function that returns value of the InterpolationTimePeriod object.
     * The object consits of the period time and the unit od the period:
     * interpolationTimePeriodValue - value of the period between 2 PDOs
     * interpolationTimeIndex - time unit for the period
     * @param: none
     * @return: InterpolationTimerPeriodCST object
     */
    std::optional<InterpolationTimePeriod> getInterpolationTimePeriod() const;

    /**
     * @brief: Function that returns the value of the positive torque limit.
     * @param: none
     * @return: value of uint16_t
     */
    std::optional<uint16_t> getPositiveTorqueLimitValue() const;

    /**
     * @brief: Function that returns the value of the negative torque limit.
     * @param: none
     * @return: value of uint16_t
     */
    std::optional<uint16_t> getNegativeTorqueLimitValue() const;

    /**
     * @brief: Function that returns the value of the torque slope.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getTorqueSlope() const;

    /**
     * @brief Function that returns value of the max acceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getMaxAcceleration() const;

    /**
     * @brief Function that returns value of the max deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getMaxDeceleration() const;

    /**
     * @brief Function that returns value of the low velocity limit.
     * @param: none
     * @return: value of int32_t
     */
    std::optional<int32_t> getLowVelocityLimitValue() const;

    /**
     * @brief Function that returns value of the high velocity limit.
     * @param: none
     * @return: value of int32_t
     */
    std::optional<int32_t> getHighVelocityLimitValue() const;

    /**
     * @brief Function that returns value of the torque limitation option code
     * @param: none
     * @return: value of int8_t
     */
    std::optional<TorqueLimitationOptionCode> getTorqueLimitationOptionCode() const;

    /**
     * @brief Function that returns SoftwarePositionLimitCST object.
     * SoftwarePositionLimit has 2 parameters:
     * min
     * max
     * @param: none
     * @return: SoftwarePositionLimitCST object
     */
    std::optional<SoftwarePositionLimit> getSoftwarePositionLimit() const;

    /**
     * @brief Function that returns value of the quickstop deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getQuickStopDeceleration() const;

    /**
     * @brief Function that returns value of the profile deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getProfileDeceleration() const;

 private:
    void parse(const nlohmann::json& json);

    std::optional<InterpolationTimePeriod> interpolationTimePeriod_;
    std::optional<uint16_t> positiveTorqueLimitValue_;
    std::optional<uint16_t> negativeTorqueLimitValue_;
    std::optional<uint32_t> torqueSlope_;
    std::optional<uint32_t> maxAcceleration_;
    std::optional<uint32_t> maxDeceleration_;
    std::optional<int32_t>  lowVelocityLimit_;
    std::optional<int32_t>  highVelocityLimit_;
    std::optional<TorqueLimitationOptionCode> torqueLimitationOptionCode_;
    std::optional<SoftwarePositionLimit> softwarePositionLimit_;
    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<uint32_t> profileDeceleration_;

    const std::map<std::string, TorqueLimitationOptionCode> torqueCodemap_ = {
        {"NoLimit", TorqueLimitationOptionCode::NoLimitVelocityAcceleration},
        {"TorqueReducedUntilZero", TorqueLimitationOptionCode::TorqueReducedUntilZero},
        {"InsideRampLimits", TorqueLimitationOptionCode::TorqueModifiedInsideRampLimits},
        {"AccordingToSign", TorqueLimitationOptionCode::TorqueModifiedAccordingToSign},
        {"InsideMaxTorqueRange", TorqueLimitationOptionCode::TorqueModifiedInsideMaxTorqueRange}
    };
};

}  // namespace crf::devices::canopendrivers
