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
#include "CANopenDrivers/CiA402/ModesOfOperation/CSV/CSVDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cst
 * @brief Class CSVConfiguration is a class for configuring the registers of the
 * cyclic synchronous velocity mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * CSV class for writing values in the registers.
 *
 */
class CSVConfiguration {
 public:
    explicit CSVConfiguration(const nlohmann::json& json);
    explicit CSVConfiguration(const std::string& str) = delete;
    ~CSVConfiguration() = default;

    /**
     * @brief Function that returns the value of the quickstop deceleration.
     * @param: none
     * @return: value uint32_t
     */
    std::optional<uint32_t> getQuickStopDeceleration() const;

    /**
     * @brief Function that returns value of the profile deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getProfileDeceleration() const;

    /**
     * @brief Function that returns the type of the motion profile type
     * for the cyclic synchronous velocity mode.
     * Possible values :
     * TRAPEZOIDAL_CSV - linear ramp
     * SIN2_RAMP_CSV - sin^2 ramp
     * JERK_RAMP_CSV - jerk free ramp
     * JERK_LIMITED_CSV - jerk limited ramp
     * @param: none
     * @return: ProfileTypeCSV object
     */
    std::optional<ProfileType> getMotionProfileType() const;

    /**
     * @brief Function that returns the number of the jerks that will be
     * implemented for the jerk limited profile for the mode. (it also specifies how many sub-indices
     * there will be for the register 60A4 which is used for setting the jerks).
     * Value of the register goes from 1 to 6. If a value FF is set in the register that means that
     * the profile jerk use is not configured.
     */
    std::optional<uint8_t> getProfileJerkUse() const;

    /**
     * @brief Function that returns a vector of values of the profile jerks
     * for the jerk limited profile. Number of jerks can be up to 6.
     */
    std::optional<std::vector<uint32_t>> getProfileJerks() const;

    /**
     * @brief Function that returns SoftwarePositionLimitCSV object.
     * The object has 2 parameters:
     * min
     * max
     * @param: none
     * @return: SoftwarePositionLimitCSV object
     */
    std::optional<SoftwarePositionLimit> getSoftwarePositionLimit() const;

    /**
     * @brief Function that returns value of the InterpolationTimePeriod object.
     * The object consits of the period time and the unit od the period:
     * interpolationTimePeriodValue - value of the period between 2 PDOs
     * interpolationTimeIndex - time unit for the period
     * @param: none
     * @return: InterpolationTimerPeriodCSV object
     */
    std::optional<InterpolationTimePeriod> getInterpolationTimePeriod() const;

 private:
    void parse(const nlohmann::json& json);
    void parse(const std::string&) = delete;

    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<uint32_t> profileDeceleration_;
    std::optional<ProfileType> motionProfileType_;
    std::optional<uint8_t> profileJerkUse_;
    std::optional<std::vector<uint32_t>> profileJerks_;
    std::optional<SoftwarePositionLimit> softwarePositionLimit_;
    std::optional<InterpolationTimePeriod> interpolationTimePeriod_;

    const std::map<std::string, ProfileType> profileTypeMap_ = {
        {"Trapezoidal", ProfileType::Trapezoidal},
        {"Sin^2", ProfileType::Sin2},
        {"JerkFree", ProfileType::JerkFree},
        {"JerkLimited", ProfileType::JerkLimited}
    };
};

}  // namespace crf::devices::canopendrivers
