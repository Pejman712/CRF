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
#include "CANopenDrivers/CiA402/ModesOfOperation/CSP/CSPDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_csp
 * @brief Class CSPConfiguration is a class for parsing the registers of the
 * cyclic synchronous position mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * CSP class for writing values in the registers.
 *
 * All of the values keep the units from the JSON file and they should be consistent with the motor
 * units.
 *
 */
class CSPConfiguration {
 public:
    explicit CSPConfiguration(const nlohmann::json& json);
    explicit CSPConfiguration(const std::string& str) = delete;
    ~CSPConfiguration() = default;

    /**
     * @brief Function that returns PositionRangeLimitCSP object.
     * @param: none
     * @return: std::optional<PositionRangeLimitCSP> object
     */
    std::optional<PositionRangeLimit> getPositionRangeLimit() const;

    /**
     * @brief Function that returns SoftwarePositionLimitCSP object.
     * @param: none
     * @return: SoftwarePositionLimitCSP object
     */
    std::optional<SoftwarePositionLimit> getSoftwarePositionLimit() const;

    /**
     * @brief Function that returns value of the following error window.
     * @param: none
     * @return: value of uint32_t
     *
     */
    std::optional<uint32_t> getFollowingErrorWindow() const;


    /**
     * @brief Function that returns value of the following error timeout.
     * @param: none
     * @return: the following error timeout value
     *
     */
    std::optional<uint16_t> getFollowingErrorTimeout() const;

    /**
     * @brief Function that returns the value of the quickstop deceleration.
     * @param: none
     * @return: value uint32_t
     */
    std::optional<uint32_t> getQuickStopDeceleration() const;

    /**
     * @brief Function that returns the type of the motion profile type
     * for the cyclic synchronous velocity mode.
     * Possible values :
     * TRAPEZOIDAL_CSP - linear ramp
     * SIN2_RAMP_CSP - sin^2 ramp
     * JERK_RAMP_CSP - jerk free ramp
     * JERK_LIMITED_CSP - jerk limited ramp
     * @param: none
     * @return: ProfileTypeCSP object
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
     * @brief Function that returns value of the InterpolationTimePeriod object.
     * The object consits of the period time and the unit od the period:
     * interpolationTimePeriodValue - value of the period between 2 PDOs
     * interpolationTimeIndex - time unit fot the period
     * @param: none
     * @return: InterpolationTimerPeriod object
     */
    std::optional<InterpolationTimePeriod> getInterpolationTimePeriod() const;

    /**
     * @brief Function that returns value of the profile deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getProfileDeceleration() const;

 private:
    void parse(const nlohmann::json& json);

    std::optional<PositionRangeLimit> positionRangeLimit_;
    std::optional<SoftwarePositionLimit> softwarePositionLimit_;
    std::optional<uint32_t> followingErrorWindow_;
    std::optional<uint16_t> followingErrorTimeOut_;
    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<ProfileType> motionProfileType_;
    std::optional<uint8_t> profileJerkUse_;
    std::optional<std::vector<uint32_t>> profileJerks_;
    std::optional<InterpolationTimePeriod> interpolationTimePeriod_;
    std::optional<uint32_t> profileDeceleration_;

    const std::map<std::string, ProfileType> profileTypeMap_ = {
        {"Trapezoidal", ProfileType::Trapezoidal},
        {"Sin^2", ProfileType::Sin2},
        {"JerkFree", ProfileType::JerkFree},
        {"JerkLimited", ProfileType::JerkLimited}
    };

    const double secToMillisec_ = 1000;
};

}  // namespace crf::devices::canopendrivers
