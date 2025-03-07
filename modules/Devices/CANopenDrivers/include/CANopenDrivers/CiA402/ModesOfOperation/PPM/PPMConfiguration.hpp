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
#include "CANopenDrivers/CiA402/ModesOfOperation/PPM/PPMDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ppm
 * @brief Class PPMConfiguration is a class for configuring the registers of the
 * profile position mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * PPM class for writing values in the registers.
 *
 */
class PPMConfiguration {
 public:
    explicit PPMConfiguration(const nlohmann::json& json);
    explicit PPMConfiguration(const std::string& str) = delete;
    ~PPMConfiguration() = default;

    /**
     * @brief Function that returns PositionRangeLimitPPM object.
     * The object has 2 parameters:
     * min
     * max
     * @param: none
     * @return: PositionRangeLimitPPM object
     */
    std::optional<PositionRangeLimit> getPositionRangeLimit() const;

    /**
     * @brief Function that returns SoftwarePositionLimitPPM object.
     * The object has 2 parameters:
     * min
     * max
     * @param: none
     * @return: SoftwarePositionLimitPPM object
     */
    std::optional<SoftwarePositionLimit> getSoftwarePositionLimit() const;

    /**
     * @brief Function that returns value of the max profile velocity.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getMaxProfileVelocity() const;

    /**
     * @brief Function that returns value of the quickstop deceleration.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getQuickStopDeceleration() const;

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
     * @brief Function that returns the type of the motion profile type
     * for the profile position mode.
     * Possible values :
     * TRAPEZOIDAL_PPM - linear ramp
     * SIN2_RAMP_PPM - sin^2 ramp
     * JERK_RAMP_PPM - jerk free ramp
     * JERK_LIMITED_PPM - jerk limited ramp
     * @param: none
     * @return: ProfileTypePPM object
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
     * @brief Function that returns the end velocity. End velocity is the velocity at
     * which the motor reaches the target position.
     * @param: none
     * @return: value of uint32_t
     */
    std::optional<uint32_t> getEndVelocity() const;

 private:
    void parse(const nlohmann::json& robotConfig);
    void parse(const std::string&) = delete;

    std::optional<PositionRangeLimit> positionRangeLimit_;
    std::optional<SoftwarePositionLimit> softwarePositionLimit_;
    std::optional<uint32_t> maxProfileVelocity_;
    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<uint32_t> maxAcceleration_;
    std::optional<uint32_t> maxDeceleration_;
    std::optional<ProfileType> motionProfileType_;
    std::optional<uint8_t> profileJerkUse_;
    std::optional<std::vector<uint32_t>> profileJerks_;
    std::optional<uint32_t> endVelocity_;

    const std::map<std::string, ProfileType> profileMap_ {
        {"Trapezoidal", ProfileType::Trapezoidal},
        {"Sin^2", ProfileType::Sin2},
        {"JerkFree", ProfileType::JerkFree},
        {"JerkLimited", ProfileType::JerkLimited}
    };
};

}  // namespace crf::devices::canopendrivers
