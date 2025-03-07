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
#include "CANopenDrivers/CiA402/ModesOfOperation/PVM/PVMDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_pvm
 * @brief Class PVMConfiguration is a class for configuring the registers of the
 * profile velocity mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * PVM class for writing values in the registers.
 *
 */
class PVMConfiguration {
 public:
    explicit PVMConfiguration(const nlohmann::json& json);
    explicit PVMConfiguration(const std::string& str) = delete;
    ~PVMConfiguration() = default;

    /**
     * @brief Function that returns SoftwarePositionLimitPVM object.
     * The object has 2 parameters:
     * min
     * max
     * @param: none
     * @return: SoftwarePositionLimitPVM object
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
     * for the profile velocity mode.
     * Possible values :
     * Trapezoidal - linear ramp
     * Sin2 Ramp - sin^2 ramp
     * Jerk Ramp - jerk free ramp
     * Jerk Limited - jerk limited ramp
     * @param: none
     * @return: ProfileTypePVM object
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
     * @brief Function that returns the value of the sensor selection code.
     * The register determines wheter a differentiated position signal
     * or the signal from a separate velocity sensor is evaluated.
     * @param: none
     * @return: value of int16_t
    */
    std::optional<int16_t> getSensorSelectionCode() const;

    /**
     * @brief Function that returns the value of the velocity window.
     * This value specifies by how much the actual speed may vary from the target velocity that is set.
     * @param: none
     * @return: value of uint16_t
    */
    std::optional<uint16_t> getVelocityWindow() const;

    /**
     * @brief Function that returns the value of the velocity window time.
     * The velocity window time specifies how long the actual velocity and the target velocity
     * must be close to one another.
     * @param: none
     * @return: value of uint16_t
    */
    std::optional<uint16_t> getVelocityWindowTime() const;

    /**
     * @brief Function that returns the value of the velocity threshold.
     * @param: none
     * @return: value of uint16_t
    */
    std::optional<uint16_t> getVelocityThreshold() const;

    /**
     * @brief Function that returns the value of the velocity threshold time.
     * @param: none
     * @return: value of uint16_t
    */
    std::optional<uint16_t> getVelocityThresholdTime() const;

    /**
     * @brief Function that returns the value of the maximum slippage
     * of an asynchronous motor.
     * @param: none
     * @return: value of int32_t
    */
    std::optional<int32_t> getMaxSlippage() const;

 private:
    void parse(const nlohmann::json& json);
    void parse(const std::string&) = delete;

    std::optional<SoftwarePositionLimit> softwarePositionLimit_;
    std::optional<uint32_t> maxProfileVelocity_;
    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<uint32_t> maxAcceleration_;
    std::optional<uint32_t> maxDeceleration_;
    std::optional<ProfileType> motionProfileType_;
    std::optional<uint8_t> profileJerkUse_;
    std::optional<std::vector<uint32_t>> profileJerks_;
    std::optional<int16_t> sensorSelectionCode_;
    std::optional<uint16_t> velocityWindow_;
    std::optional<uint16_t> velocityWindowTime_;
    std::optional<uint16_t> velocityThreshold_;
    std::optional<uint16_t> velocityThresholdTime_;
    std::optional<uint16_t> maxSlippage_;

    const std::map<std::string, ProfileType> profileTypeMap_ = {
        {"Trapezoidal", ProfileType::Trapezoidal},
        {"Sin^2", ProfileType::Sin2},
        {"JerkFree", ProfileType::JerkFree},
        {"JerkLimited", ProfileType::JerkLimited}
    };

    const double secToMillisec_ = 1000;
};

}  // namespace crf::devices::canopendrivers
