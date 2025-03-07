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
#include "CANopenDrivers/CiA402/ModesOfOperation/IPM/IPMDefinitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_ipm
 * @brief Class IPMConfiguration is a class for configuring the registers of the
 * interpolated position mode. The point of the class is to get the values of the
 * configuration registers of the mode from the json file so that they can be later used in the
 * IPM class for writing values in the registers.
 *
 */
class IPMConfiguration {
 public:
    explicit IPMConfiguration(const nlohmann::json& json);
    explicit IPMConfiguration(const std::string& str) = delete;
    ~IPMConfiguration() = default;

    /**
     * @brief Function that returns PositionRangeLimitIPM object.
     * The object has 2 parameters:
     * min
     * max
     * @param: none
     * @return: PositionRangeLimitIPM object
    */
    std::optional<PositionRangeLimit> getPositionRangeLimit() const;

    /**
     * @brief Function that returns SoftwarePositionLimitIPM object.
     * SoftwarePositionLimit has 2 parameters:
     * min
     * max
     * @param: none
     * @return: SoftwarePositionLimitIPM object
    */
    std::optional<SoftwarePositionLimit> getSoftwarePositionLimit() const;

    /**
     * @brief Function that returns value of the home offset.
     * This object indicates the configured difference between the zero position for the application
     * and the machine home position (found during homing).
     * @param: none
     * @return: value of int32_t
    */
    std::optional<int32_t> getHomeOffset() const;

    /**
     * @brief: Function that returns value of the interpolation sub mode select.
     * This object indicates the actually chosen interpolation mode. If linear interpolation is the
     * only algorithm avalaible, then it is not necessary to implement this object. If a manufacturer -
     * specific interpolation mode is selected, the corresponding interpolation data record shall be
     * implemented in the manufacturer-specific profile area of the object dictionary.
     *
    */
    std::optional<int16_t> getInterpolationSubModeSelect() const;

    /**
     * @brief: Function that returns the InterpolationDataConfiguration object.
     * Atributes of the object are:
     * maximumBufferSize - maximum size of the buffer,
     * actualBufferSize - actual size of the buffer,
     * bufferOrganization - organiz
        uint16_t bufferPosition;
        uint8_t sizeOfDataRecord;
        uint8_t bufferClear;
    * @param: none
    * @return InterpolationDataConfiguration object.
    */
    std::optional<InterpolationDataConfiguration> getInterpolationDataConfiguration() const;

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
    std::optional<int32_t> homeOffset_;
    std::optional<int16_t> interpolationSubModeSelect_;
    std::optional<InterpolationDataConfiguration> interpolationDataConfiguration_;
    std::optional<InterpolationTimePeriod> interpolationTimePeriod_;
    std::optional<uint32_t> maxProfileVelocity_;
    std::optional<uint32_t> quickStopDeceleration_;
    std::optional<uint32_t> maxAcceleration_;
    std::optional<uint32_t> maxDeceleration_;
    std::optional<uint32_t> endVelocity_;

    const std::map<std::string, InterpolationSubmode> interpolationModeMap_ = {
        {"LinearInterpolation", InterpolationSubmode::LinearInterpolation}
    };
};

}  // namespace crf::devices::canopendrivers
