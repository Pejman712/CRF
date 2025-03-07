/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <string>
#include <optional>
#include <cstdint>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/ModesOfOperation/CST/CSTConfiguration.hpp"

namespace crf::devices::canopendrivers {

CSTConfiguration::CSTConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<InterpolationTimePeriod> CSTConfiguration::getInterpolationTimePeriod() const {
    return interpolationTimePeriod_;
}

std::optional<uint16_t> CSTConfiguration::getPositiveTorqueLimitValue() const {
    return positiveTorqueLimitValue_;
}

std::optional<uint16_t> CSTConfiguration::getNegativeTorqueLimitValue() const {
    return negativeTorqueLimitValue_;
}

std::optional<uint32_t> CSTConfiguration::getTorqueSlope() const {
    return torqueSlope_;
}

std::optional<uint32_t> CSTConfiguration::getMaxAcceleration() const {
    return maxAcceleration_;
}

std::optional<uint32_t> CSTConfiguration::getMaxDeceleration() const {
    return maxDeceleration_;
}

std::optional<int32_t> CSTConfiguration::getLowVelocityLimitValue() const {
    return lowVelocityLimit_;
}

std::optional<int32_t> CSTConfiguration::getHighVelocityLimitValue() const {
    return highVelocityLimit_;
}

std::optional<TorqueLimitationOptionCode> CSTConfiguration::getTorqueLimitationOptionCode() const {
    return torqueLimitationOptionCode_;
}

std::optional<SoftwarePositionLimit> CSTConfiguration::getSoftwarePositionLimit() const {
    return softwarePositionLimit_;
}

std::optional<uint32_t> CSTConfiguration::getQuickStopDeceleration() const {
    return quickStopDeceleration_;
}

std::optional<uint32_t> CSTConfiguration::getProfileDeceleration() const {
    return profileDeceleration_;
}

// Private

void CSTConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double posUnits = json.at("PositionUnitConversion").get<double>();
        double velUnits = json.at("VelocityUnitConversion").get<double>();
        double accUnits = json.at("AccelerationUnitConversion").get<double>();
        double tqeUnits = json.at("TorqueUnitConversion").get<double>();

        // Parsing and conversion to driver units
        // Implicit conversions! Overflow may occur if you are not careful!
        if (json.contains("InterpolationTimePeriod")) {
            InterpolationTimePeriod interpolation;
            interpolation.value = json.at("InterpolationTimePeriod").at("Value").get<uint8_t>();
            interpolation.index = json.at("InterpolationTimePeriod").at("Index").get<int8_t>();
            interpolationTimePeriod_ = interpolation;
        }
        if (json.contains("PositiveTorqueLimitValue")) {
            positiveTorqueLimitValue_ =
                tqeUnits * json.at("PositiveTorqueLimitValue").get<double>();
        }
        if (json.contains("NegativeTorqueLimitValue")) {
            negativeTorqueLimitValue_ =
                tqeUnits * json.at("NegativeTorqueLimitValue").get<double>();
        }
        if (json.contains("TorqueSlope")) {
            torqueSlope_ = tqeUnits * json.at("TorqueSlope").get<double>();
        }
        if (json.contains("MaxAcceleration")) {
            maxAcceleration_ = accUnits * json.at("MaxAcceleration").get<double>();
        }
        if (json.contains("MaxDeceleration")) {
            maxDeceleration_ = accUnits * json.at("MaxDeceleration").get<double>();
        }
        if (json.contains("LowVelocityLimitValue")) {
            lowVelocityLimit_ = velUnits * json.at("LowVelocityLimitValue").get<double>();
        }
        if (json.contains("HighVelocityLimitValue")) {
            highVelocityLimit_ = velUnits * json.at("HighVelocityLimitValue").get<double>();
        }
        if (json.contains("TorqueLimitationOptionCode")) {
            torqueLimitationOptionCode_ =
                torqueCodemap_.at(json.at("TorqueLimitationOptionCode").get<std::string>());
        }
        if (json.contains("SoftwarePositionLimit")) {
            SoftwarePositionLimit limit;
            limit.min = posUnits * json.at("SoftwarePositionLimit").at("Min").get<double>();
            limit.max = posUnits * json.at("SoftwarePositionLimit").at("Max").get<double>();
            softwarePositionLimit_ = limit;
        }
        if (json.contains("QuickStopDeceleration")) {
            quickStopDeceleration_ = accUnits * json.at("QuickStopDeceleration").get<double>();
        }
        if (json.contains("ProfileDeceleration")) {
            profileDeceleration_ = accUnits * json.at("ProfileDeceleration").get<double>();
        }
    } catch(const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to CST has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
