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

#include "CANopenDrivers/CiA402/ModesOfOperation/PVM/PVMConfiguration.hpp"

namespace crf::devices::canopendrivers {

PVMConfiguration::PVMConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<SoftwarePositionLimit> PVMConfiguration::getSoftwarePositionLimit() const {
    return softwarePositionLimit_;
}

std::optional<uint32_t> PVMConfiguration::getMaxProfileVelocity() const {
    return maxProfileVelocity_;
}

std::optional<uint32_t> PVMConfiguration::getQuickStopDeceleration() const {
    return quickStopDeceleration_;
}

std::optional<uint32_t> PVMConfiguration::getMaxAcceleration() const {
    return maxAcceleration_;
}

std::optional<uint32_t> PVMConfiguration::getMaxDeceleration() const {
    return maxDeceleration_;
}

std::optional<ProfileType> PVMConfiguration::getMotionProfileType() const {
    return motionProfileType_;
}

std::optional<uint8_t> PVMConfiguration::getProfileJerkUse() const {
    return profileJerkUse_;
}

std::optional<std::vector<uint32_t>> PVMConfiguration::getProfileJerks() const {
    return profileJerks_;
}

std::optional<int16_t> PVMConfiguration::getSensorSelectionCode() const {
    return sensorSelectionCode_;
}

std::optional<uint16_t> PVMConfiguration::getVelocityWindow() const {
    return velocityWindow_;
}

std::optional<uint16_t> PVMConfiguration::getVelocityWindowTime() const {
    return velocityWindowTime_;
}

std::optional<uint16_t> PVMConfiguration::getVelocityThreshold() const {
    return velocityThreshold_;
}

std::optional<uint16_t> PVMConfiguration::getVelocityThresholdTime() const {
    return velocityThresholdTime_;
}

std::optional<int32_t> PVMConfiguration::getMaxSlippage() const {
    return maxSlippage_;
}

// Private

void PVMConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double posUnits = json.at("PositionUnitConversion").get<double>();
        double velUnits = json.at("VelocityUnitConversion").get<double>();
        double accUnits = json.at("AccelerationUnitConversion").get<double>();

        if (json.contains("SoftwarePositionLimit")) {
            SoftwarePositionLimit limit;
            limit.min = posUnits * json.at("SoftwarePositionLimit").at("Min").get<double>();
            limit.max = posUnits * json.at("SoftwarePositionLimit").at("Max").get<double>();
            softwarePositionLimit_ = limit;
        }
        if (json.contains("MaxProfileVelocity")) {
            maxProfileVelocity_ = velUnits * json.at("MaxProfileVelocity").get<double>();
        }
        if (json.contains("QuickStopDeceleration")) {
            quickStopDeceleration_ = json.at("QuickStopDeceleration").get<double>();
        }
        if (json.contains("MaxAcceleration")) {
            maxAcceleration_ = accUnits * json.at("MaxAcceleration").get<double>();
        }
        if (json.contains("MaxDeceleration")) {
            maxDeceleration_ = accUnits * json.at("MaxDeceleration").get<double>();
        }
        if (json.contains("MotionProfileType")) {
            motionProfileType_ =
                profileTypeMap_.at(json.at("MotionProfileType").get<std::string>());
        }
        if (json.contains("SensorSelectionCode")) {
            sensorSelectionCode_ = json.at("SensorSelectionCode").get<int16_t>();
        }
        if (json.contains("VelocityWindow")) {
            velocityWindow_ = velUnits * json.at("VelocityWindow").get<double>();
        }
        if (json.contains("VelocityWindowTime")) {
            velocityWindowTime_ = secToMillisec_ * json.at("VelocityWindowTime").get<double>();
        }
        if (json.contains("VelocityThreshold")) {
            velocityThreshold_ = velUnits * json.at("VelocityThreshold").get<double>();
        }
        if (json.contains("VelocityThresholdTime")) {
            velocityThresholdTime_ =
                secToMillisec_ * json.at("VelocityThresholdTime").get<double>();
        }
        if (json.contains("MaxSlippage")) {
            maxSlippage_ = velUnits * json.at("MaxSlippage").get<double>();
        }
    } catch (const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to PVM has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
