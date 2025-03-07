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

#include "CANopenDrivers/CiA402/ModesOfOperation/PPM/PPMConfiguration.hpp"

namespace crf::devices::canopendrivers {

PPMConfiguration::PPMConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<PositionRangeLimit> PPMConfiguration::getPositionRangeLimit() const {
    return positionRangeLimit_;
}

std::optional<SoftwarePositionLimit> PPMConfiguration::getSoftwarePositionLimit() const {
    return softwarePositionLimit_;
}

std::optional<uint32_t> PPMConfiguration::getMaxProfileVelocity() const {
    return maxProfileVelocity_;
}

std::optional<uint32_t> PPMConfiguration::getQuickStopDeceleration() const {
    return quickStopDeceleration_;
}

std::optional<uint32_t> PPMConfiguration::getMaxAcceleration() const {
    return maxAcceleration_;
}

std::optional<uint32_t> PPMConfiguration::getMaxDeceleration() const {
    return maxDeceleration_;
}

std::optional<ProfileType> PPMConfiguration::getMotionProfileType() const {
    return motionProfileType_;
}

std::optional<uint8_t> PPMConfiguration::getProfileJerkUse() const {
    return profileJerkUse_;
}

std::optional<std::vector<uint32_t>> PPMConfiguration::getProfileJerks() const {
    return profileJerks_;
}

std::optional<uint32_t> PPMConfiguration::getEndVelocity() const {
    return endVelocity_;
}

// Private

void PPMConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double posUnits = json.at("PositionUnitConversion").get<double>();
        double velUnits = json.at("VelocityUnitConversion").get<double>();
        double accUnits = json.at("AccelerationUnitConversion").get<double>();
        double jrkUnits = json.at("JerkUnitConversion").get<double>();

        // Parsing and conversion to driver units
        if (json.contains("PositionRangeLimit")) {
            PositionRangeLimit limit;
            limit.min = posUnits * json.at("PositionRangeLimit").at("Min").get<double>();
            limit.max = posUnits * json.at("PositionRangeLimit").at("Max").get<double>();
            positionRangeLimit_ = limit;
        }
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
            quickStopDeceleration_ = accUnits * json.at("QuickStopDeceleration").get<double>();
        }
        if (json.contains("MaxAcceleration")) {
            maxAcceleration_ = accUnits * json.at("MaxAcceleration").get<double>();
        }
        if (json.contains("MaxDeceleration")) {
            maxDeceleration_ = accUnits * json.at("MaxDeceleration").get<double>();
        }
        if (json.contains("MotionProfileType")) {
            motionProfileType_ = profileMap_.at(json.at("MotionProfileType").get<std::string>());
        }
        if (json.contains("ProfileJerkUse")) {
            profileJerkUse_ = json.at("ProfileJerkUse").get<uint8_t>();
        }
        if (json.contains("ProfileJerks")) {
            std::vector<double> jerks = json.at("ProfileJerks").get<std::vector<double>>();
            profileJerks_ = std::vector<uint32_t>(jerks.size());
            for (uint64_t i = 0; i < jerks.size(); i++) {
                profileJerks_.value()[i] = jerks[i] * jrkUnits;
            }
        }
        if (json.contains("EndVelocity")) {
            endVelocity_ = velUnits * json["EndVelocity"].get<double>();
        }
    } catch (const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to PPM has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
