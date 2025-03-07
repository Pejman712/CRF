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

#include "CANopenDrivers/CiA402/ModesOfOperation/CSV/CSVConfiguration.hpp"

namespace crf::devices::canopendrivers {

CSVConfiguration::CSVConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<uint32_t> CSVConfiguration::getQuickStopDeceleration() const {
    return quickStopDeceleration_;
}

std::optional<uint32_t> CSVConfiguration::getProfileDeceleration() const {
    return profileDeceleration_;
}

std::optional<ProfileType> CSVConfiguration::getMotionProfileType() const {
    return motionProfileType_;
}

std::optional<uint8_t> CSVConfiguration::getProfileJerkUse() const {
    return profileJerkUse_;
}

std::optional<std::vector<uint32_t>> CSVConfiguration::getProfileJerks() const {
    return profileJerks_;
}

std::optional<SoftwarePositionLimit> CSVConfiguration::getSoftwarePositionLimit() const {
    return softwarePositionLimit_;
}

std::optional<InterpolationTimePeriod> CSVConfiguration::getInterpolationTimePeriod() const {
    return interpolationTimePeriod_;
}

// Private

void CSVConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double posUnits = json.at("PositionUnitConversion").get<double>();
        double accUnits = json.at("AccelerationUnitConversion").get<double>();
        double jrkUnits = json.at("JerkUnitConversion").get<double>();

        // Parsing and conversion to driver units
        if (json.contains("SoftwarePositionLimit")) {
            SoftwarePositionLimit limit;
            limit.min = posUnits * json.at("SoftwarePositionLimit").at("Min").get<double>();
            limit.max = posUnits * json.at("SoftwarePositionLimit").at("Max").get<double>();
            softwarePositionLimit_ = limit;
        }
        if (json.contains("QuickStopDeceleration")) {
            quickStopDeceleration_ = accUnits * json.at("QuickStopDeceleration").get<double>();
        }
        if (json.contains("MotionProfileType")) {
            motionProfileType_ =
                profileTypeMap_.at(json.at("MotionProfileType").get<std::string>());
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
        if (json.contains("ProfileDeceleration")) {
            profileDeceleration_ = accUnits * json.at("ProfileDeceleration").get<double>();
        }
    } catch(const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to CSV has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
