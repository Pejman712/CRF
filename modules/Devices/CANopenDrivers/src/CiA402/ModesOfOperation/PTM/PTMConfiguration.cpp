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

#include "CANopenDrivers/CiA402/ModesOfOperation/PTM/PTMConfiguration.hpp"

namespace crf::devices::canopendrivers {

PTMConfiguration::PTMConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<uint16_t> PTMConfiguration::getPositiveTorqueLimitValue() const {
    return positiveTorqueLimitValue_;
}

std::optional<uint16_t> PTMConfiguration::getNegativeTorqueLimitValue() const {
    return negativeTorqueLimitValue_;
}

std::optional<uint32_t> PTMConfiguration::getTorqueSlope() const {
    return torqueSlope_;
}

std::optional<TorqueProfileTypePTM> PTMConfiguration::getTorqueProfileType() const {
    return torqueProfileType_;
}

// Private

void PTMConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double tqeUnits = json.at("TorqueUnitConversion").get<double>();

        if (json.contains("PositiveTorqueLimitValue")) {
            positiveTorqueLimitValue_ =
                tqeUnits * json.at("PositiveTorqueLimitValue").get<uint16_t>();
        }
        if (json.contains("NegativeTorqueLimitValue")) {
            negativeTorqueLimitValue_ =
                tqeUnits * json.at("NegativeTorqueLimitValue").get<uint16_t>();
        }
        if (json.contains("TorqueSlope")) {
            torqueSlope_ = tqeUnits * json.at("TorqueSlope").get<double>();
        }
        if (json.contains("TorqueProfileType")) {
            torqueProfileType_ = profileMap_.at(json.at("TorqueProfileType").get<std::string>());
        }
    } catch(const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to PTM has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
