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

#include "CANopenDrivers/CiA402/ModesOfOperation/VOM/VOMConfiguration.hpp"

namespace crf::devices::canopendrivers {

VOMConfiguration::VOMConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<VelocityMinMaxAmountVOM> VOMConfiguration::getVelocityMinMaxAmount() const {
    return velocityMinMaxAmount_;
}

std::optional<VelocityQuickStopVOM> VOMConfiguration::getVelocityQuickStop() const {
    return velocityQuickStop_;
}

std::optional<VelocitySetPointFactorVOM> VOMConfiguration::getVelocitySetPointFactor() const {
    return velocitySetPointFactor_;
}

std::optional<DimensionFactorVOM> VOMConfiguration::getDimensionFactor() const {
    return dimensionFactor_;
}

// Private

void VOMConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double velUnits = json.at("VelocityUnitConversion").get<double>();

        if (json.contains("VelocityMinMaxAmount")) {
            VelocityMinMaxAmountVOM vel;
            vel.min = velUnits * json.at("VelocityMinMaxAmount").at("Min").get<double>();
            vel.max = velUnits * json.at("VelocityMinMaxAmount").at("Max").get<double>();
            velocityMinMaxAmount_ = vel;
        }
        if (json.contains("VelocityQuickStop")) {
            VelocityQuickStopVOM vel;
            vel.deltaSpeed = velUnits * json.at("VelocityQuickStop").at("DeltaSpeed").get<uint32_t>();  // NOLINT
            vel.deltaTime = json.at("VelocityQuickStop").at("DeltaTime").get<uint16_t>();  // sec
            velocityQuickStop_ = vel;
        }
        if (json.contains("SetPointFactor")) {
            VelocitySetPointFactorVOM factor;
            factor.numerator = json.at("SetPointFactor").at("Numerator").get<int16_t>();
            factor.denominator = json.at("SetPointFactor").at("Denominator").get<int16_t>();  // NOLINT
            velocitySetPointFactor_ = factor;
        }
        if (json.contains("DimensionFactor")) {
            DimensionFactorVOM factor;
            factor.numerator = json.at("DimensionFactor").at("Numerator").get<int32_t>();
            factor.denominator = json.at("DimensionFactor").at("Denominator").get<int32_t>();
            dimensionFactor_ = factor;
        }
    } catch (const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to VOM has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
