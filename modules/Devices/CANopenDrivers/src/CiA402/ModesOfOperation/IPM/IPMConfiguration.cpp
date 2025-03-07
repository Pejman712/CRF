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

#include "CANopenDrivers/CiA402/ModesOfOperation/IPM/IPMConfiguration.hpp"

namespace crf::devices::canopendrivers {

IPMConfiguration::IPMConfiguration(const nlohmann::json& json) {
    parse(json);
}

std::optional<PositionRangeLimit> IPMConfiguration::getPositionRangeLimit() const {
    return positionRangeLimit_;
}
std::optional<SoftwarePositionLimit> IPMConfiguration::getSoftwarePositionLimit() const {
    return softwarePositionLimit_;
}

std::optional<int32_t> IPMConfiguration::getHomeOffset() const {
    return homeOffset_;
}

std::optional<int16_t> IPMConfiguration::getInterpolationSubModeSelect() const {
    return interpolationSubModeSelect_;
}

std::optional<InterpolationDataConfiguration> IPMConfiguration::getInterpolationDataConfiguration() const {  // NOLINT
    return interpolationDataConfiguration_;
}

std::optional<InterpolationTimePeriod> IPMConfiguration::getInterpolationTimePeriod() const {
    return interpolationTimePeriod_;
}

std::optional<uint32_t> IPMConfiguration::getMaxProfileVelocity() const {
    return maxProfileVelocity_;
}

std::optional<uint32_t> IPMConfiguration::getQuickStopDeceleration() const {
    return quickStopDeceleration_;
}

std::optional<uint32_t> IPMConfiguration::getMaxAcceleration()const  {
    return maxAcceleration_;
}

std::optional<uint32_t> IPMConfiguration::getMaxDeceleration() const {
    return maxDeceleration_;
}

std::optional<uint32_t> IPMConfiguration::getEndVelocity() const {
    return endVelocity_;
}

// Private

void IPMConfiguration::parse(const nlohmann::json& json) {
    try {
        // Get conversion values from the json
        double posUnits = json.at("PositionUnitConversion").get<double>();
        double velUnits = json.at("VelocityUnitConversion").get<double>();
        double accUnits = json.at("AccelerationUnitConversion").get<double>();

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
        if (json.contains("HomeOffset")) {
            homeOffset_ = posUnits * json.at("HomeOffset").get<double>();
        }
        if (json.contains("InterpolationSubModeSelect")) {
            interpolationSubModeSelect_ =
                interpolationModeMap_.at(json.at("InterpolationSubModeSelect").get<std::string>());
        }
        if (json.contains("InterpolationDataConfiguration")) {
            nlohmann::json inter = json.at("InterpolationDataConfiguration");
            InterpolationDataConfiguration data;
            data.maximumBufferSize = inter.at("MaximumBufferSize").get<uint32_t>();
            data.actualBufferSize = inter.at("ActualBufferSize").get<uint32_t>();
            data.bufferOrganization = inter.at("BufferOrganization").get<uint8_t>();
            data.bufferPosition = inter.at("BufferPosition").get<uint16_t>();
            data.sizeOfDataRecord = inter.at("SizeOfDataRecord").get<uint8_t>();
            data.bufferClear = inter.at("BufferClear").get<uint8_t>();
            interpolationDataConfiguration_ = data;
        }
        if (json.contains("InterpolationTimePeriod")) {
            InterpolationTimePeriod time;
            time.value = json.at("InterpolationTimePeriod").at("Value").get<uint8_t>();
            time.index = json.at("InterpolationTimePeriod").at("Index").get<int8_t>();
            interpolationTimePeriod_ = time;
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
            maxAcceleration_ = accUnits * json.at("MaxDeceleration").get<double>();
        }
        if (json.contains("EndVelocity")) {
            endVelocity_ = velUnits * json.at("EndVelocity").get<double>();
        }
    } catch(const std::exception& e) {
        throw std::invalid_argument(
            "The JSON file provided to IPM has error: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
