/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <optional>
#include <cstdint>
#include <any>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/ModesOfOperation/CST/CST.hpp"

namespace crf::devices::canopendrivers {

CST::CST(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> CST::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> CST::setTargetRegisters(const int16_t& torqueOffset) {
    targetRegisters_[0] = {CiA402::TorqueOffset, Subindex::SUB0, torqueOffset, Type::INT_16};
    return targetRegisters_;
}

bool CST::isDriveFollowingCommandValue(const uint16_t& statusWord) const {
    return (statusWord & StatusWordCSTMask::DriveFollowsCommandValue) ==
        StatusWordCST::DriveFollowsCommandValue;
}

// Private

void CST::parseConfigRegisters() {
    if (config_.getInterpolationTimePeriod()) {
        InterpolationTimePeriod time = config_.getInterpolationTimePeriod().value();
        configRegisters_.emplace_back(
            CiA402::InterpolationTimePeriod, Subindex::SUB1, time.value, Type::UINT_8);
        configRegisters_.emplace_back(
            CiA402::InterpolationTimePeriod, Subindex::SUB2, time.index, Type::INT_8);
    }
    if (config_.getSoftwarePositionLimit()) {
        SoftwarePositionLimit limit = config_.getSoftwarePositionLimit().value();
        configRegisters_.emplace_back(
            CiA402::SoftwarePositionLimit, Subindex::SUB1, limit.min, Type::INT_32);
        configRegisters_.emplace_back(
            CiA402::SoftwarePositionLimit, Subindex::SUB2, limit.max, Type::INT_32);
    }
    if (config_.getLowVelocityLimitValue()) {
        int32_t limit =  config_.getLowVelocityLimitValue().value();
        configRegisters_.emplace_back(
            CiA402::LowVelocityLimitValue, Subindex::SUB0, limit, Type::INT_32);
    }
    if (config_.getHighVelocityLimitValue()) {
        int32_t limit =  config_.getHighVelocityLimitValue().value();
        configRegisters_.emplace_back(
            CiA402::HighVelocityLimitValue, Subindex::SUB0, limit, Type::INT_32);
    }
    if (config_.getMaxAcceleration()) {
        uint32_t acc =  config_.getMaxAcceleration().value();
        configRegisters_.emplace_back(CiA402::MaxAcceleration, Subindex::SUB0, acc, Type::UINT_32);
    }
    if (config_.getMaxDeceleration()) {
        uint32_t dec =  config_.getMaxDeceleration().value();
        configRegisters_.emplace_back(CiA402::MaxDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getQuickStopDeceleration()) {
        uint32_t dec = config_.getQuickStopDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::QuickStopDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getProfileDeceleration()) {
        uint32_t dec = config_.getProfileDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::ProfileDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getPositiveTorqueLimitValue()) {
        uint16_t limit = config_.getPositiveTorqueLimitValue().value();
        configRegisters_.emplace_back(
            CiA402::PositiveTorqueLimitValue, Subindex::SUB0, limit, Type::UINT_16);
    }
    if (config_.getNegativeTorqueLimitValue()) {
        uint16_t limit = config_.getNegativeTorqueLimitValue().value();
        configRegisters_.emplace_back(
            CiA402::NegativeTorqueLimitValue, Subindex::SUB0, limit, Type::UINT_16);
    }
    if (config_.getTorqueSlope()) {
        uint32_t slope =  config_.getTorqueSlope().value();
        configRegisters_.emplace_back(
            CiA402::TorqueSlope, Subindex::SUB0, slope, Type::UINT_32);
    }
    if (config_.getTorqueLimitationOptionCode()) {
        TorqueLimitationOptionCode code = config_.getTorqueLimitationOptionCode().value();
        configRegisters_.emplace_back(
            CiA402::TorqueLimitationOptionCode, Subindex::SUB0,
            static_cast<int8_t>(code), Type::INT_8);
    }
}

}  // namespace crf::devices::canopendrivers
