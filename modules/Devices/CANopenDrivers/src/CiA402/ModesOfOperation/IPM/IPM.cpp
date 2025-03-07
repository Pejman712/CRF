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

#include "CANopenDrivers/CiA402/ModesOfOperation/IPM/IPM.hpp"

namespace crf::devices::canopendrivers {

IPM::IPM(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> IPM::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> IPM::setTargetRegisters(
    const uint32_t& profileVelocity,
    const uint32_t& profileAcceleration,
    const uint32_t& profileDeceleration) {
    targetRegisters_[0] =
        {CiA402::ProfileVelocity, Subindex::SUB0, profileVelocity, Type::UINT_32};
    targetRegisters_[1] =
        {CiA402::ProfileAcceleration, Subindex::SUB0, profileAcceleration, Type::UINT_32};
    targetRegisters_[2] =
        {CiA402::ProfileDeceleration, Subindex::SUB0, profileDeceleration, Type::UINT_32};
    return targetRegisters_;
}

ControlWordIPM IPM::setIPM() const {
    return ControlWordIPM::EnableIPM;
}

ControlWordIPM IPM::activateWord() const {
    return ControlWordIPM::Execute;
}

ControlWordIPM IPM::deactivateWord() const {
    return ControlWordIPM::Halt;
}

ControlWordIPM IPM::unsetIPM() const {
    return ControlWordIPM::DisableIPM;
}

bool IPM::isTargetReached(const uint16_t& statusWord) const {
    return (statusWord & StatusWordIPMMask::TargetReached) == StatusWordIPM::TargetReached;
}

bool IPM::isIPMModeSet(const uint16_t& statusWord) const {
    return (statusWord & StatusWordIPMMask::IPMModeActive) == StatusWordIPM::IPMModeActive;
}

bool IPM::isFollowingError(const uint16_t& statusWord) const {
    return (statusWord & StatusWordIPMMask::FollowingError) == StatusWordIPM::FollowingError;
}

// Private

void IPM::parseConfigRegisters() {
    if (config_.getInterpolationTimePeriod()) {
        InterpolationTimePeriod time = config_.getInterpolationTimePeriod().value();
        configRegisters_.emplace_back(
            CiA402::InterpolationTimePeriod, Subindex::SUB1, time.value, Type::UINT_8);
        configRegisters_.emplace_back(
            CiA402::InterpolationTimePeriod, Subindex::SUB2, time.index, Type::INT_8);
    }
    if (config_.getPositionRangeLimit()) {
        PositionRangeLimit limit = config_.getPositionRangeLimit().value();
        configRegisters_.emplace_back(
            CiA402::PositionRangeLimit, Subindex::SUB1, limit.min, Type::INT_32);
        configRegisters_.emplace_back(
            CiA402::PositionRangeLimit, Subindex::SUB2, limit.max, Type::INT_32);
    }
    if (config_.getSoftwarePositionLimit()) {
        SoftwarePositionLimit limit = config_.getSoftwarePositionLimit().value();
        configRegisters_.emplace_back(
            CiA402::SoftwarePositionLimit, Subindex::SUB1, limit.min, Type::INT_32);
        configRegisters_.emplace_back(
            CiA402::SoftwarePositionLimit, Subindex::SUB2, limit.max, Type::INT_32);
    }
    if (config_.getHomeOffset()) {
        configRegisters_.emplace_back(
            CiA402::HomeOffset, Subindex::SUB0, config_.getHomeOffset().value(), Type::INT_32);
    }
    if (config_.getInterpolationSubModeSelect()) {
        int16_t mode = config_.getInterpolationSubModeSelect().value();
        configRegisters_.emplace_back(
            CiA402::InterpolationSubModeSelect, Subindex::SUB0, mode, Type::INT_16);
    }
    if (config_.getInterpolationDataConfiguration()) {
        InterpolationDataConfiguration data = config_.getInterpolationDataConfiguration().value();
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB1, data.maximumBufferSize, Type::UINT_32);
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB2, data.actualBufferSize, Type::UINT_32);
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB3, data.bufferOrganization, Type::UINT_8);
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB4, data.bufferPosition, Type::UINT_16);
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB5, data.sizeOfDataRecord, Type::UINT_8);
        configRegisters_.emplace_back(
            CiA402::InterpolationDataConfiguration,
            Subindex::SUB6, data.bufferClear, Type::UINT_8);
    }
    if (config_.getMaxProfileVelocity()) {
        uint32_t vel = config_.getMaxProfileVelocity().value();
        configRegisters_.emplace_back(
            CiA402::MaxProfileVelocity, Subindex::SUB0, vel, Type::UINT_32);
    }
    if (config_.getQuickStopDeceleration()) {
        uint32_t dec = config_.getQuickStopDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::QuickStopDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getMaxAcceleration()) {
        uint32_t acc =  config_.getMaxAcceleration().value();
        configRegisters_.emplace_back(CiA402::MaxAcceleration, Subindex::SUB0, acc, Type::UINT_32);
    }
    if (config_.getMaxDeceleration()) {
        uint32_t dec =  config_.getMaxDeceleration().value();
        configRegisters_.emplace_back(CiA402::MaxDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getEndVelocity()) {
        uint32_t vel = config_.getEndVelocity().value();
        configRegisters_.emplace_back(CiA402::EndVelocity, Subindex::SUB0, vel, Type::UINT_32);
    }
}

}  // namespace crf::devices::canopendrivers
