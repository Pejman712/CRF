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

#include "CANopenDrivers/CiA402/ModesOfOperation/PPM/PPM.hpp"

namespace crf::devices::canopendrivers {

PPM::PPM(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> PPM::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> PPM::setTargetRegisters(
    const uint32_t& profileVelocity, const uint32_t& profileAcceleration,
    const uint32_t& profileDeceleration, const PositionReference& reference) {
    targetRegisters_[0] =
        {CiA402::ProfileVelocity, Subindex::SUB0, profileVelocity, Type::UINT_32};
    targetRegisters_[1] =
        {CiA402::ProfileAcceleration, Subindex::SUB0, profileAcceleration, Type::UINT_32};
    targetRegisters_[2] =
        {CiA402::ProfileDeceleration, Subindex::SUB0, profileDeceleration, Type::UINT_32};
    reference_ = reference;
    return targetRegisters_;
}

ControlWordPPM PPM::activateWord() const {
    return ControlWordPPM::Execute;
}

ControlWordPPM PPM::deactivateWord() const {
    return ControlWordPPM::Halt;
}

ControlWordPPM PPM::activateEndlessMovementWord() const {
    return ControlWordPPM::ActivateEndlessMovement;
}

ControlWordPPM PPM::deactivateEndlessMovementWord() const {
    return ControlWordPPM::DeactivateEndlessMovement;
}

ControlWordPPM PPM::assumeNewTargetPositonWord() const {
    if (reference_ == PositionReference::Relative) {
        return static_cast<ControlWordPPM>(
            ControlWordPPM::Relative | ControlWordPPM::AssumeTargetPosition);
    } else {
        return ControlWordPPM::AssumeTargetPosition;
    }
}

ControlWordPPM PPM::activateChangeSetImmediatelyWord() const {
    if (reference_ == PositionReference::Relative) {
        return static_cast<ControlWordPPM>(
            ControlWordPPM::Relative | ControlWordPPM::ActivateChangeSetPoint);
    } else {
        return ControlWordPPM::ActivateChangeSetPoint;
    }
}

ControlWordPPM PPM::deactivateChangeSetImmediatelyWord() const {
    return ControlWordPPM::DeactivateChangeSetPoint;
}

ControlWordPPM PPM::activateChangeSetPoint() const {
    if (reference_ == PositionReference::Relative) {
        return static_cast<ControlWordPPM>(
            ControlWordPPM::Relative | ControlWordPPM::ActivateLaterChangeSetPoint);
    } else {
        return ControlWordPPM::ActivateLaterChangeSetPoint;
    }
}

ControlWordPPM PPM::deactivateChangeSetPoint() const {
    return ControlWordPPM::DeactivateLaterChangeSetPoint;
}

bool PPM::isTargetReached(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPPMMask::TargetReachedMaskPPM) ==
        StatusWordPPM::TargetReachedPPM;
}

bool PPM::isSetPointAcknowledged(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPPMMask::SetpointAcknowledgeMaskPPM) ==
        StatusWordPPM::SetpointAcknowledgePPM;
}

bool PPM::isFollowingError(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPPMMask::FollowingErrorMaskPPM) ==
        StatusWordPPM::FollowingErrorPPM;
}

// Private

void PPM::parseConfigRegisters() {
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
    if (config_.getMaxProfileVelocity()) {
        uint32_t vel = config_.getMaxProfileVelocity().value();
        configRegisters_.emplace_back(
            CiA402::MaxProfileVelocity, Subindex::SUB0, vel, Type::UINT_32);
    }
    if (config_.getMaxAcceleration()) {
        uint32_t acc = config_.getMaxAcceleration().value();
        configRegisters_.emplace_back(
            CiA402::MaxAcceleration, Subindex::SUB0, acc, Type::UINT_32);
    }
    if (config_.getMaxDeceleration()) {
        uint32_t dec = config_.getMaxDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::MaxDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getMotionProfileType()) {
        int16_t motion = static_cast<int16_t>(config_.getMotionProfileType().value());
        configRegisters_.emplace_back(
            CiA402::MotionProfileType, Subindex::SUB0, motion, Type::INT_16);
    }
    if (config_.getQuickStopDeceleration()) {
        uint32_t dec = config_.getQuickStopDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::QuickStopDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getProfileJerkUse()) {
        uint8_t jerk = config_.getProfileJerkUse().value();
        configRegisters_.emplace_back(CiA402::ProfileJerkUse, Subindex::SUB0, jerk, Type::UINT_8);
    }
    if (config_.getProfileJerks()) {
        std::vector<uint32_t> jerks = config_.getProfileJerks().value();
        for (uint64_t i = 0; i < jerks.size(); i++) {
            configRegisters_.emplace_back(CiA402::ProfileJerk, i + 1, jerks[i], Type::UINT_32);
        }
    }
    if (config_.getEndVelocity()) {
        uint32_t vel = config_.getEndVelocity().value();
        configRegisters_.emplace_back(CiA402::EndVelocity, Subindex::SUB0, vel, Type::UINT_32);
    }
}

}  // namespace crf::devices::canopendrivers
