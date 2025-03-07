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

#include "CANopenDrivers/CiA402/ModesOfOperation/PVM/PVM.hpp"

namespace crf::devices::canopendrivers {

PVM::PVM(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> PVM::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> PVM::setTargetRegisters(
    const uint32_t& profileAcceleration, const uint32_t& profileDeceleration) {
    targetRegisters_[0] =
        {CiA402::ProfileAcceleration, Subindex::SUB0, profileAcceleration, Type::UINT_32};
    targetRegisters_[1] =
        {CiA402::ProfileDeceleration, Subindex::SUB0, profileDeceleration, Type::UINT_32};
    return targetRegisters_;
}

ControlWordPVM PVM::activateWord() const {
    return ControlWordPVM::Execute;
}

ControlWordPVM PVM::deactivateWord() const {
    return ControlWordPVM::Halt;
}

bool PVM::isTargetReached(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPVMMask::TargetReached) == StatusWordPVM::TargetReached;
}

bool PVM::isSpeedLimited(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPVMMask::SpeedLimited) == StatusWordPVM::SpeedLimited;
}

bool PVM::isSpeedZero(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPVMMask::SpeedIsZero) == StatusWordPVM::SpeedIsZero;
}

bool PVM::isMaxSlippageReached(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPVMMask::MaxSlippageError) == StatusWordPVM::MaxSlippageError;
}

// Private

void PVM::parseConfigRegisters() {
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
    if (config_.getQuickStopDeceleration()) {
        uint32_t dec = config_.getQuickStopDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::QuickStopDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getMaxAcceleration()) {
        uint32_t acc = config_.getMaxAcceleration().value();
        configRegisters_.emplace_back(
            CiA402::MaxAcceleration, Subindex::SUB0, acc, Type::UINT_32);
    }
    if (config_.getMaxDeceleration()) {
        uint32_t dec =  config_.getMaxDeceleration().value();
        configRegisters_.emplace_back(CiA402::MaxDeceleration, Subindex::SUB0, dec, Type::UINT_32);
    }
    if (config_.getMotionProfileType()) {
        int16_t motion = static_cast<int16_t>(config_.getMotionProfileType().value());
        configRegisters_.emplace_back(
            CiA402::MotionProfileType, Subindex::SUB0, motion, Type::INT_16);
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
    if (config_.getSensorSelectionCode()) {
        int16_t code = config_.getSensorSelectionCode().value();
        configRegisters_.emplace_back(
            CiA402::SensorSelectionCode, Subindex::SUB0, code, Type::INT_16);
    }
    if (config_.getVelocityWindow()) {
        uint16_t window = config_.getVelocityWindow().value();
        configRegisters_.emplace_back(
            CiA402::VelocityWindow, Subindex::SUB0, window, Type::UINT_16);
    }
    if (config_.getVelocityWindowTime()) {
        uint16_t time = config_.getVelocityWindowTime().value();
        configRegisters_.emplace_back(
            CiA402::VelocityWindowTime, Subindex::SUB0, time, Type::UINT_16);
    }
    if (config_.getVelocityThreshold()) {
        uint16_t thres = config_.getVelocityThreshold().value();
        configRegisters_.emplace_back(
            CiA402::VelocityThreshold, Subindex::SUB0, thres, Type::UINT_16);
    }
    if (config_.getVelocityThresholdTime()) {
        uint16_t time = config_.getVelocityThresholdTime().value();
        configRegisters_.emplace_back(
            CiA402::VelocityThresholdTime, Subindex::SUB0, time, Type::UINT_16);
    }
    if (config_.getMaxSlippage()) {
        configRegisters_.emplace_back(
            CiA402::MaxSlippage, Subindex::SUB0, config_.getMaxSlippage().value(), Type::INT_32);
    }
}

}  // namespace crf::devices::canopendrivers
