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

#include "CANopenDrivers/CiA402/ModesOfOperation/PTM/PTM.hpp"

namespace crf::devices::canopendrivers {

PTM::PTM(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> PTM::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> PTM::setTargetRegisters(const int16_t& targetTorque) {
    targetRegisters_[0] = {CiA402::TargetTorque, Subindex::SUB0, targetTorque, Type::INT_16};
    return targetRegisters_;
}

ControlWordPTM PTM::activateWord() const {
    return ControlWordPTM::Execute;
}

ControlWordPTM PTM::deactivateWord() const {
    return ControlWordPTM::Halt;
}

bool PTM::isTargetReached(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPTMMask::TargetReached) == StatusWordPTM::TargetReached;
}

bool PTM::isTorqueLimited(const uint16_t& statusWord) const {
    return (statusWord & StatusWordPTMMask::TorqueLimited) == StatusWordPTM::TorqueLimited;
}

// Private

void PTM::parseConfigRegisters() {
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
        uint32_t slope = config_.getTorqueSlope().value();
        configRegisters_.emplace_back(CiA402::TorqueSlope, Subindex::SUB0, slope, Type::UINT_32);
    }
    if (config_.getTorqueProfileType()) {
        int16_t type = static_cast<int16_t>(config_.getTorqueProfileType().value());
        configRegisters_.emplace_back(
            CiA402::TorqueProfileType, Subindex::SUB0, type, Type::INT_16);
    }
}

}  // namespace crf::devices::canopendrivers
