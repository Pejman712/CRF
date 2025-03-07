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

#include "CANopenDrivers/CiA402/ModesOfOperation/VOM/VOM.hpp"

namespace crf::devices::canopendrivers {

VOM::VOM(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> VOM::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> VOM::setTargetRegisters(
    const uint32_t& deltaSpeedAcc, const uint16_t& deltaTimeAcc,
    const uint32_t deltaSpeedDec, const uint16_t deltaTimeDec) {
    targetRegisters_[0] =
        {CiA402::VLVelocityAcceleration, Subindex::SUB1, deltaSpeedAcc, Type::UINT_32};
    targetRegisters_[1] =
        {CiA402::VLVelocityAcceleration, Subindex::SUB2, deltaTimeAcc, Type::UINT_16};
    targetRegisters_[2] =
        {CiA402::VLVelocityDeceleration, Subindex::SUB1, deltaSpeedDec, Type::UINT_32};
    targetRegisters_[3] =
        {CiA402::VLVelocityDeceleration, Subindex::SUB2, deltaTimeDec, Type::UINT_16};
    return targetRegisters_;
}

ControlWordVOM VOM::activateWord() const {
    return ControlWordVOM::Execute;
}

ControlWordVOM VOM::deactivateWord() const {
    return ControlWordVOM::Halt;
}

bool VOM::isSpeedLimited(const uint16_t& statusWord) const {
    return (statusWord & StatusWordVOMMask::SpeedLimited) == StatusWordVOM::SpeedLimited;
}

// Private

void VOM::parseConfigRegisters() {
    if (config_.getVelocityMinMaxAmount()) {
        VelocityMinMaxAmountVOM minMax = config_.getVelocityMinMaxAmount().value();
        configRegisters_.emplace_back(
            CiA402::VelocityMinMaxAmount,
            Subindex::SUB1, static_cast<uint32_t>(minMax.min), Type::UINT_32);
        configRegisters_.emplace_back(
            CiA402::VelocityMinMaxAmount,
            Subindex::SUB2, static_cast<uint32_t>(minMax.max), Type::UINT_32);
    }
    if (config_.getVelocityQuickStop()) {
        VelocityQuickStopVOM vel = config_.getVelocityQuickStop().value();
        configRegisters_.emplace_back(
            CiA402::VLVelocityQuickStop, Subindex::SUB1, vel.deltaSpeed, Type::UINT_32);
        configRegisters_.emplace_back(
            CiA402::VLVelocityQuickStop, Subindex::SUB2, vel.deltaTime, Type::UINT_16);
    }
    if (config_.getVelocitySetPointFactor()) {
        VelocitySetPointFactorVOM factor = config_.getVelocitySetPointFactor().value();
        configRegisters_.emplace_back(
            CiA402::VLSetPointFactor, Subindex::SUB1, factor.numerator, Type::INT_16);
        configRegisters_.emplace_back(
            CiA402::VLSetPointFactor, Subindex::SUB2, factor.denominator, Type::INT_16);
    }
    if (config_.getDimensionFactor()) {
        DimensionFactorVOM factor = config_.getDimensionFactor().value();
        configRegisters_.emplace_back(
            CiA402::VLDimensionFactor, Subindex::SUB1, factor.numerator, Type::INT_32);
        configRegisters_.emplace_back(
            CiA402::VLDimensionFactor, Subindex::SUB2, factor.denominator, Type::INT_32);
    }
}

}  // namespace crf::devices::canopendrivers
