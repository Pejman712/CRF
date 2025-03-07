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

#include "CANopenDrivers/CiA402/ModesOfOperation/CSV/CSV.hpp"

namespace crf::devices::canopendrivers {

CSV::CSV(const nlohmann::json& config):
    config_(config) {
    configRegisters_.reserve(numberOfConfigRegisters_);
    targetRegisters_ = std::vector<RegisterValues>(numberOfTargetRegisters_);
    parseConfigRegisters();
}

std::vector<RegisterValues> CSV::getConfigRegisters() const {
    return configRegisters_;
}

std::vector<RegisterValues> CSV::setTargetRegisters(
    const int32_t& velocityOffset, const int16_t& torqueOffset) {
    targetRegisters_[0] = {CiA402::VelocityOffset, Subindex::SUB0, velocityOffset, Type::INT_32};
    targetRegisters_[1] = {CiA402::TorqueOffset, Subindex::SUB0, torqueOffset, Type::INT_16};
    return targetRegisters_;
}

bool CSV:: isDriveFollowingCommandValue(const uint16_t& statusWord) const {
    return (statusWord & StatusWordCSVMask::DriveFollowsCommandValue) ==
        StatusWordCSV::DriveFollowsCommandValue;
}

// Private

void CSV::parseConfigRegisters() {
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
    if (config_.getMotionProfileType()) {
        int16_t motion = static_cast<int16_t>(config_.getMotionProfileType().value());
        configRegisters_.emplace_back(
            CiA402::MotionProfileType, Subindex::SUB0, motion, Type::INT_16);
    }
    if (config_.getProfileDeceleration()) {
        uint32_t dec = config_.getProfileDeceleration().value();
        configRegisters_.emplace_back(
            CiA402::ProfileDeceleration, Subindex::SUB0, dec, Type::UINT_32);
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
}

}  // namespace crf::devices::canopendrivers
