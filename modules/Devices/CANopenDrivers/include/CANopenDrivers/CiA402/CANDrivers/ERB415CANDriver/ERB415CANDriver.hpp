/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <future>

#include <nlohmann/json.hpp>
#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/master.hpp>

#include "CANopenDrivers/CiA402/CANDrivers/CiA402CANDriver/CiA402CANDriver.hpp"

#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_erb_can_driver
 * @brief More specialized driver for the ERB motors.
 *
 */
class ERB415CANDriver :public CiA402CANDriver{
 public:
    ERB415CANDriver(
        std::shared_ptr<lely::canopen::AsyncMaster> master, // NOLINT
        const uint64_t& id,
        const nlohmann::json& j);
    ~ERB415CANDriver() override;

    crf::expected<bool> setProfilePosition(double pos, double vel, double acc, double dec,
        PositionReference reference = PositionReference::Absolute) override;
    crf::expected<bool> setInterpolatedPosition(
        double pos, double vel, double acc, double dec) override;

 private:
    nlohmann::json json_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::devices::canopendrivers
