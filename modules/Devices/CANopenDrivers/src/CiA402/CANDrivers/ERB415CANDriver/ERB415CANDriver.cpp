/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <thread>
#include <future>
#include <cstdint>
#include <any>

#include <nlohmann/json.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>

#include "CANopenDrivers/CiA402/CANDrivers/ERB415CANDriver/ERB415CANDriver.hpp"

namespace crf::devices::canopendrivers {

ERB415CANDriver::ERB415CANDriver(
    std::shared_ptr<lely::canopen::AsyncMaster> master,
    const uint64_t& id,
    const nlohmann::json& j):
    CiA402CANDriver(master, id, j),
    logger_("ERB415CANDriver - " + std::to_string(id)) {
    logger_->debug("CTor");
}

ERB415CANDriver::~ERB415CANDriver() {
    logger_->debug("DTor");
}

crf::expected<bool> ERB415CANDriver::setProfilePosition(
    double pos, double vel, double acc, double dec, PositionReference reference) {
    logger_->debug("setProfilePosition");
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ppmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::ProfilePositionMode);
    if (!res) return res;

    int32_t position = pos * positionUnit_;
    uint32_t velocity = vel * velocityUnit_;

    crf::expected<bool> result = writeSDO<uint32_t>(
        CiA402::ProfileVelocity, Subindex::SUB0, velocity);
    if (!result) {
        logger_->error("Failed to write SDO to set profile velocity");
        return result;
    }
    tpdo_mapped[CiA402::TargetPosition][Subindex::SUB0] = position;
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] =
        static_cast<uint16_t>(ppm_->assumeNewTargetPositonWord());
    return true;
}

crf::expected<bool> ERB415CANDriver::setInterpolatedPosition(
    double pos, double vel, double acc, double dec) {
    logger_->debug("setInterpolatedPosition");
    if (getStatusWord() != StatusWord::OperationEnabled) return crf::Code::NotInitialized;
    if (!ipmAvailable_) return crf::Code::MethodNotAllowed;
    crf::expected<bool> res = setModeOfOperation(ModeOfOperation::InterpolatedPositionMode);
    if (!res) return res;

    int32_t position = pos * positionUnit_;

    tpdo_mapped[CiA402::InterpolationDataRecord][Subindex::SUB1] = position;
    tpdo_mapped[CiA402::ControlWord][Subindex::SUB0] = static_cast<uint16_t>(ipm_->setIPM());
    return true;
}

}  // namespace crf::devices::canopendrivers
