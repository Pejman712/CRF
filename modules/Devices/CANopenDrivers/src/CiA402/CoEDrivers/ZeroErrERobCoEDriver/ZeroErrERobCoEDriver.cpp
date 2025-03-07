/*
 * © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <cmath>
#include <chrono>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CoEDrivers/ZeroErrERobCoEDriver/ZeroErrERobCoEDriver.hpp"

namespace crf::devices::canopendrivers {

ZeroErrERobCoEDriver::ZeroErrERobCoEDriver(
    std::shared_ptr<CoEMaster> master, const uint64_t &id, const nlohmann::json& j):
    CiA402CoEDriver(master, id, j),
    logger_("ZeroErrERobCoEDriver") {
    logger_->debug("CTor");
}

ZeroErrERobCoEDriver::~ZeroErrERobCoEDriver() {
    logger_->debug("DTor");
}

crf::expected<double> ZeroErrERobCoEDriver::getTorque() {
    int32_t tor = rxpdo_->gearTorqueValue;
    logger_->debug("getTorque: 0x{:08X}", tor);
    return static_cast<double>(tor / milliNmtoNm_);
}

// Protected

bool ZeroErrERobCoEDriver::bindIOMap() {
    logger_->debug("bindIOMap");
    std::optional<uint8_t*> in = master_->retrieveInputs(id_);
    std::optional<uint8_t*> out = master_->retrieveOutputs(id_);
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        return false;
    }
    rxpdo_ = reinterpret_cast<RxPDO*>(in.value());
    txpdo_ = reinterpret_cast<TxPDO*>(out.value());

    // Bind references to the correspondent address
    controlWord_ = &txpdo_->controlWord;
    modeOfOperation_ = &txpdo_->modeOfOperation;
    targetPosition_ = &txpdo_->targetPosition;
    targetVelocity_ = &txpdo_->targetVelocity;
    targetTorque_ = &txpdo_->targetTorque;

    statusWord_ = &rxpdo_->statusWord;
    modeOfOperationDisplay_ = &rxpdo_->modeOfOperationDisplay;
    positionActualValue_ = &rxpdo_->positionActualValue;
    velocityActualValue_ = &rxpdo_->velocityActualValue;
    currentActualValue_ = &rxpdo_->currentActualValue;

    *controlWord_ = static_cast<ControlWord>(ControlWord::Shutdown & ControlWord::Halt);
    *modeOfOperation_ = ModeOfOperation::ProfilePositionMode;
    *targetPosition_ = *positionActualValue_;
    *targetVelocity_ = 0x00000000;
    *targetTorque_ = 0x0000;

    logger_->info("PDOs binded");
    return true;
}

}  // namespace crf::devices::canopendrivers
