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

#include "CANopenDrivers/CiA402/CoEDrivers/ELMOGoldCoEDriver/ELMOGoldCoEDriver.hpp"

namespace crf::devices::canopendrivers {

ELMOGoldCoEDriver::ELMOGoldCoEDriver(
    std::shared_ptr<CoEMaster> master, const uint64_t &id, const nlohmann::json& j):
    CiA402CoEDriver(master, id, j),
    logger_("ELMOGoldCoEDriver") {
    logger_->debug("CTor");
}

ELMOGoldCoEDriver::~ELMOGoldCoEDriver() {
    logger_->debug("DTor");
}

// Protected

bool ELMOGoldCoEDriver::bindIOMap() {
    logger_->debug("bindIOMap");
    std::optional<uint8_t*> in = master_->retrieveInputs(id_);
    std::optional<uint8_t*> out = master_->retrieveOutputs(id_);
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        return false;
    }
    RxPDO* rxpdo = reinterpret_cast<RxPDO*>(in.value());
    TxPDO* txpdo = reinterpret_cast<TxPDO*>(out.value());

    // Bind references to the correspondent address
    controlWord_ = &txpdo->controlWord;
    modeOfOperation_ = &txpdo->modeOfOperation;
    targetPosition_ = &txpdo->targetPosition;
    targetVelocity_ = &txpdo->targetVelocity;
    targetTorque_ = &txpdo->targetTorque;

    statusWord_ = &rxpdo->statusWord;
    modeOfOperationDisplay_ = &rxpdo->modeOfOperationDisplay;
    positionActualValue_ = &rxpdo->positionActualValue;
    velocityActualValue_ = &rxpdo->velocityActualValue;
    torqueActualValue_ = &rxpdo->torqueActualValue;
    currentActualValue_ = &rxpdo->currentActualValue;

    *controlWord_ = static_cast<ControlWord>(ControlWord::Shutdown & ControlWord::Halt);
    *modeOfOperation_ = ModeOfOperation::ProfilePositionMode;
    *targetPosition_ = *positionActualValue_;
    *targetVelocity_ = 0x00000000;
    *targetTorque_ = 0x0000;

    logger_->info("PDOs binded");
    return true;
}

}  // namespace crf::devices::canopendrivers
