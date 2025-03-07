/*
 * © Copyright CERN 2024.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <cmath>
#include <chrono>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CoEDrivers/CiA402CoEDriver/CiA402CoEDriver.hpp"

#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::devices::canopendrivers {

class ELMOPlatinumCoEDriver : public CiA402CoEDriver{
 public:
    ELMOPlatinumCoEDriver(
        std::shared_ptr<CoEMaster> master, const uint64_t &id, const nlohmann::json& j);
    ~ELMOPlatinumCoEDriver() override;

 private:
#pragma pack(push, 1)
    struct TxPDO {
        uint16_t controlWord;
        int32_t targetPosition;
        int32_t targetVelocity;
        int16_t targetTorque;
        int8_t modeOfOperation;
    };
#pragma pack(pop)

#pragma pack(push, 1)
    struct RxPDO {
        uint16_t statusWord;
        int32_t positionActualValue;
        int32_t velocityActualValue;
        int16_t torqueActualValue;
        int16_t currentActualValue;
        int8_t modeOfOperationDisplay;
    };
#pragma pack(pop)

    crf::utility::logger::EventLogger logger_;

    bool bindIOMap() override;
};

}  // namespace crf::devices::canopendrivers
