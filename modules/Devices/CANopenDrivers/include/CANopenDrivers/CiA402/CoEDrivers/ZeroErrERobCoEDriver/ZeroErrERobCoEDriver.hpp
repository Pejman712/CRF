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

class ZeroErrERobCoEDriver : public CiA402CoEDriver{
 public:
    ZeroErrERobCoEDriver(
        std::shared_ptr<CoEMaster> master, const uint64_t &id, const nlohmann::json& j);
    ~ZeroErrERobCoEDriver() override;

    crf::expected<double> getTorque() override;

 private:
#pragma pack(push, 1)
    struct TxPDO {
        uint16_t controlWord;
        int8_t modeOfOperation;
        uint8_t placeholder;
        int32_t targetPosition;
        int32_t targetVelocity;
        int16_t targetTorque;
    };
#pragma pack(pop)

#pragma pack(push, 1)
    struct RxPDO {
        uint16_t statusWord;
        int8_t modeOfOperationDisplay;
        uint8_t placeholder;
        int32_t positionActualValue;
        int32_t velocityActualValue;
        int32_t gearTorqueValue;
        int16_t currentActualValue;
    };
#pragma pack(pop)

    TxPDO* txpdo_;
    RxPDO* rxpdo_;

    crf::utility::logger::EventLogger logger_;

    bool bindIOMap() override;

    const double milliNmtoNm_ = 1000;
};

}  // namespace crf::devices::canopendrivers
