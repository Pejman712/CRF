/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include "CommunicationUtility/ExpectedJSONConverter.hpp"

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "CANopenDrivers/CiA402/ICiA402Driver.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two_communication_point
 * @brief Communication point manager for the CiA402Driver associated with the "Device Profile for Drives and Motion Control."
 * One of the key aspects of this profile are the below modes of operation
 */
class CiA402Manager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    CiA402Manager() = delete;
    CiA402Manager(std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver> cia402driver,  // NOLINT
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    CiA402Manager(const CiA402Manager& other) = delete;
    CiA402Manager(CiA402Manager&& other) = delete;
    ~CiA402Manager();

    crf::expected<bool> setProfilePosition(
        const uint32_t& priority,
        const double& position,
        const double& velocity,
        const double& acceleration,
        const double& deceleration,
        const PositionReference& positionReference);
    crf::expected<bool> setProfileVelocity(
        const uint32_t& priority,
        const double& velocity,
        const double& acceleration,
        const double& deceleration);
    crf::expected<bool> setProfileTorque(
        const uint32_t& priority,
        const double& torque);

    crf::expected<bool> setVelocity(
        const uint32_t& priority,
        const double& velocity,
        const double& deltaSpeedAcc,
        const double& deltaTimeAcc,
        const double& deltaSpeedDec,
        const double& deltaTimeDec);
    crf::expected<bool> setMaximumTorque(
        const uint32_t& priority,
        const double& maxTorque);

    crf::expected<bool> setInterpolatedPosition(
        const uint32_t& priority,
        const double& position,
        const double& velocity,
        const double& acceleration,
        const double& deceleration);
    crf::expected<bool> setModeOfOperation(
        const uint32_t& priority,
        const ModeOfOperation& mode);

    crf::expected<bool> setCyclicPosition(
        const uint32_t& priority,
        const double& position,
        const double& posOffset,
        const double& velOffset,
        const double& torOffset);
    crf::expected<bool> setCyclicVelocity(
        const uint32_t& priority,
        const double& position,
        const double& velOffset,
        const double& torOffset);
    crf::expected<bool> setCyclicTorque(
        const uint32_t& priority,
        const double& position,
        const double& torOffset);

    crf::expected<bool> quickStop(const uint32_t& priority);
    crf::expected<bool> stop(const uint32_t& priority);
    crf::expected<bool> resetFault(const uint32_t& priority);
    crf::expected<bool> resetQuickStop(const uint32_t& priority);

    nlohmann::json getStatus() override;

 private:
    std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver> cia402driver_;
};

}  // namespace crf::devices::canopendrivers
