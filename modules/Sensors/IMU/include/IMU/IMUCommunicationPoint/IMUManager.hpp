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
#include "IMU/IIMU.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_imu_communication_point
 * @brief
 */
class IMUManager: public crf::utility::devicemanager::DeviceManagerWithPriorityAccess {  // NOLINT
 public:
    IMUManager() = delete;
    IMUManager(std::shared_ptr<crf::sensors::imu::IIMU> imu,
        const std::chrono::milliseconds& inizializationTimeout  = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout  = std::chrono::seconds(10));
    IMUManager(const IMUManager& other) = delete;
    IMUManager(IMUManager&& other) = delete;
    ~IMUManager();

    crf::expected<bool> calibrate(const uint32_t& priority);

    nlohmann::json getStatus();

 private:
    std::shared_ptr<crf::sensors::imu::IIMU> imu_;
};

}  // namespace crf::sensors::imu
