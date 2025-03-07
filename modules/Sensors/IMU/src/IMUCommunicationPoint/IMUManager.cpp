/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "IMU/IMUCommunicationPoint/IMUManager.hpp"
#include "IMU/IMUSignalJSONConverter.hpp"

namespace crf::sensors::imu {

IMUManager::IMUManager(
    std::shared_ptr<crf::sensors::imu::IIMU> imu,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(imu, initializationTimeout, controlAccessTimeout),
    imu_(imu) {
    logger_ = crf::utility::logger::EventLogger("IMUManager");
    logger_->debug("CTor");
}

IMUManager::~IMUManager() {
    logger_->debug("DTor");
}

crf::expected<bool> IMUManager::calibrate(const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return imu_->calibrate();
}

nlohmann::json IMUManager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["priorityUnderControl"] = simpleAccessControl_.getHighestPriority();
    statusJSON["signals"] =  imu_->getSignal();

    return statusJSON;
}

}  // namespace crf::sensors::imu
