/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <array>
#include <memory>
#include <thread>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"
#include "IMU/IIMU.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_vmu931
 * @brief 
 * 
 */
class VMU931: public IIMU {
 public:
    explicit VMU931(
      std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial);
    ~VMU931();

    bool initialize() override;
    bool deinitialize() override;
    IMUSignals getSignal() override;
    crf::expected<bool> calibrate() override;

 private:
    std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial_;
    utility::logger::EventLogger logger_;
    bool initialized_;
    IMUSignals imuData_;
    bool stopReceiverThread_;
    std::thread receiverThread_;

    void receiver();
    bool parseMessage(const std::string& message, int bufferSize);
    bool startDataBroadcast();
    bool writeMsgToDevice(const std::string& msg);
};

}  // namespace crf::sensors::imu
