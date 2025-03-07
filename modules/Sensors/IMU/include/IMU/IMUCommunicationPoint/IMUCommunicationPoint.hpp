/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "IMU/IMUCommunicationPoint/IMUManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_imu_communication_point
 * @brief TODO
 */
class IMUCommunicationPoint: public crf::utility::devicemanager::PriorityAccessCommunicationPoint {  // NOLINT
 public:
    IMUCommunicationPoint() = delete;
    explicit IMUCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::sensors::imu::IMUManager> manager);
    ~IMUCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::sensors::imu::IMUManager> manager_;

    utility::logger::EventLogger logger_;

    void calibrate(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::sensors::imu
