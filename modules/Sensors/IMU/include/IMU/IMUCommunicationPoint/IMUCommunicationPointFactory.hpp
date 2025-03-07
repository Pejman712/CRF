/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "IMU/IMUCommunicationPoint/IMUManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup imu_communication_point
 * @brief
 */
class IMUCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    IMUCommunicationPointFactory() = delete;
    explicit IMUCommunicationPointFactory(
        std::shared_ptr<crf::sensors::imu::IMUManager> manager);
    ~IMUCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::sensors::imu::IMUManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::sensors::imu
