/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>

#include "IMU/IMUCommunicationPoint/IMUCommunicationPoint.hpp"
#include "IMU/IMUCommunicationPoint/IMUCommunicationPointFactory.hpp"

namespace crf::sensors::imu {

IMUCommunicationPointFactory::IMUCommunicationPointFactory(
    std::shared_ptr<IMUManager> manager) :
    manager_(manager),
    logger_("IMUCommunicationPointFactory") {
    logger_->debug("CTor");
}

IMUCommunicationPointFactory::~IMUCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    IMUCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<IMUCommunicationPoint>(socket, manager_);
    return commpoint;
}

}   // namespace crf::sensors::imu
