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
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two_communication_point
 * @brief Communication point factory of the CiA402Driver communication point
 */
class CiA402CommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    CiA402CommunicationPointFactory() = delete;
    explicit CiA402CommunicationPointFactory(
        std::shared_ptr<crf::devices::canopendrivers::CiA402Manager> manager);
    ~CiA402CommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::devices::canopendrivers::CiA402Manager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::devices::canopendrivers
