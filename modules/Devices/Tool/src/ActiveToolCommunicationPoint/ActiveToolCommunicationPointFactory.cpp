/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>

#include "Tool/ActiveToolCommunicationPoint/ActiveToolCommunicationPoint.hpp"
#include "Tool/ActiveToolCommunicationPoint/ActiveToolCommunicationPointFactory.hpp"

namespace crf::devices::tool {

ActiveToolCommunicationPointFactory::ActiveToolCommunicationPointFactory(
    std::shared_ptr<ActiveToolManager> manager) :
    manager_(manager),
    logger_("ActiveToolCommunicationPointFactory") {
    logger_->debug("CTor");
}

ActiveToolCommunicationPointFactory::~ActiveToolCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    ActiveToolCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<ActiveToolCommunicationPoint>(socket, manager_);
    return commpoint;
}

}   // namespace crf::devices::tool
