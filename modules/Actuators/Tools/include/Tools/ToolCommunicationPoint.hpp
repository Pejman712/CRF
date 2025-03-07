#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Tools/ITool.hpp"
#include "ComponentAccessControl/IComponentAccessControl.hpp"

namespace crf {
namespace devices {
namespace tools {

class ToolCommunicationPoint : public utility::commoninterfaces::IInitializable {
 public:
    ToolCommunicationPoint() = delete;
    ToolCommunicationPoint(std::shared_ptr<ITool> tool,
        std::shared_ptr<IPC> inputIPC, std::shared_ptr<IPC> outputIPC,
        uint32_t priority,
        std::shared_ptr<communication::componentaccesscontrol::IComponentAccessControl> accessControl,  // NOLINT
        const std::chrono::milliseconds& publisherRate = std::chrono::milliseconds(250));
    ToolCommunicationPoint(const ToolCommunicationPoint&) = delete;
    ToolCommunicationPoint(ToolCommunicationPoint&&) = delete;
    ~ToolCommunicationPoint() override;

    bool initialize() override;
    bool deinitialize() override;

 private:
    utility::logger::EventLogger logger_;

    std::shared_ptr<IPC> inputIPC_;
    std::shared_ptr<IPC> outputIPC_;
    std::thread receiverThread_;
    std::thread senderThread_;
    std::chrono::milliseconds publisherRate_;

    uint32_t priority_;
    std::shared_ptr<communication::componentaccesscontrol::IComponentAccessControl> accessControl_;
    bool hasAccess_;

    const std::chrono::milliseconds readFailWait_;
    bool initialized_;

    std::shared_ptr<ITool> tool_;

    std::atomic<bool> stopThreads_;
    void receiver();
    void sender();

     std::map<std::string, std::function<
        bool(ToolCommunicationPoint*,
            const communication::datapackets::JSONPacket&)> > commandHandlers_;

    bool startControlRequestHandler(
        const communication::datapackets::JSONPacket&);
    bool stopControlRequestHandler(
        const communication::datapackets::JSONPacket&);
    bool setValueRequestHandler(
        const communication::datapackets::JSONPacket&);
    bool getValueRequestHandler(
        const communication::datapackets::JSONPacket&);

    template<typename T>
    bool setToolValue(const std::string& name, const communication::datapackets::JSONPacket& json);
};

}  // namespace tools
}  // namespace devices
}  // namespace crf
