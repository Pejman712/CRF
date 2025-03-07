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

#include "Tools/ToolCommunicationPoint.hpp"

namespace crf {
namespace devices {
namespace tools {

ToolCommunicationPoint::ToolCommunicationPoint(std::shared_ptr<ITool> tool,
    std::shared_ptr<IPC> inputIPC, std::shared_ptr<IPC> outputIPC,
    uint32_t priority,
    std::shared_ptr<communication::componentaccesscontrol::IComponentAccessControl> accessControl,
    const std::chrono::milliseconds& publisherRate) :
        logger_("ToolCommunicationPoint"),
        inputIPC_(inputIPC),
        outputIPC_(outputIPC),
        receiverThread_(),
        senderThread_(),
        publisherRate_(publisherRate),
        priority_(priority),
        accessControl_(accessControl),
        hasAccess_(false),
        readFailWait_(std::chrono::milliseconds(100)),
        initialized_(false),
        tool_(tool),
        stopThreads_(false),
        commandHandlers_() {
            logger_->debug("CTor");

            commandHandlers_.insert({"start", &ToolCommunicationPoint::startControlRequestHandler});
            commandHandlers_.insert({"stop", &ToolCommunicationPoint::stopControlRequestHandler});
            commandHandlers_.insert({"set", &ToolCommunicationPoint::setValueRequestHandler});
            commandHandlers_.insert({"get", &ToolCommunicationPoint::getValueRequestHandler});
}

ToolCommunicationPoint::~ToolCommunicationPoint() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool ToolCommunicationPoint::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    stopThreads_ = false;

    senderThread_ = std::thread(&ToolCommunicationPoint::sender, this);
    receiverThread_ = std::thread(&ToolCommunicationPoint::receiver, this);

    initialized_ = true;
    return true;
}

bool ToolCommunicationPoint::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    stopThreads_ = true;

    senderThread_.join();
    receiverThread_.join();

    initialized_ = false;
    return true;
}

void ToolCommunicationPoint::receiver() {
    logger_->debug("receiver");

    std::string buffer;
    Packets::PacketHeader header;
    communication::datapackets::JSONPacket json;

    while (!stopThreads_) {
        if (!inputIPC_->read(buffer, header)) {
            logger_->warn("Error reading from inputIPC");
            std::this_thread::sleep_for(readFailWait_);
            continue;
        }

        if (header.type() != communication::datapackets::JSON_PACKET_TYPE) {
            logger_->warn("Not supported packet type");
            continue;
        }

        if (!json.deserialize(buffer)) {
            logger_->warn("Not valid json format");
            continue;
        }

        std::string command;
        try {
            command = json.data_.at("cmd").get<std::string>();
        } catch (const std::exception& e) {
            logger_->warn("Cannot get \"cmd\" field from received json: {}", e.what());
            continue;
        }
        if (commandHandlers_.find(command) == commandHandlers_.end()) { continue; }
        commandHandlers_[command](this, json);
    }
}

void ToolCommunicationPoint::sender() {
    logger_->debug("sender");
    auto availableFields = tool_->getValueNames();

    while (!stopThreads_) {
        communication::datapackets::JSONPacket outputPacket;
        for (size_t i=0; i < availableFields.size(); i++) {
            auto value = tool_->getValue(availableFields[i]);
            auto type = tool_->getValueType(availableFields[i]);
            if ((!value) || (!type)) continue;

            switch (type.get()) {
                case ToolValueTypes::BOOL:
                    outputPacket.data_[availableFields[i]] = boost::any_cast<bool>(value.get());
                    break;
                case ToolValueTypes::INT:
                    outputPacket.data_[availableFields[i]] = boost::any_cast<int>(value.get());
                    break;
                case ToolValueTypes::FLOAT:
                    outputPacket.data_[availableFields[i]] = boost::any_cast<float>(value.get());
                    break;
            }
        }

        if (!outputIPC_->write(outputPacket.serialize(), outputPacket.getHeader())) {
            logger_->warn("Failed to write status packet");
        }
        std::this_thread::sleep_for(publisherRate_);
    }
}

bool ToolCommunicationPoint::startControlRequestHandler(
    const communication::datapackets::JSONPacket&) {
    hasAccess_ = accessControl_->requestAccess(priority_);
    return accessControl_->requestAccess(priority_);
}

bool ToolCommunicationPoint::stopControlRequestHandler(
    const communication::datapackets::JSONPacket&) {
    hasAccess_ = false;
    return accessControl_->releaseAccess(priority_);
}

bool ToolCommunicationPoint::setValueRequestHandler(
    const communication::datapackets::JSONPacket& json) {
    if ((!hasAccess_) || (!accessControl_->requestAccess(priority_))) {
        hasAccess_ = false;
        logger_->warn("A higher priority communication point holds the access");
        return false;
    }

    std::string fieldName;
    try {
        fieldName = json.data_.at("name").get<std::string>();
    } catch (const std::exception&) {
        logger_->warn("Failed to parse name field");
        return false;
    }

    auto type = tool_->getValueType(fieldName);
    if (!type) {
        logger_->warn("Requested field name does not exists: {}", fieldName);
        return false;
    }

    switch (type.get()) {
        case ToolValueTypes::BOOL:
            return setToolValue<bool>(fieldName, json);
        case ToolValueTypes::INT:
            return setToolValue<int>(fieldName, json);
        case ToolValueTypes::FLOAT:
            return setToolValue<float>(fieldName, json);
    }

    return false;
}

bool ToolCommunicationPoint::getValueRequestHandler(
    const communication::datapackets::JSONPacket& json) {
    std::string fieldName;
    try {
        fieldName = json.data_.at("name").get<std::string>();
    } catch (const std::exception&) {
        logger_->warn("Failed to parse name field");
        return false;
    }

    auto type = tool_->getValueType(fieldName);
    if (!type) {
        logger_->warn("Requested field name does not exists");
        return false;
    }

    auto value = tool_->getValue(fieldName);
    if (!value) {
        logger_->warn("Failed to get request value");
        return false;
    }

    communication::datapackets::JSONPacket outputPacket;
    switch (type.get()) {
        case ToolValueTypes::BOOL:
            outputPacket.data_[fieldName] = boost::any_cast<bool>(value.get());
            break;
        case ToolValueTypes::INT:
            outputPacket.data_[fieldName] = boost::any_cast<int>(value.get());
            break;
        case ToolValueTypes::FLOAT:
            outputPacket.data_[fieldName] = boost::any_cast<float>(value.get());
            break;
    }

    if (!outputIPC_->write(outputPacket.serialize(), outputPacket.getHeader())) {
        logger_->warn("Failed to write get packet");
        return false;
    }
    return true;
}

template<typename T>
bool ToolCommunicationPoint::setToolValue(const std::string& name,
    const communication::datapackets::JSONPacket& json) {
    T value;
    try {
        value = json.data_.at("value").get<T>();
    } catch (const std::exception&) {
        logger_->warn("Failed to parse value field");
        return false;
    }
    return tool_->setValue(name, value);
}


template bool ToolCommunicationPoint::setToolValue<bool>(const std::string& name,
    const communication::datapackets::JSONPacket& json);
template bool ToolCommunicationPoint::setToolValue<int>(const std::string& name,
    const communication::datapackets::JSONPacket& json);
template bool ToolCommunicationPoint::setToolValue<float>(const std::string& name,
    const communication::datapackets::JSONPacket& json);

}  // namespace tools
}  // namespace devices
}  // namespace crf
