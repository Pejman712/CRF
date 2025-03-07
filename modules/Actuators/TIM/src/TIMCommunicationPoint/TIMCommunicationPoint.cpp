/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>

#include "TIM/TIMCommunicationPoint/TIMCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::tim {

TIMCommunicationPoint::TIMCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::actuators::tim::TIMManager> timManager) :
    PriorityAccessCommunicationPoint(socket, timManager),
    timManager_(timManager),
    logger_("TIMCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "setCurrentPosition",
        std::bind(&TIMCommunicationPoint::setCurrentPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setTargetPosition",
        std::bind(&TIMCommunicationPoint::setTargetPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setTargetVelocity",
        std::bind(&TIMCommunicationPoint::setTargetVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "moveToTarget",
        std::bind(&TIMCommunicationPoint::moveToTargetHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "jog",
        std::bind(&TIMCommunicationPoint::jogHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stop",
        std::bind(&TIMCommunicationPoint::stopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "emergencyStop",
        std::bind(&TIMCommunicationPoint::emergencyStopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "extendChargingArm",
        std::bind(&TIMCommunicationPoint::extendChargingArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "retractChargingArm",
        std::bind(&TIMCommunicationPoint::retractChargingArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "startCharging",
        std::bind(&TIMCommunicationPoint::startChargingHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stopCharging",
        std::bind(&TIMCommunicationPoint::stopChargingHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "enableEconomyMode",
        std::bind(&TIMCommunicationPoint::enableEconomyModeHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "disableEconomyMode",
        std::bind(&TIMCommunicationPoint::disableEconomyModeHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "rebootRobotArmWagon",
        std::bind(&TIMCommunicationPoint::rebootRobotArmWagonHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setObstacleArea",
        std::bind(&TIMCommunicationPoint::setObstacleAreaHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "devicesRetracted",
        std::bind(&TIMCommunicationPoint::devicesRetractedHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "allowMovement",
        std::bind(&TIMCommunicationPoint::allowMovementHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "acknowledgeAlarms",
        std::bind(&TIMCommunicationPoint::acknowledgeAlarmsHandler,
            this, std::placeholders::_1)});
}

TIMCommunicationPoint::~TIMCommunicationPoint() {
}

void TIMCommunicationPoint::setCurrentPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setCurrentPositionHandler");
    uint32_t priority = 0;
    float position = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        position = packet.data["position"].get<float>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->setCurrentPosition(position, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::setTargetPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setTargetPositionHandler");
    uint32_t priority = 0;
    float position = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        position = packet.data["position"].get<float>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->setTargetPosition(position, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::setTargetVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setTargetVelocityHandler");
    uint32_t priority = 0;
    float velocity = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        velocity = packet.data["velocity"].get<float>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->setTargetVelocity(velocity, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::moveToTargetHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("moveToTargetHandler");
    uint32_t priority = 0;
    float position = 0;
    float velocity = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        position = packet.data["position"].get<float>();
        velocity = packet.data["velocity"].get<float>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->moveToTarget(position, velocity, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::jogHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("jogHandler");
    uint32_t priority = 0;
    float velocity = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        velocity = packet.data["velocity"].get<float>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->jog(velocity, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::stopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->stop(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::emergencyStopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("emergencyStopHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->emergencyStop(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::extendChargingArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("extendChargingArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->extendChargingArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::retractChargingArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("retractChargingArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->retractChargingArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::startChargingHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startChargingHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->startCharging(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::stopChargingHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopChargingHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->stopCharging(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::enableEconomyModeHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("enableEconomyModeHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->enableEconomyMode(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::disableEconomyModeHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("disableEconomyModeHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->disableEconomyMode(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::rebootRobotArmWagonHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("rebootRobotArmWagonHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->rebootRobotArmWagon(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::setObstacleAreaHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setObstacleAreaHandler");
    uint32_t priority = 0;
    LHCObstacle obstacle;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        obstacle.identifier(packet.data["obstacle"]["identifier"].get<int>());
        obstacle.type(packet.data["obstacle"]["type"].get<LHCObstacleType>());
        obstacle.startPosition(packet.data["obstacle"]["startPosition"].get<float>());
        obstacle.endPosition(packet.data["obstacle"]["endPosition"].get<float>());
        obstacle.maximumVelocity(packet.data["obstacle"]["maximumVelocity"].get<float>());
        obstacle.mustRetractDevices(packet.data["obstacle"]["mustRetractDevices"].get<bool>());
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->setObstacleArea(obstacle, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::devicesRetractedHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("devicesRetractedHandler");
    uint32_t priority = 0;
    bool devicesStatus = false;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        devicesStatus = packet.data["devicesStatus"].get<bool>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->devicesRetracted(devicesStatus, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::allowMovementHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("allowMovementHandler");
    uint32_t priority = 0;
    bool allow = false;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        allow = packet.data["allow"].get<bool>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->allowMovement(allow, priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMCommunicationPoint::acknowledgeAlarmsHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("acknowledgeAlarmsHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data["priority"].get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timManager_->acknowledgeAlarms(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

}  // namespace crf::actuators::tim
