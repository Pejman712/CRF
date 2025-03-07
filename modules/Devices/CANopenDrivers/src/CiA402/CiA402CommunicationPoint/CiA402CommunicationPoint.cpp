/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include <nlohmann/json.hpp>

#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPoint.hpp"

namespace crf::devices::canopendrivers {

CiA402CommunicationPoint::CiA402CommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<CiA402Manager> manager):
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(socket, manager),
    socket_(socket),
    manager_(manager),
    logger_("CiA402CommunicationPoint") {
    logger_->debug("CTor");

    jsonCommandHandlers_.insert({
        "setProfilePosition",
        std::bind(&CiA402CommunicationPoint::setProfilePositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setProfileVelocity",
        std::bind(&CiA402CommunicationPoint::setProfileVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setProfileTorque",
        std::bind(&CiA402CommunicationPoint::setProfileTorqueHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setVelocity",
        std::bind(&CiA402CommunicationPoint::setVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setMaximumTorque",
        std::bind(&CiA402CommunicationPoint::setMaximumTorqueHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setInterpolatedPosition",
        std::bind(&CiA402CommunicationPoint::setInterpolatedPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setModeOfOperation",
        std::bind(&CiA402CommunicationPoint::setModeOfOperationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setCyclicPosition",
        std::bind(&CiA402CommunicationPoint::setCyclicPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setCyclicVelocity",
        std::bind(&CiA402CommunicationPoint::setCyclicVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setCyclicTorque",
        std::bind(&CiA402CommunicationPoint::setCyclicTorqueHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "quickStop",
        std::bind(&CiA402CommunicationPoint::quickStopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stop",
        std::bind(&CiA402CommunicationPoint::stopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resetFault",
        std::bind(&CiA402CommunicationPoint::resetFaultHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resetQuickStop",
        std::bind(&CiA402CommunicationPoint::resetQuickStopHandler,
            this, std::placeholders::_1)});
}

CiA402CommunicationPoint::~CiA402CommunicationPoint() {
    logger_->debug("DTor");
    PriorityAccessCommunicationPoint::deinitialize();
}

void CiA402CommunicationPoint::deinitialiseDeviceManagerOnSocketClose() {
    logger_->info("All clients disconnected, stopping motor");
    manager_->stop(1);
}

void CiA402CommunicationPoint::heartbeatLost() {
    logger_->critical("Heartbeat has been lost! Quick Stopping the motor");
    manager_->quickStop(1);
}

void CiA402CommunicationPoint::setProfilePositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfilePositionHandler");
    int priority;
    double position;
    double velocity;
    double acceleration;
    double deceleration;
    PositionReference reference;
    try {
        priority = packet.data.at("priority").get<int>();
        position = packet.data.at("position").get<double>();
        velocity = packet.data.at("velocity").get<double>();
        acceleration = packet.data.at("acceleration").get<double>();
        deceleration = packet.data.at("deceleration").get<double>();
        reference = packet.data.at("positionReference").get<PositionReference>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setProfilePosition(
            priority, position, velocity, acceleration, deceleration, reference));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setProfileVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    int priority;
    double velocity;
    double acceleration;
    double deceleration;
    try {
        priority = packet.data.at("priority").get<int>();
        velocity = packet.data.at("velocity").get<double>();
        acceleration = packet.data.at("acceleration").get<double>();
        deceleration = packet.data.at("deceleration").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setProfileVelocity(
            priority, velocity, acceleration, deceleration));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setProfileTorqueHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileTorqueHandler");
    int priority;
    double torque;
    try {
        priority = packet.data.at("priority").get<int>();
        torque = packet.data.at("torque").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setProfileTorque(
            priority, torque));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setVelocityHandler");
    int priority;
    double velocity;
    double deltaSpeedAcc;
    double deltaTimeAcc;
    double deltaSpeedDec;
    double deltaTimeDec;
    try {
        priority = packet.data.at("priority").get<int>();
        velocity = packet.data.at("velocity").get<double>();
        deltaSpeedAcc = packet.data.at("velocity").get<double>();
        deltaTimeAcc = packet.data.at("velocity").get<double>();
        deltaSpeedDec = packet.data.at("velocity").get<double>();
        deltaTimeDec = packet.data.at("velocity").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setVelocity(
            priority, velocity, deltaSpeedAcc, deltaTimeAcc, deltaSpeedDec, deltaTimeDec));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setMaximumTorqueHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setMaximumTorqueHandler: {}", packet.data);
    int priority;
    double torque;
    try {
        priority = packet.data.at("priority").get<int>();
        torque = packet.data.at("maxTorque").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setMaximumTorque(
            priority, torque));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setInterpolatedPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setInterpolatedPositionHandler");
    int priority;
    double position;
    double velocity;
    double acceleration;
    double deceleration;
    try {
        priority = packet.data.at("priority").get<int>();
        position = packet.data.at("position").get<double>();
        velocity = packet.data.at("velocity").get<double>();
        acceleration = packet.data.at("acceleration").get<double>();
        deceleration = packet.data.at("deceleration").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setInterpolatedPosition(
            priority, position, velocity, acceleration, deceleration));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setModeOfOperationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setModeOfOperationHandler");
    int priority;
    ModeOfOperation mode;
    mode = packet.data.at("mode").get<ModeOfOperation>();
    try {
        priority = packet.data.at("priority").get<int>();
        mode = packet.data.at("mode").get<ModeOfOperation>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setModeOfOperation(
            priority, mode));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setCyclicPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setCyclicPositionHandler");
    int priority;
    double position;
    double posOffset;
    double velOffset;
    double torOffset;
    try {
        priority = packet.data.at("priority").get<int>();
        position = packet.data.at("position").get<double>();
        posOffset = packet.data.at("posOffset").get<double>();
        velOffset = packet.data.at("velOffset").get<double>();
        torOffset = packet.data.at("torOffset").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setCyclicPosition(
            priority, position, posOffset, velOffset, torOffset));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setCyclicVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setCyclicVelocityHandler");
    int priority;
    double velocity;
    double velOffset;
    double torOffset;
    try {
        priority = packet.data.at("priority").get<int>();
        velocity = packet.data.at("velocity").get<double>();
        velOffset = packet.data.at("velOffset").get<double>();
        torOffset = packet.data.at("torOffset").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setCyclicVelocity(
            priority, velocity, velOffset, torOffset));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::setCyclicTorqueHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setCyclicTorqueHandler");
    int priority;
    double torque;
    double torOffset;
    try {
        priority = packet.data.at("priority").get<int>();
        torque = packet.data.at("torque").get<double>();
        torOffset = packet.data.at("torOffset").get<double>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->setCyclicTorque(
            priority, torque, torOffset));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::quickStopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("quickStopHandler");
    int priority;
    try {
        priority = packet.data.at("priority").get<int>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->quickStop(
            priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::stopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopHandler");
    int priority;
    try {
        priority = packet.data.at("priority").get<int>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->stop(
            priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::resetFaultHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resetFaultHandler");
    int priority;
    try {
        priority = packet.data.at("priority").get<int>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->resetFault(
            priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

void CiA402CommunicationPoint::resetQuickStopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resetQuickStopHandler");
    int priority;
    try {
        priority = packet.data.at("priority").get<int>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet, manager_->resetQuickStop(
            priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::InternalServerError);
        return;
    }
    return;
}

}  // namespace crf::devices::canopendrivers
