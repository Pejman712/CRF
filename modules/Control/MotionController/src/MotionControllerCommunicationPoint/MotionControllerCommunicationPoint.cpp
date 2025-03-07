/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
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
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.hpp"

namespace crf::control::motioncontroller {

MotionControllerCommunicationPoint::MotionControllerCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<MotionControllerManager> manager):
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    logger_("MotionControllerCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "startJointsControl",
        std::bind(&MotionControllerCommunicationPoint::startJointsControlLoopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "startTaskControl",
        std::bind(&MotionControllerCommunicationPoint::startTaskControlLoopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "appendPath",
        std::bind(&MotionControllerCommunicationPoint::appendPathHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setVelocity",
        std::bind(&MotionControllerCommunicationPoint::setVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setTorque",
        std::bind(&MotionControllerCommunicationPoint::setTorqueHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setProfileVelocity",
        std::bind(&MotionControllerCommunicationPoint::setProfileVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setProfileAcceleration",
        std::bind(&MotionControllerCommunicationPoint::setProfileAccelerationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "softStop",
        std::bind(&MotionControllerCommunicationPoint::softStopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "hardStop",
        std::bind(&MotionControllerCommunicationPoint::hardStopHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setParameters",
        std::bind(&MotionControllerCommunicationPoint::setParametersHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "getConfiguration",
        std::bind(&MotionControllerCommunicationPoint::getConfigurationHandler,
            this, std::placeholders::_1)});
}

MotionControllerCommunicationPoint::~MotionControllerCommunicationPoint() {
    logger_->debug("DTor");
}

void MotionControllerCommunicationPoint::startJointsControlLoopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startJointsControlLoopHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet, manager_->startJointsControlLoop(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::startTaskControlLoopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startTaskControlLoopHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet, manager_->startTaskControlLoop(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::appendPathHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("appendPathHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        std::string type = packet.data.at("type").get<std::string>();
        if (type == "joints") {
            std::vector<crf::utility::types::JointPositions> path = packet.data.
                at("data").get<std::vector<crf::utility::types::JointPositions>>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->appendPath(priority, path));
        } else if (type == "task") {
            std::vector<crf::utility::types::TaskPose> path = packet.data.
                at("data").get<std::vector<crf::utility::types::TaskPose>>();
            crf::control::motioncontroller::TrajectoryExecutionMethod method = packet.data.
                at("method").get<crf::control::motioncontroller::TrajectoryExecutionMethod>();
            crf::control::motioncontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::motioncontroller::PointReferenceFrame>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->appendPath(priority, path, method, reference));
        } else {
            sendJSONError(packet, crf::Code::BadRequest);
            return;
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::setVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        std::string type = packet.data.at("type").get<std::string>();
        if (type == "joints") {
            crf::utility::types::JointVelocities velocity = packet.data.
                at("data").get<crf::utility::types::JointVelocities>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setVelocity(priority, velocity));
        } else if (type == "task") {
            crf::utility::types::TaskVelocity velocity = packet.data.
                at("data").get<crf::utility::types::TaskVelocity>();
            crf::control::motioncontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::motioncontroller::PointReferenceFrame>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setVelocity(priority, velocity, reference));
        } else {
            sendJSONError(packet, crf::Code::BadRequest);
            return;
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::setTorqueHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setTorqueHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        std::string type = packet.data.at("type").get<std::string>();
        if (type == "joints") {
            crf::utility::types::JointForceTorques torque = packet.data.
                at("data").get<crf::utility::types::JointForceTorques>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setTorque(priority, torque));
        } else if (type == "task") {
            crf::utility::types::TaskForceTorque torque = packet.data.
                at("data").get<crf::utility::types::TaskForceTorque>();
            crf::control::motioncontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::motioncontroller::PointReferenceFrame>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setTorque(priority, torque, reference));
        } else {
            sendJSONError(packet, crf::Code::BadRequest);
            return;
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::setProfileVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        std::string type = packet.data.at("type").get<std::string>();
        if (type == "joints") {
            crf::utility::types::JointVelocities velocity = packet.data.
                at("data").get<crf::utility::types::JointVelocities>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setProfileVelocity(priority, velocity));
        } else if (type == "task") {
            crf::utility::types::TaskVelocity velocity = packet.data.
                at("data").get<crf::utility::types::TaskVelocity>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setProfileVelocity(priority, velocity));
        } else {
            sendJSONError(packet, crf::Code::BadRequest);
            return;
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::setProfileAccelerationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        std::string type = packet.data.at("type").get<std::string>();
        if (type == "joints") {
            crf::utility::types::JointAccelerations acceleration = packet.data.
                at("data").get<crf::utility::types::JointAccelerations>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setProfileAcceleration(priority, acceleration));
        } else if (type == "task") {
            crf::utility::types::TaskAcceleration acceleration = packet.data.
                at("data").get<crf::utility::types::TaskAcceleration>();
            sendJSONReply<crf::expected<bool>>(packet,
                manager_->setProfileAcceleration(priority, acceleration));
        } else {
            sendJSONError(packet, crf::Code::BadRequest);
            return;
        }
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::softStopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet, manager_->softStop(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::hardStopHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet, manager_->hardStop(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::setParametersHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setProfileVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        nlohmann::json params = packet.data.at("data");
        sendJSONReply<crf::expected<bool>>(packet, manager_->setParameters(priority, params));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void MotionControllerCommunicationPoint::getConfigurationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("getConfigurationHandler");
    try {
        sendJSONReply<nlohmann::json>(packet, manager_->getConfiguration());
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

}  // namespace crf::control::motioncontroller
