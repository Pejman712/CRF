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

#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPoint.hpp"

namespace crf::control::robotbasecontroller {

RobotBaseControllerCommunicationPoint::RobotBaseControllerCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<RobotBaseControllerManager> manager):
    crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    logger_("RobotBaseControllerCommunicationPoint") {
    logger_->debug("CTor");

    jsonCommandHandlers_.insert({
        "lockControl",
        std::bind(&RobotBaseControllerCommunicationPoint::lockControlRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "unlockControl",
        std::bind(&RobotBaseControllerCommunicationPoint::unlockControlRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setMode",
        std::bind(&RobotBaseControllerCommunicationPoint::setControllerModeRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setPos",
        std::bind(&RobotBaseControllerCommunicationPoint::setPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setVel",
        std::bind(&RobotBaseControllerCommunicationPoint::setVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "interrupt",
        std::bind(&RobotBaseControllerCommunicationPoint::interruptTrajectoryHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setMaxVel",
        std::bind(&RobotBaseControllerCommunicationPoint::setMaximumVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setMaxAcc",
        std::bind(&RobotBaseControllerCommunicationPoint::setMaximumAccelerationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stageVel",
        std::bind(&RobotBaseControllerCommunicationPoint::setStageVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stagePos",
        std::bind(&RobotBaseControllerCommunicationPoint::setStagePositionHandler,
            this, std::placeholders::_1)});

    stopThreads_ = false;
    trajectoryResultThread_ =
        std::thread(&RobotBaseControllerCommunicationPoint::resultCheck, this);
}

RobotBaseControllerCommunicationPoint::~RobotBaseControllerCommunicationPoint() {
    logger_->debug("DTor");
    stopThreads_ = true;
    if (trajectoryResultThread_.joinable()) {
        cvTrajectory_.notify_one();
        trajectoryResultThread_.join();
    }
}

void RobotBaseControllerCommunicationPoint::resultCheck() {
    logger_->debug("resultCheck()");
    while (!stopThreads_) {
        std::unique_lock<std::mutex> lk(mtx_);
        cvTrajectory_.wait(lk);
        bool finished = false;
        while (!finished && !stopThreads_) {
            if (trajectoryResult_.valid()) {
                if (trajectoryResult_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {  //NOLINT
                    std::scoped_lock<std::mutex> lock(trajectoryMtx_);
                    sendJSONEvent<bool>("trajectoryResult", trajectoryResult_.get());
                    finished = true;
                }
            }
        }
    }
}

void RobotBaseControllerCommunicationPoint::lockControlRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("lockControlRequestHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    sendJSONReply<bool>(packet, manager_->lockControl(priority));
    return;
}

void RobotBaseControllerCommunicationPoint::unlockControlRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("unlockControlRequestHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    sendJSONReply<bool>(packet, manager_->unlockControl(priority));
    return;
}

void RobotBaseControllerCommunicationPoint::setControllerModeRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setControllerModeRequestHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    try {
        ControllerMode mode = packet.data.at("mode").get<ControllerMode>();
        if (mode == ControllerMode::Velocity) {
            sendJSONReply<bool>(packet,
                manager_->setControllerMode(priority, mode), true);
        } else {
            sendJSONError(packet, "Wrong controller mode selected");
        }
    } catch (const std::exception& e) {
        logger_->warn("Error parsing controller mode data: {}", e.what());
        sendJSONError(packet, "Error with max velocity data");
        return;
    }
    return;
}

void RobotBaseControllerCommunicationPoint::setPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setPositionHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::scoped_lock<std::mutex> lock(trajectoryMtx_);
    if (trajectoryResult_.valid()) {
        if (trajectoryResult_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            logger_->error("A trajectory is already executing");
            sendJSONError(packet, "Previous trajectory running");
            return;
        }
        trajectoryResult_.get();
    }
    try {
        std::vector<crf::utility::types::TaskPose> positions =
            packet.data.at("data").get<std::vector<crf::utility::types::TaskPose>>();
        for (uint8_t i = 0; i < positions.size(); i++) {
            if (positions.at(i).getPosition()(2) != 0 || positions.at(i).getCardanXYZ()[0] != 0 ||
                positions.at(i).getCardanXYZ()[1] != 0) {
                sendJSONError(packet, "Wrong parameters");
                return;
            }
        }
        trajectoryResult_ = manager_->setPosition(priority, positions);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints trajectory data: {}", e.what());
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    if (!trajectoryResult_.valid()) {
        logger_->error("There was an error executing the trajectory");
        sendJSONReply<bool>(packet, false);
        return;
    }
    sendJSONReply<bool>(packet, true);
    cvTrajectory_.notify_one();
    return;
}

void RobotBaseControllerCommunicationPoint::setVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setJointVelocitiesHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    if (trajectoryResult_.valid()) {
        std::scoped_lock<std::mutex> lock(trajectoryMtx_);
        if (trajectoryResult_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            sendJSONError(packet,
                "Another trajectory is already in execution");
            return;
        }
        trajectoryResult_.get();
    }
    try {
        crf::utility::types::TaskVelocity velocity =
            packet.data.at("data").get<crf::utility::types::TaskVelocity>();
        sendJSONReply<bool>(packet,
            manager_->setVelocity(priority, velocity), true);
    } catch (const std::exception& e) {
        sendJSONError(packet, "Wrong velocity data");
        return;
    }
    return;
}

void RobotBaseControllerCommunicationPoint::interruptTrajectoryHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("interruptTrajectoryHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    if (!trajectoryResult_.valid()) {
        logger_->warn("Trajectory is not in execution");
        sendJSONError(packet, "No trajectory executing");
        return;
    }
    sendJSONReply<bool>(packet, manager_->interruptTrajectory(priority));
    return;
}

void RobotBaseControllerCommunicationPoint::setMaximumVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setMaximumVelocityHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    try {
        crf::utility::types::TaskVelocity velocity = packet.data.
            at("data").get<crf::utility::types::TaskVelocity>();
        sendJSONReply<bool>(packet,
            manager_->setMaximumVelocity(priority, velocity), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task velocity data: {}", e.what());
        sendJSONError(packet, "Error with max velocity data");
        return;
    }
    return;
}

void RobotBaseControllerCommunicationPoint::setMaximumAccelerationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setMaximumAccelerationHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    try {
        crf::utility::types::TaskAcceleration acceleration = packet.data.
            at("data").get<crf::utility::types::TaskAcceleration>();
        sendJSONReply<bool>(packet,
            manager_->setMaximumAcceleration(priority, acceleration), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task acceleration data: {}", e.what());
        sendJSONError(packet, "Error with max acceleration data");
        return;
    }
    return;
}

void RobotBaseControllerCommunicationPoint::setStageVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setStageVelocityHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    try {
        float velocity = packet.data.at("data").get<float>();
        sendJSONReply<bool>(packet, manager_->setStageVelocity(priority, velocity));
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task acceleration data: {}", e.what());
        sendJSONError(packet, "Error with max acceleration data");
        return;
    }
    return;
}

void RobotBaseControllerCommunicationPoint::setStagePositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setStageVelocityHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    try {
        float position = packet.data.at("data").get<float>();
        sendJSONReply<bool>(packet, manager_->setStagePosition(priority, position));
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task acceleration data: {}", e.what());
        sendJSONError(packet, "Error with max acceleration data");
        return;
    }
    return;
}

}  // namespace crf::control::robotbasecontroller
