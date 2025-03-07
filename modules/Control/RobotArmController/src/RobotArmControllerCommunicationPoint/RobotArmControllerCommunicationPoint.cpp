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
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPoint.hpp"

namespace crf::control::robotarmcontroller {

RobotArmControllerCommunicationPoint::RobotArmControllerCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<RobotArmControllerManager> manager):
    crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    trajectoryResult_(std::future<bool>()),
    logger_("RobotArmControllerCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "lockControl",
        std::bind(&RobotArmControllerCommunicationPoint::lockControlRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "unlockControl",
        std::bind(&RobotArmControllerCommunicationPoint::unlockControlRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setMode",
        std::bind(&RobotArmControllerCommunicationPoint::setControllerModeRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setPos",
        std::bind(&RobotArmControllerCommunicationPoint::setPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setVel",
        std::bind(&RobotArmControllerCommunicationPoint::setVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setAcc",
        std::bind(&RobotArmControllerCommunicationPoint::setAccelerationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "interrupt",
        std::bind(&RobotArmControllerCommunicationPoint::interruptTrajectoryHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setJoiMaxVel",
        std::bind(&RobotArmControllerCommunicationPoint::setJointsMaximumVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setJoiMaxAcc",
        std::bind(&RobotArmControllerCommunicationPoint::setJointsMaximumAccelerationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setTaskMaxVel",
        std::bind(&RobotArmControllerCommunicationPoint::setTaskMaximumVelocityHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setTaskMaxAcc",
        std::bind(&RobotArmControllerCommunicationPoint::setTaskMaximumAccelerationHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "gripPos",
        std::bind(&RobotArmControllerCommunicationPoint::setGripperPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "gripVel",
        std::bind(&RobotArmControllerCommunicationPoint::setGripperVelocityHandler,
            this, std::placeholders::_1)});

    stopThreads_ = false;
    trajectoryResultThread_ =
        std::thread(&RobotArmControllerCommunicationPoint::resultCheck, this);
}

RobotArmControllerCommunicationPoint::~RobotArmControllerCommunicationPoint() {
    logger_->debug("DTor");
    stopThreads_ = true;
    if (trajectoryResultThread_.joinable()) {
        cvTrajectory_.notify_one();
        trajectoryResultThread_.join();
    }
}

void RobotArmControllerCommunicationPoint::resultCheck() {
    logger_->debug("resultCheck()");
    while (!stopThreads_) {
        std::unique_lock<std::mutex> lk(mtx_);
        cvTrajectory_.wait(lk);
        bool finished = false;
        while (!finished && !stopThreads_) {
            if (!trajectoryResult_.valid()) {
                continue;
            }
            if (trajectoryResult_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {  //NOLINT
                std::scoped_lock<std::mutex> lock(trajectoryMtx_);
                sendJSONEvent<bool>("trajectoryResult", trajectoryResult_.get());
                finished = true;
            }
        }
    }
}

void RobotArmControllerCommunicationPoint::lockControlRequestHandler(
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

void RobotArmControllerCommunicationPoint::unlockControlRequestHandler(
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


void RobotArmControllerCommunicationPoint::setControllerModeRequestHandler(
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
        crf::control::robotarmcontroller::ControllerMode mode =
            packet.data.at("mode").get<crf::control::robotarmcontroller::ControllerMode>();
        if (mode == crf::control::robotarmcontroller::ControllerMode::Velocity) {
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

void RobotArmControllerCommunicationPoint::setPositionHandler(
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
    std::string type;
    try {
        type = packet.data.at("type").get<std::string>();
    } catch (const std::exception& e) {
        logger_->warn("Error parsing type of position data: {}", e.what());
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
        if (type == "joints") {
            std::vector<crf::utility::types::JointPositions> position = packet.data.
                at("data").get<std::vector<crf::utility::types::JointPositions>>();
            trajectoryResult_ = manager_->setPosition(priority, position);
        } else if (type == "task") {
            std::vector<crf::utility::types::TaskPose> position = packet.data.
                at("data").get<std::vector<crf::utility::types::TaskPose>>();
            crf::control::robotarmcontroller::TrajectoryExecutionMethod method = packet.data.
                at("method").get<crf::control::robotarmcontroller::TrajectoryExecutionMethod>();
            crf::control::robotarmcontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::robotarmcontroller::PointReferenceFrame>();
            trajectoryResult_ = manager_->setPosition(priority, position, method, reference);
        } else {
            sendJSONError(packet, "Wrong data type");
            return;
        }
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints trajectory data: {}", e.what());
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

void RobotArmControllerCommunicationPoint::setVelocityHandler(
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
    std::string type;
    try {
        type = packet.data.at("type").get<std::string>();
    } catch (const std::exception& e) {
        logger_->warn("Error parsing type of position data: {}", e.what());
        return;
    }
    if (trajectoryResult_.valid()) {
        std::scoped_lock<std::mutex> lock(trajectoryMtx_);
        if (trajectoryResult_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            logger_->error("A trajectory is already executing");
            return;
        }
        trajectoryResult_.get();
    }
    try {
        if (type == "joints") {
            crf::utility::types::JointVelocities velocity = packet.data.
                at("data").get<crf::utility::types::JointVelocities>();
            sendJSONReply<bool>(packet, manager_->setVelocity(priority, velocity));
        } else if (type == "task") {
            crf::utility::types::TaskVelocity velocity = packet.data.
                at("data").get<crf::utility::types::TaskVelocity>();
            crf::control::robotarmcontroller::TrajectoryExecutionMethod method = packet.data.
                at("method").get<crf::control::robotarmcontroller::TrajectoryExecutionMethod>();
            crf::control::robotarmcontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::robotarmcontroller::PointReferenceFrame>();
            sendJSONReply<bool>(packet,
                manager_->setVelocity(priority, velocity, method, reference), true);
        } else {
            sendJSONError(packet, "Wrong data type");
            return;
        }
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints velocities data: {}", e.what());
        sendJSONError(packet, "Wrong data type");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setAccelerationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setJointAccelerationsHandler");
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
    std::string type;
    try {
        type = packet.data.at("type").get<std::string>();
    } catch (const std::exception& e) {
        logger_->warn("Error parsing type of position data: {}", e.what());
        return;
    }
    if (trajectoryResult_.valid()) {
        std::scoped_lock<std::mutex> lock(trajectoryMtx_);
        if (trajectoryResult_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            logger_->error("A trajectory is already executing");
            return;
        }
        trajectoryResult_.get();
    }
    try {
        if (type == "joints") {
            crf::utility::types::JointAccelerations accelerations = packet.data.
                at("data").get<crf::utility::types::JointAccelerations>();
            sendJSONReply<bool>(packet,
                manager_->setAcceleration(priority, accelerations), true);
        } else if (type == "task") {
            crf::utility::types::TaskVelocity velocity = packet.data.
                at("data").get<crf::utility::types::TaskVelocity>();
            crf::control::robotarmcontroller::TrajectoryExecutionMethod method = packet.data.
                at("method").get<crf::control::robotarmcontroller::TrajectoryExecutionMethod>();
            crf::control::robotarmcontroller::PointReferenceFrame reference = packet.data.
                at("reference").get<crf::control::robotarmcontroller::PointReferenceFrame>();
            sendJSONReply<bool>(packet,
                manager_->setVelocity(priority, velocity, method, reference), true);
        } else {
            sendJSONError(packet, "Wrong data type");
            return;
        }
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints velocities data: {}", e.what());
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::interruptTrajectoryHandler(
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

void RobotArmControllerCommunicationPoint::setJointsMaximumVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setJointsMaximumVelocityHandler");
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
        utility::types::JointVelocities velocity = packet.data.
            at("data").get<utility::types::JointVelocities>();
        sendJSONReply<bool>(packet,
            manager_->setJointsMaximumVelocity(priority, velocity), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints velocity data: {}", e.what());
        sendJSONError(packet, "Error with max velocity data");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setJointsMaximumAccelerationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setJointsMaximumAccelerationHandler");
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
        crf::utility::types::JointAccelerations acceleration = packet.data.
            at("data").get<crf::utility::types::JointAccelerations>();
        sendJSONReply<bool>(packet,
            manager_->setJointsMaximumAcceleration(priority, acceleration), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing joints acceleration data: {}", e.what());
        sendJSONError(packet, "Error with max acceleration data");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setTaskMaximumVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setTaskMaximumVelocityHandler");
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
            manager_->setTaskMaximumVelocity(priority, velocity), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task velocity data: {}", e.what());
        sendJSONError(packet, "Error with max velocity data");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setTaskMaximumAccelerationHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setTaskMaximumAccelerationHandler");
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
            manager_->setTaskMaximumAcceleration(priority, acceleration), true);
    } catch (const std::exception& e) {
        logger_->warn("Error parsing task acceleration data: {}", e.what());
        sendJSONError(packet, "Error with max acceleration data");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setGripperPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setGripperPositionHandler");
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
        sendJSONReply<bool>(packet, manager_->setGripperPosition(priority, position));
    } catch (const std::exception& e) {
        logger_->warn("Error parsing Gripper Position: {}", e.what());
        sendJSONError(packet, "Error with gripper position data");
        return;
    }
    return;
}

void RobotArmControllerCommunicationPoint::setGripperVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setGripperPositionHandler");
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
        sendJSONReply<bool>(packet, manager_->setGripperVelocity(priority, velocity));
    } catch (const std::exception& e) {
        logger_->warn("Error parsing Gripper Velocity: {}", e.what());
        sendJSONError(packet, "Error with gripper velocity data");
        return;
    }
    return;
}

}  // namespace crf::control::robotarmcontroller
