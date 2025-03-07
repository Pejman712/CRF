/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "RobotBaseController/RobotBaseControllerClient/RobotBaseControllerClient.hpp"

namespace crf::control::robotbasecontroller {

RobotBaseControllerClient::RobotBaseControllerClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout,
    const float& frequency,
    const uint32_t& priority):
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    frequency_(frequency),
    priority_(priority),
    initialized_(false),
    stopThread_(false),
    grabberThread_(),
    error_(),
    mtx_(),
    jsonCV_(),
    actionRequested_(0),
    statusPosition_(),
    statusVelocity_(),
    logger_("RobotBaseControllerClient") {
        logger_->debug("CTor");
}

RobotBaseControllerClient::~RobotBaseControllerClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool RobotBaseControllerClient::initialize() {
    logger_->debug("initialize()");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!socket_->open()) {
        logger_->warn("Failed to connect");
        return false;
    }
    stopThread_ = false;
    grabberThread_ = std::thread(&RobotBaseControllerClient::grabber, this);
    initialized_ = true;
    if (frequency_ > 0) {
        startStream(frequency_);
    }
    return true;
}

bool RobotBaseControllerClient::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    socket_->close();
    stopThread_ = true;
    if (grabberThread_.joinable()) {
        grabberThread_.join();
    }
    if (frequency_ > 0) {
        stopStream();
    }
    initialized_ = false;
    return true;
}

std::future<bool> RobotBaseControllerClient::setPosition(
    const crf::utility::types::TaskPose& position) {
    logger_->debug("setPosition");
    std::vector<crf::utility::types::TaskPose> path;
    path.push_back(position);
    return setPosition(path);
}

std::future<bool> RobotBaseControllerClient::setPosition(
    const std::vector<crf::utility::types::TaskPose>& positions) {
    logger_->debug("setPosition(vec)");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::future<bool>();;
    }
    if (!lockControl()) {
        return std::future<bool>();;
    }
    return std::async(std::launch::async, [this, positions]() {
        return sendPosition(positions);
    });
}

bool RobotBaseControllerClient::setVelocity(
    const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setVelocity)");
    std::vector<crf::utility::types::TaskVelocity> path;
    path.push_back(velocity);
    return setVelocity(path);
}

bool RobotBaseControllerClient::setVelocity(
    const std::vector<crf::utility::types::TaskVelocity>& velocities) {
    logger_->debug("setVelocity(vec)");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "setVel";
    json.data["priority"] = priority_;
    json.data["data"] = velocities;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    actionRequested_++;
    std::unique_lock<std::mutex> activateLock(activateMutex_);
    if (activateCV_.wait_for(activateLock, serverReplyTimeout_) == std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    if (!activateResult_) {
        logger_->error("Not able to set velocity");
        return false;
    }
    activateLock.unlock();
    unlockControl();
    return true;
}

bool RobotBaseControllerClient::interruptTrajectory() {
    logger_->debug("interruptTrajectory");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "interrupt";
    json.data["priority"] = priority_;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    actionRequested_++;
    std::unique_lock<std::mutex> activateLock(activateMutex_);
    if (activateCV_.wait_for(activateLock, serverReplyTimeout_) == std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    if (!activateResult_) {
        logger_->error("Not able to stop the trajectory");
        return false;
    }
    activateLock.unlock();
    unlockControl();
    return true;
}

crf::utility::types::TaskPose RobotBaseControllerClient::getPosition() {
    logger_->debug("getPosition");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return crf::utility::types::TaskPose();
    }
    if (frequency_ <= 0) {
        communication::datapackets::JSONPacket json;
        json.data["cmd"] = "getStat";
        std::unique_lock<std::mutex> socketLock(socketMutex_);
        socket_->write(json, json.getHeader());
        socketLock.unlock();

        std::unique_lock<std::mutex> statusLock(statusMutex_);
        if (statusCV_.wait_for(statusLock, serverReplyTimeout_) == std::cv_status::timeout) {
            logger_->error("Timeout for server reply");
            return crf::utility::types::TaskPose();
        }
        statusLock.unlock();
    }
    return statusPosition_;
}

crf::utility::types::TaskVelocity RobotBaseControllerClient::getVelocity() {
    logger_->debug("getTaskVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return crf::utility::types::TaskVelocity();
    }
    if (frequency_ <= 0) {
        communication::datapackets::JSONPacket json;
        json.data["cmd"] = "getStat";
        std::unique_lock<std::mutex> socketLock(socketMutex_);
        socket_->write(json, json.getHeader());
        socketLock.unlock();

        std::unique_lock<std::mutex> statusLock(statusMutex_);
        if (statusCV_.wait_for(statusLock, serverReplyTimeout_) == std::cv_status::timeout) {
            logger_->error("Timeout for server reply");
            return crf::utility::types::TaskVelocity();
        }
        statusLock.unlock();
    }
    return statusVelocity_;
}

bool RobotBaseControllerClient::setMaximumVelocity(
    const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setMaximumVelocity");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "setMaxVel";
    json.data["priority"] = priority_;
    json.data["data"] = velocity;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    actionRequested_++;
    std::unique_lock<std::mutex> activateLock(activateMutex_);
    if (activateCV_.wait_for(activateLock, serverReplyTimeout_) == std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    if (!activateResult_) {
        logger_->error("Not able to activate the stabilizer");
        return false;
    }
    activateLock.unlock();
    unlockControl();
    return true;
}

bool RobotBaseControllerClient::setMaximumAcceleration(
    const crf::utility::types::TaskAcceleration& acceleration) {
    logger_->debug("setMaximumAcceleration");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "setMaxAcc";
    json.data["priority"] = priority_;
    json.data["data"] = acceleration;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    actionRequested_++;
    std::unique_lock<std::mutex> activateLock(activateMutex_);
    if (activateCV_.wait_for(activateLock, serverReplyTimeout_) == std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    if (!activateResult_) {
        logger_->error("Not able to activate the stabilizer");
        return false;
    }
    activateLock.unlock();
    unlockControl();
    return true;
}

void RobotBaseControllerClient::grabber() {
    while (!stopThread_) {
        if (!socket_->isOpen()) {
            continue;
        }

        auto retval = socket_->read();
        if (!retval) {
            continue;
        }

        std::string status;
        auto buffer = retval.value().first;
        auto header = retval.value().second;
        if (header.type() != communication::datapackets::JSON_PACKET_TYPE) {
            continue;
        }

        communication::datapackets::JSONPacket json;
        if (!json.deserialize(buffer)) {
            logger_->warn("Failed to deserialize json packet: {}", buffer);
            continue;
        }

        mtx_.lock();
        try {
            if (json.data["command"].get<std::string>() != "reply") {
                logger_->warn("Unknown command: {}", json.data["command"].get<std::string>());
            }
            std::string replyCommand = json.data["replyCommand"].get<std::string>();
            if (replyCommand == "lockControl") {
                std::unique_lock<std::mutex> lockControlLock(lockControlMutex_);
                lockControlResult_ = json.data["message"].get<bool>();
                lockControlCV_.notify_one();
            } else if (replyCommand == "unlockControl") {
                std::unique_lock<std::mutex> unlockControlLock(unlockControlMutex_);
                unlockControlResult_ = json.data["message"].get<bool>();
                unlockControlCV_.notify_one();
            } else if (replyCommand == "activate") {
                /*
                 * The notify might launch before the wait, to solve this we check if the wait
                 * is ready and then launch (jplayang)
                 */
                auto start = std::chrono::high_resolution_clock::now();
                while (actionRequested_ == 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                    auto elapsed = std::chrono::high_resolution_clock::now();
                    if (start - elapsed > serverReplyTimeout_) {
                        logger_->error("Received activate without request");
                        break;
                    }
                }
                if (actionRequested_ != 0) {
                    std::unique_lock<std::mutex> activateLock(activateMutex_);
                    activateResult_ = json.data["message"].get<bool>();
                    activateCV_.notify_one();
                    actionRequested_--;
                }
            } else if (replyCommand == "trajectoryResult") {
                std::unique_lock<std::mutex> trajectoryLock(trajectoryMutex_);
                trajectoryResult_ = json.data["message"].get<bool>();
                trajectoryCV_.notify_all();
            } else if (replyCommand == "streamer" || replyCommand == "getStatus") {
                statusCV_.notify_one();
                nlohmann::json tempJSON = json.data["message"];
                statusPosition_ = tempJSON["taskPose"].get<crf::utility::types::TaskPose>(); // NOLINT
                statusVelocity_ = tempJSON["taskVelocity"].get<crf::utility::types::TaskVelocity>(); // NOLINT
            } else if (replyCommand == "error") {
                error_ = json.data["message"];
                logger_->error("Error on the communication point: {}", error_);
            }
        } catch (const std::exception& ex) {
            logger_->warn("Failed reading json response: {}", ex.what());
        }
        mtx_.unlock();
    }
}

bool RobotBaseControllerClient::lockControl() {
    logger_->debug("lockControl");
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "lockControl";
    json.data["priority"] = priority_;
    json.data["mode"] = crf::control::robotbasecontroller::ControllerMode::Velocity;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();
    std::unique_lock<std::mutex> lockControlLock(lockControlMutex_);
    if (lockControlCV_.wait_for(lockControlLock, serverReplyTimeout_) == std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    logger_->debug("lockControl, activated");
    if (!lockControlResult_) {
        logger_->error("Not able to get control of the controller with the given priority");
        return false;
    }
    lockControlLock.unlock();
    return true;
}

bool RobotBaseControllerClient::unlockControl() {
    logger_->debug("unlockControl");
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "unlockControl";
    json.data["priority"] = priority_;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::unique_lock<std::mutex> unlockControlLock(unlockControlMutex_);
    if (unlockControlCV_.wait_for(unlockControlLock, serverReplyTimeout_) ==
        std::cv_status::timeout) {
            logger_->error("Timeout for server reply");
            return false;
    }
    logger_->debug("unlockControl, activated");
    if (!unlockControlResult_) {
        logger_->error("Not able to release the control of the controller");
        return false;
    }
    unlockControlLock.unlock();
    return true;
}

bool RobotBaseControllerClient::sendPosition(
    const std::vector<crf::utility::types::TaskPose>& positions) {
    logger_->debug("sendPosition");
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "setPos";
    json.data["priority"] = priority_;
    json.data["data"] = positions;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), true);
    socketLock.unlock();

    std::unique_lock<std::mutex> trajectoryLock(trajectoryMutex_);
    trajectoryCV_.wait(trajectoryLock);
    if (!trajectoryResult_) {
        logger_->error("Not able to execute the trajectory");
        return false;
    }
    trajectoryLock.unlock();
    return true;
}

bool RobotBaseControllerClient::startStream(const int& freq) {
    logger_->debug("startStream");
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "startStream";
    json.data["frequency"] = freq;
    return socket_->write(json, json.getHeader(), false);
}

bool RobotBaseControllerClient::stopStream() {
    logger_->debug("stopStream");
    communication::datapackets::JSONPacket json;
    json.data["cmd"] = "stopStream";
    return socket_->write(json, json.getHeader(), false);
}

}  // namespace crf::control::robotbasecontroller
