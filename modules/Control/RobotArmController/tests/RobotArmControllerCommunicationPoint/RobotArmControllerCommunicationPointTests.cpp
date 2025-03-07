/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <optional>
#include <fstream>

#include <nlohmann/json.hpp>

#include "../RobotArmBehaviourForTests/RobotArmBehaviourForTests.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPoint.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/SocketMock.hpp"

using crf::control::robotarmcontroller::IRobotArmController;

class RobotArmControllerCommunicationPointShould : public RobotArmBehaviourForTests {
 protected:
    RobotArmControllerCommunicationPointShould():
        RobotArmBehaviourForTests(),
        simulatedJointForceTorques_(NUM_JOINTS),
        logger_("RobotArmControllerCommunicationPointShould"),
        socketMock_(new NiceMock<crf::communication::sockets::SocketMock>),
        packetSocket_(new crf::communication::datapacketsocket::PacketSocket(socketMock_)),
        statusJSON_(),
        isSocketOpen_(false),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        writeMutex_(),
        writeCv_(),
        dataWritten_(false),
        bytesWritten_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        robotArmMock_.reset(new NiceMock<RobotArmMock>);
    }

    ~RobotArmControllerCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        robotArmMock_.reset(new NiceMock<crf::actuators::robotarm::RobotArmMock>);
        socketMock_.reset(new NiceMock<crf::communication::sockets::SocketMock>);
        packetSocket_.reset(new crf::communication::datapacketsocket::PacketSocket(socketMock_));
        configureRobotArmDefaultBehavior();
        configurePacketSocket();

        statusJSON_["taskAcceleration"] = crf::utility::types::TaskAcceleration({0, 0, 0, 0, 0, 0});  // NOLINT
        statusJSON_["taskVelocity"] = crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0});
        Eigen::Matrix3d statusJSONMatrix;
        statusJSONMatrix << 0.9999999998650762,     -1.469273945758246e-05, -7.346491160712805e-06,
                            1.4692793427721867e-05,  0.9999999998650762,     7.346383221573885e-06,
                            7.346383221226959e-06,  -7.346491161059725e-06,  0.9999999999460295;
        statusJSON_["taskPose"] = crf::utility::types::TaskPose(
            {-5.552445427179396e-06, -0.00979967309954301, 1.2460999639754617},
            statusJSONMatrix);
            // same rotation in different representations is as following:
            // crf::math::rotation::CardanXYZ({-7.346491161324052e-06,-7.346383221293041e-06,1.4692793428646991e-05}));  // NOLINT
            // Eigen::Quaterniond({0.9999999999595227, -3.673218595807085e-06, -3.6732185956336235e-06,7.346383221623445e-06})); // NOLINT
        statusJSON_["jointAccelerations"] = std::vector<double>(NUM_JOINTS, 0);
        statusJSON_["jointPositions"] = std::vector<double>(NUM_JOINTS, 0);
        statusJSON_["jointForceTorques"] = std::vector<double>(NUM_JOINTS, 0);
        statusJSON_["jointVelocities"] = std::vector<double>(NUM_JOINTS, 0);
        statusJSON_["priorityUnderControl"] = 0;
        statusJSON_["status"] = "initialized";
        statusJSON_["mode"] = 2;  // Velocity

        std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager(
            new crf::control::robotarmcontroller::RobotArmControllerManager(
                robotArmMock_));
        sut_.reset(
            new crf::control::robotarmcontroller::RobotArmControllerCommunicationPoint(
                packetSocket_,
                manager));
    }

    void configurePacketSocket() {
        logger_->debug("configurePacketSocket");
        bytesWritten_.clear();
        bytesToRead_.clear();
        ON_CALL(*socketMock_, isOpen()).WillByDefault(Invoke([this]() {
            return isSocketOpen_;
        }));
        ON_CALL(*socketMock_, open()).WillByDefault(Invoke([this]() {
            isSocketOpen_ = true;
            return true;
        }));
        ON_CALL(*socketMock_, close()).WillByDefault(Invoke([this]() {
            std::unique_lock<std::mutex> lock(readMutex_);
            isSocketOpen_ = false;
            readCv_.notify_all();
            return true;
        }));
        ON_CALL(*socketMock_, read(_)).WillByDefault(Invoke([this](int length) {
            std::unique_lock<std::mutex> lock(readMutex_);
            while (bytesToRead_.length() < static_cast<size_t>(length)) {
                readCv_.wait_for(lock, std::chrono::milliseconds(10));
                if (!isSocketOpen_) {
                    return std::string();
                }
            }
            std::string bytes = bytesToRead_.substr(0, length);
            bytesToRead_ = bytesToRead_.substr(length, bytesToRead_.length());
            return bytes;
        }));
        ON_CALL(*socketMock_, write(_, _)).WillByDefault(
            Invoke([this](std::string buffer, bool ack) {
                std::unique_lock<std::mutex> lock(writeMutex_);
                if (!isSocketOpen_) {
                    return false;
                }
                bytesWritten_.append(buffer);
                dataWritten_ = true;
                writeCv_.notify_all();
                return true;
        }));
    }

    void writePacket(const crf::communication::datapackets::IPacket& packet) {
        std::unique_lock<std::mutex> lock(readMutex_);
        bytesToRead_.append("~~~~~~");
        bytesToRead_.append(packet.getHeader().serialize());
        bytesToRead_.append(packet.serialize());
        readCv_.notify_all();
    }

    std::optional<crf::communication::datapackets::JSONPacket> readJsonPacket() {
        {
            std::unique_lock<std::mutex> lock(writeMutex_);
            dataWritten_ = false;
            if (bytesWritten_.length() == 0) {
                if (!writeCv_.wait_for(lock, std::chrono::seconds(2), [this] {
                    return dataWritten_; })) {
                        return std::nullopt;
                }
            }
        }

        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return std::nullopt;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return std::nullopt;
        }

        crf::communication::datapackets::JSONPacket json;
        std::string jsonBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!json.deserialize(jsonBytes)) {
            logger_->warn("Failed to deserialize packet: {}", jsonBytes);
            return std::nullopt;
        }
        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());
        return json;
    }

    JointForceTorques simulatedJointForceTorques_;

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<crf::control::robotarmcontroller::RobotArmControllerCommunicationPoint>
        sut_;

    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;

    nlohmann::json statusJSON_;
    bool isSocketOpen_;
    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;
};

TEST_F(RobotArmControllerCommunicationPointShould, initializeDeinitializeSequence) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, sendErrorIfMissingCommand) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["comnd"] = "getStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Cannot get command field from received json");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, sendErrorIfUnknownCommand) {
    ASSERT_TRUE(sut_->initialize());

    std::string unknownCommand = "wubba lubba dub dub";
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = unknownCommand;
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Unknown command: " + unknownCommand);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, correctlyGetStatus) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, sendErrorIfWrongStreamParameters) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startStreamStatus";
    json.data["frequency"] = "50";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

    json.data["frequency"] = 0;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Frequency not valid");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, correctlyStartStopStatusStream) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 100.0f;

    crf::communication::datapackets::JSONPacket stopStreamJSON;
    stopStreamJSON.data["command"] = "stopStreamStatus";

    writePacket(startStreamJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    writePacket(stopStreamJSON);

    ASSERT_FALSE(readJsonPacket());

    writePacket(startStreamJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    ASSERT_TRUE(sut_->deinitialize());

    ASSERT_FALSE(readJsonPacket());
}

TEST_F(RobotArmControllerCommunicationPointShould, correctlyStartStatusStreamWithDesiredFrequency) {  // NOLINT
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 25.0f;

    writePacket(startStreamJSON);

    const int numberOfPacketsToRead = 51;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfPacketsToRead; i++) {
        auto response = readJsonPacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().data["command"], "reply");
        ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
        ASSERT_EQ(response.value().data["message"], statusJSON_);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    EXPECT_GE(duration, std::chrono::milliseconds(2000).count());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, stopStreamIfSocketCloses) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 100.0f;

    writePacket(startStreamJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    isSocketOpen_ = false;
    readJsonPacket();

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, intializeWithDefaultModeVelocity) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket getStatusJSON;
    getStatusJSON.data["command"] = "getStatus";

    writePacket(getStatusJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_EQ(response.value().data["message"]["status"], "initialized");

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notUnlockControlIfNotLocked) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "unlockControl";
    lockControlJSON.data["priority"] = -4;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], false);

    crf::communication::datapackets::JSONPacket getStatusJSON;
    getStatusJSON.data["command"] = "getStatus";

    writePacket(getStatusJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_EQ(response.value().data["message"]["status"], "initialized");

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notUnlockControlIfPriorityIsDifferent) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 1;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], false);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, lockAndUnlockCorrectly) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notSetPositionWithoutLockingControl) {
    ASSERT_TRUE(sut_->initialize());

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose(
        {3, 0, 37},
        crf::math::rotation::CardanXYZ({15, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;
    setPositionJSON.data["type"] = "task";
    setPositionJSON.data["method"] = 2;  // Joint Loop
    setPositionJSON.data["reference"] = 2;  // TCP


    writePacket(setPositionJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPos");
    ASSERT_EQ(response.value().data["message"], false);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, setPositionWithCorrectParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::JointPositions> positions;
    positions.push_back(crf::utility::types::JointPositions({1, 0, 0, 0, 0, 0}));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["type"] = "joints";
    setPositionJSON.data["data"] = positions;

    writePacket(setPositionJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPos");
    ASSERT_EQ(response.value().data["message"], true);

    // Wait for result
    response = std::optional<crf::communication::datapackets::JSONPacket>();
    while (!response) {
        response = readJsonPacket();
    }

    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "trajectoryResult");
    ASSERT_EQ(response.value().data["message"], true);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notSetPositionWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose(
        {3, 0, 37},
        crf::math::rotation::CardanXYZ({15, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["type"] = "task";
    setPositionJSON.data["data"] = positions;
    setPositionJSON.data["method"] = 2;  // Joint Loop
    setPositionJSON.data["reference"] = 2;  // TCP

    writePacket(setPositionJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPos");
    ASSERT_EQ(response.value().data["message"], true);

    // Wait for result
    response = std::optional<crf::communication::datapackets::JSONPacket>();
    while (!response) {
        response = readJsonPacket();
    }

    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "trajectoryResult");
    ASSERT_EQ(response.value().data["message"], false);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, setVelocityWithCorrectParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity wrong
    std::vector<crf::utility::types::TaskVelocity> velocities;
    velocities.push_back(crf::utility::types::TaskVelocity({0.1, 0, 0, 0, 0, 0}));
    crf::communication::datapackets::JSONPacket setVelocityJSON;
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["type"] = "task";
    setVelocityJSON.data["data"] = velocities;
    setVelocityJSON.data["method"] = 3;  // Task Loop
    setVelocityJSON.data["reference"] = 2;  // TCP

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong data type");

    // Set velocity right
    crf::utility::types::TaskVelocity vel({0.1, 0, 0, 0, 0, 0});
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["type"] = "task";
    setVelocityJSON.data["data"] = vel;
    setVelocityJSON.data["method"] = 3;  // Task Loop
    setVelocityJSON.data["reference"] = 2;  // TCP

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVel");
    ASSERT_EQ(response.value().data["message"], true);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notSetVelocityWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    std::vector<crf::utility::types::TaskVelocity> vect;
    vect.push_back(crf::utility::types::TaskVelocity({0, 0, 500, 0, 0, 0}));
    crf::communication::datapackets::JSONPacket setVelocityJSON;
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["type"] = "task";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = vect;
    setVelocityJSON.data["method"] = 3;  // Task Loop
    setVelocityJSON.data["reference"] = 2;  // TCP

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong data type");

    // Set velocity
    crf::utility::types::TaskVelocity vel2({0.0, 0, 0, 0, 1, 0});
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["type"] = "task";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = vel2;
    setVelocityJSON.data["method"] = 3;  // Task Loop
    setVelocityJSON.data["reference"] = 2;  // TCP

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVel");
    ASSERT_EQ(response.value().data["message"], true);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, interruptTrajectoryCorrectly) {
    ASSERT_TRUE(sut_->initialize());

    // Interrupt
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "interrupt";
    setPositionJSON.data["priority"] = 10;

    writePacket(setPositionJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "No trajectory executing");

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose(
        {0.1, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0})));
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["type"] = "task";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;
    setPositionJSON.data["method"] = 2;  // Joint Loop
    setPositionJSON.data["reference"] = 2;  // TCP

    writePacket(setPositionJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPos");
    ASSERT_EQ(response.value().data["message"], true);

    // Interrupt
    setPositionJSON.data["command"] = "interrupt";
    setPositionJSON.data["priority"] = 10;

    writePacket(setPositionJSON);

    // Trajectory finished
    response = readJsonPacket();
    while (response) {
        if (response.value().data["replyCommand"] == "trajectoryResult") {
            ASSERT_EQ(response.value().data["command"], "reply");
            ASSERT_EQ(response.value().data["replyCommand"], "trajectoryResult");
            ASSERT_EQ(response.value().data["message"], false);
        }
        if (response.value().data["replyCommand"] == "interrupt") {
            ASSERT_EQ(response.value().data["command"], "reply");
            ASSERT_EQ(response.value().data["replyCommand"], "interrupt");
            ASSERT_EQ(response.value().data["message"], true);
        }
        if (response.value().data["replyCommand"] == "trajectoryResult") {
            ASSERT_EQ(response.value().data["command"], "reply");
            ASSERT_EQ(response.value().data["replyCommand"], "trajectoryResult");
            ASSERT_EQ(response.value().data["message"], false);
        }

        response = readJsonPacket();
    }

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notSetMaxVelocityWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    crf::utility::types::TaskVelocity maxVel({0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
    crf::communication::datapackets::JSONPacket setVelocityJSON;
    setVelocityJSON.data["command"] = "setTaskMaxVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = maxVel;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTaskMaxVel");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    maxVel = crf::utility::types::TaskVelocity({0, 0, -0.1, 0, 0, 0});
    setVelocityJSON.data["command"] = "setTaskMaxVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = maxVel;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTaskMaxVel");
    ASSERT_EQ(response.value().data["message"], false);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotArmControllerCommunicationPointShould, notSetMaxAccelerationWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set acceleration
    crf::utility::types::TaskAcceleration maxAcc({0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
    crf::communication::datapackets::JSONPacket setAccelerationJSON;
    setAccelerationJSON.data["command"] = "setTaskMaxAcc";
    setAccelerationJSON.data["priority"] = 10;
    setAccelerationJSON.data["data"] = maxAcc;

    writePacket(setAccelerationJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTaskMaxAcc");
    ASSERT_EQ(response.value().data["message"], true);

    // Set acceleration
    maxAcc = crf::utility::types::TaskAcceleration({0, 0, 0, 0, 0, 0});
    setAccelerationJSON.data["command"] = "setTaskMaxAcc";
    setAccelerationJSON.data["priority"] = 10;
    setAccelerationJSON.data["data"] = maxAcc;

    writePacket(setAccelerationJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTaskMaxAcc");
    ASSERT_EQ(response.value().data["message"], false);

    // Unlock Control
    crf::communication::datapackets::JSONPacket unlockControlJSON;
    unlockControlJSON.data["command"] = "unlockControl";
    unlockControlJSON.data["priority"] = 10;

    writePacket(unlockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}
