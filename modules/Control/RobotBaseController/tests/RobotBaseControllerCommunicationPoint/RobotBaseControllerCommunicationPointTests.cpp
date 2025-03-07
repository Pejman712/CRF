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

#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPoint.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"

#include "Types/Types.hpp"

#include "RobotBase/RobotBaseMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class RobotBaseControllerCommunicationPointShould : public ::testing::Test {
 protected:
    RobotBaseControllerCommunicationPointShould() :
        logger_("RobotBaseControllerCommunicationPointShould"),
        robotBaseConfiguration_(new crf::actuators::robotbase::RobotBaseConfiguration),
        robotBaseMock_(new NiceMock<crf::actuators::robotbase::RobotBaseMock>),
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
    }

    ~RobotBaseControllerCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        statusJSON_["position"] = crf::utility::types::TaskPose({0, 0, std::numeric_limits<double>::quiet_NaN()},  // NOLINT
        crf::math::rotation::CardanXYZ({std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0}));  // NOLINT
        statusJSON_["velocity"] = crf::utility::types::TaskVelocity({0, 0, 0, 0, 0, 0});
        statusJSON_["priorityUnderControl"] = 0;
        statusJSON_["status"] = "initialized";
        statusJSON_["mode"] = 2;  // Velocity

        robotBaseMock_.reset(new NiceMock<crf::actuators::robotbase::RobotBaseMock>);
        socketMock_.reset(new NiceMock<crf::communication::sockets::SocketMock>);
        packetSocket_.reset(new crf::communication::datapacketsocket::PacketSocket(socketMock_));
        configureRobotBase();
        configurePacketSocket();
        std::shared_ptr<crf::control::robotbasecontroller::RobotBaseControllerManager> manager(
            new crf::control::robotbasecontroller::RobotBaseControllerManager(
                robotBaseMock_));
        sut_.reset(
            new crf::control::robotbasecontroller::RobotBaseControllerCommunicationPoint(
                packetSocket_,
                manager));
    }

    void configureRobotBase() {
        logger_->debug("configureRobotBase");

        ON_CALL(*robotBaseMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*robotBaseMock_, deinitialize()).WillByDefault(Return(true));

        std::string testFileDirName = __FILE__;
        testFileDirName = testFileDirName.substr(0, testFileDirName.find("modules/"));
        testFileDirName += "modules/Actuators/RobotBase/tests/config/";
        testFileDirName.append("goodCernBot2Config.json");
        std::ifstream config(testFileDirName);
        ASSERT_TRUE(robotBaseConfiguration_->parse(nlohmann::json::parse(config)));
        ON_CALL(*robotBaseMock_, getConfiguration()).WillByDefault(Return(robotBaseConfiguration_));
        ON_CALL(*robotBaseMock_, setTaskVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*robotBaseMock_, getTaskPose()).WillByDefault(Return(basePosition_));
        ON_CALL(*robotBaseMock_, getTaskVelocity()).WillByDefault(Return(baseVelocity_));
        ON_CALL(*robotBaseMock_, stopBase()).WillByDefault(Return(true));
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

    std::unique_ptr<crf::control::robotbasecontroller::RobotBaseControllerCommunicationPoint> sut_;
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::actuators::robotbase::RobotBaseConfiguration> robotBaseConfiguration_;
    std::shared_ptr<crf::actuators::robotbase::RobotBaseMock> robotBaseMock_;
    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;

    crf::utility::types::TaskVelocity baseVelocity_;
    crf::utility::types::TaskPose basePosition_;

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

TEST_F(RobotBaseControllerCommunicationPointShould, initializeDeinitializeSequence) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, sendErrorIfMissingCommand) {
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

TEST_F(RobotBaseControllerCommunicationPointShould, sendErrorIfUnknownCommand) {
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

TEST_F(RobotBaseControllerCommunicationPointShould, correctlyGetStatus) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");

    ASSERT_EQ(
        nlohmann::to_string(response.value().data["message"]), nlohmann::to_string(statusJSON_));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, sendErrorIfWrongStreamParameters) {
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

TEST_F(RobotBaseControllerCommunicationPointShould, correctlyStartStopStatusStream) {
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
    ASSERT_EQ(
        nlohmann::to_string(response.value().data["message"]), nlohmann::to_string(statusJSON_));

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(
        nlohmann::to_string(response.value().data["message"]), nlohmann::to_string(statusJSON_));

    writePacket(stopStreamJSON);

    ASSERT_FALSE(readJsonPacket());

    writePacket(startStreamJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(
        nlohmann::to_string(response.value().data["message"]), nlohmann::to_string(statusJSON_));

    ASSERT_TRUE(sut_->deinitialize());

    ASSERT_FALSE(readJsonPacket());
}

TEST_F(RobotBaseControllerCommunicationPointShould, correctlyStartStatusStreamWithDesiredFrequency) {  // NOLINT
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
        ASSERT_EQ(
            nlohmann::to_string(response.value().data["message"]),
            nlohmann::to_string(statusJSON_));
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    EXPECT_GE(duration, std::chrono::milliseconds(2000).count());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, stopStreamIfSocketCloses) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 100.0f;

    writePacket(startStreamJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(
        nlohmann::to_string(response.value().data["message"]), nlohmann::to_string(statusJSON_));

    isSocketOpen_ = false;
    readJsonPacket();

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, intializeWithDefaultModeVelocity) {
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

TEST_F(RobotBaseControllerCommunicationPointShould, failIfSetModeIncorrectly) {
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

    crf::communication::datapackets::JSONPacket getStatusJSON;
    getStatusJSON.data["command"] = "getStatus";

    writePacket(getStatusJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_EQ(response.value().data["message"]["status"], "initialized");

    crf::communication::datapackets::JSONPacket setModeJSON;
    setModeJSON.data["command"] = "setMode";
    setModeJSON.data["priority"] = 1;
    setModeJSON.data["mode"] = 2;  // Velocity

    writePacket(setModeJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMode");
    ASSERT_EQ(response.value().data["message"], true);

    setModeJSON.data["command"] = "setMode";
    setModeJSON.data["priority"] = 1;
    setModeJSON.data["mode"] = 0;

    writePacket(setModeJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong controller mode selected");


    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, notUnlockControlIfNotLocked) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "unlockControl";
    lockControlJSON.data["priority"] = -4;
    lockControlJSON.data["mode"] = 1;

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

TEST_F(RobotBaseControllerCommunicationPointShould, notUnlockControlIfPriorityIsDifferent) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 1;
    lockControlJSON.data["mode"] = 2;

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

TEST_F(RobotBaseControllerCommunicationPointShould, lockAndUnlockCorrectly) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

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

TEST_F(RobotBaseControllerCommunicationPointShould, notSetPositionWithoutLockingControl) {
    ASSERT_TRUE(sut_->initialize());

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({3, 0, 37},
        crf::math::rotation::CardanXYZ({15, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;

    writePacket(setPositionJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, setPositionWithCorrectParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({0.03, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
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

TEST_F(RobotBaseControllerCommunicationPointShould, notSetPositionWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({3, 0, 37},
        crf::math::rotation::CardanXYZ({15, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;

    writePacket(setPositionJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

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

TEST_F(RobotBaseControllerCommunicationPointShould, setVelocityWithCorrectParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

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
    setVelocityJSON.data["data"] = velocities;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong velocity data");

    // Set velocity right
    crf::utility::types::TaskVelocity vel({0.1, 0, 0, 0, 0, 0});
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = vel;

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

TEST_F(RobotBaseControllerCommunicationPointShould, notSetVelocityWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    crf::utility::types::TaskVelocity vel({0, 0, 0.1, 0, 0, 0});
    crf::communication::datapackets::JSONPacket setVelocityJSON;
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = vel;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVel");
    ASSERT_EQ(response.value().data["message"], false);

    // Set velocity
    crf::utility::types::TaskVelocity vel2({0.0, 0, 0, 0, 0.1, 0});
    setVelocityJSON.data["command"] = "setVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = vel2;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVel");
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

TEST_F(RobotBaseControllerCommunicationPointShould, interruptTrajectoryCorrectly) {
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
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({0.1, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0})));
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;

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

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "interrupt");
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

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotBaseControllerCommunicationPointShould, notSetMaxVelocityWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    crf::utility::types::TaskVelocity maxVel({0, 0, 0.1, 0, 0, 0});
    crf::communication::datapackets::JSONPacket setVelocityJSON;
    setVelocityJSON.data["command"] = "setMaxVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = maxVel;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMaxVel");
    ASSERT_EQ(response.value().data["message"], true);

    // Set velocity
    maxVel = crf::utility::types::TaskVelocity({0, 0, -0.1, 0, 0, 0});
    setVelocityJSON.data["command"] = "setMaxVel";
    setVelocityJSON.data["priority"] = 10;
    setVelocityJSON.data["data"] = maxVel;

    writePacket(setVelocityJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMaxVel");
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

TEST_F(RobotBaseControllerCommunicationPointShould, notSetMaxAccelerationWithWrongParameters) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set acceleration
    crf::utility::types::TaskAcceleration maxAcc({0, 0, 0.1, 0, 0, 0});
    crf::communication::datapackets::JSONPacket setAccelerationJSON;
    setAccelerationJSON.data["command"] = "setMaxAcc";
    setAccelerationJSON.data["priority"] = 10;
    setAccelerationJSON.data["data"] = maxAcc;

    writePacket(setAccelerationJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMaxAcc");
    ASSERT_EQ(response.value().data["message"], true);

    // Set acceleration
    maxAcc = crf::utility::types::TaskAcceleration({0, 0, -0.1, 0, 0, 0});
    setAccelerationJSON.data["command"] = "setMaxAcc";
    setAccelerationJSON.data["priority"] = 10;
    setAccelerationJSON.data["data"] = maxAcc;

    writePacket(setAccelerationJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMaxAcc");
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

TEST_F(RobotBaseControllerCommunicationPointShould, DISABLED_changeControllerModeWithSamePriority) {
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;  // Velocity

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({1, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
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

    // Lock control
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;  // Velocity

    writePacket(lockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
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

TEST_F(RobotBaseControllerCommunicationPointShould, DISABLED_changeControllerModeIfPrioritySuperior) {  // NOLINT
    ASSERT_TRUE(sut_->initialize());

    // Lock control
    crf::communication::datapackets::JSONPacket lockControlJSON;
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 10;
    lockControlJSON.data["mode"] = 2;  // Velocity

    writePacket(lockControlJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    // Set position
    std::vector<crf::utility::types::TaskPose> positions;
    positions.push_back(crf::utility::types::TaskPose({1, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0})));
    crf::communication::datapackets::JSONPacket setPositionJSON;
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
    setPositionJSON.data["data"] = positions;

    writePacket(setPositionJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPos");
    ASSERT_EQ(response.value().data["message"], true);

    // Lock control
    lockControlJSON.data["command"] = "lockControl";
    lockControlJSON.data["priority"] = 1;
    lockControlJSON.data["mode"] = 2;  // Velocity

    writePacket(lockControlJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "trajectoryResult");
    ASSERT_EQ(response.value().data["message"], false);

    // Set position
    setPositionJSON.data["command"] = "setPos";
    setPositionJSON.data["priority"] = 10;
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
    unlockControlJSON.data["priority"] = 1;

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

