/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <future>
#include <memory>
#include <string>
#include <chrono>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Sockets/TCP/TCPServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPointFactory.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"

#include "MotionController/MotionControllerMockConfiguration.hpp"
#include "Sockets/SocketMock.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::control::motioncontroller::MotionControllerCommunicationPoint;
using crf::control::motioncontroller::MotionControllerManager;
using crf::control::motioncontroller::MotionControllerMockConfiguration;

class MotionControllerCommunicationPointShould: public ::testing::Test {
 protected:
    MotionControllerCommunicationPointShould() :
        logger_("MotionControllerCommunicationPointShould"),
        isSocketOpen_(false) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~MotionControllerCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        socketMock_.reset(new NiceMock<crf::communication::sockets::SocketMock>);
        packetSocket_.reset(new crf::communication::datapacketsocket::PacketSocket(socketMock_));
        configurePacketSocket();

        controller_ = std::make_shared<MotionControllerMockConfiguration>(6);

        controller_->configureMock();

        std::shared_ptr<MotionControllerManager> manager(
            new MotionControllerManager(controller_));

        sut_.reset(
            new crf::control::motioncontroller::MotionControllerCommunicationPoint(
                packetSocket_,
                manager));
    }

    void lockControl() {
        crf::communication::datapackets::JSONPacket lockControlJSON;
        lockControlJSON.data["command"] = "lockControl";
        lockControlJSON.data["priority"] = 10;

        writePacket(lockControlJSON);

        auto response = readJsonPacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().data["command"], "reply");
        ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
        ASSERT_EQ(response.value().data["message"], true);
    }

    void unlockControl() {
        crf::communication::datapackets::JSONPacket unlockControlJSON;
        unlockControlJSON.data["command"] = "unlockControl";
        unlockControlJSON.data["priority"] = 10;

        writePacket(unlockControlJSON);

        auto response = readJsonPacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().data["command"], "reply");
        ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
        ASSERT_EQ(response.value().data["message"], true);
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
        if (!isSocketOpen_) return std::nullopt;
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


    std::unique_ptr<crf::control::motioncontroller::MotionControllerCommunicationPoint>
        sut_;

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;
    std::shared_ptr<MotionControllerMockConfiguration> controller_;

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

TEST_F(MotionControllerCommunicationPointShould, initializeAndDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, appendJointsPathCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, appendPath(_)).Times(1);

    // Append Path
    crf::utility::types::JointPositions pos({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "appendPath";
    json.data["priority"] = 10;
    json.data["type"] = "joints";
    json.data["data"] = {pos};

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "appendPath");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, appendTaskPathCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, appendPath(_, _, _)).Times(1);

    // Append Path
    crf::utility::types::TaskPose pos(
        {0, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0}));
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "appendPath";
    json.data["priority"] = 10;
    json.data["type"] = "task";
    json.data["data"] = {pos};
    json.data["method"] = 1;
    json.data["reference"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "appendPath");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setJointVelocitiesCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setVelocity(_)).Times(1);

    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = 10;
    json.data["type"] = "joints";
    json.data["data"] = vel;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVelocity");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setTaskVelocityCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setVelocity(_, _)).Times(1);

    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = 10;
    json.data["type"] = "task";
    json.data["data"] = vel;
    json.data["reference"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVelocity");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setJointForceTorquesCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setTorque(_)).Times(1);

    // Append Path
    crf::utility::types::JointForceTorques tqe({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setTorque";
    json.data["priority"] = 10;
    json.data["type"] = "joints";
    json.data["data"] = tqe;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTorque");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setTaskForceTorqueCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setTorque(_, _)).Times(1);

    // Append Path
    crf::utility::types::TaskForceTorque tqe({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setTorque";
    json.data["priority"] = 10;
    json.data["type"] = "task";
    json.data["data"] = tqe;
    json.data["reference"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setTorque");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setJointsProfileVelocityCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setProfileVelocity(
        An<const crf::utility::types::JointVelocities&>())).Times(1);

    // Append Path
    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileVelocity";
    json.data["priority"] = 10;
    json.data["type"] = "joints";
    json.data["data"] = vel;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileVelocity");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setTaskProfileVelocityCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setProfileVelocity(
        An<const crf::utility::types::TaskVelocity&>())).Times(1);

    // Append Path
    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileVelocity";
    json.data["priority"] = 10;
    json.data["type"] = "task";
    json.data["data"] = vel;
    json.data["reference"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileVelocity");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setJointsProfileAccelerationCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setProfileAcceleration(
        An<const crf::utility::types::JointAccelerations&>())).Times(1);

    // Append Path
    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileAcceleration";
    json.data["priority"] = 10;
    json.data["type"] = "joints";
    json.data["data"] = vel;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileAcceleration");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, setTaskProfileAccelerationCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, setProfileAcceleration(
        An<const crf::utility::types::TaskAcceleration&>())).Times(1);

    // Append Path
    crf::utility::types::JointVelocities vel({0, 0, 0, 0, 0, 0});
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileAcceleration";
    json.data["priority"] = 10;
    json.data["type"] = "task";
    json.data["data"] = vel;
    json.data["reference"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileAcceleration");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, callSoftStopCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, softStop()).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "softStop";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "softStop");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(MotionControllerCommunicationPointShould, callHardStopCorrectly) {
    EXPECT_CALL(*controller_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*controller_, hardStop()).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "hardStop";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "hardStop");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*controller_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}
