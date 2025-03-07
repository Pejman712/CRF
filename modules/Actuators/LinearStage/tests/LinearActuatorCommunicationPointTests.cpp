/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Sebastien Collomb CERN BE/CEM/MRO 2023
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
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPoint.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPointFactory.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorManager.hpp"

#include "LinearStage/LinearActuatorMockConfiguration.hpp"
#include "Sockets/SocketMock.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::actuators::linearactuator::LinearActuatorCommunicationPoint;
using crf::actuators::linearactuator::LinearActuatorManager;
using crf::actuators::linearactuator::LinearActuatorMockConfiguration;

class LinearActuatorCommunicationPointShould: public ::testing::Test {
 protected:
    LinearActuatorCommunicationPointShould() :
        logger_("LinearActuatorCommunicationPointShould"),
        isSocketOpen_(false), dataWritten_(false) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~LinearActuatorCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        socketMock_.reset(new NiceMock<crf::communication::sockets::SocketMock>);
        packetSocket_.reset(new crf::communication::datapacketsocket::PacketSocket(socketMock_));
        configurePacketSocket();

        actuator_ = std::make_shared<LinearActuatorMockConfiguration>();

        actuator_->configureMock();

        std::shared_ptr<LinearActuatorManager> manager(
            new LinearActuatorManager(actuator_));

        sut_.reset(
            new crf::actuators::linearactuator::LinearActuatorCommunicationPoint(
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

    std::unique_ptr<crf::actuators::linearactuator::LinearActuatorCommunicationPoint>
        sut_;

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;
    std::shared_ptr<LinearActuatorMockConfiguration> actuator_;

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

TEST_F(LinearActuatorCommunicationPointShould, initializeAndDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(LinearActuatorCommunicationPointShould, setPositionCorrectly) {
    EXPECT_CALL(*actuator_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*actuator_, setPosition(_)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setPosition";
    json.data["priority"] = 10;
    json.data["position"] = 1.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setPosition");
    nlohmann::json jsonResult;
    jsonResult["code"] = 200;
    jsonResult["detail"] = 0;
    jsonResult["value"] = true;
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*actuator_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(LinearActuatorCommunicationPointShould, setVelocityCorrectly) {
    EXPECT_CALL(*actuator_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*actuator_, setVelocity(_)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = 10;
    json.data["velocity"] = 1.0;

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

    EXPECT_CALL(*actuator_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}
