/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <chrono>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Sockets/TCP/TCPServer.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPoint.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPointFactory.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"

#include "CANopenDrivers/CiA402/CiA402DriverMockConfiguration.hpp"
#include "Sockets/SocketMock.hpp"
#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"
#include "crf/expected.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::devices::canopendrivers::CiA402CommunicationPoint;
using crf::devices::canopendrivers::CiA402Manager;
using crf::devices::canopendrivers::CiA402DriverMockConfiguration;

class CiA402CommunicationPointShould: public ::testing::Test {
 protected:
    CiA402CommunicationPointShould() :
        logger_("CiA402CommunicationPointShould"),
        isSocketOpen_(false), dataWritten_(false) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CiA402CommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        socketMock_.reset(new NiceMock<crf::communication::sockets::SocketMock>);
        packetSocket_.reset(new crf::communication::datapacketsocket::PacketSocket(socketMock_));
        configurePacketSocket();

        cia402driver_ = std::make_shared<CiA402DriverMockConfiguration>();

        cia402driver_->configureMock();

        std::shared_ptr<CiA402Manager> manager(
            new CiA402Manager(cia402driver_));

        sut_.reset(
            new crf::devices::canopendrivers::CiA402CommunicationPoint(
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

    std::unique_ptr<crf::devices::canopendrivers::CiA402CommunicationPoint>
        sut_;

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;
    std::shared_ptr<CiA402DriverMockConfiguration> cia402driver_;

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

TEST_F(CiA402CommunicationPointShould, initializeAndDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setProfilePositionCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setProfilePosition(_, _, _, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfilePosition";
    json.data["priority"] = 10;
    json.data["position"] = 1.0;
    json.data["velocity"] = 1.0;
    json.data["acceleration"] = 1.0;
    json.data["deceleration"] = 0.0;
    json.data["positionReference"] = crf::devices::canopendrivers::PositionReference::Absolute;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfilePosition");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setProfileVelocityCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setProfileVelocity(_, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileVelocity";
    json.data["priority"] = 10;
    json.data["velocity"] = 1.0;
    json.data["acceleration"] = 1.0;
    json.data["deceleration"] = 0.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileVelocity");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setProfileTorqueCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setProfileTorque(_)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileTorque";
    json.data["priority"] = 10;
    json.data["torque"] = 2.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setProfileTorque");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setVelocityCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setVelocity(_, _, _, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = 10;
    json.data["velocity"] = 1.0;
    json.data["deltaSpeedAcc"] = 1.0;
    json.data["deltaTimeAcc"] = 1.0;
    json.data["deltaSpeedDec"] = 0.0;
    json.data["deltaTimeDec"] = 0.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setVelocity");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setMaximumTorqueCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setMaximumTorque(_)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setMaximumTorque";
    json.data["priority"] = 10;
    json.data["maxTorque"] = 2.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setMaximumTorque");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setInterpolatedPositionCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setInterpolatedPosition(_, _, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setInterpolatedPosition";
    json.data["priority"] = 10;
    json.data["position"] = 1.0;
    json.data["velocity"] = 1.0;
    json.data["acceleration"] = 2.0;
    json.data["deceleration"] = 1.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setInterpolatedPosition");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setModeOfOperation) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setModeOfOperation(_)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setModeOfOperation";
    json.data["priority"] = 10;
    json.data["mode"] = crf::devices::canopendrivers::ModeOfOperation::ProfilePositionMode;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setModeOfOperation");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setCyclicPositionCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setCyclicPosition(_, _, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setCyclicPosition";
    json.data["priority"] = 10;
    json.data["position"] = 1.0;
    json.data["posOffset"] = 1.0;
    json.data["velOffset"] = 1.0;
    json.data["torOffset"] = 1.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setCyclicPosition");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setCyclicVelocityCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setCyclicVelocity(_, _, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setCyclicVelocity";
    json.data["priority"] = 10;
    json.data["velocity"] = 1.0;
    json.data["velOffset"] = 1.0;
    json.data["torOffset"] = 1.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setCyclicVelocity");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, setCyclicTorqueCorrectly) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, setCyclicTorque(_, _)).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "setCyclicTorque";
    json.data["priority"] = 10;
    json.data["torque"] = 1.0;
    json.data["torOffset"] = 1.0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "setCyclicTorque");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, checkIfQuickStop) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, quickStop()).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "quickStop";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "quickStop");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, checkIfStop) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    // Called again after all clients disconnect
    EXPECT_CALL(*cia402driver_, stop()).Times(2);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "stop";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "stop");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, checkResetFault) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, resetFault()).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "resetFault";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "resetFault");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, checkResetQuickStop) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    EXPECT_CALL(*cia402driver_, resetQuickStop()).Times(1);

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "resetQuickStop";
    json.data["priority"] = 10;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "resetQuickStop");
    ASSERT_EQ(response.value().data["message"].get<crf::expected<bool>>(),
        crf::expected<bool>(true));

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CommunicationPointShould, getStatus) {
    EXPECT_CALL(*cia402driver_, initialize()).Times(1);
    ASSERT_TRUE(sut_->initialize());

    lockControl();

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");

    nlohmann::json jsonResult;
    jsonResult["status"] = "initialize";

    jsonResult["position"] = crf::expected<double>(0);
    jsonResult["velocity"] = crf::expected<double>(0);
    jsonResult["torque"] = crf::expected<double>(0);
    jsonResult["maxTorque"] = crf::expected<double>(0);
    jsonResult["inFault"] = false;
    jsonResult["inQuickStop"] = false;
    jsonResult["statusWord"] = "Operation Enabled";
    jsonResult["modeOfOperation"] = 0;
    jsonResult["motorStatus"] = std::vector<crf::ResponseCode>({crf::ResponseCode(crf::Code::OK)});
    ASSERT_EQ(response.value().data["message"], jsonResult);

    unlockControl();

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    EXPECT_CALL(*cia402driver_, deinitialize()).Times(1);
    ASSERT_TRUE(sut_->deinitialize());
}
