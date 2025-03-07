/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 * Contributor: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
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

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerClient/StatusStreamerClient.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

#include "DeviceManager/DeviceManagerMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class StatusStreamerClientShould : public ::testing::Test {
 protected:
    StatusStreamerClientShould() :
        logger_("StatusStreamerClientShould"),
        socketMock_(new NiceMock<crf::communication::sockets::SocketMock>),
        packetSocket_(new crf::communication::datapacketsocket::PacketSocket(socketMock_)),
        isSocketOpen_(false),
        dataWritten_(false) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~StatusStreamerClientShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        configureSocket();
    }

    void configureSocket() {
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

    std::unique_ptr<crf::utility::devicemanager::StatusStreamerClient> sut_;
    crf::utility::logger::EventLogger logger_;
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

TEST_F(StatusStreamerClientShould, initializeDeinitializeSequence) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, opensTheSocketIfItWasNotOpen) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, printInLoggerIfWrongCommand) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["comnd"] = "getStatus";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, printInLoggerIfCommandIsNotReply) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "How You Doing?";
    writePacket(json);

    json.data["command"] = "reply";
    json.data["definaetlyNotAReplyCommand"] = "What Are You Looking At?";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, printInLoggerIfReplyCommandDoesNotMatch) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "reply";
    json.data["replyCommand"] = "e4e5ke2";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, printInLoggerIfReplyIsAnError) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "reply";
    json.data["replyCommand"] = "error";
    json.data["message"] = "Menuda liada loco";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, streamStatusAnswerGoesThroughStatusUpdate) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 0));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "reply";
    json.data["replyCommand"] = "streamStatus";
    json.data["message"] = "Check Logger to see function not overwritten";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerClientShould, correctlyStartStreamWithDesiredFrequency) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 1));

    ASSERT_TRUE(sut_->initialize());

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "startStreamStatus");
    ASSERT_EQ(response.value().data["frequency"], 1);

    ASSERT_TRUE(sut_->deinitialize());

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "stopStreamStatus");

    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 10));

    ASSERT_TRUE(sut_->initialize());

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "startStreamStatus");
    ASSERT_EQ(response.value().data["frequency"], 10);

    ASSERT_TRUE(sut_->deinitialize());

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "stopStreamStatus");
}

TEST_F(StatusStreamerClientShould, correctlyStopStream) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerClient(
        packetSocket_, std::chrono::milliseconds(10000), 1));

    ASSERT_TRUE(sut_->initialize());

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "startStreamStatus");
    ASSERT_EQ(response.value().data["frequency"], 1);

    ASSERT_TRUE(sut_->deinitialize());

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "stopStreamStatus");
}
