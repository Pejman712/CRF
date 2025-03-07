/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "IPC/IPC.hpp"
#include "IPC/NetworkIPC.hpp"

#include "Mocks/Communication/NetworkServerMock.hpp"
#include "Mocks/Utility/DummyPacket.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)

using crf::communication::ipc::NetworkIPC;
using crf::communication::networkserver::NetworkServerMock;

using testing::_;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class NetworkIPCShould: public ::testing::Test {
 protected:
    NetworkIPCShould(): logger_("NetworkIPCShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        message_ = "Dummy msg";
        header_.length = message_.length();
        serverMock_.reset(new NiceMock<NetworkServerMock>);
        connected_ = false;
        ON_CALL(*serverMock_, isConnected()).WillByDefault(Invoke([this] {
            return connected_;
        }));
        ON_CALL(*serverMock_, disconnect()).WillByDefault(Invoke([this] {
            connected_ = false;
            return true;
        }));
        ON_CALL(*serverMock_, acceptConnection(_)).WillByDefault(Invoke([this](bool) {
            if (connected_) return false;
            connected_ = true;
            return true;
        }));
        ON_CALL(*serverMock_, send(_, _)).WillByDefault(
            Invoke([this](const Packets::PacketHeader&, const std::string&) {
                return true;
        }));
        ON_CALL(*serverMock_, receive(_, _)).WillByDefault(
            Invoke([this](Packets::PacketHeader* header, std::string* buffer) {
                header->length = header_.length;
                buffer->resize(header->length);
                buffer->assign(message_.c_str(), header->length);
                return true;
        }));
    }
    ~NetworkIPCShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::mutex m_;
    std::condition_variable clientConnectedCV_;
    std::string message_;
    Packets::PacketHeader header_;
    bool connected_;
    std::shared_ptr<NetworkServerMock> serverMock_;
    std::unique_ptr<NetworkIPC> ipc_;
};

TEST_F(NetworkIPCShould, returnFalseIfOpenOrCloseTwice) {
    ipc_.reset(new NetworkIPC(serverMock_));
    ASSERT_TRUE(ipc_->open());
    ASSERT_FALSE(ipc_->open());
    ASSERT_TRUE(ipc_->close());
    ASSERT_FALSE(ipc_->close());
}

TEST_F(NetworkIPCShould, returnFalseForReadWriteIfNotOpen) {
    ipc_.reset(new NetworkIPC(serverMock_));
    ASSERT_FALSE(ipc_->write(message_, header_));
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
}

TEST_F(NetworkIPCShould, returnTrueForWriteAndReadOperationsIfOpen) {
    ipc_.reset(new NetworkIPC(serverMock_));
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    ASSERT_TRUE(ipc_->open());
    ASSERT_TRUE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_TRUE(ipc_->write(message_, header_));
    ASSERT_EQ(message_, receivedMsg);
    ASSERT_EQ(header_.length, receivedHeader.length);
    ASSERT_TRUE(ipc_->close());
}

TEST_F(NetworkIPCShould, closeItselfOnDestruction) {
    ipc_.reset(new NetworkIPC(serverMock_));
    ASSERT_TRUE(ipc_->open());
    ipc_.reset();
    ASSERT_FALSE(connected_);
}

// Test disabled due to a constant file. Needs to be checked
TEST_F(NetworkIPCShould, DISABLED_returnFalseOnReadWriteDuringAutomaticReconnectionAndOnReconnectionFailure) {  // NOLINT
    ipc_.reset(new NetworkIPC(serverMock_));
    ASSERT_TRUE(ipc_->open());
    connected_ = false;
    EXPECT_CALL(*serverMock_, acceptConnection(_)).WillOnce(Invoke(
        [this] (bool) {
            std::unique_lock<std::mutex> l(m_);
            if (clientConnectedCV_.wait_for(l, CV_TIMEOUT) == std::cv_status::timeout) {
                throw std::runtime_error("Some deadlock fuckup in the test");
            }
            connected_ = true;
            return true;
        })).WillRepeatedly(DoDefault());
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(ipc_->write(message_, header_));
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(ipc_->write(message_, header_));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    clientConnectedCV_.notify_one();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_TRUE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_TRUE(ipc_->write(message_, header_));
    connected_ = false;
    EXPECT_CALL(*serverMock_, acceptConnection(_)).WillOnce(Invoke(
        [this] (bool) {
            std::unique_lock<std::mutex> l(m_);
            if (clientConnectedCV_.wait_for(l, CV_TIMEOUT) == std::cv_status::timeout) {
                throw std::runtime_error("Some deadlock fuckup in the test");
            }
            return false;
        })).WillRepeatedly(DoDefault());
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(ipc_->write(message_, header_));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    clientConnectedCV_.notify_one();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // autoreconnection finished immediately this time, so TRUE
    ASSERT_TRUE(ipc_->write(message_, header_));
}

TEST_F(NetworkIPCShould, closeItselfOnDestructionAlsoDuringAutoReconnection) {
    ipc_.reset(new NetworkIPC(serverMock_));
    ASSERT_TRUE(ipc_->open());
    connected_ = false;
    EXPECT_CALL(*serverMock_, acceptConnection(_)).WillOnce(Invoke(
        [this] (bool) {
            logger_->info("acceptConnection");
            std::unique_lock<std::mutex> l(m_);
            if (clientConnectedCV_.wait_for(l, CV_TIMEOUT) == std::cv_status::timeout) {
                throw std::runtime_error("Some deadlock fuckup in the test");
            }
            connected_ = true;
            return true;
        })).WillRepeatedly(DoDefault());
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    ASSERT_FALSE(ipc_->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(ipc_->write(message_, header_));
    ipc_.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    logger_->info("wake up");
    clientConnectedCV_.notify_one();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_FALSE(connected_);
}
