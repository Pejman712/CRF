/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/TcpClient.hpp"

#include "Mocks/Utility/DummyPacket.hpp"
#include "Mocks/Utility/SocketInterfaceMock.hpp"
#include "TcpClientTests.hpp"

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Ne;

using crf::communication::networkclient::INetworkClient;
using crf::communication::networkclient::TcpClient;
using crf::utility::commutility::SocketInterfaceMock;


int TcpClientShould::assignAddrinfo(const char*, const char*,
    const struct addrinfo*, struct addrinfo **res) {
    *res = &addrinfo_;
    return 0;
}

int TcpClientShould::assignDataToTheReceiver(int, void* buff, size_t length, int) {
    logger_->debug("length: {}", length);
    Packets::DummyPacket packet;
    // hack to avoid undefined references to static members in variadic templates
    unsigned int packetSize = packet.size;
    if (length == Packets::PacketHeader::size) {
        Packets::PacketHeader header = packet.getHeader();
        logger_->debug("copying header to mem ...");
        std::memcpy(buff, header.serialize().c_str(), length);
        return length;
    } else if (length == packetSize) {
        packet.x_ = 6;
        packet.y_ = 9;
        std::memcpy(buff, packet.serialize().c_str(), packetSize);
        return packet.size;
    } else {
        logger_->warn("Length: {} != packet.size: {}", length, packetSize);
        std::string s(length, 'x');
        std::memcpy(buff, s.c_str(), length);
        return length;
    }
}

TcpClientShould::TcpClientShould(): logger_("TcpClientShould"), addrinfo_(), sockaddr_() {
    logger_->info("{0} BEGIN",
        testing::UnitTest::GetInstance()->current_test_info()->name());
    sockaddr_.sa_family = 69;
    addrinfo_.ai_addr = &sockaddr_;
    somePort_ = 6969;
    someClientSocket_ = 666;
    someAddr_ = "192.168.0.69";
    socketInterfaceMock_.reset(new NiceMock<SocketInterfaceMock>);
    ON_CALL(*socketInterfaceMock_, getaddrinfo(_, _, _, _)).WillByDefault(Invoke(
        std::bind(&TcpClientShould::assignAddrinfo, this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4)));
    ON_CALL(*socketInterfaceMock_, getsockopt(_, _, _, _, _)).WillByDefault(Invoke(
        [](int socket, int, int, const void*, socklen_t*) {
            if (socket <=0) {
                return -1;
            } else {
                return 0;
            }
        }));
    ON_CALL(*socketInterfaceMock_, socket(_, _, _)).WillByDefault(Return(someClientSocket_));
    ON_CALL(*socketInterfaceMock_, connect(someClientSocket_, _, _)).WillByDefault(Return(0));
    ON_CALL(*socketInterfaceMock_, connect(Ne(someClientSocket_), _, _))
        .WillByDefault(Return(-1));
    ON_CALL(*socketInterfaceMock_, send(_, _, _, _)).WillByDefault(Invoke(
        [](int, const void*, size_t length, int) {
            return length;
        }));
    ON_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillByDefault(Invoke(
        std::bind(&TcpClientShould::assignDataToTheReceiver, this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4)));
}

TcpClientShould::~TcpClientShould() {
    logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
}

TEST_F(TcpClientShould, throwIfFailedToGetAddrinfo) {
    EXPECT_CALL(*socketInterfaceMock_, getaddrinfo(_, _, _, _)).WillOnce(Return(-1));
    EXPECT_ANY_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
}

TEST_F(TcpClientShould, notThrowIfEverythingOk) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
}

TEST_F(TcpClientShould, returnFalseIfNotPreviouslyConnected) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->isConnected());
}

TEST_F(TcpClientShould, returnFalseIfConnectWithNotSupportedNonBlockingMode) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->connect(false));
}

TEST_F(TcpClientShould, returnFalseIfFailedToCreateSocket) {
    EXPECT_CALL(*socketInterfaceMock_, socket(_, _, _)).WillOnce(Return(-1));
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->connect());
}

TEST_F(TcpClientShould, returnFalseIfFailedToConnectForSomeReason) {
    EXPECT_CALL(*socketInterfaceMock_, connect(_, _, _)).WillOnce(Return(-1));
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->connect());
}

TEST_F(TcpClientShould, returnTrueIfConnectAndReturnFalseIfConnectAgain) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_FALSE(sut_->connect());
    ASSERT_TRUE(sut_->isConnected());
}

TEST_F(TcpClientShould, beAbleToConnectAndDisconnectMultipleTimes) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());
    ASSERT_FALSE(sut_->disconnect());

    ASSERT_TRUE(sut_->connect());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());

    ASSERT_TRUE(sut_->connect());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());
}

TEST_F(TcpClientShould, notAllowToSendReceiveIfNotConnected) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    Packets::DummyPacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, packet.serialize()));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(TcpClientShould, sendAndReceiveAppropriateData) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::DummyPacket packet;
    packet.x_ = 69;
    packet.y_ = 6.9;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_TRUE(sut_->send(header, packet.serialize()));
    ASSERT_TRUE(sut_->receive(&header, &buff));
    ASSERT_FALSE(buff.empty());
    ASSERT_TRUE(packet.deserialize(buff));
}

TEST_F(TcpClientShould, returnFalseWhenHeaderIsGarbage) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::PacketHeader header;
    std::string buff;
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Invoke(
        [] (int, void* buff, size_t length, int) {
            std::string s(length, 'x');
            std::memcpy(buff, s.c_str(), length);
            return length;
        }));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

/*
 * Dead client socket detection
 */
TEST_F(TcpClientShould, returnFalseWhenSocketReadFailsAndDisconnectItself) {
    ASSERT_NO_THROW(sut_.reset(new TcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::PacketHeader header;
    std::string buff;
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Return(0));
    ASSERT_FALSE(sut_->receive(&header, &buff));
    ASSERT_FALSE(sut_->isConnected());
}
