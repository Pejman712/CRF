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
#include "NetworkServer/TcpServer.hpp"

#include "Mocks/Utility/DummyPacket.hpp"
#include "Mocks/Utility/SocketInterfaceMock.hpp"

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;
using crf::utility::commutility::SocketInterfaceMock;


class TcpServerShould: public ::testing::Test {
 public:
    int assignDataToTheReceiver(int, void* buff, size_t length, int) {
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

 protected:
    TcpServerShould(): logger_("TcpServerShould") {
        somePort_ = 6969;
        someServerSocket_ = 666;
        someClientSocket_ = 667;
        socketInterfaceMock_.reset(new NiceMock<SocketInterfaceMock>);
        ON_CALL(*socketInterfaceMock_, socket(_, _, _)).WillByDefault(Return(someServerSocket_));
        ON_CALL(*socketInterfaceMock_, bind(_, _, _)).WillByDefault(Return(0));
        ON_CALL(*socketInterfaceMock_, listen(_, _)).WillByDefault(Return(0));
        ON_CALL(*socketInterfaceMock_, getsockopt(_, _, _, _, _)).WillByDefault(Invoke(
            [](int socket, int, int, const void*, socklen_t*) {
                if (socket <=0) {
                    return -1;
                } else {
                    return 0;
                }
            }));
        ON_CALL(*socketInterfaceMock_, setsockopt(_, _, _, _, _)).WillByDefault(Invoke(
            [](int socket, int, int, const void*, socklen_t) {
                if (socket <=0) {
                    return -1;
                } else {
                    return 0;
                }
            }));
        ON_CALL(*socketInterfaceMock_, accept(_, _, _)).WillByDefault(Return(someClientSocket_));
        ON_CALL(*socketInterfaceMock_, send(_, _, _, _)).WillByDefault(Invoke(
            [](int, const void*, size_t length, int) {
                return length;
            }));
        ON_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillByDefault(Invoke(
            std::bind(&TcpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4)));
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<SocketInterfaceMock> > socketInterfaceMock_;
    int somePort_;
    int someServerSocket_;
    int someClientSocket_;
    std::unique_ptr<INetworkServer> sut_;
};

TEST_F(TcpServerShould, throwIfFailedToCreateAndConfigureServerSocket) {
    EXPECT_CALL(*socketInterfaceMock_, socket(_, _, _))
        .WillOnce(Return(-1)).WillRepeatedly(Return(someClientSocket_));
    EXPECT_ANY_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    EXPECT_CALL(*socketInterfaceMock_, bind(_, _, _))
        .WillOnce(Return(-1)).WillRepeatedly(Return(0));
    EXPECT_ANY_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    EXPECT_CALL(*socketInterfaceMock_, listen(_, _))
    .WillOnce(Return(-1)).WillRepeatedly(Return(0));
    EXPECT_ANY_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
}

TEST_F(TcpServerShould, notThrowIfEverythingOk) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
}

TEST_F(TcpServerShould, returnFalseIfNotPreviouslyConnected) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->isConnected());
}

TEST_F(TcpServerShould, returnFalseIfAcceptConnectionWithNotSupportedNonBlockingMode) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->acceptConnection(false));
}

TEST_F(TcpServerShould, returnTrueWhenAcceptConnectionAndReturnFalseIfAcceptAgain) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_FALSE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
}

TEST_F(TcpServerShould, beAbleToAcceptConnectionAndDisconnectMultipleTimes) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());
    ASSERT_FALSE(sut_->disconnect());

    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());

    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_FALSE(sut_->isConnected());
}

TEST_F(TcpServerShould, notAllowToSendReceiveIfNotConnected) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    Packets::DummyPacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, packet.serialize()));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(TcpServerShould, sendAndReceiveAppropriateData) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
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

TEST_F(TcpServerShould, returnFalseWhenHeaderIsGarbage) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
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
TEST_F(TcpServerShould, returnFalseWhenSocketReadFailsAndDisconnectItself) {
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::PacketHeader header;
    std::string buff;
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Return(0));
    ASSERT_FALSE(sut_->receive(&header, &buff));
    ASSERT_FALSE(sut_->isConnected());
}

/*
 * You can test this demo with netcat:
 *      > netcat localhost 6969
 */
TEST_F(TcpServerShould, DISABLED_demoSenderAndReceiver) {
    int howManyTimes = 1;
    ASSERT_NO_THROW(sut_.reset(new TcpServer(somePort_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::DummyPacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    for (int i = 0; i < howManyTimes; i++) {
        ASSERT_TRUE(sut_->receive(&header, &buff));
        ASSERT_FALSE(buff.empty());
        packet.deserialize(buff);
        header = packet.getHeader();
        ASSERT_TRUE(sut_->send(header, packet.serialize()));
    }
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_TRUE(sut_->acceptConnection());
    for (int i = 0; i < howManyTimes; i++) {
        ASSERT_TRUE(sut_->receive(&header, &buff));
        ASSERT_FALSE(buff.empty());
        packet.deserialize(buff);
        header = packet.getHeader();
        ASSERT_TRUE(sut_->send(header, packet.serialize()));
    }
}
