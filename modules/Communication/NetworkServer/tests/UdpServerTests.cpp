/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "NetworkServer/UdpServer.hpp"
#include "NetworkServer/UDPSocketPackets.hpp"

#include "Mocks/Utility/DummyPacket.hpp"
#include "Mocks/Utility/SocketInterfaceMock.hpp"
#include <arpa/inet.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Ref;
using ::testing::Pointee;

using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::UdpServer;
using crf::utility::commutility::SocketInterfaceMock;


class UdpServerShould: public ::testing::Test {
 public:
     int assignDataToTheReceiver(int, void* buff, size_t length, int, struct sockaddr *clientAddr,
        socklen_t *) {
        std::memcpy(clientAddr, reinterpret_cast<sockaddr*>(&someClientAddress_),
            sizeof(struct sockaddr_in));
        if (isConnected_) {
            Packets::DummyPacket packet;
            // hack to avoid undefined references to static members in variadic templates
            unsigned int packetSize = packet.size;
            packet.x_ = 6;
            packet.y_ = 9;
            if (length == Packets::PacketHeader::size) {
                Packets::PacketHeader header = packet.getHeader();
                std::memcpy(buff, header.serialize().c_str(), length);
                return length;
            } else if (length == packetSize) {
                std::memcpy(buff, packet.serialize().c_str(), packetSize);
                return packet.size;
            } else {
                logger_->warn("Length: {} != packet.size: {}", length, packetSize);
                std::string s(length, 'x');
                std::memcpy(buff, s.c_str(), length);
                return length;
            }
        } else {
            Packets::UDPConnectionPacket packet {};
            // hack to avoid undefined references to static members in variadic templates
            unsigned int packetSize = packet.size;

            packet.action = Packets::UDPConnectionPacket::UDP_CONNECTION_STATE::CONNECT;
            if (length == Packets::PacketHeader::size) {
                Packets::PacketHeader header = packet.getHeader();
                std::memcpy(buff, header.serialize().c_str(), length);
                return length;
            } else if (length == packetSize) {
                std::memcpy(buff, packet.serialize().c_str(), packetSize);
                isConnected_ = true;
                return packet.size;
            } else {
                logger_->warn("Length: {} != packet.size: {}", length, packetSize);
                std::string s(length, 'x');
                std::memcpy(buff, s.c_str(), length);
                return length;
            }
        }
    }

 protected:
    UdpServerShould(): logger_("UdpServerShould"),
        isConnected_ {false},
        someClientAddress_{} {
        somePort_ = 6969;
        someServerSocket_ = 666;
        someClientAddress_.sin_family = AF_INET;
        someClientAddress_.sin_port = htons(250);
        someClientAddress_.sin_addr.s_addr = htonl(4700);

        socketInterfaceMock_.reset(new NiceMock<SocketInterfaceMock>);
        ON_CALL(*socketInterfaceMock_, socket(_, _, _)).WillByDefault(Return(someServerSocket_));
        ON_CALL(*socketInterfaceMock_, bind(_, _, _)).WillByDefault(Return(0));
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
                if (socket <= 0) {
                    return -1;
                } else {
                    return 0;
                }
            }));
        ON_CALL(*socketInterfaceMock_, sendto(_, _, _, _, _, _)).WillByDefault(Invoke(
            [](int, const void*, size_t length, int, const struct sockaddr *, socklen_t) {
                return length;
            }));
         ON_CALL(*socketInterfaceMock_, recvfrom(_, _, _, _, _, _)).WillByDefault(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6)));
    }
    crf::utility::logger::EventLogger logger_;
    bool isConnected_;
    sockaddr_in someClientAddress_;

    int somePort_;
    int someServerSocket_;
    std::shared_ptr<NiceMock<SocketInterfaceMock> > socketInterfaceMock_;
    std::unique_ptr<INetworkServer> sut_;
};

TEST_F(UdpServerShould, throwIfFailedToCreateAndConfigureServerSocket) {
    EXPECT_CALL(*socketInterfaceMock_, socket(_, _, _))
        .WillOnce(Return(-1)).WillOnce(Return(someServerSocket_));
    EXPECT_ANY_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    EXPECT_CALL(*socketInterfaceMock_, bind(_, _, _))
        .WillOnce(Return(-1));
    EXPECT_ANY_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
}

TEST_F(UdpServerShould, notThrowIfEverythingOk) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
}

TEST_F(UdpServerShould, returnFalseIfNotPreviouslyConnected) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->isConnected());
}

TEST_F(UdpServerShould, returnFalseIfAcceptConnectionWithNotSupportedNonBlockingMode) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_FALSE(sut_->acceptConnection(false));
}

TEST_F(UdpServerShould, returnTrueWhenAcceptConnectionAndReturnFalseIfAcceptAgain) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ASSERT_FALSE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
}

TEST_F(UdpServerShould, beAbleToAcceptConnectionAndDisconnectMultipleTimes) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    bool ret = sut_->disconnect();
    ASSERT_TRUE(ret);
    isConnected_ = sut_->isConnected();
    ASSERT_FALSE(isConnected_);
    ASSERT_FALSE(sut_->disconnect());

    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ret = sut_->disconnect();
    ASSERT_TRUE(ret);
    isConnected_ = sut_->isConnected();
    ASSERT_FALSE(isConnected_);

    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());
    ret = sut_->disconnect();
    ASSERT_TRUE(ret);
    isConnected_ = sut_->isConnected();
    ASSERT_FALSE(isConnected_);
}

TEST_F(UdpServerShould, sendsDisconnectMessageWhenDisconnect) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    ASSERT_TRUE(sut_->isConnected());

    // DC packet
    Packets::UDPConnectionPacket dcpacket {};
    dcpacket.action = Packets::UDPConnectionPacket::UDP_CONNECTION_STATE::DISCONNECT;
    std::string bytes;
    bytes.append(dcpacket.getHeader().serialize());
    bytes.append(dcpacket.serialize());

    std::string receivedMsg;
    sockaddr_in receivedClienAddress_{};

    EXPECT_CALL(*socketInterfaceMock_, sendto(_, _, _, _, _, _)).WillOnce(Invoke(
        [&receivedMsg, &receivedClienAddress_](int, const void *message, size_t length, int,
            const struct sockaddr *dest_addr, socklen_t) {
            receivedMsg = std::string(reinterpret_cast<const char*>(message), length);
            receivedClienAddress_.sin_port =
                                    reinterpret_cast<const sockaddr_in*>(dest_addr)->sin_port;
            receivedClienAddress_.sin_addr =
                        reinterpret_cast<const sockaddr_in*>(dest_addr)->sin_addr;
            return length;
        }));
    ASSERT_TRUE(sut_->disconnect());
    ASSERT_EQ(receivedMsg, bytes);
    ASSERT_EQ(receivedClienAddress_.sin_port, someClientAddress_.sin_port);
    ASSERT_EQ(receivedClienAddress_.sin_addr.s_addr, someClientAddress_.sin_addr.s_addr);
}

TEST_F(UdpServerShould, notAllowToSendReceiveIfNotConnected) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    Packets::DummyPacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, packet.serialize()));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(UdpServerShould, sendAndReceiveAppropriateData) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
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

TEST_F(UdpServerShould, returnFalseWhenHeaderIsGarbage) {
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::PacketHeader header;
    std::string buff;
    EXPECT_CALL(*socketInterfaceMock_, recvfrom(_, _, _, _, _, _)).WillRepeatedly(Invoke(
        [] (int, void* buff, size_t length, int, struct sockaddr *, socklen_t *) {
            std::string s(length, 'x');
            std::memcpy(buff, s.c_str(), length);
            return length;
        }));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

/*
 * You can test this demo with netcat:
 *      > netcat localhost 6969
 */
TEST_F(UdpServerShould, DISABLED_demoSenderAndReceiver) {
    int howManyTimes = 1;
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_)));
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

TEST_F(UdpServerShould, sendToReturnsNegativeBytes) {
    EXPECT_CALL(*socketInterfaceMock_, sendto(_, _, _, _, _, _)).WillOnce(Return(-1)).
        WillOnce(Return(1));
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::DummyPacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, packet.serialize()));
}

TEST_F(UdpServerShould, receivesANumberOfBytesSmallerThanPacketHeader) {
    EXPECT_CALL(*socketInterfaceMock_, recvfrom(_, _, _, _, _, _)).
        WillOnce(Return(Packets::PacketHeader::size)).WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).WillOnce(Return(0));
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::PacketHeader header;
    std::string buff;
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(UdpServerShould, receivesNegativerBytesForTheBodyOfTheMessage) {
    EXPECT_CALL(*socketInterfaceMock_, recvfrom(_, _, _, _, _, _)).WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).WillOnce(Return(-1));
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
    Packets::PacketHeader header;
    std::string buff;
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(UdpServerShould, receiveBadFristPacketHeader) {
    EXPECT_CALL(*socketInterfaceMock_, recvfrom(_, _, _, _, _, _)).
        WillOnce(Return(0)).
        WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6))).
        WillOnce(Invoke(
            std::bind(&UdpServerShould::assignDataToTheReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                std::placeholders::_5,
                std::placeholders::_6)));
    ASSERT_NO_THROW(sut_.reset(new UdpServer(somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->acceptConnection());
}
