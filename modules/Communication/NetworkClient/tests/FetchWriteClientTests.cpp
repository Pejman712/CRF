/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/FetchWritePacket.hpp"
#include "NetworkClient/FetchWriteClient.hpp"

#include "Mocks/Utility/DummyPacket.hpp"
#include "Mocks/Utility/SocketInterfaceMock.hpp"
#include "TcpClientTests.hpp"

using ::testing::_;
using ::testing::AtLeast;
using ::testing::DoDefault;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Ne;

using crf::communication::networkclient::INetworkClient;
using crf::communication::networkclient::FetchWriteClient;
using crf::utility::commutility::SocketInterfaceMock;


class FetchWriteClientShould: public TcpClientShould {
 public:
    int assignAckToReceiver(int, void* buff, size_t length, int, uint8_t err) {
        logger_->debug("length: {}", length);
        if (length != 16) {
            logger_->warn("Asked to receive: {} bytes, but ACK should be 16 bytes", length);
            return -1;
        }
        /*
         * Technically ACK should have similar structure to the FETCH/WRITE telegram
         * but meeh ... I only check the fail bit anyway ...
         * Maybe someone can work on it later a little bit?
         */
        std::memset(buff, 0, length);
        (reinterpret_cast<uint8_t*>(buff))[8] = err;
        return length;
    }
    int assignDataToReceiver(int, void* buff, size_t length, int) {
        logger_->debug("length: {}", length);
        for (int i = 0; i < length; i++) {
            (reinterpret_cast<uint8_t*>(buff))[i] = i+1;
        }
        return length;
    }

 protected:
    FetchWriteClientShould(): TcpClientShould(), logger_("FetchWriteClientShould") {
        ON_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillByDefault(Invoke(
            std::bind(&FetchWriteClientShould::assignAckToReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4,
                0)));
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(FetchWriteClientShould, notThrowIfEverythingOk) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
}

TEST_F(FetchWriteClientShould, returnFalseIfUnsupportedPacket) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::DummyPacket unsupportedPacket{};
    Packets::PacketHeader unsupportedHeader = unsupportedPacket.getHeader();
    ASSERT_FALSE(sut_->send(unsupportedHeader, unsupportedPacket.serialize()));
    std::string unsupportedBuff;
    ASSERT_FALSE(sut_->receive(&unsupportedHeader, &unsupportedBuff));
}

TEST_F(FetchWriteClientShould, notAllowToSendReceiveIfNotConnected) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    Packets::FetchWritePacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, packet.serialize()));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(FetchWriteClientShould, notAllowToSendReceiveIfFailedToDeserializePacket) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::FetchWritePacket packet;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff;
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(FetchWriteClientShould, notAllowToSendReceiveIfDataLengthIsWrong) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::FetchWritePacket packet;
    packet.dataLength_ = 0;
    Packets::PacketHeader header = packet.getHeader();
    std::string buff = packet.serialize();
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
    packet.dataLength_ = 3;
    packet.data_ = std::vector<uint8_t>({1, 2, 3});
    buff = packet.serialize();
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(FetchWriteClientShould, failToSendReceiveIfAckContainsError) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::FetchWritePacket packet;
    packet.dataLength_ = 4;
    packet.data_ = std::vector<uint8_t>({1, 2, 3, 4});
    Packets::PacketHeader header = packet.getHeader();
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Invoke(
        std::bind(&FetchWriteClientShould::assignAckToReceiver, this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4,
            2)));
    std::string buff = packet.serialize();
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(FetchWriteClientShould, returnTrueAndSuccessfullyWriteFetchData) {
    ASSERT_NO_THROW(sut_.reset(new FetchWriteClient(someAddr_, somePort_, socketInterfaceMock_)));
    ASSERT_TRUE(sut_->connect());
    Packets::FetchWritePacket packet;
    packet.dataBlockNumber_ = 43;
    packet.startAddress_ = 3;
    packet.dataLength_ = 4;
    uint16_t dividedDataLength = 2;
    packet.data_ = std::vector<uint8_t>({0, 0, 0, 0});
    Packets::PacketHeader header = packet.getHeader();
    std::string buff = packet.serialize();
    uint8_t sentTelegram[16];
    EXPECT_CALL(*socketInterfaceMock_, send(_, _, _, _)).WillOnce(Invoke(
        [&sentTelegram](int, const void* data, size_t length, int) {
            std::memcpy(sentTelegram, data, length);
            return length;
        })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->send(header, buff));
    ASSERT_EQ(sentTelegram[9], packet.dataBlockNumber_);
    ASSERT_EQ(sentTelegram[10], (reinterpret_cast<uint8_t*>(&packet.startAddress_))[1]);
    ASSERT_EQ(sentTelegram[11], (reinterpret_cast<uint8_t*>(&packet.startAddress_))[0]);
    ASSERT_EQ(sentTelegram[12], (reinterpret_cast<uint8_t*>(&dividedDataLength))[1]);
    ASSERT_EQ(sentTelegram[13], (reinterpret_cast<uint8_t*>(&dividedDataLength))[0]);
    EXPECT_CALL(*socketInterfaceMock_, send(_, _, _, _)).WillOnce(Invoke(
        [&sentTelegram](int, const void* data, size_t length, int) {
            std::memcpy(sentTelegram, data, length);
            return length;
        })).WillRepeatedly(DoDefault());
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillOnce(DoDefault())
        .WillRepeatedly(Invoke(
            std::bind(&FetchWriteClientShould::assignDataToReceiver, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4)));
    ASSERT_TRUE(sut_->receive(&header, &buff));
    ASSERT_EQ(sentTelegram[9], packet.dataBlockNumber_);
    ASSERT_EQ(sentTelegram[10], (reinterpret_cast<uint8_t*>(&packet.startAddress_))[1]);
    ASSERT_EQ(sentTelegram[11], (reinterpret_cast<uint8_t*>(&packet.startAddress_))[0]);
    ASSERT_EQ(sentTelegram[12], (reinterpret_cast<uint8_t*>(&dividedDataLength))[1]);
    ASSERT_EQ(sentTelegram[13], (reinterpret_cast<uint8_t*>(&dividedDataLength))[0]);
    packet.deserialize(buff);
    ASSERT_EQ(packet.data_, std::vector<uint8_t>({1, 2, 3, 4}));
}

TEST(FetchWritePacketShould, serializeAndDeserializeCorrectly) {
    Packets::FetchWritePacket expectedPacket;
    expectedPacket.data_ = std::vector<uint8_t>({1, 2, 3, 4});
    expectedPacket.dataLength_ = 4;
    Packets::FetchWritePacket deserializedPacket;

    ASSERT_TRUE(deserializedPacket.deserialize(expectedPacket.serialize()));
    ASSERT_EQ(expectedPacket, deserializedPacket);
    expectedPacket.data_ = std::vector<uint8_t>();
    expectedPacket.dataLength_ = 0;
    ASSERT_TRUE(deserializedPacket.deserialize(expectedPacket.serialize()));
    ASSERT_EQ(expectedPacket, deserializedPacket);
    ASSERT_TRUE(expectedPacket.deserialize(deserializedPacket.serialize()));
    ASSERT_EQ(expectedPacket, deserializedPacket);
}

TEST(FetchWritePacketShould, returnWellFormedJsonString) {
    Packets::FetchWritePacket packet;
    ASSERT_NO_THROW(nlohmann::json::parse(packet.toJSONString()));
    packet.data_ = std::vector<uint8_t>({1});
    ASSERT_NO_THROW(nlohmann::json::parse(packet.toJSONString()));
    packet.data_ = std::vector<uint8_t>({1, 2, 3, 4, 5, 6});
    ASSERT_NO_THROW(nlohmann::json::parse(packet.toJSONString()));
}
