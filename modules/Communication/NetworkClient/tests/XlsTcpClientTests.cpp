/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "CommUtility/PacketTypes.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/XlsTcpClient.hpp"

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
using crf::communication::networkclient::XlsTcpClient;
using crf::utility::commutility::SocketInterfaceMock;

class XlsTcpClientShould: public TcpClientShould {
 protected:
    XlsTcpClientShould(): TcpClientShould(), logger_("XlsTcpClientShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    void SetUp() override {
        // I cannot use this macro in the constructor
        ASSERT_NO_THROW(sut_.reset(new XlsTcpClient(someAddr_, somePort_, socketInterfaceMock_)));
    }
    ~XlsTcpClientShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(XlsTcpClientShould, returnFalseIfUnsupportedPacket) {
    ASSERT_TRUE(sut_->connect());
    Packets::DummyPacket unsupportedPacket{};
    Packets::PacketHeader unsupportedHeader = unsupportedPacket.getHeader();
    ASSERT_FALSE(sut_->send(unsupportedHeader, unsupportedPacket.serialize()));
    std::string unsupportedBuff;
    ASSERT_FALSE(sut_->receive(&unsupportedHeader, &unsupportedBuff));
}

TEST_F(XlsTcpClientShould, notAllowToSendReceiveIfNotConnected) {
    std::string buff("xxx");
    Packets::PacketHeader header{};
    header.type = Packets::XLS_ADAPTER_PACKET_TYPE;
    header.length = buff.size();
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(XlsTcpClientShould, returnFalseIfFailedToSendOrReceive) {
    EXPECT_CALL(*socketInterfaceMock_, send(_, _, _, _)).WillRepeatedly(Return(-1));
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Return(-1));
    ASSERT_TRUE(sut_->connect());
    std::string buff("someCrazyDatagram");
    Packets::PacketHeader header{};
    header.type = Packets::XLS_ADAPTER_PACKET_TYPE;
    header.length = buff.size();
    ASSERT_FALSE(sut_->send(header, buff));
    ASSERT_FALSE(sut_->receive(&header, &buff));
}

TEST_F(XlsTcpClientShould, returnTrueIfSucceedToSend) {
    ASSERT_TRUE(sut_->connect());
    std::string buff("someCrazyDatagram");
    Packets::PacketHeader header{};
    header.type = Packets::XLS_ADAPTER_PACKET_TYPE;
    header.length = buff.size();
    ASSERT_TRUE(sut_->send(header, buff));
}

TEST_F(XlsTcpClientShould, returnTrueAndReceiveAppropriateData) {
    std::string expectedData("xxxyyyzzz");
    EXPECT_CALL(*socketInterfaceMock_, recv(_, _, _, _)).WillRepeatedly(Invoke(
        [expectedData](int, void* buff, size_t length, int) {
            std::memcpy(buff, expectedData.data(), expectedData.size());
            return length;
        }));
    ASSERT_TRUE(sut_->connect());
    Packets::PacketHeader header{};
    header.type = Packets::XLS_ADAPTER_PACKET_TYPE;
    header.length = expectedData.size();
    std::string buff;
    ASSERT_TRUE(sut_->receive(&header, &buff));
    ASSERT_EQ(expectedData, buff);
}
