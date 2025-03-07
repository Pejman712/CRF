/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <bitset>
#include <memory>

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "XlsAdapter/XlsAdapter.hpp"
#include "XlsAdapter/XlsMessage.hpp"

#include "Mocks/Communication/NetworkClientMock.hpp"

using testing::_;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::communication::networkclient::NetworkClientMock;
using crf::sensors::xlsadapter::IXlsAdapter;
using crf::sensors::xlsadapter::XlsAdapter;
using crf::sensors::xlsadapter::XlsMessage;

class XlsAdapterShould: public ::testing::Test {
 public:
    void pushing_back(std::vector<uint8_t>* vector, float value) {
        for (int i = 0; i < sizeof value; i++) {
            vector->push_back((reinterpret_cast<uint8_t*>(&value))[i]);
        }
    }
    bool subscribeSent_;

 protected:
    XlsAdapterShould(): logger_("XlsAdapterShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        subscribeSent_ = false;
        networkClientMock_.reset(new NiceMock<NetworkClientMock>);
        ON_CALL(*networkClientMock_, connect(_)).WillByDefault(Return(true));
        ON_CALL(*networkClientMock_, disconnect()).WillByDefault(Return(true));
        ON_CALL(*networkClientMock_, send(_, _)).WillByDefault(Invoke(
            [this](const Packets::PacketHeader& header, const std::string& buffer) {
                if (header.type() != Packets::XLS_ADAPTER_PACKET_TYPE) {
                    return false;
                }
                std::bitset<8> enq(buffer[0]);
                std::bitset<8> exp("11111111");
                // check if it's subscription
                if (enq == exp) {
                    subscribeSent_ = true;
                }
                return true;
            }));
        ON_CALL(*networkClientMock_, receive(_, _)).WillByDefault(Invoke(
            [this](Packets::PacketHeader* header, std::string* buffer) {
                return simulateDataReply(header, buffer);
            }));
        sut_.reset(new XlsAdapter(networkClientMock_));
        expectedData_ = {};
    }
    ~XlsAdapterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    bool simulateDataReply(Packets::PacketHeader* header, std::string* buffer) {
        if (header->type != Packets::XLS_ADAPTER_PACKET_TYPE) {
            return false;
        }
        if (header->length <= 0) {
            return false;
        }
        if (!subscribeSent_) {
            return false;
        }
        std::vector<uint8_t> data_block;
        if (repliesReceived_== 0) {
            repliesReceived_++;
            for (int i = 0; i < 12; i++) {
                data_block.push_back(0x00);
            }
            data_block[4] = 0x18;
            data_block[7] = 0x40;
        }
        if (repliesReceived_ == 1) {
            repliesReceived_++;
            data_block.push_back(0x02);
            data_block.push_back(0x00);
            data_block.push_back(0x54);
            data_block.push_back(0x00);
            std::vector<uint32_t> customHeader = {0x0000004D, 0x00000014};
            // Which Table, Number of Averages
            for (int j = 0; j < customHeader.size(); j++) {
                for (int i = 0; i < sizeof(decltype(customHeader)::value_type); i++) {
                    data_block.push_back(((reinterpret_cast<uint8_t*>(&customHeader)[j])[i]));
                }
            }
        } else {
            // add rest of data message
            uint32_t code = 0x00000018;
            for (int i = 0; i < sizeof code; i++) {
                data_block.push_back((reinterpret_cast<uint8_t*>(&code))[i]);
            }
            float data = expectedData_.at(0);
            for (int j = 0; j < 20; j++) {
                pushing_back(&data_block, data);
            }
            code = 0x00000019;
            for (int i = 0; i < sizeof code; i++) {
                data_block.push_back((reinterpret_cast<uint8_t*>(&code))[i]);
            }
            float data2 = expectedData_.at(1);
            for (int j = 0; j < 20; j++) {
                 pushing_back(&data_block, data2);
            }
        }
        std::string stringBuff(data_block.begin(), data_block.end());
        buffer->assign(stringBuff);
        return true;
    }
    bool simulateSetModeOKReply(Packets::PacketHeader* header, std::string* buffer) {
        if (header->type != Packets::XLS_ADAPTER_PACKET_TYPE) {
            return false;
        }
        if (header->length <= 0) {
            return false;
        }
        std::vector<uint8_t> data_block;
        if (repliesReceived_== 0) {
            repliesReceived_++;
            for (int i = 0; i < 12; i++) {
                data_block.push_back(0x00);
            }
            data_block[4] = 0x1C;
            data_block[7] = 0x40;
        }
        if (repliesReceived_ == 1) {
            repliesReceived_++;
            data_block.push_back(0x00);
            data_block.push_back(0x00);
            data_block.push_back(0x0C);
            data_block.push_back(0x00);
            std::vector<uint32_t> customHeader = {0x00000001, 0x0000000A, 0x00000000};
            // Which Table, Number of Averages
            for (int j = 0; j < customHeader.size(); j++) {
                for (int i = 0; i < sizeof(decltype(customHeader)::value_type); i++) {
                    data_block.push_back(((reinterpret_cast<uint8_t*>(&customHeader)[j])[i]));
                }
            }
        }
        std::string stringBuff(data_block.begin(), data_block.end());
        buffer->assign(stringBuff);
        return true;
    }
    bool simulateSetModeErrorReply(Packets::PacketHeader* header, std::string* buffer) {
        if (header->type != Packets::XLS_ADAPTER_PACKET_TYPE) {
            return false;
        }
        if (header->length <= 0) {
            return false;
        }
        std::vector<uint8_t> data_block;
        if (repliesReceived_== 0) {
            repliesReceived_++;
            for (int i = 0; i < 12; i++) {
                data_block.push_back(0x00);
            }
            data_block[4] = 0x1C;
            data_block[7] = 0x40;
        }
        if (repliesReceived_ == 1) {
            repliesReceived_++;
            for (int i = 0; i < 16; i++) {
                data_block.push_back(0x00);
            }
            data_block[0] = 0x02;
            data_block[2] = 0x0C;
            // Data containing CustomHeader + Error Code Info
            data_block[12] = 0x09;
        } else {
            for (int i = 0; i < 24; i++) {
                data_block.push_back(0x00);
            }
            data_block[12] = 0x01;
            data_block[20] = 0x08;
        }
        std::string stringBuff(data_block.begin(), data_block.end());
        buffer->assign(stringBuff);
        return true;
    }
    crf::utility::logger::EventLogger logger_;
    int repliesReceived_ = 0;
    std::shared_ptr<NiceMock<NetworkClientMock> > networkClientMock_;
    std::unique_ptr<IXlsAdapter> sut_;
    std::vector<float> expectedData_;
};

TEST_F(XlsAdapterShould, returnFalseIfInitializedOrDeinitializedTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    // again ...
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(XlsAdapterShould, returnFalseIfUnableToConnect) {
    EXPECT_CALL(*networkClientMock_, connect(_)).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(XlsAdapterShould, returnEmptyVectorIfNotInitialized) {
    ASSERT_EQ(0, sut_->getData().size());
}

TEST_F(XlsAdapterShould, returnFalseWhenTryToChangeModeAndNotInitialized) {
    ASSERT_FALSE(sut_->changeMode(IXlsAdapter::OneWire));
    ASSERT_FALSE(sut_->changeMode(IXlsAdapter::TwoWires));
}

TEST_F(XlsAdapterShould, returnEmptyVectorIfNetworkCommFails) {
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*networkClientMock_, send(_, _)).WillOnce(Return(false))
        .WillRepeatedly(DoDefault());
    EXPECT_CALL(*networkClientMock_, receive(_, _)).WillOnce(Return(false))
        .WillRepeatedly(DoDefault());
    ASSERT_EQ(0, sut_->getData().size());
    ASSERT_EQ(0, sut_->getData().size());
}

TEST_F(XlsAdapterShould, returnEmptyVectorIfNotSubscribed) {
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*networkClientMock_, receive(_, _)).WillOnce(Return(false))
        .WillRepeatedly(DoDefault());
    std::vector<float> receivedData = sut_->getData();
    ASSERT_EQ(0, receivedData.size());
}

TEST_F(XlsAdapterShould, returnExpectedDataIfEverythingOk) {
    expectedData_ = {34.00054, 34.00039};
    ASSERT_TRUE(sut_->initialize());
    std::vector<float> receivedData = sut_->getData();
    ASSERT_FLOAT_EQ(expectedData_[0], receivedData[0]);
    ASSERT_FLOAT_EQ(expectedData_[1], receivedData[1]);
}

TEST_F(XlsAdapterShould, returnFalseIfModeSettingSentAndReplyWithErrors) {
    ASSERT_TRUE(sut_->initialize());
    // With Errors
    EXPECT_CALL(*networkClientMock_, receive(_, _)).WillRepeatedly(Invoke(
        [this](Packets::PacketHeader* header, std::string* buffer) {
            return simulateSetModeErrorReply(header, buffer);
        }));
    ASSERT_FALSE(sut_->changeMode(IXlsAdapter::OneWire));
}

TEST_F(XlsAdapterShould, returnTrueIfModeSettingOneSentAndReplyWithoutErrors) {
    ASSERT_TRUE(sut_->initialize());
    // Without Errors
    EXPECT_CALL(*networkClientMock_, receive(_, _)).WillRepeatedly(Invoke(
        [this](Packets::PacketHeader* header, std::string* buffer) {
            return simulateSetModeOKReply(header, buffer);
        }));
    ASSERT_TRUE(sut_->changeMode(IXlsAdapter::OneWire));
    repliesReceived_ = 0;  // let's try again with different argument
    ASSERT_TRUE(sut_->changeMode(IXlsAdapter::TwoWires));
}

TEST_F(XlsAdapterShould, returnFalseIfModeSettingTwoSentAndNotSubscribedBefore) {
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*networkClientMock_, send(_, _)).WillOnce(Invoke(
        [this](const Packets::PacketHeader& header, const std::string& buffer) {
            if (header.type() != Packets::XLS_ADAPTER_PACKET_TYPE) {
                return false;
            }
            XlsAdapterShould::subscribeSent_ = false;
            return true;
        })).WillRepeatedly(DoDefault());
    ASSERT_FALSE(sut_->changeMode(IXlsAdapter::TwoWires));
}
