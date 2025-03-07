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
#include "EventLogger/EventLogger.hpp"
#include "XlsAdapter/XlsMessage.hpp"

using crf::sensors::xlsadapter::XlsMessage;

class XlsMessageShould: public ::testing::Test {
 public:
    std::string createDataAnswer(float data, float data2) {
        std::vector<uint8_t> telegram;
        for (int i=0; i < 16; i++) {
            telegram.push_back(0x00);
        }
        telegram[4] = 0x18;
        telegram[7] = 0x40;
        telegram[12] = 0x0002;
        telegram[14] = 0x002C;
        std::vector<uint32_t> customHeader = {0x0000004D, 0x0000000A};  // Which Table, Number of Averages  // NOLINT
        for (int j= 0; j < customHeader.size(); j++) {
            for (int i= 0; i < sizeof(decltype(customHeader)::value_type); i++) {
                telegram.push_back(((reinterpret_cast<uint8_t*>(&customHeader)[j])[i]));
            }
        }
        uint32_t code = 0x00000018;
        for (int i= 0; i < sizeof code; i++) {
            telegram.push_back((reinterpret_cast<uint8_t*>(&code))[i]);
        }
        int Averages = static_cast<int>(customHeader)[1];
        for (int j= 0; j < Averages; j++) {
            for (int i= 0; i < sizeof data; i++) {
                telegram.push_back((reinterpret_cast<uint8_t*>(&data))[i]);
            }
        }
        code = 0x00000019;
        for (int i= 0; i < sizeof code; i++) {
            telegram.push_back((reinterpret_cast<uint8_t*>(&code))[i]);
        }
        for (int j= 0; j < Averages; j++) {
            for (int i= 0; i < sizeof data2; i++) {
                telegram.push_back((reinterpret_cast<uint8_t*>(&data2))[i]);
            }
        }
        std::string stringBuff(telegram.begin(), telegram.end());
        return stringBuff;
    }
    std::string simulateSetModeOKReply() {
        std::vector<uint8_t> telegram;
        for (int i = 0; i < 28; i++) {
            telegram.push_back(0x00);
        }
        telegram[4]= 0x1C;
        telegram[7]= 0x00;
        telegram[12] = 0x00;
        telegram[14] = 0x0C;
        std::vector<uint32_t> customHeader = {0x00000001, 0x0000000A, 0x00000000};  // Which Table, Number of Averages  //NOLINT
        for (int j= 0; j < customHeader.size(); j++) {
            for (int i= 1; i < sizeof(decltype(customHeader)::value_type); i++) {
                telegram[15+j*sizeof(decltype(customHeader)::value_type)+i]
                    = (((reinterpret_cast<uint8_t*>(&customHeader)[j])[i]));
            }
        }
        std::string stringBuff(telegram.begin(), telegram.end());
        return stringBuff;
    }

 protected:
    XlsMessageShould(): logger_("XlsMessageShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~XlsMessageShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(XlsMessageShould, returnAppropriateHeader) {
    XlsMessage message;
    Packets::PacketHeader header = message.getXlsPacketHeader();
    ASSERT_EQ(Packets::XLS_ADAPTER_PACKET_TYPE, header.type);
}


TEST_F(XlsMessageShould, haveProperTelegramStructure) {
    XlsMessage message;
    std::string buffer = message.createSubscribeMessage();
    // check that Endianness of first byte is correct
    std::bitset<8> enq(buffer[0]);
    EXPECT_EQ(std::bitset<8>("11111111"), enq);
    // check that Telegram contains data on packInfo_ = 0x40
    std::bitset<8> enq2(buffer[7]);
    EXPECT_EQ(std::bitset<8>("01000000"), enq2);
}

TEST_F(XlsMessageShould, setBitsCorrectlyInSetModeMessage) {
    XlsMessage message;
    uint32_t mode = 0x00000003;
    std::string buffer = message.createSetModeMessage(mode);
    // check that Code table is S Codes (=3)
    std::bitset<8> enq(buffer[20]);
    EXPECT_EQ(std::bitset<8>("00000011"), enq);
    // check that Code is 90 (Measurement Mode)
    std::bitset<8> modeset(buffer[28]);
    EXPECT_EQ(std::bitset<8>("01011010"), modeset);
    // check that Code is the desired one
    std::bitset<8> code(buffer[32]);
    std::bitset<8> enp((reinterpret_cast<uint8_t*>(&mode))[0]);
    EXPECT_EQ(enp, code);
    mode = 0x00000001;
    XlsMessage message2;
    buffer = message2.createSetModeMessage(mode);
    // check that Code is the desired one
    std::bitset<8> exp(buffer[32]);
    std::bitset<8> anothermode((reinterpret_cast<uint8_t*>(&mode))[0]);
    EXPECT_EQ(exp, anothermode);
}

TEST_F(XlsMessageShould, returnSubscribeStringWithCorrectLength) {
    XlsMessage message;
    int lengthSubscribeCommand = 20;
    std::string buffer = message.createSubscribeMessage();
    ASSERT_EQ(lengthSubscribeCommand, buffer.size());
}

TEST_F(XlsMessageShould, returnSetModeStringWithCorrectLength) {
    XlsMessage message2;
    std::string buffer2 = message2.createSetModeMessage(0x00000003);
    int lengthSetCommand = 12+4+(3*4)+8;
    ASSERT_EQ(lengthSetCommand, buffer2.size());
}

TEST_F(XlsMessageShould, returnHeaderBytesToBeRequestedIfNotAllReceived) {
    XlsMessage message;
    std::string buffer;
    buffer.resize(12);
    buffer[4] = 0x20;
    buffer[7] = 0x40;
    ASSERT_EQ((int)buffer[4]-buffer.size(), message.getBytesToBeRequested(buffer));
}

TEST_F(XlsMessageShould, returnZeroIfAllBytesToBeRequestedAreReceived) {
    XlsMessage message;
    std::string buffer = message.createSubscribeMessage();
    ASSERT_EQ(0, message.getBytesToBeRequested(buffer));
}

TEST_F(XlsMessageShould, returnOkIfSetModeSuccess) {
    XlsMessage message;
    std::string buffer = simulateSetModeOKReply();
    ASSERT_TRUE(message.checkReply(buffer));
}

TEST_F(XlsMessageShould, returnFalseIfSetModeReplyReturnsErrorMessage) {
    XlsMessage message;
    std::string buffer = message.createSetModeMessage(0x00000001);
    buffer[12] = 0x01;  // One Error Received
    buffer[14] = 0x0C;
    buffer[24] = 0x01;
    buffer.resize(static_cast<int>(buffer)[4] +
        (static_cast<int>(buffer)[12]*static_cast<int>(buffer)[14]));
    ASSERT_FALSE(message.checkReply(buffer));
    buffer[12] = 0x02;  // More than one error
    buffer[14] = 0x0C;
    buffer[24] = 0x09;  // Code identifying error
    buffer.resize(static_cast<int>(buffer)[4] +
        (static_cast<int>(buffer)[12]*static_cast<int>(buffer)[14]));
    ASSERT_FALSE(message.checkReply(buffer));
}

TEST_F(XlsMessageShould, returnInterpretedDataCorrectlyForDesiredMCodes) {
    XlsMessage message;
    float expectedData = 34.00054;
    float expectedData2 = 44.00054;
    // SimulateAnswer
    std::string buffer = createDataAnswer(expectedData, expectedData2);
    std::vector<float> data = message.interpretData(buffer);
    ASSERT_FLOAT_EQ(data[0], expectedData);
    ASSERT_FLOAT_EQ(data[1], expectedData2);
}
