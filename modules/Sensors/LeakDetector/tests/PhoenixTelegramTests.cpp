/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <bitset>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LeakDetector/PhoenixTelegram.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::Return;

using crf::sensors::leakdetector::PhoenixTelegram;

/*
 * Method from CRC calc from the device manufacturer.
 * Defined in CrcCalcForTest.cpp
 */
unsigned char CalculateCRC(unsigned char* CRCData, uint16_t CRCLen);

class PhoenixTelegramShould: public ::testing::Test {
 protected:
    PhoenixTelegramShould(): logger_("PhoenixTelegramShould") {
        allCommands_.push_back(PhoenixTelegram::start);
        allCommands_.push_back(PhoenixTelegram::stop);
        allCommands_.push_back(PhoenixTelegram::leakRateMbarLS);
        allCommands_.push_back(PhoenixTelegram::internalPressure1SelUnit);
        allCommands_.push_back(PhoenixTelegram::internalPressure1Mbar);
    }
    crf::utility::logger::EventLogger logger_;  // just to debug
    PhoenixTelegram sut_;
    std::vector<PhoenixTelegram::Cmd> allCommands_;
};

TEST_F(PhoenixTelegramShould, alwaysStartTelegramWith0x05) {
    for (auto cmd : allCommands_) {
        std::vector<uint8_t> telegram = sut_.makeTelegram(cmd);
        std::bitset<8> enq(telegram[0]);
        EXPECT_EQ(std::bitset<8>("00000101"), enq);
    }
}

TEST_F(PhoenixTelegramShould, alwaysSetSecondByteToTelegramLengthMinus2) {
    for (auto cmd : allCommands_) {
        std::vector<uint8_t> telegram = sut_.makeTelegram(cmd);
        std::bitset<8> len(telegram[1]);
        EXPECT_EQ(std::bitset<8>(telegram.size()-2), len);
    }
}

TEST_F(PhoenixTelegramShould, configureStartStopClearCommandsAsWriteCommands) {
    std::vector<uint8_t> telegram = sut_.makeTelegram(PhoenixTelegram::start);
    std::bitset<8> cmdH(telegram[3]);
    EXPECT_EQ(std::bitset<8>("00100000"), cmdH);
    telegram = sut_.makeTelegram(PhoenixTelegram::stop);
    cmdH = telegram[3];
    EXPECT_EQ(std::bitset<8>("00100000"), cmdH);
    telegram = sut_.makeTelegram(PhoenixTelegram::clearError);
    cmdH = telegram[3];
    EXPECT_EQ(std::bitset<8>("00100000"), cmdH);
}

TEST_F(PhoenixTelegramShould, configureGetterCommandsAsReadCommands) {
    std::vector<uint8_t> telegram = sut_.makeTelegram(PhoenixTelegram::leakRateMbarLS);
    std::bitset<8> cmdH(telegram[3]);
    EXPECT_EQ(std::bitset<8>("00000000"), cmdH);
    telegram = sut_.makeTelegram(PhoenixTelegram::internalPressure1SelUnit);
    cmdH = telegram[3];
    EXPECT_EQ(std::bitset<8>("00000000"), cmdH);
    telegram = sut_.makeTelegram(PhoenixTelegram::internalPressure1Mbar);
    cmdH = telegram[3];
    EXPECT_EQ(std::bitset<8>("00000000"), cmdH);
}

TEST_F(PhoenixTelegramShould, returnFalseIfTelegramBadSlaveRepliedWithErrOrBadCrcAndReturnTrueIfOk) {  // NOLINT
    unsigned int crc;
    /*
     * scenario: telegram too short
     */
    EXPECT_FALSE(sut_.checkSlaveResponse(std::vector<uint8_t>{1, 2}));
    std::vector<uint8_t> slaveResponse;
    slaveResponse.push_back(0x69);  // slave response, should be 0x02, 0x69 is considered some garbage  // NOLINT
    slaveResponse.push_back(0x06);  // LEN = 6 - one DATA byte for err code
    slaveResponse.push_back(0x00);  // StwH - everything OK ...0x8E
    slaveResponse.push_back(0x02);  // StwL - STANDBY ...
    slaveResponse.push_back(0x00);  // CmdH - read
    slaveResponse.push_back(0x81);  // CmdL - leakRateMbarLS, or whatever else ...
    slaveResponse.push_back(0x01);  // let's say CRC failure
    crc = CalculateCRC(slaveResponse.data(), slaveResponse.size());
    slaveResponse.push_back(crc);
    /*
     * scenario: first byte incorrect
     */
    EXPECT_FALSE(sut_.checkSlaveResponse(slaveResponse));
    slaveResponse[0] = 0x02;  // OK, now first byte is OK
    slaveResponse[1] = 0x69;  // buuuut LEN byte does not correspond to the telegram size
    crc = CalculateCRC(slaveResponse.data(), slaveResponse.size()-1);
    slaveResponse.back() = crc;
    /*
     * scenario: LEN incorrect
     */
    EXPECT_FALSE(sut_.checkSlaveResponse(slaveResponse));
    slaveResponse[1] = 0x06;  // good, now we have a correct LEN
    slaveResponse[2] = 0x8E;  // slave replied with ERR
    slaveResponse[6] = 0x0A;  // type of error: cmd doesn't exist
    crc = CalculateCRC(slaveResponse.data(), slaveResponse.size()-1);
    slaveResponse.back() = crc;
    /*
     * scenario: slave replied with ERR
     */
    EXPECT_FALSE(sut_.checkSlaveResponse(slaveResponse));
    slaveResponse[2] = 0x00;  // OK, now slave reply with sth normal
    // buuuut ... we do not recalculate checksum, therefore checksum is BAD ...
    /*
     * scenario: bad checksum
     */
    EXPECT_FALSE(sut_.checkSlaveResponse(slaveResponse));
    crc = CalculateCRC(slaveResponse.data(), slaveResponse.size()-1);
    slaveResponse.back() = crc;  // now CRC is fine
    /*
     * scenario: everything fine
     */
    EXPECT_TRUE(sut_.checkSlaveResponse(slaveResponse));
}

TEST_F(PhoenixTelegramShould, returnGoodFloatFromSlaveResponse) {
        std::vector<uint8_t> slaveResponse;
        float expectedFloat = 69.69;
        slaveResponse.push_back(0x02);  // slave response
        slaveResponse.push_back(0x09);  // LEN = 9 (4 bytes for float)
        slaveResponse.push_back(0x05);  // StwH - FINE
        slaveResponse.push_back(0x81);  // StwL - FINE, RUNUP
        slaveResponse.push_back(0x00);  // CmdH - read
        slaveResponse.push_back(0x81);  // CmdL - leakRateMbarLS
        uint8_t* ptr = reinterpret_cast<uint8_t*>(&expectedFloat+1);
        do {
            ptr--;
            slaveResponse.push_back(*ptr);
        } while (ptr != reinterpret_cast<uint8_t*>(&expectedFloat));
        uint8_t crc = CalculateCRC(slaveResponse.data(), slaveResponse.size());
        slaveResponse.push_back(crc);
        EXPECT_FLOAT_EQ(expectedFloat, sut_.getFloatFromTelegram(slaveResponse));
}
