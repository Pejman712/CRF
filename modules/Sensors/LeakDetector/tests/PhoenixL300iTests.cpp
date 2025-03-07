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

#include "Mocks/Communication/IpcMock.hpp"
#include "Mocks/Communication/SerialCommunicationMock.hpp"

#include "LeakDetector/PhoenixL300i.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::Return;

using crf::communication::serialcommunication::SerialCommunicationMock;
using crf::communication::serialcommunication::SerialCommunication;
using crf::sensors::leakdetector::PhoenixL300i;
using crf::sensors::leakdetector::ILeakDetector;

/*
 * Method from CRC calc from the device manufacturer.
 * Defined in CrcCalcForTest.cpp
 */
unsigned char CalculateCRC(unsigned char* CRCData, uint16_t CRCLen);

class PhoenixL300iShould: public ::testing::Test {
 public:
    int assignTelegramToArg(std::string* buf, int length, std::vector<uint8_t> telegram) {
        if (length < telegram.size()) {
            return -1;
        }
        for (int i = 0; i < telegram.size(); i++) {
            (*buf)[i] = telegram[i];
        }
        return 0;
    }

 protected:
    PhoenixL300iShould(): logger_("PhoenixL300iShould") {
        ipcMock_.reset(new IpcMock);
        sCommMock_.reset(new SerialCommunicationMock());
        expectSendStartCommandAndReplyOK();
        sut_.reset(new PhoenixL300i(ipcMock_, sCommMock_));
        ON_CALL(*sCommMock_, write(_)).WillByDefault(Invoke(
            [](const std::string& s) { return s.size(); }));
    }

    void expectSendStartCommandAndReplyOK() {
        std::vector<uint8_t> expectedTelegram;
        expectedTelegram.push_back(0x02);  // slave response
        expectedTelegram.push_back(0x05);  // LEN = 5
        expectedTelegram.push_back(0x05);  // StwH - FINE
        expectedTelegram.push_back(0x81);  // StwL - FINE, RUNUP
        expectedTelegram.push_back(0x20);  // CmdH - write
        expectedTelegram.push_back(0x01);  // CmdL - start
        uint8_t crc = CalculateCRC(expectedTelegram.data(), expectedTelegram.size());
        expectedTelegram.push_back(crc);

        EXPECT_CALL(*sCommMock_, write(_)).Times(AtLeast(1))
            .WillOnce(::testing::Return(6));
        EXPECT_CALL(*sCommMock_, read(_, _)).Times(AtLeast(1))
            .WillOnce(Invoke(std::bind(
                &PhoenixL300iShould::assignTelegramToArg,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                expectedTelegram)));
    }

    std::vector<uint8_t> prepareSlaveFloatResponse(float value) {
        std::vector<uint8_t> slaveResponse;
        slaveResponse.push_back(0x02);  // slave response
        slaveResponse.push_back(0x09);  // LEN = 9 (5 + 4 bytes for float)
        slaveResponse.push_back(0x05);  // StwH - FINE
        slaveResponse.push_back(0x81);  // StwL - FINE, RUNUP
        slaveResponse.push_back(0x00);  // CmdH - read
        slaveResponse.push_back(0x81);  // CmdL - leakRateMbarLS - hardcoded, doesn't matter that much  // NOLINT
        uint8_t* ptr = reinterpret_cast<uint8_t*>(&value+1);
        do {
            ptr--;
            slaveResponse.push_back(*ptr);
        } while (ptr != reinterpret_cast<uint8_t*>(&value));
        uint8_t crc = CalculateCRC(slaveResponse.data(), slaveResponse.size());
        slaveResponse.push_back(crc);
        return slaveResponse;
    }

    std::vector<uint8_t> prepareSlaveClearErrorResponse(bool success) {
        std::vector<uint8_t> slaveResponse;
        slaveResponse.push_back(0x02);  // slave response
        slaveResponse.push_back(success ? 0x05 : 0x06);  // LEN = 5 for success, 6 for err
        slaveResponse.push_back(success ? 0x05 : 0x8E);  // 0x05 - OK, 0x8[anything] - ERR
        slaveResponse.push_back(0x81);  // StwL - FINE, RUNUP
        slaveResponse.push_back(0x00);  // CmdH - read
        slaveResponse.push_back(0x05);  // CmdL - clearError
        if (!success) {
            slaveResponse.push_back(0x16);  // dec 22 - cmd not allowed
        }
        uint8_t crc = CalculateCRC(slaveResponse.data(), slaveResponse.size());
        slaveResponse.push_back(crc);
        return slaveResponse;
    }

    std::shared_ptr<IpcMock> ipcMock_;
    std::shared_ptr<SerialCommunicationMock> sCommMock_;
    std::unique_ptr<ILeakDetector> sut_;
    crf::utility::logger::EventLogger logger_;  // just to debug
};

TEST_F(PhoenixL300iShould, getExpectedLeakRateFromGoodSlaveResponse) {
    float expectedFloat = 69;
    std::vector<uint8_t> expectedTelegram = prepareSlaveFloatResponse(expectedFloat);
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedTelegram)));
    /*
     * In all tests using floating point comparison this macro should be used
     * rather than normal EXPECT_EQ; It verifies LHS against RHS to be "almost" equal :)
     */
    EXPECT_FLOAT_EQ(expectedFloat, sut_->getLeakRate());
}

TEST_F(PhoenixL300iShould, get0LeakRateFromBadSlaveResponse) {
    float someFloat = 69;
    std::vector<uint8_t> expectedTelegram = prepareSlaveFloatResponse(someFloat);
    expectedTelegram.back() = 0;  // bad CRC in the end
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedTelegram)));
    EXPECT_FLOAT_EQ(0.0, sut_->getLeakRate());
}

TEST_F(PhoenixL300iShould, getExpectedInternalPressureFromGoodSlaveResponse) {
    float expectedFloat = 1234.5678;
    std::vector<uint8_t> expectedTelegram = prepareSlaveFloatResponse(expectedFloat);
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedTelegram)));
    EXPECT_FLOAT_EQ(expectedFloat, sut_->getInternalPressure());
}

TEST_F(PhoenixL300iShould, get0InternalPressureFromBadSlaveResponse) {
    float someFloat = 1234.5678;
    std::vector<uint8_t> expectedTelegram = prepareSlaveFloatResponse(someFloat);
    expectedTelegram.resize(expectedTelegram.size() - 2);  // let's cut-off last 2 bytes
    expectedTelegram[1] = expectedTelegram[1] - 2;
    // and recalc CRC to fake good telegram
    expectedTelegram.back() = CalculateCRC(expectedTelegram.data(), expectedTelegram.size()-1);
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedTelegram)));
    EXPECT_FLOAT_EQ(0.0, sut_->getInternalPressure());
}

TEST_F(PhoenixL300iShould, returnTrueUponClearErrorSuccessAndFalseUponFailure) {
    std::vector<uint8_t> expectedResponseIfSuccess = prepareSlaveClearErrorResponse(true);
    std::vector<uint8_t> expectedResponseIfFailure = prepareSlaveClearErrorResponse(false);
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedResponseIfSuccess)));
    EXPECT_TRUE(sut_->clearError());
    ON_CALL(*sCommMock_, read(_, _)).WillByDefault(Invoke(std::bind(
        &PhoenixL300iShould::assignTelegramToArg,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        expectedResponseIfFailure)));
    EXPECT_FALSE(sut_->clearError());
}

TEST_F(PhoenixL300iShould, DISABLED_getSomeLeakRates) {
    auto serial = std::make_shared<SerialCommunication>("/dev/ttyUSB0", 38400);
    ASSERT_TRUE(serial->initialize());
    sut_.reset(new PhoenixL300i(ipcMock_, serial));
    for (int i=0; i < 10; i++) {
        logger_->info("LeakRate: {}", sut_->getLeakRate());
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(200));
    }
}
