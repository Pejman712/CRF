/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 * Contributor: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunicationMock.hpp"
#include "RPSensor/AtomtexBDKG24/AtomtexBDKG24.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::Invoke;

class AtomtexBDKG24Should: public ::testing::Test {
 protected:
    AtomtexBDKG24Should() :
        logger_("AtomtexBDKG24Should") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        serialMock_.reset(
            new NiceMock<crf::communication::serialcommunication::SerialCommunicationMock>);
        ON_CALL(*serialMock_, initialize()).WillByDefault(Return(true));
        ON_CALL(*serialMock_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*serialMock_, read(_, _)).WillByDefault(Invoke(
            [this](std::string* buffer, int length) {
                buffer->resize(answerBuffer_.length());
                buffer->assign(answerBuffer_.c_str(), answerBuffer_.length());
                return answerBuffer_.length();
        }));
        ON_CALL(*serialMock_, write(_)).WillByDefault(Invoke(
            [this](const std::string& buff) {
                if (buff[1] == 0x04) {
                    answerBuffer_.resize(29);
                    answerBuffer_.assign(reinterpret_cast<const char*>(
                        answerBufferDoseRateChar), 29);
                } else if (buff[1] == 0x05) {
                    answerBuffer_.resize(8);
                    answerBuffer_.assign(reinterpret_cast<const char*>(
                        answerBufferResetChar), 8);
                }
                return buff.length();
        }));
    }

    ~AtomtexBDKG24Should() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    const unsigned char answerBufferDoseRateChar[29] = {
        0x01, 0x04, 0x18,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x3d, 0x61, 0x11, 0x3f,
        0x00, 0x00, 0x00, 0x00,
        0x3d, 0x61, 0x11, 0x3f,
        0x00, 0x00
    };

    const unsigned char answerBufferResetChar[8] = {
        0x01, 0x05, 0x00, 0x23,
        0xFF, 0x00, 0x00, 0x00
    };

    crf::utility::logger::EventLogger logger_;
    std::string answerBuffer_;

    std::shared_ptr<crf::communication::serialcommunication::SerialCommunicationMock> serialMock_;
    std::unique_ptr<crf::sensors::rpsensor::AtomtexBDKG24> sut_;
};

TEST_F(AtomtexBDKG24Should, returnTrueOnInitializeDeinitializeTest) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(AtomtexBDKG24Should, initializeFailsIfSerialDoesNotWork) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));
    ON_CALL(*serialMock_, initialize()).WillByDefault(Return(false));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(AtomtexBDKG24Should, initializeFailsIfRPSensorDoesNotWork) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));
    ON_CALL(*serialMock_, read(_, _)).WillByDefault(Return(0));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(AtomtexBDKG24Should, returnNegativeIfNotInitialized) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));

    ASSERT_FALSE(sut_->getDoseRate());
    ASSERT_FALSE(sut_->getCumulativeDose());
    ASSERT_FALSE(sut_->resetCumulativeDose());
}

TEST_F(AtomtexBDKG24Should, returnNegativeIfSerialWriteFailsTest) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));
    ON_CALL(*serialMock_, write(_)).WillByDefault(Invoke(
        [this](const std::string& buff) {
            return -1;
    }));

    ASSERT_FALSE(sut_->initialize());

    ASSERT_FALSE(sut_->getDoseRate());
    ASSERT_FALSE(sut_->getCumulativeDose());
    ASSERT_FALSE(sut_->resetCumulativeDose());
}

TEST_F(AtomtexBDKG24Should, returnNegativeIfSerialReadFailsTest) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));
    ON_CALL(*serialMock_, read(_, _)).WillByDefault(Invoke(
        [this](std::string* buffer, int length) {
            return -1;
    }));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->getDoseRate());
    ASSERT_FALSE(sut_->getCumulativeDose());
    ASSERT_FALSE(sut_->resetCumulativeDose());
}

TEST_F(AtomtexBDKG24Should, returnCorrectDataTest) {
    sut_.reset(new crf::sensors::rpsensor::AtomtexBDKG24(serialMock_));
    ASSERT_TRUE(sut_->initialize());

    ASSERT_NEAR(sut_->getDoseRate().value(), 0.0549481, 1e-5);
    ASSERT_NEAR(sut_->getCumulativeDose().value(), 0.0549481, 1e-5);
    ASSERT_TRUE(sut_->resetCumulativeDose());
}
