/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

#include "Radars/FraunhoferRadar/FraunhoferRadar.hpp"

#include "Mocks/Communication/SerialCommunicationMock.hpp"

using crf::sensors::fraunhoferradar::FraunhoferRadar;
using crf::sensors::fraunhoferradar::IRadar;
using crf::communication::serialcommunication::SerialCommunicationMock;

using testing::_;
using testing::Invoke;
using testing::Return;
using testing::NiceMock;
using testing::A;
using testing::InSequence;

class FraunhoferRadarShould: public ::testing::Test {
 protected:
    FraunhoferRadarShould():
      testAddress_(__FILE__),
      serialCommunicationMock_(new NiceMock<SerialCommunicationMock>),
      logger_("FraunhoferRadarShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testAddress_ = testAddress_.substr(0, testAddress_.find("tests/"));
        testAddress_ += "tests/Configurations/";
        std::ifstream param(testAddress_ + "Sensors/Radar/RadarConfig.json");
        param >> parameters_;
        ON_CALL(*serialCommunicationMock_, initialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*serialCommunicationMock_, deinitialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*serialCommunicationMock_, read(_, _)).WillByDefault(Invoke(
            [this](std::string* buff, int length) {
              *buff = "A";
              return 1;
            }));
    }
    ~FraunhoferRadarShould() {
        logger_->info("{} END with {}",
          testing::UnitTest::GetInstance()->current_test_info()->name(),
          testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<IRadar> sut_;
    nlohmann::json parameters_;
    std::string testAddress_;
    std::shared_ptr<SerialCommunicationMock> serialCommunicationMock_;
    std::string mockString_;
};

TEST_F(FraunhoferRadarShould, returnEmptyScanIfNotInitialized) {
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    auto radarData = sut_->getFrame();
    ASSERT_TRUE(radarData.empty());
}

TEST_F(FraunhoferRadarShould, returnExceptionIfCorruptedRadarConfigFile) {
    std::ifstream param(testAddress_ + "Sensors/Radar/RadarConfigCorrupted.json");
    param >> parameters_;
    ASSERT_THROW(sut_.reset(
      new FraunhoferRadar(parameters_, serialCommunicationMock_)), std::invalid_argument);
}

TEST_F(FraunhoferRadarShould, returnFalseIfSerialPortCouldNotInitialize) {
    EXPECT_CALL(*serialCommunicationMock_, initialize()).WillOnce(::testing::Return(false));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FraunhoferRadarShould, returnFalseIfPLL_RampLengthParameterIsTooHigh) {
    std::ifstream param(testAddress_ + "Sensors/Radar/RadarConfigWrongParameters.json");
    param >> parameters_;
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FraunhoferRadarShould, returnFalseIfBandWidthParameterIsTooHigh) {
    std::ifstream param(testAddress_ + "Sensors/Radar/RadarConfigWrongParameters2.json");
    param >> parameters_;
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FraunhoferRadarShould, returnFalseIfCannotReadFromSerial) {
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(
      ::testing::Return(-1));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FraunhoferRadarShould, returnCorrectNyquistFrequency) {
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_NEAR(sut_->getMaxObservationFrequency(), 9.25, 0.01);
}

TEST_F(FraunhoferRadarShould, returnFalseIfInitializedOrDeinitializedTwice) {
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;}));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(FraunhoferRadarShould, throwRuntimeErrorIfRadarBusy) {
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "2"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "3"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;}));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_THROW(sut_->getFrame(), std::runtime_error);
}

TEST_F(FraunhoferRadarShould, throwRuntimeErrorIfCouldNotStartRamp) {
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "3"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;}));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_THROW(sut_->getFrame(), std::runtime_error);
}

TEST_F(FraunhoferRadarShould, throwRuntimeErrorIfExpectedByteAmountDidNotMatch) {
    InSequence s;
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "2"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;}));
    EXPECT_CALL(*serialCommunicationMock_, read(_, _))
      .Times(20)
      .WillRepeatedly(Invoke([this](std::string* buff, int length) {*buff = 1; return 2;}));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_THROW(sut_->getFrame(), std::runtime_error);
}

TEST_F(FraunhoferRadarShould, returnCorectScanSize) {
    InSequence s;
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "1"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "0"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "2"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\r"; return 1;})).WillOnce(Invoke(
      [this](std::string* buff, int length) {*buff = "\n"; return 1;}));
    EXPECT_CALL(*serialCommunicationMock_, read(_, _))
      .Times(19)
      .WillRepeatedly(Invoke([this](std::string* buff, int length) {
        mockString_.resize(4095); *buff = mockString_; return mockString_.length();}));
    EXPECT_CALL(*serialCommunicationMock_, read(_, _)).WillOnce(Invoke(
      [this](std::string* buff, int length) {
        mockString_.resize(2195); *buff = mockString_; return mockString_.length();}));
    sut_.reset(new FraunhoferRadar(parameters_, serialCommunicationMock_));
    ASSERT_TRUE(sut_->initialize());
    auto radarData = sut_->getFrame();
    ASSERT_EQ(radarData.size(), 10);
    ASSERT_EQ(radarData[0].size(), 4000);
}
