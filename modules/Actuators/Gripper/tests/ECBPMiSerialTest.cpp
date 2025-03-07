
/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alessandro Vascelli CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunicationMock.hpp"
#include "Gripper/ECBPMi/ECBPMiSerial.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::Invoke;

class ECBPMiShould: public ::testing::Test {
 protected:
    ECBPMiShould() :
    logger_("ECBPMiShould"),
    initializeResult_(true) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        urdfPath_ = __FILE__;
        urdfPath_ = urdfPath_.substr(0, urdfPath_.find("cpproboticframework"));
        urdfPath_ += "cpproboticframework/modules/Actuators/Gripper/config/ECBPMi/ECBPMi.urdf";
    }

    void SetUp() {
        serialMock_.reset(
            new NiceMock<crf::communication::serialcommunication::SerialCommunicationMock>);
        ON_CALL(*serialMock_, initialize()).WillByDefault([this]() {
            return initializeResult_;
        });
        ON_CALL(*serialMock_, deinitialize()).WillByDefault(Return(true));

        ON_CALL(*serialMock_, read(_, _)).WillByDefault(Return(1));
        ON_CALL(*serialMock_, write(_)).WillByDefault(Return(1));
    }

    ~ECBPMiShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::communication::serialcommunication::SerialCommunicationMock> serialMock_;
    std::string urdfPath_;
    std::unique_ptr<crf::actuators::gripper::ECBPMiSerial> sut_;
    bool initializeResult_;
};

TEST_F(ECBPMiShould, thorwExceptionIfWrongURDF) {
    ASSERT_THROW(sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, "test")),
        std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_)));
}

TEST_F(ECBPMiShould, returnTrueOnInitialize) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    initializeResult_ = true;
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(ECBPMiShould, returnNegativeIfNotInitialized) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    initializeResult_ = false;
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(ECBPMiShould, returnTrueIfAlreadyDeInitialized) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    initializeResult_ = false;
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ECBPMiShould, sendUnlockWhenInitializing) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    const unsigned char unlock[77] = { };
    std::string requestString(reinterpret_cast<const char*>(unlock), 77);

    ON_CALL(*serialMock_, write(requestString)).WillByDefault(Return(1));
    initializeResult_ = true;
    ASSERT_TRUE(serialMock_->write(requestString));
}

TEST_F(ECBPMiShould, sendStandbyWhenDenitializing) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    const unsigned char standBy[7] = { };
    std::string requestString(reinterpret_cast<const char*>(standBy), 7);

    ON_CALL(*serialMock_, write(requestString)).WillByDefault(Return(1));

    initializeResult_ = false;
    ASSERT_TRUE(serialMock_->write(requestString));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ECBPMiShould, returnNegativeIfWriteFails) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));

    const unsigned char command[7] = { };
    std::string requestString(reinterpret_cast<const char*>(command), 7);

    ON_CALL(*serialMock_, write(requestString)).WillByDefault(Return(-1));
    initializeResult_ = false;
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->activate());
    ASSERT_FALSE(sut_->deactivate());
}

TEST_F(ECBPMiShould, sendActivateCommand) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));
    initializeResult_ = true;
    ASSERT_TRUE(sut_->initialize());

    const unsigned char activate[7] = { };
    std::string requestString(reinterpret_cast<const char*>(activate), 7);
    ON_CALL(*serialMock_, write(requestString)).WillByDefault(Return(1));

    ASSERT_TRUE(sut_->activate());
}

TEST_F(ECBPMiShould, sendDeActivateCommand) {
    sut_.reset(new crf::actuators::gripper::ECBPMiSerial(serialMock_, urdfPath_));
    initializeResult_ = true;
    ASSERT_TRUE(sut_->initialize());

    const unsigned char deactivate[7] = { };
    std::string requestString(reinterpret_cast<const char*>(deactivate), 7);
    ON_CALL(*serialMock_, write(requestString)).WillByDefault(Return(1));

    ASSERT_TRUE(sut_->deactivate());
}
