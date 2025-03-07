/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "LinearStage/CANOpenLinearStage/CANOpenLinearStage.hpp"
#include "CANOpenDevices/CANOpenMotors/CANOpenMotorMock.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;

using crf::actuators::linearstage::LinearStageConfiguration;
using crf::actuators::linearstage::CANOpenLinearStage;
using crf::devices::canopendevices::CANOpenMotorMock;

class CANOpenLinearStageShould: public ::testing::Test {
 protected:
    CANOpenLinearStageShould():
        logger_("CANOpenLinearStageShould"),
        motor_(new NiceMock<CANOpenMotorMock>),
        configuration_(new LinearStageConfiguration) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        goodConfiguration_["rotationToLinearRatio"] = 1;
        goodConfiguration_["minimumPosition"] = 0;
        goodConfiguration_["maximumPosition"] = 23;
        goodConfiguration_["maximumVelocity"] = 290;
        goodConfiguration_["maximumAcceleration"] = 20;
        goodConfiguration_["maximumDeceleration"] = 21;

        configuration_.reset(new LinearStageConfiguration);
        configuration_->parse(goodConfiguration_);
    }


    ~CANOpenLinearStageShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<CANOpenMotorMock> motor_;
    std::shared_ptr<LinearStageConfiguration> configuration_;
    std::unique_ptr<CANOpenLinearStage> sut_;

    nlohmann::json goodConfiguration_;
};

TEST_F(CANOpenLinearStageShould, initializeDeinitializeSequence) {
    EXPECT_CALL(*motor_, initialize()).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, deinitialize()).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(2).WillRepeatedly(Return(false));
    EXPECT_CALL(*motor_, faultReset()).Times(0);
    EXPECT_CALL(*motor_, shutdown()).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(2).WillRepeatedly(Return(true));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_FALSE(sut_->getActualVelocity());
    ASSERT_FALSE(sut_->getActualPosition());
    ASSERT_FALSE(sut_->setTargetPosition(250));
    ASSERT_FALSE(sut_->setTargetVelocity(250));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(CANOpenLinearStageShould, resetFaultsDuringInitialization) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, deinitialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(1).WillRepeatedly(Return(true));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CANOpenLinearStageShould, deinitializedInDestructor) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, deinitialize()).Times(1).WillRepeatedly(Return(false));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(1).WillRepeatedly(Return(true));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfMotorDoesnt) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtResetFault) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtEnable) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtSetProfileAcceleration) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtSetProfileDecceleration) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtSetProfileVelocity) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, doesntInitializeIfCANtSetPositionLimits) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, faultReset()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(1).WillRepeatedly(Return(false));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CANOpenLinearStageShould, correctlySetAndGets) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, deinitialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(false));
    EXPECT_CALL(*motor_, faultReset()).Times(0);
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(1).WillRepeatedly(Return(true));

    EXPECT_CALL(*motor_, setVelocity(250)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPosition(250, false)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, getVelocity()).Times(1).WillRepeatedly(Return(250));
    EXPECT_CALL(*motor_, getPosition()).Times(1).WillRepeatedly(Return(250));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_TRUE(sut_->initialize());
    sut_->setTargetPosition(250);
    sut_->setTargetVelocity(250);

    ASSERT_NEAR(sut_->getActualPosition().get(), 250, 1e-3);
    ASSERT_NEAR(sut_->getActualVelocity().get(), 250, 1e-3);
}

TEST_F(CANOpenLinearStageShould, returnsBoostNoneIfReadFromMotorFails) {
    EXPECT_CALL(*motor_, initialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, deinitialize()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, inFault()).Times(1).WillRepeatedly(Return(false));
    EXPECT_CALL(*motor_, faultReset()).Times(0);
    EXPECT_CALL(*motor_, shutdown()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, enableOperation()).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setProfileVelocity(_)).Times(1).WillRepeatedly(Return(true));
    EXPECT_CALL(*motor_, setPositionLimits(_)).Times(1).WillRepeatedly(Return(true));

    EXPECT_CALL(*motor_, getVelocity()).Times(1).WillRepeatedly(Return(std::nullopt));
    EXPECT_CALL(*motor_, getPosition()).Times(1).WillRepeatedly(Return(std::nullopt));

    sut_.reset(new CANOpenLinearStage(motor_, configuration_));
    ASSERT_TRUE(sut_->initialize());

    ASSERT_FALSE(sut_->getActualPosition());
    ASSERT_FALSE(sut_->getActualVelocity());
}
