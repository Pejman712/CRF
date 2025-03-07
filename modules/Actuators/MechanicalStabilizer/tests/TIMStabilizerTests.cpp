/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATMotorMock.hpp"
#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"
#include "MechanicalStabilizer/TIMStabilizer/TIMStabilizer.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATMotorMock;
using crf::actuators::mechanicalstabilizer::IMechanicalStabilizer;
using crf::actuators::mechanicalstabilizer::TIMStabilizer;

class TIMStabilizerShould : public ::testing::Test {
 protected:
    TIMStabilizerShould() :
        logger_("TIMStabilizerShould"),
        motor_(new NiceMock<EtherCATMotorMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~TIMStabilizerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        cycleTime_ = 20;
        totalTime_ = 100;
        ON_CALL(*motor_, initialize()).WillByDefault(Return(true));
        ON_CALL(*motor_, inFault()).WillByDefault(Return(false));
        ON_CALL(*motor_, faultReset()).WillByDefault(Return(true));
        ON_CALL(*motor_, shutdown()).WillByDefault(Return(true));
        ON_CALL(*motor_, setMotorRatedCurrent(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMotorRatedTorque(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxCurrent(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxTorque(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setQuickstopDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setProfileAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setProfileDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setModeOfOperation(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, enableOperation()).WillByDefault(Return(true));
        ON_CALL(*motor_, isEnabled()).WillByDefault(Return(true));
        ON_CALL(*motor_, setVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, internalLimitActive()).WillByDefault(Return(true));
        ON_CALL(*motor_, setTorque(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, targetReached()).WillByDefault(Return(true));
        ON_CALL(*motor_, disableOperation()).WillByDefault(Return(true));
        ON_CALL(*motor_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*motor_, stop()).WillByDefault(Return(true));
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<EtherCATMotorMock> motor_;
    std::unique_ptr<IMechanicalStabilizer> sut_;
    int cycleTime_;
    int totalTime_;
};

TEST_F(TIMStabilizerShould, initializeDeinitializeSequence) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfInFault) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, inFault()).WillOnce(Return(true));
    EXPECT_CALL(*motor_, faultReset()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, inFault()).WillOnce(Return(true));
    EXPECT_CALL(*motor_, faultReset()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    EXPECT_CALL(*motor_, inFault()).WillRepeatedly(Return(false));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantShutdown) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, shutdown()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, shutdown()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMotorRatedCurrent) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMotorRatedCurrent(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMotorRatedCurrent(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMotorRatedTorque) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMotorRatedTorque(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMotorRatedTorque(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMaxCurrent) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMaxTorque) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMaxTorque(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxTorque(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetQuickstopDeceleration) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMaxAcceleration) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetMaxDeceleration) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetProfileAcceleration) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetProfileDeceleration) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantEnableOperation) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, enableOperation()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, enableOperation()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItIsntEnabled) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, isEnabled()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, isEnabled()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfItCantSetVelocity) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, setVelocity(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setVelocity(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, failedToInitializeIfNotInternalLimitActive) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    EXPECT_CALL(*motor_, internalLimitActive()).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, internalLimitActive()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, CantOpenIfNotInitialized) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_FALSE(sut_->deactivate());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deactivate());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, CantCloseIfNotInitialized) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_FALSE(sut_->activate());
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, internalLimitActive()).WillOnce(Return(false));
    ASSERT_TRUE(sut_->activate());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, CantCheckIsOpenIfNotInitialized) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_FALSE(sut_->isDeactivated());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->isDeactivated());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, CantCheckIsClosedIfNotInitialized) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_FALSE(sut_->isActivated());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->isActivated().value());
    EXPECT_CALL(*motor_, internalLimitActive()).Times(2).WillRepeatedly(Return(false));
    ASSERT_TRUE(sut_->activate());
    ASSERT_TRUE(sut_->isActivated().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, openStabilizerWatchDogTimeout) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, internalLimitActive()).WillRepeatedly(Return(false));
    ASSERT_TRUE(sut_->activate());
    ASSERT_FALSE(sut_->deactivate());
    ASSERT_FALSE(sut_->isDeactivated().value());
    ASSERT_FALSE(sut_->isActivated().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, closeStabilizerWatchDogTimeout) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, targetReached()).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->activate());
    ASSERT_FALSE(sut_->isActivated().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, ifOpenNotClosed) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_TRUE(sut_->initialize());

    EXPECT_CALL(*motor_, internalLimitActive())
        .Times(3)
        .WillOnce(Return(false))
        .WillOnce(Return(true))
        .WillOnce(Return(false));

    ASSERT_TRUE(sut_->activate());
    ASSERT_TRUE(sut_->deactivate());
    ASSERT_FALSE(sut_->isActivated().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMStabilizerShould, targetReachedButStillOpen) {
    sut_.reset(new TIMStabilizer(motor_, cycleTime_, totalTime_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, internalLimitActive()).Times(2).WillRepeatedly(Return(true));
    ASSERT_FALSE(sut_->activate());
    ASSERT_FALSE(sut_->isActivated().value());
    ASSERT_TRUE(sut_->deinitialize());
}
