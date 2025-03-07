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
#include "Shielding/IShielding.hpp"
#include "Shielding/RadioactiveSourceShielding/RadioactiveSourceShielding.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATMotorMock;

class RadioactiveSourceShieldingShould : public ::testing::Test {
 protected:
    RadioactiveSourceShieldingShould() :
        logger_("RadioactiveSourceShieldingShould"),
        motor_(new NiceMock<EtherCATMotorMock>) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~RadioactiveSourceShieldingShould() {
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
        ON_CALL(*motor_, setProfileVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setPosition(_, _)).WillByDefault(Return(true));
        ON_CALL(*motor_, targetReached()).WillByDefault(Return(true));
        ON_CALL(*motor_, disableOperation()).WillByDefault(Return(true));
        ON_CALL(*motor_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*motor_, stop()).WillByDefault(Return(true));
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<EtherCATMotorMock> motor_;
    std::unique_ptr<crf::actuators::shielding::IShielding> sut_;
    int cycleTime_;
    int totalTime_;
};

TEST_F(RadioactiveSourceShieldingShould, initializeDeinitializeSequence) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfInFault) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

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


TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantShutdown) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, shutdown()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, shutdown()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMotorRatedCurrent) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMotorRatedCurrent(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMotorRatedCurrent(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMotorRatedTorque) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMotorRatedTorque(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMotorRatedTorque(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMaxCurrent) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMaxTorque) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMaxTorque(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxTorque(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetQuickstopDeceleration) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMaxAcceleration) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetMaxDeceleration) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetProfileAcceleration) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetProfileDeceleration) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetProfileVelocity) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setProfileVelocity(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileVelocity(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, failedToInitializeIfItCantSetModeOfOperation) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    EXPECT_CALL(*motor_, setModeOfOperation(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setModeOfOperation(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, CantOpenIfNotInitialized) {
    EXPECT_CALL(*motor_, getPosition()).WillRepeatedly(Return(-4000));
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->open());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->open());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, CantCloseIfNotInitialized) {
    EXPECT_CALL(*motor_, getPosition()).WillRepeatedly(Return(-527557));
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->close());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->close());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, CantCheckIsOpenIfNotInitialized) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->isOpen());
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, getPosition()).WillOnce(Return(-527557));
    ASSERT_TRUE(sut_->isOpen().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, CantCheckIsClosedIfNotInitialized) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->isClosed());
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, getPosition()).WillOnce(Return(-4000));
    ASSERT_TRUE(sut_->isClosed().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, ReturnNotOpenIfWrongPosition) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->isOpen());
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, getPosition()).WillOnce(Return(5000));
    ASSERT_FALSE(sut_->isOpen().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, ReturnNotCloseIfWrongPosition) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_FALSE(sut_->isClosed());
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, getPosition()).WillOnce(Return(-527557));
    ASSERT_FALSE(sut_->isClosed().value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, openStabilizerWatchDogTimeout) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, targetReached()).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->open());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RadioactiveSourceShieldingShould, closeStabilizerWatchDogTimeout) {
    sut_.reset(new crf::actuators::shielding::RadioactiveSourceShielding(motor_, cycleTime_,
        totalTime_));

    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*motor_, targetReached()).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->close());
    ASSERT_TRUE(sut_->deinitialize());
}
