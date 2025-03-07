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
#include "Mocks/Devices/EtherCATMotorMock.hpp"
#include "Tools/Screwdriver.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATMotorMock;
using crf::devices::tools::Screwdriver;
using crf::devices::tools::ToolValueTypes;

class ScrewdriverShould : public ::testing::Test {
 protected:
    ScrewdriverShould() :
        logger_("ScrewdriverShould"),
        motorInitialized_(false),
        motor_(new NiceMock<EtherCATMotorMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~ScrewdriverShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        ON_CALL(*motor_, initialize()).WillByDefault(Invoke([this]() {
            if (!motorInitialized_) {
                motorInitialized_ = true;
                return true;
            } else {
                return false;
            }
        }));
        ON_CALL(*motor_, setModeOfOperation(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, inFault()).WillByDefault(Return(false));
        ON_CALL(*motor_, faultReset()).WillByDefault(Return(true));
        ON_CALL(*motor_, shutdown()).WillByDefault(Return(true));
        ON_CALL(*motor_, enableOperation()).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxCurrent(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxTorque(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setQuickstopDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setMaxDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setProfileAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, setProfileDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*motor_, deinitialize()).WillByDefault(Invoke([this]() {
            if (motorInitialized_) {
                motorInitialized_ = false;
                return true;
            } else {
                return false;
            }
        }));
    }

    crf::utility::logger::EventLogger logger_;

    bool motorInitialized_;
    std::shared_ptr<EtherCATMotorMock> motor_;
    std::unique_ptr<Screwdriver> sut_;
};

TEST_F(ScrewdriverShould, initializeDeinitializeSequence) {
    sut_.reset(new Screwdriver(motor_, 1));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(ScrewdriverShould, failedToInitialize) {
    sut_.reset(new Screwdriver(motor_, 1));

    EXPECT_CALL(*motor_, setModeOfOperation(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setModeOfOperation(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

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

    EXPECT_CALL(*motor_, shutdown()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, shutdown()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, enableOperation()).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, enableOperation()).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxCurrent(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setQuickstopDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setMaxDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileAcceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillOnce(Return(false));
    ASSERT_FALSE(sut_->initialize());
    EXPECT_CALL(*motor_, setProfileDeceleration(_)).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ScrewdriverShould, notImplementedFunctions) {
    sut_.reset(new Screwdriver(motor_, 1));
    std::string functionString = "velocity";
    bool valueBool = false;
    int valueInt = 1;

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setValue(functionString, valueBool));
    ASSERT_FALSE(sut_->setValue(functionString, valueInt));
    ASSERT_EQ(sut_->getValue(functionString), boost::none);
}

TEST_F(ScrewdriverShould, setValueWatchdogFail) {
    EXPECT_CALL(*motor_, setVelocity(_)).WillRepeatedly(Return(true));
    sut_.reset(new Screwdriver(motor_, 1));
    std::string functionString = "velocity";
    float valueFloat = 0.5;

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setValue(functionString, valueFloat));
    EXPECT_CALL(*motor_, stop()).Times(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_TRUE(sut_->setValue(functionString, valueFloat));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_TRUE(sut_->setValue(functionString, valueFloat));
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    EXPECT_CALL(*motor_, stop()).Times(2);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ScrewdriverShould, setValueNoVelocity) {
    sut_.reset(new Screwdriver(motor_, 1));
    std::string functionString = "position";
    float valueFloat = 0.5;

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setValue(functionString, valueFloat));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ScrewdriverShould, getValues) {
    sut_.reset(new Screwdriver(motor_, 1));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getValueNames()[0], "velocity");
    ASSERT_EQ(sut_->getValueType("velocity"), ToolValueTypes::FLOAT);
    ASSERT_TRUE(sut_->deinitialize());
}




