/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "LinearStage/EtherCATLinearActuator/EtherCATLinearActuator.hpp"
#include "EtherCATDevices/EtherCATMotorMockConfiguration.hpp"

#include "EventLogger/EventLogger.hpp"

using crf::actuators::linearactuator::EtherCATLinearActuator;
using crf::devices::ethercatdevices::EtherCATMotorMockConfiguration;

using testing::_;

class EtherCATLinearActuatorTests: public ::testing::Test {
 protected:
    EtherCATLinearActuatorTests():
        logger_("EtherCATLinearActuatorTests") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~EtherCATLinearActuatorTests() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        motor_ = std::make_shared<EtherCATMotorMockConfiguration>(motorID_);
        motor_->configureMock();
        sut_ = std::make_unique<EtherCATLinearActuator>(motor_);

        motor_->initialize();
    }

    std::shared_ptr<EtherCATMotorMockConfiguration> motor_;
    std::unique_ptr<EtherCATLinearActuator> sut_;

    crf::utility::logger::EventLogger logger_;

    const int motorID_ = 1;
};

TEST_F(EtherCATLinearActuatorTests, initializeDeinitializeSequence) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());

    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATLinearActuatorTests, setPositionCorrectly) {
    EXPECT_CALL(*motor_, setPosition(_, _, _, _, _)).Times(1);
    EXPECT_CALL(*motor_, getPosition()).Times(1);

    auto result = sut_->setPosition(5000);
    ASSERT_FALSE(result);
    ASSERT_EQ(result.get_response(), crf::Code::NotInitialized);

    auto position = sut_->getPosition();
    ASSERT_FALSE(position);
    ASSERT_EQ(position.get_response(), crf::Code::NotInitialized);

    ASSERT_TRUE(sut_->initialize());

    result = sut_->setPosition(5000);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());

    position = sut_->getPosition();
    ASSERT_TRUE(position);
    ASSERT_EQ(position.value(), 5000);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(EtherCATLinearActuatorTests, setVelocityCorrectly) {
    EXPECT_CALL(*motor_, setVelocity(_, _, _)).Times(1);
    EXPECT_CALL(*motor_, getVelocity()).Times(1);

    auto result = sut_->setVelocity(5000);
    ASSERT_FALSE(result);
    ASSERT_EQ(result.get_response(), crf::Code::NotInitialized);

    auto velocity = sut_->getVelocity();
    ASSERT_FALSE(velocity);
    ASSERT_EQ(velocity.get_response(), crf::Code::NotInitialized);

    ASSERT_TRUE(sut_->initialize());

    result = sut_->setVelocity(5000);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());

    velocity = sut_->getVelocity();
    ASSERT_TRUE(velocity);
    ASSERT_EQ(velocity.value(), 5000);

    ASSERT_TRUE(sut_->deinitialize());
}
