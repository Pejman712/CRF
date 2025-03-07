/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <future>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "TIMRPWagon/TIMRPWagonMockConfiguration.hpp"
#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::missionutility::DeployableTIMRPWagonArm;
using crf::actuators::timrpwagon::TIMRPWagonMock;
using crf::actuators::timrpwagon::TIMRPWagonMockConfiguration;

class DeployableTIMRPWagonArmShould: public ::testing::Test {
 protected:
    DeployableTIMRPWagonArmShould() :
        logger_("DeployableTIMRPWagonArmShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        RPWagonMockConfig_.configureTIMRPWagonMock();
        RPWagonMockConfig_.setMovementTime(std::chrono::milliseconds(200));
        sut_.reset(new DeployableTIMRPWagonArm(RPWagonMockConfig_.getMock()));
    }

    ~DeployableTIMRPWagonArmShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    TIMRPWagonMockConfiguration RPWagonMockConfig_;
    std::unique_ptr<DeployableTIMRPWagonArm> sut_;
};

TEST_F(DeployableTIMRPWagonArmShould, DeployCorrectlyWhenCalled) {
    RPWagonMockConfig_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Retracted);

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());

    ASSERT_TRUE(sut_->deploy());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());
}

TEST_F(DeployableTIMRPWagonArmShould, RetractCorrectlyWhenCalled) {
    RPWagonMockConfig_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Deployed);

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());

    ASSERT_TRUE(sut_->retract());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());
}

TEST_F(DeployableTIMRPWagonArmShould, isMovingCalledCorrectly) {
    RPWagonMockConfig_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Retracted);

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());

    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->deploy();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_TRUE(sut_->isMoving());

    ASSERT_TRUE(res.get());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());
}
