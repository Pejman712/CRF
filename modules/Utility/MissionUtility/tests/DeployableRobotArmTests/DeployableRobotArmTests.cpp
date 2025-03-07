/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include<fstream>

#include <memory>
#include <future>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "RobotArmController/IRobotArmController.hpp"
#include "RobotArmController/RobotArmControllerMockConfiguration.hpp"
#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::missionutility::DeployableRobotArm;
using crf::control::robotarmcontroller::RobotArmControllerMock;
using crf::control::robotarmcontroller::RobotArmControllerMockConfiguration;

class DeployableRobotArmShould: public ::testing::Test {
 protected:
    DeployableRobotArmShould() :
        logger_("DeployableRobotArmShould"),
        RobotArmControllerMockConfig_(9) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        std::string armConfig = __FILE__;
        armConfig = armConfig.substr(0, armConfig.find("MissionUtility"));
        armConfig += "MissionUtility/tests/config/TestArmTrayectories.json";
        std::ifstream armDeployableRobotArm(armConfig);
        configFile_ = nlohmann::json::parse(armDeployableRobotArm);

        RobotArmControllerMockConfig_.configureRobotArmControllerMock();
        sut_.reset(new DeployableRobotArm(RobotArmControllerMockConfig_.getMock(), configFile_));
    }

    ~DeployableRobotArmShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    nlohmann::json configFile_;
    RobotArmControllerMockConfiguration RobotArmControllerMockConfig_;
    std::unique_ptr<DeployableRobotArm> sut_;
};

TEST_F(DeployableRobotArmShould, DeployCorrectlyWhenCalled) {
    RobotArmControllerMockConfig_.setPosition(
        crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());

    ASSERT_TRUE(sut_->deploy());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());
}

TEST_F(DeployableRobotArmShould, RetractCorrectlyWhenCalled) {
    RobotArmControllerMockConfig_.setPosition(
        crf::utility::types::JointPositions({0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());

    ASSERT_TRUE(sut_->retract());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());
}

TEST_F(DeployableRobotArmShould, isMovingCalledCorrectly) {
    RobotArmControllerMockConfig_.setPosition(
        crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());

    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->deploy();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ASSERT_TRUE(sut_->isMoving());

    ASSERT_TRUE(res.get());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());
}
