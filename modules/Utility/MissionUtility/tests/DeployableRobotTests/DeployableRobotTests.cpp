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

#include "MotionController/IMotionController.hpp"
#include "MotionController/MotionControllerMockConfiguration.hpp"
#include "MissionUtility/DeployableRobot/DeployableRobot.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::missionutility::DeployableRobot;
using crf::control::motioncontroller::MotionControllerMock;
using crf::control::motioncontroller::MotionControllerMockConfiguration;

class DeployableRobotShould: public ::testing::Test {
 protected:
    DeployableRobotShould() :
        logger_("DeployableRobotShould") {
        motionControllerMockConfig_ = std::make_shared<MotionControllerMockConfiguration>(9);
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        std::string armConfig = __FILE__;
        armConfig = armConfig.substr(0, armConfig.find("MissionUtility"));
        armConfig += "MissionUtility/tests/config/TestArmTrayectories.json";
        std::ifstream armDeployableRobot(armConfig);
        configFile_ = nlohmann::json::parse(armDeployableRobot);

        motionControllerMockConfig_->configureMock();
        sut_ = std::make_unique<DeployableRobot>(motionControllerMockConfig_, configFile_);
    }

    ~DeployableRobotShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    nlohmann::json configFile_;
    std::shared_ptr<MotionControllerMockConfiguration> motionControllerMockConfig_;
    std::unique_ptr<DeployableRobot> sut_;
};

TEST_F(DeployableRobotShould, DeployCorrectlyWhenCalled) {
    motionControllerMockConfig_->setInitialPosition(
        crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());
    ASSERT_TRUE(sut_->deploy());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());
}

TEST_F(DeployableRobotShould, RetractCorrectlyWhenCalled) {
    motionControllerMockConfig_->setInitialPosition(
        crf::utility::types::JointPositions({0, -0.2, 0, 0, 0, 0, 0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->isDeployed());
    ASSERT_FALSE(sut_->isRetracted());

    ASSERT_TRUE(sut_->retract());

    ASSERT_TRUE(sut_->isRetracted());
    ASSERT_FALSE(sut_->isDeployed());
}
