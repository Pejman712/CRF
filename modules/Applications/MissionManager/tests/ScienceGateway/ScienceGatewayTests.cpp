/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/
#include <thread>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "MissionManager/ScienceGateway/ScienceGateway.hpp"
#include "MotionController/MotionControllerMockConfiguration.hpp"
#include "MissionUtility/DeployableRobot/DeployableRobot.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::An;
using testing::Matcher;
using testing::AtLeast;

using crf::control::motioncontroller::MotionControllerMockConfiguration;
using crf::utility::missionutility::DeployableRobot;
using crf::applications::missionmanager::sciencegateway::ScienceGateway;

class ScienceGatewayShould: public ::testing::Test {
 protected:
    ScienceGatewayShould():
        logger_("ScienceGatewayShould"),
        motioncontroller_(std::make_shared<MotionControllerMockConfiguration>(7)) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~ScienceGatewayShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0,
            testDirName.find("ScienceGateway/ScienceGatewayTests.cpp"));
        std::ifstream missionData_1(
            testDirName + "config/ScienceGateway/testScienceGatewayTrajectories.json");  // NOLINT
        configFileTraj_ = nlohmann::json::parse(missionData_1);
        motioncontroller_->configureMock();
        deployable_ = std::make_unique<DeployableRobot>(motioncontroller_, configFileTraj_);
        std::ifstream missionData_2(
            testDirName + "config/ScienceGateway/testFileScienceGateway.json");
        configFileMission_ = nlohmann::json::parse(missionData_2);
        sut_.reset(new ScienceGateway(deployable_, configFileMission_));
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable_;
    std::shared_ptr<MotionControllerMockConfiguration> motioncontroller_;

    nlohmann::json configFileTraj_;
    nlohmann::json configFileMission_;
    std::unique_ptr<ScienceGateway> sut_;
};

TEST_F(ScienceGatewayShould, DeployRetractLoop) {
    motioncontroller_->setInitialPosition(
        configFileTraj_["RetractedPosition"].get<crf::utility::types::JointPositions>());
    nlohmann::json missionJson;
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::Easy;
    ASSERT_TRUE(sut_->start());
    // In ini
    ASSERT_TRUE(sut_->setStatus(missionJson));
    // In mission easy
    ASSERT_TRUE(sut_->next());
    // Deploy
    ASSERT_TRUE(sut_->goHome());
    // Retract
    ASSERT_TRUE(sut_->setStatus(missionJson));
    // Select mission easy again
    ASSERT_TRUE(sut_->next());
    // Deploy
    ASSERT_TRUE(sut_->stop());
    // Retract and deini
}

TEST_F(ScienceGatewayShould, ReturnTrueIfInitializeOrDeinitializeMultipleTime) {
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->stop());
    ASSERT_TRUE(sut_->stop());
}

TEST_F(ScienceGatewayShould, ReturnFalseIfWrongOrderOrWrongMissionNum) {
    nlohmann::json missionJson;
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::Easy;
    ASSERT_FALSE(sut_->setStatus(missionJson));
    ASSERT_FALSE(sut_->next());

    ASSERT_TRUE(sut_->start());
    ASSERT_FALSE(sut_->next());
    ASSERT_TRUE(sut_->stop());
    ASSERT_FALSE(sut_->setStatus(missionJson));

    ASSERT_TRUE(sut_->start());
    missionJson["MissionSelect"] =
        crf::applications::missionmanager::sciencegateway::MissionSelect::NotSet;
    ASSERT_TRUE(sut_->setStatus(missionJson));
}
