/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
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

#include "MissionManager/RPSurveyLHC/RPSurveyLHC.hpp"
#include "TIM/TIMMockConfiguration.hpp"
#include "RobotArmController/RobotArmControllerMockConfiguration.hpp"
#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"
#include "TIMRPWagon/TIMRPWagonMockConfiguration.hpp"
#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"
#include "RPSensor/RPSensorMockConfiguration.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::An;
using testing::Matcher;
using testing::AtLeast;

using crf::actuators::tim::TIMMockConfiguration;
using crf::sensors::rpsensor::RPSensorMockConfiguration;
using crf::actuators::timrpwagon::TIMRPWagonMockConfiguration;
using crf::utility::missionutility::DeployableTIMRPWagonArm;
using crf::control::robotarmcontroller::RobotArmControllerMockConfiguration;
using crf::utility::missionutility::DeployableRobotArm;
using crf::applications::missionmanager::rpsurveylhc::RPSurveyLHC;

class RPSurveyLHCShould: public ::testing::Test {
 protected:
    RPSurveyLHCShould():
        logger_("RPSurveyLHCShould"),
        timArmMock_(9),
        rpSensorMock_(std::make_shared<RPSensorMockConfiguration>()),
        useRPArmWagon_(false) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~RPSurveyLHCShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void InitializeMocks() {
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("RPSurveyLHC/RPSurveyLHCTests.cpp"));
        timMock_.configureTIMMock();
        rpSensorMock_->configureMock();
        if (useRPArmWagon_) {
            std::ifstream missionData(testDirName + "config/RPSurveyLHC/testFileRPWagon.json");
            configFile_ = nlohmann::json::parse(missionData);
            rpArmMock_.configureTIMRPWagonMock();
            deployable_ .reset(new DeployableTIMRPWagonArm(rpArmMock_.getMock()));
        } else {
            std::ifstream missionData(testDirName + "config/RPSurveyLHC/testTIMArmTrayectories.json");  // NOLINT
            configFile_ = nlohmann::json::parse(missionData);
            timArmMock_.configureRobotArmControllerMock();
            deployable_.reset(new DeployableRobotArm(timArmMock_.getMock(), configFile_));
        }
        std::ifstream missionData(testDirName + "config/RPSurveyLHC/testFileTIMArm.json");
        configFile_ = nlohmann::json::parse(missionData);
        sut_.reset(new RPSurveyLHC(
            timMock_.getMock(), deployable_, rpSensorMock_, configFile_));
    }

    void retractDeployableDevice() {
        if (useRPArmWagon_) {
            rpArmMock_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Retracted);
        } else {
            timArmMock_.setPosition(crf::utility::types::JointPositions(
                {0, 0, 0, 0, 0, 0, 0, 0, 0}));
        }
    }

    void deployDeployableDevice() {
        if (useRPArmWagon_) {
            rpArmMock_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Deployed);
        } else {
            timArmMock_.setPosition(crf::utility::types::JointPositions(
                {0, -0.25, 0, 0, 0, 0, 0, 0, 0}));
        }
    }

    crf::utility::logger::EventLogger logger_;

    TIMMockConfiguration timMock_;
    std::shared_ptr<RPSensorMockConfiguration> rpSensorMock_;
    TIMRPWagonMockConfiguration rpArmMock_;
    RobotArmControllerMockConfiguration timArmMock_;
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable_;

    bool useRPArmWagon_;
    nlohmann::json configFile_;
    std::unique_ptr<RPSurveyLHC> sut_;
};

TEST_F(RPSurveyLHCShould, TIMArm_NotInitializeIfSurveyDataIsNotPresent) {
    useRPArmWagon_ = false;
    InitializeMocks();
    ASSERT_FALSE(sut_->start());
}

TEST_F(RPSurveyLHCShould, TIMArm_NotDeployWhenTwoObstaclesAreTooCloseFor) {
    useRPArmWagon_ = false;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(129);
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 120, 121, 1, true));
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 150, 151, 1, true));

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 130;
    dcumjson["surveyData"]["ending_dcum"] = 132;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const std::vector<crf::utility::types::JointPositions>&>(_))).Times(0);
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCShould, TIMArm_NotDeployWhenTwoObstaclesAreTooCloseBackwards) {
    useRPArmWagon_ = false;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(140);
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 120, 121, 1, true));
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 150, 151, 1, true));

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 140;
    dcumjson["surveyData"]["ending_dcum"] = 138;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const std::vector<crf::utility::types::JointPositions>&>(_))).Times(0);
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCShould, TIMArm_RetractWhenGointToStart) {
    useRPArmWagon_ = false;
    InitializeMocks();
    deployDeployableDevice();
    timMock_.setTIMStartingPosition(100);

    nlohmann::json json;
    json["surveyData"]["starting_dcum"] = 102;
    json["surveyData"]["ending_dcum"] = 1;
    json["surveyData"]["parking_dcum"] = 1;
    ASSERT_TRUE(sut_->setStatus(json));
    ASSERT_TRUE(sut_->start());
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(AtLeast(1));
    ASSERT_TRUE(sut_->next());
    while (!sut_->getStatus()["retracted"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, TIMArm_DeployWhenGoingToEnd) {
    useRPArmWagon_ = false;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(60);

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 60;
    dcumjson["surveyData"]["ending_dcum"] = 62;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->next());
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(AtLeast(1));
    ASSERT_TRUE(sut_->next());
    while (!sut_->getStatus()["deployed"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, TIMArm_RetractIfAskedTo) {
    useRPArmWagon_ = false;
    InitializeMocks();
    deployDeployableDevice();

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    nlohmann::json json;
    json["retractArm"] = true;
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(AtLeast(1));
    ASSERT_TRUE(sut_->setStatus(json));
    while (!sut_->getStatus()["retracted"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, TIMArm_DeployIfAskedTo) {
    useRPArmWagon_ = false;
    InitializeMocks();
    retractDeployableDevice();

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    nlohmann::json json;
    json["deployArm"] = true;
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(AtLeast(1));
    ASSERT_TRUE(sut_->setStatus(json));
    while (!sut_->getStatus()["deployed"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, TIMArm_notDeployIfInUnknownPosButRetractPossible) {
    useRPArmWagon_ = false;
    InitializeMocks();
    timArmMock_.setPosition(crf::utility::types::JointPositions({0, 0, 0, 1, 0, 1, 0, 0, -0.5}));

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 10;
    dcumjson["surveyData"]["ending_dcum"] = 1;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());

    nlohmann::json json;
    json["deployArm"] = true;
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(0);
    ASSERT_TRUE(sut_->setStatus(json));
    nlohmann::json json2;
    json2["retractArm"] = true;
    EXPECT_CALL(*timArmMock_.getMock(), setPosition(
        Matcher<const crf::utility::types::JointPositions&>(_))).Times(AtLeast(1));
    ASSERT_TRUE(sut_->setStatus(json2));
}

TEST_F(RPSurveyLHCShould, TIMArm_callEmergencyStopAndInterruptInEmergency) {
    useRPArmWagon_ = false;
    InitializeMocks();
    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    EXPECT_CALL(*timArmMock_.getMock(), interruptTrajectory()).Times(AtLeast(1));
    EXPECT_CALL(*timMock_.getMock(), emergencyStop()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->emergency());
}

TEST_F(RPSurveyLHCShould, TIMArm_pauseCorrectly) {
    useRPArmWagon_ = false;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(28.5);

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 40;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->next());
    EXPECT_CALL(*timMock_.getMock(), stop()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->pause());
    while (sut_->getStatus()["status"] !=
    crf::applications::missionmanager::MissionStatus::Paused) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, RPWagonArm_NotInitializeIfSurveyDataIsNotPresent) {
    useRPArmWagon_ = true;
    InitializeMocks();
    ASSERT_FALSE(sut_->start());
}

TEST_F(RPSurveyLHCShould, RPWagonArm_NotDeployWhenTwoObstaclesAreTooClose) {
    useRPArmWagon_ = true;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(129);
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 120, 121, 1, true));
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 150, 151, 1, true));

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 130;
    dcumjson["surveyData"]["ending_dcum"] = 132;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    EXPECT_CALL(*rpArmMock_.getMock(), deployRPArm()).Times(0);
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCShould, RPWagonArm_NotDeployWhenTwoObstaclesAreTooCloseBackwards) {
    useRPArmWagon_ = true;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(140);
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 120, 121, 1, true));
    timMock_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 150, 151, 1, true));

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 140;
    dcumjson["surveyData"]["ending_dcum"] = 138;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    EXPECT_CALL(*rpArmMock_.getMock(), deployRPArm()).Times(0);
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCShould, RPWagonArm_RetractWhenGointToStart) {
    useRPArmWagon_ = true;
    InitializeMocks();
    timMock_.setTIMStartingPosition(100);
    deployDeployableDevice();

    nlohmann::json json;
    json["surveyData"]["starting_dcum"] = 102;
    json["surveyData"]["ending_dcum"] = 100;
    json["surveyData"]["parking_dcum"] = 1;
    ASSERT_TRUE(sut_->setStatus(json));
    ASSERT_TRUE(sut_->start());
    EXPECT_CALL(*rpArmMock_.getMock(), retractRPArm()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->next());
    while (!sut_->getStatus()["retracted"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, RPWagonArm_DeployWhenGoingToEnd) {
    useRPArmWagon_ = true;
    InitializeMocks();
    retractDeployableDevice();
    timMock_.setTIMStartingPosition(60);

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 60;
    dcumjson["surveyData"]["ending_dcum"] = 62;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->next());
    EXPECT_CALL(*rpArmMock_.getMock(), deployRPArm()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->next());
    while (!sut_->getStatus()["deployed"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, RPWagonArm_RetractIfAskedTo) {
    useRPArmWagon_ = true;
    InitializeMocks();
    deployDeployableDevice();

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    nlohmann::json json;
    json["retractArm"] = true;
    EXPECT_CALL(*rpArmMock_.getMock(), retractRPArm()).Times(1);
    ASSERT_TRUE(sut_->setStatus(json));
    while (!sut_->getStatus()["retracted"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, RPWagonArm_DeployIfAskedTo) {
    useRPArmWagon_ = true;
    InitializeMocks();
    retractDeployableDevice();

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    nlohmann::json json;
    json["deployArm"] = true;
    EXPECT_CALL(*rpArmMock_.getMock(), deployRPArm()).Times(1);
    ASSERT_TRUE(sut_->setStatus(json));
    while (!sut_->getStatus()["deployed"]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST_F(RPSurveyLHCShould, RPWagonArm_notDeployIfInUnknownPosButYesRetract) {
    useRPArmWagon_ = true;
    InitializeMocks();
    rpArmMock_.setPosition(crf::actuators::timrpwagon::RPArmPosition::InTheMiddle);

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 10;
    dcumjson["surveyData"]["ending_dcum"] = 1;
    dcumjson["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());

    nlohmann::json json;
    json["deployArm"] = true;
    ASSERT_TRUE(sut_->setStatus(json));
    nlohmann::json json2;
    json2["retractArm"] = true;
    EXPECT_CALL(*rpArmMock_.getMock(), retractRPArm()).Times(1);
    ASSERT_TRUE(sut_->setStatus(json2));
}

TEST_F(RPSurveyLHCShould, RPWagonArm_callEmergencyStopAndInterruptInEmergency) {
    useRPArmWagon_ = true;
    InitializeMocks();
    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 50;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    EXPECT_CALL(*rpArmMock_.getMock(), stopRPArm()).Times(AtLeast(1));
    EXPECT_CALL(*timMock_.getMock(), emergencyStop()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->emergency());
}

TEST_F(RPSurveyLHCShould, RPWagonArm_pauseCorrectly) {
    useRPArmWagon_ = true;
    InitializeMocks();
    timMock_.setTIMStartingPosition(28.5);
    retractDeployableDevice();

    nlohmann::json dcumjson;
    dcumjson["surveyData"]["starting_dcum"] = 40;
    dcumjson["surveyData"]["ending_dcum"] = 42;
    dcumjson["surveyData"]["parking_dcum"] = 200;
    ASSERT_TRUE(sut_->setStatus(dcumjson));
    ASSERT_TRUE(sut_->start());
    ASSERT_TRUE(sut_->next());
    EXPECT_CALL(*timMock_.getMock(), stop()).Times(AtLeast(1));
    ASSERT_TRUE(sut_->pause());
    while (sut_->getStatus()["status"] !=
    crf::applications::missionmanager::MissionStatus::Paused) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
