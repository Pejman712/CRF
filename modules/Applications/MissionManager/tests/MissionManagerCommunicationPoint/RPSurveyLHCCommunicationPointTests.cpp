/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
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

#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/TCP/TCPSocket.hpp"
#include "Sockets/TCP/TCPServer.hpp"

#include "TIM/TIMMockConfiguration.hpp"
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "TIM/TIMCommunicationPoint/TIMCommunicationPointFactory.hpp"
#include "TIM/TIMClient/TIMClient.hpp"

#include "TIMRPWagon/TIMRPWagonMockConfiguration.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPointFactory.hpp"
#include "TIMRPWagon/TIMRPWagonClient/TIMRPWagonClient.hpp"
#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"
#include "RobotArm/RobotArmMockConfiguration.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"
#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"

#include "RPSensor/RPSensorMockConfiguration.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorCommunicationPointFactory.hpp"
#include "RPSensor/RPSensorClient/RPSensorClient.hpp"

#include "MissionManager/RPSurveyLHC/RPSurveyLHC.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::An;
using testing::Matcher;
using testing::AtLeast;

using crf::communication::sockets::TCPServer;
using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::TCPSocket;
using crf::communication::datapacketsocket::PacketSocket;

using crf::actuators::tim::TIMMockConfiguration;
using crf::actuators::tim::TIMManager;
using crf::actuators::tim::TIMCommunicationPointFactory;
using crf::actuators::tim::TIMClient;

using crf::actuators::timrpwagon::TIMRPWagonMockConfiguration;
using crf::actuators::timrpwagon::TIMRPWagonManager;
using crf::actuators::timrpwagon::TIMRPWagonCommunicationPointFactory;
using crf::actuators::timrpwagon::TIMRPWagonClient;
using crf::utility::missionutility::DeployableTIMRPWagonArm;
using crf::actuators::robotarm::RobotArmMockConfiguration;
using crf::control::robotarmcontroller::RobotArmControllerManager;
using crf::control::robotarmcontroller::RobotArmControllerCommunicationPointFactory;
using crf::control::robotarmcontroller::RobotArmControllerClient;
using crf::utility::missionutility::DeployableRobotArm;
using crf::utility::missionutility::IDeployableDevice;

using crf::sensors::rpsensor::RPSensorMockConfiguration;
using crf::sensors::rpsensor::RPSensorManager;
using crf::sensors::rpsensor::RPSensorCommunicationPointFactory;
using crf::sensors::rpsensor::RPSensorClient;

using crf::applications::missionmanager::rpsurveylhc::RPSurveyLHC;

class RPSurveyLHCCommunicationPointShould: public ::testing::Test {
 protected:
    RPSurveyLHCCommunicationPointShould():
        logger_("RPSurveyLHCCommunicationPointShould"),
        timArmMock_(9),
        rpSensorMock_(std::make_shared<RPSensorMockConfiguration>()) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~RPSurveyLHCCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void InitializeCommunications(bool useRPArmWagon) {
        uint32_t priority = 1;
        float streamerFreq = 0;
        std::chrono::milliseconds replyTimeout(2000);
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find(
            "MissionManagerCommunicationPoint/RPSurveyLHCCommunicationPointTests.cpp"));

        // TIM
        // Communication Point
        int timPort = 14000;
        timMock_.configureTIMMock();
        auto managerTim = std::make_shared<TIMManager>(timMock_.getMock());
        auto factoryTim = std::make_shared<TIMCommunicationPointFactory>(managerTim);
        auto serverSocketTim = std::make_shared<TCPServer>(timPort);
        serverTIM_.reset(new CommunicationPointServer(serverSocketTim, factoryTim));
        if (!serverTIM_->initialize()) {
            logger_->error("Failed to initialize network server");
            return;
        }
        // Client
        auto clientSocketTIM = std::make_shared<TCPSocket>("localhost", timPort);
        auto socketTIM = std::make_shared<PacketSocket>(clientSocketTIM);
        tim_.reset(new TIMClient(socketTIM, replyTimeout, streamerFreq, priority));

        // Deployable Device
        if (useRPArmWagon) {
            // RP Wagon Arm Communication Point
            std::ifstream missionData(testDirName + "config/RPSurveyLHC/testFileRPWagon.json");
            configFile_ = nlohmann::json::parse(missionData);
            int rpArmPort = 13003;
            rpArmMock_.configureTIMRPWagonMock();
            auto managerRPArm = std::make_shared<TIMRPWagonManager>(rpArmMock_.getMock());
            auto factoryRPArm = std::make_shared<TIMRPWagonCommunicationPointFactory>(managerRPArm);
            auto serverSocketRPArm = std::make_shared<TCPServer>(rpArmPort);
            serverRPArm_.reset(new CommunicationPointServer(serverSocketRPArm, factoryRPArm));
            if (!serverRPArm_->initialize()) {
                logger_->error("Failed to initialize network server");
                return;
            }
            // RP Wagon Arm Client
            auto clientSocketRPArm = std::make_shared<TCPSocket>("localhost", rpArmPort);
            auto socketRPArm = std::make_shared<PacketSocket>(clientSocketRPArm);
            rpArm_.reset(new TIMRPWagonClient(socketRPArm, replyTimeout, streamerFreq, priority));
            // Deployable Device Client
            deployable_.reset(new DeployableTIMRPWagonArm(rpArm_));
        } else {
            // TIM Arm Controller Communication Point
            std::ifstream missionData(
                testDirName + "config/RPSurveyLHC/testTIMArmTrayectories.json");
            configFile_ = nlohmann::json::parse(missionData);
            int timArmPort = 13002;
            timArmMock_.configureRobotArmMock();
            auto managerTIMArm = std::make_shared<RobotArmControllerManager>(timArmMock_.getMock());
            auto factoryTIMArm = std::make_shared<RobotArmControllerCommunicationPointFactory>(managerTIMArm);  // NOLINT
            auto serverSocketTIMArm = std::make_shared<TCPServer>(timArmPort);
            serverTIMArm_.reset(new CommunicationPointServer(serverSocketTIMArm, factoryTIMArm));
            if (!serverTIMArm_->initialize()) {
                logger_->error("Failed to initialize network server");
                return;
            }
            // TIM Arm Controller Client
            auto clientSocketTIMArm = std::make_shared<TCPSocket>("localhost", timArmPort);
            auto socketTIMArm = std::make_shared<PacketSocket>(clientSocketTIMArm);
            timArm_.reset(new RobotArmControllerClient(socketTIMArm, replyTimeout, streamerFreq, priority));  // NOLINT
            // Deployable Device Client
            deployable_.reset(new DeployableRobotArm(timArm_, configFile_));
        }

        // RPSensor
        // Communication Point
        std::ifstream missionData(testDirName + "config/RPSurveyLHC/testFileTIMArm.json");
        configFile_ = nlohmann::json::parse(missionData);
        int rpSensorPort = 18000;
        rpSensorMock_->configureMock();
        auto managerRPSensor = std::make_shared<RPSensorManager>(rpSensorMock_);
        auto factoryRPSensor = std::make_shared<RPSensorCommunicationPointFactory>(managerRPSensor);
        auto serverSocketRPSensor = std::make_shared<TCPServer>(rpSensorPort);
        serverRPSensor_.reset(new CommunicationPointServer(serverSocketRPSensor, factoryRPSensor));
        if (!serverRPSensor_->initialize()) {
            logger_->error("Failed to initialize network server");
            return;
        }
        // Client
        auto clientSocketRPSensor = std::make_shared<TCPSocket>("localhost", rpSensorPort);
        auto socketRPSensor = std::make_shared<PacketSocket>(clientSocketRPSensor);
        rpSensor_.reset(new RPSensorClient(socketRPSensor, replyTimeout, streamerFreq, priority));

        // Create Mission
        sut_.reset(new RPSurveyLHC(tim_, deployable_, rpSensor_, configFile_));
    }

    crf::utility::logger::EventLogger logger_;

    TIMMockConfiguration timMock_;
    std::unique_ptr<CommunicationPointServer> serverTIM_;
    std::shared_ptr<TIMClient> tim_;

    TIMRPWagonMockConfiguration rpArmMock_;
    std::unique_ptr<CommunicationPointServer> serverRPArm_;
    std::shared_ptr<TIMRPWagonClient> rpArm_;
    RobotArmMockConfiguration timArmMock_;
    std::unique_ptr<CommunicationPointServer> serverTIMArm_;
    std::shared_ptr<RobotArmControllerClient> timArm_;
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable_;

    std::shared_ptr<RPSensorMockConfiguration> rpSensorMock_;
    std::unique_ptr<CommunicationPointServer> serverRPSensor_;
    std::shared_ptr<RPSensorClient> rpSensor_;

    nlohmann::json configFile_;
    std::shared_ptr<RPSurveyLHC> sut_;
};

/**
 * These tests are disabled as they rely on our communications and the runners cannot
 * assure that they will pass reliably. After performing changes to the mission
 * these tests should be activated and checked in the local computer.
 * (jplayang)
 */

TEST_F(RPSurveyLHCCommunicationPointShould, DISABLED_RunTIMArmFullTestWithoutProblems) {
    InitializeCommunications(false);
    timArmMock_.setPosition(crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}));
    timMock_.setTIMStartingPosition(100);

    nlohmann::json statusJSON;
    statusJSON["surveyData"]["starting_dcum"] = 100;
    statusJSON["surveyData"]["ending_dcum"] = 110;
    statusJSON["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(statusJSON));
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCCommunicationPointShould, DISABLED_RunTIMArmFullTestWithoutProblemsBackwards) {
    InitializeCommunications(false);
    timArmMock_.setPosition(crf::utility::types::JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}));
    timMock_.setTIMStartingPosition(110);

    nlohmann::json statusJSON;
    statusJSON["surveyData"]["starting_dcum"] = 110;
    statusJSON["surveyData"]["ending_dcum"] = 100;
    statusJSON["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(statusJSON));
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCCommunicationPointShould, DISABLED_RunRPWagonArmFullTestWithoutProblems) {
    InitializeCommunications(true);
    rpArmMock_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Retracted);
    timMock_.setTIMStartingPosition(100);

    nlohmann::json statusJSON;
    statusJSON["surveyData"]["starting_dcum"] = 100;
    statusJSON["surveyData"]["ending_dcum"] = 110;
    statusJSON["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(statusJSON));
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}

TEST_F(RPSurveyLHCCommunicationPointShould, DISABLED_RunRPWagonArmFullTestWithoutProblemsBackwards) {  // NOLINT
    InitializeCommunications(true);
    rpArmMock_.setPosition(crf::actuators::timrpwagon::RPArmPosition::Retracted);
    timMock_.setTIMStartingPosition(110);

    nlohmann::json statusJSON;
    statusJSON["surveyData"]["starting_dcum"] = 110;
    statusJSON["surveyData"]["ending_dcum"] = 100;
    statusJSON["surveyData"]["parking_dcum"] = 0;
    ASSERT_TRUE(sut_->setStatus(statusJSON));
    ASSERT_TRUE(sut_->start());  // Initialize
    ASSERT_TRUE(sut_->next());  // Got to start
    ASSERT_TRUE(sut_->next());  // Got to end
}
