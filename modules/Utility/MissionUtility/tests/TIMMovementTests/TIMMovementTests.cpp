/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <future>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIM/ITIM.hpp"
#include "TIM/TIMMockConfiguration.hpp"
#include "MissionUtility/TIMMovement/TIMMovement.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::missionutility::TIMMovement;
using crf::actuators::tim::TIMMock;
using crf::actuators::tim::TIMMockConfiguration;

class TIMMovementShould: public ::testing::Test {
 protected:
    TIMMovementShould() :
        logger_("TIMMovementShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        std::string armConfig = __FILE__;
        armConfig = armConfig.substr(0, armConfig.find("MissionUtility"));
        armConfig += "MissionUtility/tests/config/TestParameters.json";
        std::ifstream armTIMDeployableArm(armConfig);
        configFile_ = nlohmann::json::parse(armTIMDeployableArm);

        TIMMockConfig_.configureTIMMock();
        // TIMMovement expects the object to be initialized before-hand
        TIMMockConfig_.getMock()->initialize();
        sut_.reset(
            new TIMMovement(TIMMockConfig_.getMock(), configFile_));
    }

    ~TIMMovementShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    nlohmann::json configFile_;
    TIMMockConfiguration TIMMockConfig_;
    std::unique_ptr<TIMMovement> sut_;
};

TEST_F(TIMMovementShould, startAndStopObstacleDetectionCorrectly) {
    TIMMockConfig_.setTIMStartingPosition(1000);
    TIMMockConfig_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 1010, 1100, 0.3, true));
    sut_->startObstacleDetection();
    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->moveToDCUM(1002);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while (!sut_->isTIMMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    ASSERT_TRUE(sut_->isEnteringObstacle());
    ASSERT_TRUE(sut_->needToRetractForNextObstacle());

    res.get();

    sut_->stopObstacleDetection();
}

TEST_F(TIMMovementShould, moveToDCUMCorrectly) {
    TIMMockConfig_.setTIMStartingPosition(1000);
    ASSERT_TRUE(sut_->moveToDCUM(1001));
}

TEST_F(TIMMovementShould, exitingObstacleCorretly) {
    TIMMockConfig_.setTIMStartingPosition(1137);
    TIMMockConfig_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 1010, 1100, 0.3, true));
    sut_->startObstacleDetection();
    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->moveToDCUM(1138);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while (!sut_->isTIMMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ASSERT_TRUE(sut_->isExitingObstacle());
    ASSERT_TRUE(res.get());
}

TEST_F(TIMMovementShould, inObstacleCorrectly) {
    TIMMockConfig_.setTIMStartingPosition(1050);
    TIMMockConfig_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 1010, 1100, 0.3, true));
    sut_->startObstacleDetection();
    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->moveToDCUM(1052);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while (!sut_->isTIMMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ASSERT_TRUE(sut_->isInObstacle());
    ASSERT_TRUE(res.get());
}

TEST_F(TIMMovementShould, notExitIfObstaclesAreTooClose) {
    TIMMockConfig_.setTIMStartingPosition(1050);
    TIMMockConfig_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(1,
        crf::actuators::tim::LHCObstacleType::NotDefined, 1010, 1040, 0.3, true));
    TIMMockConfig_.TIMAddObstacle(crf::actuators::tim::LHCObstacle(2,
        crf::actuators::tim::LHCObstacleType::NotDefined, 1055, 1065, 0.3, true));
    sut_->startObstacleDetection();
    std::future<bool> res = std::async(std::launch::async, [this]() {
        return sut_->moveToDCUM(1052);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while (!sut_->isTIMMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ASSERT_TRUE(sut_->isEnteringObstacle());
    ASSERT_TRUE(res.get());
}
